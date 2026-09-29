# freertos-hil-motor-control

A distributed real-time control system built on FreeRTOS, tested on a **Hardware-in-the-Loop (HIL) bench**: an STM32 controller drives a DC motor
that is simulated in real time by an ESP32, while a second ESP32 supervises
the whole system and streams it to a PC dashboard.

The goal is not only a working system. The timing behaviour of every task and
every link is **computed analytically first, then verified by measurement** on the hardware.

**Status:** Phases 0 and 1 complete (clock on HSE, STM32 real-time core). Phase 2 nearly complete:
setpoint acquisition, motor PWM and encoder input work, speed estimate next.
Phase 3 started: the quadrature source on ESP32 #1 works and is checked against the STM32 time base.

## Architecture

```
ESP32 #1                     STM32L476RG                    ESP32 #2
Plant simulator   <-- PWM ---  Real-time controller  -- UART -->  Supervisor
(DC motor model)  -- A/B  -->  (FreeRTOS)                         & gateway
                   quadrature                                        |
                                                                   Wi-Fi
                                                                     |
                                                                PC dashboard
```

| Node | Role |
| --- | --- |
| **STM32L476RG** | Real-time controller: speed control loop, state machine, fault detection, timing instrumentation. Its firmware is identical to what a real motor would need. |
| **ESP32 #1** | Plant simulator: fixed-step DC motor model, measures the controller's PWM and generates real quadrature encoder signals. Supports fault injection. |
| **ESP32 #2** | Supervisor: receives system state and events, detects node loss, timestamps and forwards everything to the PC. |
| **PC** | Dashboard: live curves, faults, task timeline. |

### Design principles

- Every task exists for one reason; priorities are derived from deadlines, not guessed.
- Interrupt handlers do the minimum and defer work to tasks.
- No dynamic allocation: every kernel object is created statically, and heap usage is monitored at run time.
- No stateful newlib calls (`printf`, `malloc`, `strtok`) from tasks, so `USE_NEWLIB_REENTRANT` is not needed.
- Faults are fail-stop and visible: `Error_Handler`, the stack-overflow and malloc-failed hooks, the NMI
and the clock-failure callback freeze the system, light LD2 and force the motor PWM output low
(`Safe_State`, register level). Stopping the CPU alone would leave TIM1 driving the motor at the last duty cycle.
- The supervisor observes but never drives the real-time loop: if Wi-Fi or the
PC fails, the control loop keeps running safely.
- Emergency stop is hardware, local to the controller, never through a link.
- What is not measured is not claimed.

## Results

All measurements: GCC `-O2` unless stated, FreeRTOS 10.3.1, tick 1 kHz, SYSCLK 80 MHz.
The clock source is stated per section: **HSI** for the early measurements (clock, scheduling,
PWM), **HSE** (8 MHz from the ST-LINK MCO, bypass) from the encoder work on.
Instruments: DWT cycle counter (12.5 ns), logic analyser (FX2, up to 24 MS/s), CubeIDE Live Expressions.

### Clock

| Metric | Predicted | Measured |
| --- | --- | --- |
| MCO (SYSCLK/16) on PA8, HSI source | 5 MHz | 5.0035 MHz → SYSCLK **80.06 MHz (+0.07 %)**, resolution ±0.02 % |
| `RCC_CR` with HSE bypass (HSEON, HSEBYP, HSERDY, CSSON, PLLON, PLLRDY = 1) | 0x0F0F0063 | 0x0F0F0063 |
| `RCC_PLLCFGR` (PLLSRC = HSE, M = 1, N = 20, R = 2, PLLREN = 1) | 0x01001403 | 0x01001403 |
| `RCC_CFGR` (SW = SWS = PLL, AHB/APB /1, MCO = SYSCLK/16) | 0x4100000F | 0x4100000F |
| MCO (SYSCLK/16) on PA8, HSE source | 5.0000 MHz (analyser crystal limits the resolution) | <<FILL: measured MCO frequency>> |
| Generator (ESP32 crystal) vs STM32 clock, 189.58 s | \|error\| < 50 ppm | **+18.2 ppm** (±0.5), see *Encoder input* |

[![MCO on HSI, PulseView timing decoder](captures/mco-hsi-timing.png)](captures/mco-hsi-timing.png)

The HSE error is about 40 times smaller than the HSI error measured earlier (+700 ppm).
The board needed SB16 and SB50 ON and SB55 OFF, because the ST-LINK MCO is not connected to OSC_IN
by default (UM1724 §6.7.1).

### Scheduling (task A: C=2 ms, T=10 ms; task B: C=5 ms, T=25 ms; deadline-monotonic priorities)

Measured on HSI with `-O0`, logic analyser at 1 MS/s.

| Metric | Predicted | Measured |
| --- | --- | --- |
| Task B response, not preempted | 5 ms | 4.998–4.999 ms (HSI running 0.07 % fast) |
| Task B worst-case response, preempted once by A | 7 ms (naive model) | **7.010–7.011 ms** |
| Preemption overhead (tick + 2 context switches + delay call) | not modelled | ~10 µs (first estimate) |
| Task B start jitter | ±2 ms (= C\_A) | ±2.02 ms |

[![Task A (D0) and task B (D1), PulseView timing decoder](captures/task-b-response-timing.png)](captures/task-b-response-timing.png)

### Kernel overhead

Notify-to-wake latency: a low-priority task timestamps with DWT, calls `xTaskNotifyGive`, and a higher-priority task blocked in `ulTaskNotifyTake` reads DWT when it wakes.

| Build | Samples | Min | Avg | Max |
| --- | --- | --- | --- | --- |
| `-O0` | 13 841 | 987 cycles (12.3 µs) | 987 | 987 |
| `-O2` | 11 488 | **503 cycles (6.3 µs)** | 519 | 556 cycles (7.0 µs) |

### Controller I/O

Setpoint chain: TIM3 (10 kHz) triggers ADC1 in hardware, circular DMA fills 2 × 10
samples, and each half-transfer interrupt notifies the acquisition task, which averages
the block (1 kHz setpoint). The task itself starts the hardware, so no interrupt can
ever notify a task that does not exist yet. Motor PWM: TIM1 CH2, 20 kHz, 4000 steps,
compare preload enabled; currently driven open-loop from the setpoint.

| Metric | Predicted | Measured |
| --- | --- | --- |
| Setpoint acquisition rate | 1000 blocks/s, 0 late, 0 overrun | 1000 blocks/s, 0 late, 0 overrun |
| ISR-to-task latency (DMA ISR → `xTaskNotifyFromISR` → task), steady state, binary before the encoder input, HSI | — | min 776 / avg 815 / max **847 cycles** (9.7 / 10.2 / 10.6 µs) |
| Same, first wake-ups after reset (cold instruction cache) | — | up to 1432 cycles (17.9 µs) |
| Same, binary with the encoder input, HSE, 189 577 samples | — | min 815 / avg 863 / max **1599 cycles** (10.2 / 10.8 / 20.0 µs), `lateBlocks` 0, `overruns` 0. Cause of the higher maximum not identified. |
| Same, binary with the encoder input and the fail-stop code (`Safe_State`), HSE, 340 868 samples | — | min 809 / avg 857 / max **1579 cycles** (10.1 / 10.7 / 19.7 µs). Another binary, other numbers. |
| PWM frequency (TIM1 CH2, PA9), HSI | 20 kHz (20.014 kHz with HSI +0.07 %) | 20.017 kHz on one period, ±0.08 % (analyser: 1199 samples/period) |
| PWM duty follows setpoint (open loop) | proportional | 31.7 / 67.6 / 74.2 % at three positions, ±1 sample (0.08 %) |
| PWM period while duty changes (preload enabled) | constant | constant, no glitch observed |

[![PWM at 68 % duty, PulseView PWM decoder](captures/pwm-duty-68.png)](captures/pwm-duty-68.png)

### Encoder input (TIM2, x4) and quadrature source

Source: ESP32-S3 (ESP32 #1), MCPWM, 5 kHz per channel, channel A leading B by 90°
(two operators, three comparators, no CPU involved once started).
It was first verified alone with the ESP32's own PCNT in loopback (same crystal for the generator,
the PCNT and the time base, so this checks the logic, not the absolute frequency).
Then it was read by the STM32 on TIM2 (PA15 / PA1, 100 ns input filter, x4 mode), timed by the
STM32's own clock: the count and a DWT cycle stamp are read back-to-back once per 1 ms window.

| Metric | Predicted | Measured |
| --- | --- | --- |
| PCNT loopback, 5 kHz, count over the measured window (999 903 µs) | +19 998 (20 000 × 0.999903) | +19 998 |
| Same, channels swapped (direction check) | −19 998 | −19 998 |
| Same, 4 kHz (window 999 902 µs) | +15 998.4 | +15 999 |
| STM32: counts per 1 ms window | 20 | 20 (min 19, max 21) |
| STM32: window length | 80 000 cycles | 80 000.0007 cycles |
| STM32: total count over 189.58 s (189 577 windows) | 3 791 540.0 if the clocks were equal | 3 791 609 → **+18.2 ppm** (±0.5) |

The min 19 / max 21 come from the read instant (the ISR-to-task latency varies by about 0.9 µs),
not from the generator: the cumulative count is what compares the clocks.
The +18 ppm is the difference between the two crystals (ST-LINK 8 MHz vs ESP32 40 MHz);
it cannot say which one is off without a third reference.

### Fail-stop (safe state)

`Safe_State()` sets `TIM1->BDTR.MOE` to 0, drives PA9 low, switches PA9 to a plain output and lights LD2.
It works at register level (no HAL or RTOS call), so it can run from a task, an interrupt or the NMI.
Each path was exercised 5 s after start by a temporary self-test build (`CSS_SELFTEST`, 0 in the reference build)
and read back with the core running, from the registers by address.

| State, read 5 s after start | `g_acq.blocks` | PA9 mode | TIM1 MOE | LD2 |
| --- | --- | --- | --- | --- |
| Running (reference build) | counting | 2 (alternate function) | 1 | off |
| `Error_Handler()` called from the acquisition task | frozen at 5000 | 1 (plain output) | 0 | on |
| CSS callback `HAL_RCC_CSSCallback()` called from the acquisition task | frozen at 5000 | 1 | 0 | on |
| Software NMI (`NMI_Handler`, then `Safe_State`) | frozen at 5000 | 1 | 0 | on |

PA9 in mode 1 with MOE at 0 can only come from `Safe_State`: the rest of the firmware leaves PA9 in mode 2.
The pin level was read as 0 in the input register; it was not measured with an instrument, and a single read of
a 64 % PWM would not prove anything on its own.

Not exercised: a real loss of the 8 MHz (the CSS event itself, then the NMI with the CSS flag set), and the two
FreeRTOS hooks, which call `Error_Handler()` (checked by reading the code, not triggered).
The behaviour before `Safe_State` existed was not measured either, so no defect is claimed here.

### Memory

| Metric | Measured |
| --- | --- |
| Footprint, FreeRTOS enabled, no application tasks (`-O0`) | FLASH 18.91 KB / 1024 KB, RAM 8.98 KB / 96 KB, RAM2 0 / 32 KB |
| FreeRTOS heap usage | 0 allocations: heap\_4 never initialised, malloc-failed hook never hit |
| Stack high-water mark, tasks A / B / health (256 words each) | 222 / 223 / 222 words free (~34 used, no FPU context yet) |

### Key findings so far

1. **The naive response-time model is optimistic.** `R = C_B + ⌈R/T_A⌉·C_A` predicts 7 ms; the
measurement is 7.011 ms. The model ignores kernel preemption overhead, which must be measured
and added to the analysis.
2. **A measured maximum is not a bound.** At `-O0`, every one of 13 841 latency samples was
identical because the tick, the HAL time base and all task releases are phase-locked: no
interrupt ever fell inside the measurement window. TIM3 is derived from the same clock, so the
DMA interrupt is phase-locked too. Only analysis gives a worst case.
3. **The binary matters.** The same kernel on the same hardware wakes a task twice as fast
at `-O2` as at `-O0`, and even an unrelated code change moved the ISR-to-task minimum from
795 to 776 cycles by shifting code in flash. Adding the encoder input moved the maximum from
847 to 1599 cycles. Numbers are valid for one binary only.
4. **Start-up is not steady state.** The first wake-ups after reset cost up to 1432 cycles
(cold instruction cache) against 847 in steady state. Statistics skip a 10-sample warm-up.
5. **A debugger stops the core, not the peripherals.** Halting the core let the DMA overwrite
both buffer halves (4 late blocks). TIM3, TIM1 and TIM2 are now frozen on debug halt; for TIM1 this
also disables the PWM outputs, so a halted controller never keeps driving the motor.
6. **Clock error propagates.** The +0.07 % HSI error measured on MCO reappears in task timing:
a 5 ms busy-wait counted in cycles lasts 4.9965 ms.
7. **Know the instrument's resolution.** At 24 MS/s the analyser resolves the PWM duty to
1/1199 (0.08 %), coarser than the PWM itself (1/4000). A 0.08 % step between periods is the
instrument, not the signal. A count over about 1 s resolves a rate to about ±1 count, which is
50 ppm at 20 000 counts/s: only a long accumulation reaches 1 ppm.
8. **A reading is only meaningful together with the instant it was taken.** `PLLCFGR` read with the
core halted before `SystemClock_Config` returned the reset value, contradicting `CR`.
Live Expressions reads variables one after another while the program runs: with 1 ms between two
reads, two counters that must be equal differed, and the derived ppm was wrong (1 ms out of 90 s is
11 ppm). Coherent readings are now taken with the core halted: TIM2 is frozen on halt and DWT stops
with the core, so the count and the cycle counter stay consistent.
9. **Predict with the measured window, not the nominal one.** A 1 s delay lasts 999 903 µs, so 20 000
counts/s gives 19 998, not 20 000. Comparing rates (count divided by measured time) removes the effect.
10. **A clone must reproduce the configuration.** `sdkconfig` is generated and ignored by the ESP-IDF
template, so a clone silently fell back to defaults (160 MHz, 2 MB flash, 100 Hz tick, wrong target).
The chosen settings now live in `esp32-motor-sim/sdkconfig.defaults`, and a fresh clone is built and flashed
to check it.

**Open questions**

- Task-to-task latency varies by 53 cycles at `-O2` (503–556): tick interrupt inside the window
or instruction-cache state, to be decided with a latency histogram.
- ISR-to-task maximum is 1599 cycles with the encoder binary (815 / 863 / 1599 on 189 577 samples,
identical over three runs of different length) and 1579 with the fail-stop binary (809 / 857 / 1579 on
340 868 samples). A deterministic cause is likely; not identified.
- The ADC overrun counter has never been seen to increment: the error path must be exercised
by forcing an overrun (phase 5).
- The Clock Security System is enabled and its callback goes to the safe state. The callback and the NMI
path were exercised by software; a real loss of the 8 MHz (removing SB16 while running) has not been tested,
so the detection path is not claimed.
- The FreeRTOS hooks (stack overflow, malloc failed) call `Error_Handler()`; they have not been triggered.
- PLLSAI1 stays running because the ADC clock mux points to it in CubeMX, although the ADC uses HCLK/4.
- The MCO measurement on HSE is a check of the analyser's crystal as much as of the STM32's.

## Hardware

- NUCLEO-L476RG (MB1136 C-05). **Modification done**: SB16 and SB50 ON, SB55 OFF, so that
HSE receives the 8 MHz MCO from the ST-LINK (bypass mode). On this board the MCO is not connected
to OSC_IN by default, whatever the revision (UM1724 §6.7.1).
- 2 × ESP32. ESP32 #1 is an ESP32-S3 (QFN56, revision v0.2, 16 MB flash, 8 MB embedded PSRAM).
- Potentiometer (speed setpoint): ends on 3V3 and GND, wiper on A0. Never on 5 V.
- Push buttons, LEDs, I2C RTC module (fault timestamps)
- Logic analyser (8 ch, 24 MS/s)

### Pin map (STM32)

| Pin | Arduino | Function |
| --- | --- | --- |
| PA0 | A0 | Setpoint potentiometer (ADC1 IN5) |
| PA9 | D8 | Motor PWM (TIM1 CH2, 20 kHz) |
| PA15 | — | Encoder channel A (TIM2 CH1, AF1) |
| PA1 | A1 | Encoder channel B (TIM2 CH2, AF1) |
| PA6 | D12 | Reserved: TIM1 break input (hardware emergency stop, phase 5) |
| PA8 | D7 | MCO (SYSCLK/16) |
| PA10 | D2 | `TP_TASK_A` timing pin |
| PB5 | D4 | `TP_TASK_B` timing pin |
| PB4 | D5 | `TP_TASK_C` timing pin (acquisition task) |
| PB10 | D6 | `TP_ISR` timing pin (DMA interrupt) |
| PA5 | D13 | `LED_FAULT` (LD2) |
| PA13 / PA14 / PB3 | — | SWD + SWO (reserved) |

### Pin map (ESP32-S3, ESP32 #1)

| GPIO | Function |
| --- | --- |
| 4 | Quadrature channel A output (MCPWM) → STM32 PA15 |
| 5 | Quadrature channel B output (MCPWM) → STM32 PA1 |
| 6 / 7 | PCNT test inputs, wired to GPIO 4 / 5 for the standalone loopback test only |
| GND | Common with the STM32 board. The 3V3 rails are never connected together. |

## Toolchain

- STM32: STM32CubeIDE 2.2.0, CubeMX, HAL, FreeRTOS 10.3.1 (native API; CMSIS-RTOS v2 layer only where CubeMX requires it)
- ESP32: ESP-IDF v6.1 (FreeRTOS SMP), target `esp32s3`, built from VS Code with the ESP-IDF extension
- PC: Python, PulseView / sigrok

### Building the ESP32 project

```
cd esp32-motor-sim
idf.py build
idf.py -p <port> flash monitor
```

Target, CPU frequency (240 MHz), flash size (16 MB) and tick rate (1 kHz) come from
`sdkconfig.defaults`; `sdkconfig` itself is generated and not versioned.

## Roadmap

### Phase 0 — Foundations

- [x] Repository structure
- [x] CubeMX project for NUCLEO-L476RG
- [x] Clock tree designed: HSE bypass → PLL → 80 MHz, MCO on PA8
- [x] HSE start-up failure diagnosed with debugger and UM1724
- [x] Temporary clock: HSI → PLL → 80 MHz, verified on MCO
- [x] Board modification (SB16, SB50 ON, SB55 OFF)
- [x] Switch back to HSE, verified by registers, and repeat the MCO measurement

### Phase 1 — STM32 real-time core

- [x] FreeRTOS, native API, static allocation only
- [x] HAL time base moved to TIM6, SysTick left to the kernel
- [x] Two periodic tasks with dedicated timing pins
- [x] DWT cycle counter as the measurement base
- [x] First response-time analysis, predicted then measured
- [x] Health task: heap usage and stack high-water marks
- [x] Fail-stop fault handling with visible LED
- [x] Notify-to-wake latency measured at `-O0` and `-O2`
- [ ] Latency histogram to explain the `-O2` variation and the 1599-cycle ISR maximum

### Phase 2 — Controller I/O

- [x] Setpoint acquisition: timer-triggered ADC, circular DMA, double buffer
- [x] ISR → task hand-off with task notifications, latency measured
- [x] PWM output to the motor (TIM1, 20 kHz, preload), open loop from setpoint
- [x] TIM1 / TIM2 / TIM3 frozen on debug halt
- [x] Quadrature encoder input (TIM2, encoder mode x4), checked against an independent time base
- [ ] Speed estimate from the count (fixed window; resolution 60 000 / (4·N) rpm per count at 1 ms, N = pulses per revolution)
- [ ] Health task extended to the new tasks' stacks
- [ ] Phase-1 experiment tasks (A, B, ctxRx, ctxTx) behind a build switch

### Phase 3 — Plant simulator (ESP32 #1)

- [ ] DC motor model, fixed-step integration, parameters from a real motor datasheet
- [ ] PWM duty measurement
- [x] Quadrature signal generation (MCPWM, verified with the PCNT and read by the STM32)
- [ ] Simulator characterised: step jitter, signal timing accuracy

### Phase 4 — Closed-loop control

- [ ] Speed control loop (PI) at a fixed rate
- [ ] Shared state protected by mutex, priority inversion reproduced and resolved
- [ ] I2C RTC as slow peripheral task
- [ ] Step response validated against the model

### Phase 5 — Fault management

- [ ] State machine: normal / alert / fault
- [ ] Fault injection from the simulator: stall, encoder loss, noise
- [ ] Detection deadlines specified and verified
- [ ] Hardware emergency stop
- [x] Safe state (motor output forced low, LD2 on) reachable from `Error_Handler`, the NMI and the CSS callback
- [ ] Clock-loss fault (CSS): callback and NMI path exercised by software; test with a real loss of the HSE

### Phase 6 — Supervisor and links (ESP32 #2)

- [ ] Framed UART protocol: sequence numbers, CRC, heartbeat
- [ ] Node-loss detection
- [ ] Wi-Fi gateway to the PC

### Phase 7 — Dashboard

- [ ] Live setpoint and speed curves
- [ ] Fault log with timestamps
- [ ] Task timeline built from traces

### Phase 8 — Timing analysis and measurement campaign

- [ ] Response-time analysis including measured kernel overhead
- [ ] End-to-end latency, from fault to dashboard
- [ ] CPU load per task, trace intrusiveness

### Phase 9 — Hardening

- [ ] Watchdog, degraded modes
- [ ] Long-run tests (24 h)
- [ ] Final documentation

## Repository layout

```
firmware/          STM32 controller (STM32CubeIDE project)
esp32-motor-sim/   ESP32 #1 plant simulator (ESP-IDF project)
tools/             Scripts and helpers
captures/          Logic analyser captures
```

Planned: `supervisor/` (ESP32 #2), `dashboard/` (PC).
