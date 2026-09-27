# freertos-hil-motor-control

A distributed real-time control system built on FreeRTOS, tested on a
**Hardware-in-the-Loop (HIL) bench**: an STM32 controller drives a DC motor
that is simulated in real time by an ESP32, while a second ESP32 supervises
the whole system and streams it to a PC dashboard.

The goal is not only a working system. The timing behaviour of every task and
every link is **computed analytically first, then verified by measurement**
on the hardware.

**Status:** Phase 1 complete (STM32 real-time core, first timing measurements).
Phase 0 is waiting for a board modification to switch the clock from HSI to HSE.

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
|---|---|
| **STM32L476RG** | Real-time controller: speed control loop, state machine, fault detection, timing instrumentation. Its firmware is identical to what a real motor would need. |
| **ESP32 #1** | Plant simulator: fixed-step DC motor model, measures the controller's PWM and generates real quadrature encoder signals. Supports fault injection. |
| **ESP32 #2** | Supervisor: receives system state and events, detects node loss, timestamps and forwards everything to the PC. |
| **PC** | Dashboard: live curves, faults, task timeline. |

### Design principles

- Every task exists for one reason; priorities are derived from deadlines, not guessed.
- Interrupt handlers do the minimum and defer work to tasks.
- No dynamic allocation: every kernel object is created statically, and heap usage is monitored at run time.
- No stateful newlib calls (`printf`, `malloc`, `strtok`) from tasks, so `USE_NEWLIB_REENTRANT` is not needed.
- Faults are fail-stop and visible: `Error_Handler`, stack-overflow and malloc-failed hooks freeze the system and light LD2.
- The supervisor observes but never drives the real-time loop: if Wi-Fi or the
  PC fails, the control loop keeps running safely.
- Emergency stop is hardware, local to the controller, never through a link.
- What is not measured is not claimed.

## Results

All measurements: GCC `-O2` unless stated, SYSCLK 80 MHz from **HSI** (until the
board modification), FreeRTOS 10.3.1, tick 1 kHz.
Instruments: DWT cycle counter (12.5 ns), logic analyser (FX2, up to 24 MS/s), CubeIDE Live Expressions.

### Clock

| Metric | Predicted | Measured |
|---|---|---|
| MCO (SYSCLK/16) on PA8, HSE source | 5 MHz | not available: HSE not connected on unmodified board |
| MCO (SYSCLK/16) on PA8, HSI source | 5 MHz | 5.0035 MHz → SYSCLK **80.06 MHz (+0.07 %)**, resolution ±0.02 % |

![MCO on HSI, PulseView timing decoder](captures/mco-hsi-timing.png)

### Scheduling (task A: C=2 ms, T=10 ms; task B: C=5 ms, T=25 ms; deadline-monotonic priorities)

Measured with `-O0`, logic analyser at 1 MS/s.

| Metric | Predicted | Measured |
|---|---|---|
| Task B response, not preempted | 5 ms | 4.998–4.999 ms (HSI running 0.07 % fast) |
| Task B worst-case response, preempted once by A | 7 ms (naive model) | **7.010–7.011 ms** |
| Preemption overhead (tick + 2 context switches + delay call) | not modelled | ~10 µs (first estimate) |
| Task B start jitter | ±2 ms (= C_A) | ±2.02 ms |

![Task A (D0) and task B (D1), PulseView timing decoder](captures/task-b-response-timing.png)

### Kernel overhead

Notify-to-wake latency: a low-priority task timestamps with DWT, calls
`xTaskNotifyGive`, and a higher-priority task blocked in `ulTaskNotifyTake`
reads DWT when it wakes.

| Build | Samples | Min | Avg | Max |
|---|---|---|---|---|
| `-O0` | 13 841 | 987 cycles (12.3 µs) | 987 | 987 |
| `-O2` | 11 488 | **503 cycles (6.3 µs)** | 519 | 556 cycles (7.0 µs) |

### Memory

| Metric | Measured |
|---|---|
| Footprint, FreeRTOS enabled, no application tasks (`-O0`) | FLASH 18.91 KB / 1024 KB, RAM 8.98 KB / 96 KB, RAM2 0 / 32 KB |
| FreeRTOS heap usage | 0 allocations: heap_4 never initialised, malloc-failed hook never hit |
| Stack high-water mark, tasks A / B / health (256 words each) | 222 / 223 / 222 words free (~34 used, no FPU context yet) |

### Key findings so far

1. **The naive response-time model is optimistic.** `R = C_B + ⌈R/T_A⌉·C_A` predicts 7 ms; the
   measurement is 7.011 ms. The model ignores kernel preemption overhead, which must be measured
   and added to the analysis.
2. **A measured maximum is not a bound.** At `-O0`, every one of 13 841 latency samples was
   identical because the tick, the HAL time base and all task releases are phase-locked: no
   interrupt ever fell inside the measurement window. Only analysis gives a worst case.
3. **The binary matters.** The same kernel on the same hardware wakes a task twice as fast
   at `-O2` as at `-O0`. All published numbers now come from the `-O2` build.
4. **Clock error propagates.** The +0.07 % HSI error measured on MCO reappears in task timing:
   a 5 ms busy-wait counted in cycles lasts 4.9965 ms.
5. **Open question:** at `-O2` the latency varies by 53 cycles (503–556). Two hypotheses,
   a tick interrupt occasionally inside the window or instruction-cache state, to be
   decided with a latency histogram.

## Hardware

- NUCLEO-L476RG (MB1136 C-05). **Modification planned**: SB16 and SB50 ON, SB55 OFF, so that
  HSE receives the 8 MHz MCO from the ST-LINK. On this board the MCO is not connected to OSC_IN
  by default, whatever the revision (UM1724 §6.7.1). Until then the PLL runs from HSI.
- 2 × ESP32
- Potentiometer (speed setpoint), push buttons, LEDs, I2C RTC module (fault timestamps)
- Logic analyser (8 ch, 24 MS/s)

### Pin map (STM32)

| Pin | Arduino | Function |
|---|---|---|
| PA8 | D7 | MCO (SYSCLK/16) |
| PA10 | D2 | `TP_TASK_A` timing pin |
| PB5 | D4 | `TP_TASK_B` timing pin |
| PB4 | D5 | `TP_TASK_C` timing pin |
| PB10 | D6 | `TP_ISR` timing pin |
| PA5 | D13 | `LED_FAULT` (LD2) |
| PA13 / PA14 / PB3 | — | SWD + SWO (reserved) |

## Toolchain

- STM32: STM32CubeIDE, CubeMX, HAL, FreeRTOS (native API; CMSIS-RTOS v2 layer only where CubeMX requires it)
- ESP32: ESP-IDF (FreeRTOS SMP)
- PC: Python, PulseView / sigrok

## Roadmap

### Phase 0 — Foundations
- [x] Repository structure
- [x] CubeMX project for NUCLEO-L476RG
- [x] Clock tree designed: HSE bypass → PLL → 80 MHz, MCO on PA8
- [x] HSE start-up failure diagnosed with debugger and UM1724
- [x] Temporary clock: HSI → PLL → 80 MHz, verified on MCO
- [ ] Board modification (SB16, SB50 ON, SB55 OFF)
- [ ] Switch back to HSE and repeat the MCO measurement

### Phase 1 — STM32 real-time core
- [x] FreeRTOS, native API, static allocation only
- [x] HAL time base moved to TIM6, SysTick left to the kernel
- [x] Two periodic tasks with dedicated timing pins
- [x] DWT cycle counter as the measurement base
- [x] First response-time analysis, predicted then measured
- [x] Health task: heap usage and stack high-water marks
- [x] Fail-stop fault handling with visible LED
- [x] Notify-to-wake latency measured at `-O0` and `-O2`
- [ ] Latency histogram to explain the `-O2` variation

### Phase 2 — Controller I/O
- [ ] PWM output to the motor (timer)
- [ ] Quadrature encoder input (timer in encoder mode)
- [ ] Setpoint acquisition: timer-triggered ADC with DMA
- [ ] ISR → task hand-off with task notifications

### Phase 3 — Plant simulator (ESP32 #1)
- [ ] DC motor model, fixed-step integration, parameters from a real motor datasheet
- [ ] PWM duty measurement
- [ ] Quadrature signal generation
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
- [ ] Hardware emergency stop, clock-loss fault (CSS)

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
firmware/    STM32 controller (STM32CubeIDE project)
tools/       Scripts and helpers
captures/    Logic analyser captures
```

Planned: `simulator/` (ESP32 #1), `supervisor/` (ESP32 #2), `dashboard/` (PC).
