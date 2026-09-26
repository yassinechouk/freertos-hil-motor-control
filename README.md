# freertos-hil-motor-control

A distributed real-time control system built on FreeRTOS, tested on a
**Hardware-in-the-Loop (HIL) bench**: an STM32 controller drives a DC motor
that is simulated in real time by an ESP32, while a second ESP32 supervises
the whole system and streams it to a PC dashboard.

The goal is not only a working system. The timing behaviour of every task and
every link is **computed analytically first, then verified by measurement**
on the hardware.

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
- No dynamic allocation after start-up.
- The supervisor observes but never drives the real-time loop: if Wi-Fi or the
  PC fails, the control loop keeps running safely.
- Emergency stop is hardware, local to the controller, never through a link.
- What is not measured is not claimed.

## Hardware

- NUCLEO-L476RG, **modified**: SB16 and SB50 ON, SB55 OFF, so that HSE receives
  the 8 MHz MCO from the ST-LINK (not connected by default on this board, see UM1724 §6.7.1)
- 2 × ESP32
- Potentiometer (speed setpoint), push buttons, LEDs, I2C RTC module (fault timestamps)
- Logic analyser (8 ch, 24 MS/s) for all timing measurements

## Toolchain

- STM32: STM32CubeIDE, CubeMX, HAL, FreeRTOS
- ESP32: ESP-IDF (FreeRTOS SMP)
- PC: Python

## Results

_Filled in as phases complete._

| Metric | Predicted | Measured |
|---|---|---|
| System clock (MCO, SYSCLK/16) | 5 MHz | pending board modification |
| Context switch time | — | — |
| Control task jitter | — | — |
| Control loop CPU load | — | — |
| Fault detection latency | — | — |
| End-to-end latency (fault → dashboard) | — | — |

## Roadmap

### Phase 0 — Foundations
- [x] Repository structure
- [x] CubeMX project for NUCLEO-L476RG
- [x] Clock tree: HSE bypass → PLL → 80 MHz, MCO on PA8, CSS enabled
- [ ] Board modification (SB16, SB50, SB55)
- [ ] Clock verified by measurement on MCO

### Phase 1 — STM32 real-time core
- [ ] FreeRTOS, native API, static allocation
- [ ] Two periodic tasks with dedicated timing pins
- [ ] DWT cycle counter as the measurement base
- [ ] First measurements: context switch time, task jitter

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
- [ ] Worst-case response time of each task, computed then measured
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
