# Flight Robot Autonomous Navigation Control Project

A bare-metal embedded firmware implementation for autonomous flight control of a fixed-wing robot, developed on the Avionics Lite board (STM32F103VET6). This project integrates high-speed sensor acquisition, attitude estimation, PID-based control loops, and multi-mode flight logic to enable fully autonomous missions (hover, turns, figure-eight, takeoff/landing, drop).

---

## Table of Contents

1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Hardware Components](#hardware-components)
4. [Software Components](#software-components)
5. [Build & Flash Instructions](#build--flash-instructions)
6. [Configuration & Calibration](#configuration--calibration)
7. [Usage](#usage)
8. [Directory Structure](#directory-structure)
9. [Author](#author)
10. [License](#license)

---

## Features

* **High-Speed Sensor Acquisition**: Polls IMU (MPU6000), magnetometer (MAG3110), and barometer (BMP280) at 1 kHz.
* **Attitude Estimation**: Madgwick AHRS algorithm for real‑time roll, pitch, yaw.
* **PID Control Loops**: Stabilization at 20 Hz for roll, pitch, yaw, with anti‑windup.
* **Multi-Mode Flight Logic**:

  * Manual (RC passthrough)
  * Horizontal flight, steady turns (left/right)
  * Figure-eight and circling up/down patterns
  * Automatic takeoff & landing
  * Drop mode (payload release)
* **Communication Interfaces**:

  * XBee S2C radio via USART + DMA
  * PCM RC receiver via EXTI on GPIOE pins
  * I²C bus for sensors
* **Bare-Metal Implementation**:

  * STM32 Standard Peripheral Library (SPL)
  * SysTick and TIMx interrupts for task scheduling
* **Diagnostics**: Runtime telemetry over UART + DMA for logging sensor data, control outputs, mode changes.

---

## System Architecture

1. **Initialization** (in `main.c`)

   * `BoardInit()`, GPIO, SysTick, NVIC, ADC/DMA
   * I²C1/2 and USART1 setup
   * Sensor connection check & configuration
   * Timer setup: TIM1 (PWM), TIM2 (exceptions), TIM5 (sensor), TIM6 (PID), TIM8 (RC)

2. **Main Loop**

   * Idle (`while(1);`)—all work via interrupts

3. **Interrupt Handlers**

   * **TIM5\_IRQHandler**: Sensor read & AHRS update (1 kHz)
   * **TIM6\_IRQHandler**: Control logic + PID execution (20 Hz)
   * **TIM8\_UP\_IRQHandler**: RC pulse decoding → PWM outputs
   * **TIM2\_IRQHandler**: I²C exception logging

4. **Core Algorithms**

   * **MadgwickAHRS**: Fusion of gyro, accel, mag for quaternion→Euler conversion
   * **PID\_Controller / PID\_Roll\_Controller**: Multi‑loop PID with derivative pre‑filtering

5. **Communication**

   * **USART1 + DMA**: Telemetry output
   * **XBee**: Wireless command/status
   * **I²C / EXTI**: Sensor bus and RC input

---

## Hardware Components

| Component     | Interface    | Notes                                    |
| ------------- | ------------ | ---------------------------------------- |
| STM32F103VET6 | MCU          | 72 MHz Cortex-M3                         |
| MPU6000       | I²C2         | 3-axis accel & gyro                      |
| MAG3110FCR1   | I²C2         | 3-axis magnetometer                      |
| BMP280        | I²C2         | Barometric pressure & temperature sensor |
| XBee S2C      | USART1/DMA   | 2.4 GHz ZigBee radio                     |
| RC Receiver   | EXTI (PE0‑5) | PWM pulse capture                        |
| Servo Outputs | TIM1\_CH1‑4  | Aileron, elevator, throttle, rudder      |

---

## Software Components

* **Language**: C (ANSI C99)
* **Libraries**: STM32F1 Standard Peripheral Library
* **Build Tools**: GNU Arm Embedded Toolchain (arm-none-eabi-gcc) or Keil uVision
* **Code Organization**:

  * `board/` – hardware init (GPIO, timers, NVIC)
  * `comm/`  – I²C, USART, DMA drivers
  * `sensors/` – MPU6000, MAG3110, BMP280 drivers
  * `control/` – AHRS, PID, mode logic
  * `main.c` – application entrypoint

---

## Build & Flash Instructions

1. **Clone repository**:

   ```bash
   git clone <repo_url>
   cd flight-robot-autonomy
   ```

2. **Configure toolchain**:

   * Edit `Makefile` to point `CC`, `LD`, and `OPENOCD` if needed.

3. **Build**:

   ```bash
   make all
   ```

   Generates `build/firmware.bin` and `build/firmware.elf`.

4. **Flash** via ST-LINK or OpenOCD:

   ```bash
   openocd -f board/stm32f1.cfg \
           -c "program build/firmware.bin 0x08000000 verify reset exit"
   ```

---

## Configuration & Calibration

* **Magnetometer Calibration**:

  * Enter calibration mode by toggling `CalibrationFlag` in code or via RC channel 6.
  * Collect 500 samples (rotating board), offsets auto-computed.

* **PID Tuning**:

  * Default gains in `control/mode_handlers.c`
  * Tune KP, TI, TD constants for each flight mode.

* **Flight Mode Selection**:

  * Use RC channel 6 or XBee command to cycle through modes.

---

## Usage

1. **Power on board** with sensors and XBee attached.
2. **Connect to ground station** (optional) on UART 115200 bps.
3. **Ensure sensor connections** succeed (console messages).
4. **Switch RC transmitter**: flick Mode switch to enable control logic.
5. **Monitor telemetry**: attitude, errors, control outputs.

---

## Directory Structure

```text
flight-robot-autonomy/
├─ board/             # Low-level init (GPIO, timers)
├─ comm/              # I²C, USART, DMA drivers
├─ sensors/           # Sensor drivers and calibration
├─ control/           # AHRS, PID loops, mode state machines
├─ main.c             # System initialization & infinite loop
├─ Makefile           # Build script
├─ LICENSE            # MIT License
└─ README.md          # This document
```

---

## Author

**Keigo Miyama** – Original firmware architect and developer.

---

## License

This project is released under the [MIT License](LICENSE).
