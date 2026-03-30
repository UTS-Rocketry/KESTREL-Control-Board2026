# CAS Interface Board 2026

<img width="1288" height="2591" alt="image" src="https://github.com/user-attachments/assets/fdfd3684-4780-4c69-9c9a-4beff4ede71d" />

Control Actuation System (CAS) Interface Board for the UTS Rocketry avionics stack.

This board provides closed-loop control for the airbrake and parafoil control systems, along with sensor interfaces and onboard data logging. The design is based on the STM32F405RGT6 microcontroller and integrates inertial sensing, actuator control, and non-volatile storage.

The CAS board is intended to operate as part of the UTS Rocketry flight avionics architecture.

---

## Overview

The CAS Interface Board is responsible for:

- Airbrake deployment control
- Roll stabilisation control
- Sensor acquisition
- Data logging
- Actuator interfacing

Closed-loop control is performed onboard using IMU, barometric sensor, and GPS measurements.

---

## Hardware

### Microcontroller

STM32F405RGT6

- ARM Cortex-M4F
- 168 MHz
- Hardware floating point
- 1 MB Flash
- 192 KB RAM

---

### Sensors

#### IMU

LSM6DS3TR-C

Interface:

- SPI (4-wire)

Signals:

- SCK
- MOSI
- MISO
- CS
- DRDY Interrupt

Power:

- 3.3 V

---

#### Barometer

Interface:

- I2C

Signals:

- SCL
- SDA

Power:

- 3.3 V

---

### Storage

#### MicroSD Card

Interface:

- SPI

Signals:

- SCK
- MOSI
- MISO
- CS

Purpose:

- Flight data logging

---

### Actuator Interfaces

#### Servo Outputs

- Airbrake Servo
- Roll Control Servo

Control Method:

- Timer PWM outputs

Power:

- Dedicated 5 V servo rail

Signal Voltage:

- 3.3 V logic

---

## Control System

### Airbrake Control

The CAS system predicts apogee using altitude and velocity estimates derived from sensor measurements.

Airbrake deployment is adjusted to reach the target apogee.

Inputs:

- Barometer altitude
- Estimated vertical velocity

Outputs:

- Airbrake servo PWM

## Firmware

Development tools:

- STM32CubeMX
- STM32CubeIDE
- ST-Link

Features:

- SPI IMU driver
- I2C barometer driver
- PWM servo control
- SD card logging
- Flash logging
- Interrupt-driven sensor acquisition

---
