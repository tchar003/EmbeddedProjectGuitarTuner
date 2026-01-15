# STM32 Guitar Tuner

A real time digital guitar tuner implemented on an STM32 microcontroller.  
The system samples an analog microphone signal, performs pitch detection in firmware, and provides tuning feedback to the user.

This project focuses on embedded firmware development, peripheral configuration, and hardware software integration using STM32CubeIDE.

---

## Project Overview

The tuner captures audio from a microphone front end, digitizes the signal using the STM32 ADC, estimates the fundamental frequency, and maps it to the nearest standard guitar string.

Primary objectives:
- Configure STM32 peripherals using STM32CubeIDE
- Implement real time signal processing in C
- Design and integrate an analog audio front end
- Produce a clean, reproducible embedded systems project suitable for portfolio review

---

## Features

- Real time pitch detection for standard guitar tuning (E2 to E4)
- Timer controlled ADC sampling
- Digital pitch detection implemented in firmware
- I2C LCD output for tuning feedback
- Modular and readable code structure

---

## Hardware

### Microcontroller
- STM32F411RE Nucleo board

### Audio Front End
- Electret microphone
- Op amp based preamplifier

### Peripherals Used
- ADC for audio sampling
- Timer for sample rate control
- I2C for LCD communication
- GPIO for control and status
- UART for debugging output

### Schematic
The full schematic is available in:

`` Hardware/Schematic.pdf ``


## Project Directory Structure
```
GuitarTuner/
├── Core/
│   ├── Inc/                        # Application headers
│   │   ├── lcd_driver.h            # Custom I2C LCD driver
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   └── Src/                        # Application source files
│       ├── lcd_driver.c            # LCD driver implementation
│       ├── main.c                  # Application entry point
│       ├── stm32f4xx_hal_msp.c     # HAL MSP init
│       ├── stm32f4xx_it.c          # Interrupt handlers
│       ├── syscalls.c
│       ├── sysmem.c
│       └── system_stm32f4xx.c
├── Drivers/                        # STM32 HAL drivers
├── Startup/                        # Startup and vector table
├── Debug/                          # Build artifacts (gitignored)
├── GuitarTuner.ioc                 # CubeMX configuration file
├── STM32F411RETX_FLASH.ld          # Flash linker script
└── STM32F411RETX_RAM.ld            # RAM linker script
```


---

## Firmware Architecture

### Core Modules

- lcd_driver.c
  - Custom I2C LCD driver
  - Low level command and data handling

- main.c
  - System initialization
  - Main application loop
  - Tuner state management

- stm32f4xx_hal_msp.c
  - Peripheral low level initialization

- stm32f4xx_it.c
  - Interrupt service routines

---

## Pitch Detection

The tuner uses a time domain pitch detection approach suitable for real time execution on a microcontroller.

Characteristics:
- Optimized for low computational cost
- Designed for guitar frequency range
- Deterministic execution without RTOS

Future versions may implement FFT or YIN based algorithms for improved robustness.

---

## Build and Run Instructions

### Requirements
- STM32CubeIDE
- STM32F411RE Nucleo board
- USB cable
- Microphone front end connected to ADC input

### Steps
1. Clone the repository
2. Open STM32CubeIDE
3. Import the project into your workspace
4. Build the project
5. Flash the firmware to the board
6. Observe tuning output on the LCD

---

## Validation and Testing

- Verified ADC sampling timing using oscilloscope
- Tested pitch detection with known reference tones
- Confirmed tuning response across all standard guitar strings

---

## Known Limitations

- Sensitive to ambient noise
- Requires proper microphone gain adjustment
- Single note detection only

---

## Future Improvements

- Digital filtering for noise reduction
- Automatic string detection
- Graphical tuning indicator
- More robust FFT or autocorrelation based pitch detection

---

## Relevance

This project demonstrates:
- Embedded C development on STM32
- Peripheral configuration and bring up
- Real time signal processing fundamentals
- Hardware firmware co design

---
