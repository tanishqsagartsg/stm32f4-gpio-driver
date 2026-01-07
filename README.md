# stm32f4-gpio-driver
Bare-metal GPIO driver for STM32F407 (no HAL, no CMSIS)
# STM32F407 GPIO Driver (Bare-Metal)

This repository contains a **bare-metal GPIO driver implementation**
for the STM32F407 microcontroller, written in **C from scratch**
without using STM32 HAL GPIO drivers.

The project is developed using **STM32CubeIDE** and follows a
**register-level programming approach** to understand MCU internals.

---

## Project Objective

- Learn how GPIO works at the **register level**
- Design a **reusable GPIO driver**
- Practice **modular embedded driver architecture**
- Understand how CubeIDE projects are structured
- Use **Git & GitHub** for version-controlled firmware development

---

### Custom Driver Code (MY WORK)

**All original driver implementation is located here:**
<br>
driver/
<br>
├── Inc/
<br>
│ ├── stm32f407xx.h
<br>
│ └── stm32f407xx_gpio_driver.h
<br>
│
<br>
└── Src/
<br>
└── stm32f407xx_gpio_driver.c
<br>

This folder contains:
- MCU-specific register definitions
- GPIO driver APIs
- GPIO initialization, read/write, and control logic

If you want to review **my actual work**, start from this folder.

---

## Features Implemented

- GPIO register definitions using memory-mapped structures
- Peripheral clock enable/disable macros
- GPIO pin configuration:
  - Mode
  - Speed
  - Pull-up / pull-down
  - Output type
  - Alternate function
- GPIO read/write APIs
- GPIO toggle functionality
- Modular handle-based driver design

---

## Work in Progress

The following features will be added incrementally:

- GPIO interrupt configuration (EXTI)
- NVIC interrupt handling
- Application examples:
  - LED toggle
  - Button interrupt

Each feature will be added via **separate commits** to show
incremental development.

---

## Tools & Environment

- MCU: **STM32F407xx**
- IDE: **STM32CubeIDE**
- Language: **C**
- Programming Style: **Bare-metal / Register-level**
- Version Control: **Git & GitHub**

---

## Author

**Tanishq Sagar**

This repository is part of my embedded systems learning journey,
focusing on low-level firmware development and driver design.
