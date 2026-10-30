# STM32 Two-Wheel Self-Balancing Car

A two-wheel self-balancing car based on STM32F103 + MPU6050, using an inverted-pendulum model with a triple-loop PID controller. It stands upright autonomously, moves forward/backward, turns in place, and supports 3-speed switching with OLED display and HC-05 Bluetooth remote control.

This is a personal hobby project — requirements analysis, hardware design, circuit assembly and firmware debugging were all done independently.

## Features

- **Self-balancing**: dynamic balance within ±2°, response time ≤ 50ms
- **Motion control**: forward / backward / in-place turning (differential drive), 3 speed levels
- **Bluetooth control**: HC-05 transparent transmission, command format `CMD,vel=speed,dir=type`, 115200 baud
- **Real-time display**: OLED shows speed, pitch angle and battery level (ADC voltage sampling), refresh ≥ 1Hz
- **Attitude estimation**: MPU6050 built-in DMP, 100Hz sampling

## Hardware

| Module | Model |
| --- | --- |
| MCU | STM32F103C8T6 |
| IMU | MPU6050 (I2C, DMP) |
| Motor driver | TB6612 |
| Motors | DC gear motors with encoders, ≥ 500 PPR |
| Display | 0.96" OLED (SSD1306, 128×64, I2C) |
| Wireless | HC-05 Bluetooth (transparent mode) |
| Power | 12V 2000mAh LiPo battery |

## Build & Flash

1. Open `cubemx_config/MDK-ARM/cubemx_config.uvprojx` with **Keil MDK5**
2. Build (F7) and flash via **ST-Link**
3. Power on — the car enters balance mode automatically; send commands via a Bluetooth serial app (115200)

## Control Commands

```
CMD,vel=1,dir=1    // forward, speed level 1 (1/2/3 → PWM 30%/60%/90%)
CMD,vel=1,dir=0    // backward
CMD,vel=0,dir=2    // turn in place
```
