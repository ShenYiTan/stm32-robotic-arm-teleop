# stm32-robotic-arm-teleop
5-DOF robotic arm powered by STM32F103 and ESP32-C3 over BLE, featuring dual-joystick manual control, MediaPipe gesture teleoperation, and a Python kinematic trajectory GUI

## System Architecture
The system employs a distributed control hierarchy:
1. **STM32F103C8T6 (Blue Pill):** Direct hardware actuation via 5-channel 50 Hz PWM (`TIM2` & `TIM4`) and manual joystick teleoperation via 12-bit SAR ADC (`ADC1`).
2. **ESP32-C3 SuperMini:** Transparent wireless bridge hosting a Nordic UART Service (NUS) GATT profile, forwarding serial frames at 115,200 baud.
3. **Host Computer (Python 3):** Drives bidirectional Forward/Inverse Kinematics, trajectory generation (LERP), and MediaPipe skeletal gesture tracking.

---
## Hardware Specifications

* **Microcontrollers:** STM32F103C8T6 , ESP32-C3 SuperMini 
* **Actuators:** 5× MG90S metal-gear micro servos ($500\text{--}2500\ \mu\text{s}$ pulse width)
* **Power Supply:** External 5V 3A AC-to-DC regulated power supply
* **Sensors & HMI:** 2× 2-axis analog joysticks, 2× tactile pushbuttons, 2× status LEDs
---

### Video 
https://youtu.be/fqeTW4hTvRw?si=qcoytpYTg6xN4z-_

### CAD
https://cad.onshape.com/documents/502f7c82745232714e35b467/w/64c8081d1b7d81c2f6074105/e/63d52a12e079b64350ab4d9b

