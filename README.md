# BluetoothRCCar
BluetoothRCCar
This repository contains the firmware and documentation for a two-wheeled robot car controlled wirelessly via a smartphone application. The project features an STM32-based control systemthat implements differential drive logic, pulse width modulation (PWM) for speed control, and real-time Bluetooth command parsing.

📺 Presentation Video
  - https://youtu.be/4xvo5i-WG1I This video covers the detailed hardware assembly, demonstration of movement, and a deep dive into the firmware architecture.


🚀 Project Overview 
The system is designed around the STM32L432KC microcontroller, which processes 4-byte signals sent from a smartphone.
Key Features:
  - Wireless Control: Uses the Adafruit Bluefruit LE UART Friend to bridge smartphone commands to the MCU.
  - Smooth Arc Turning: Implements differential speed control where the outside wheel spins faster than the inside wheel to create a curve rather than a sharp pivot.
  - Non-Blocking Logic: The firmware identifies the "!" character to start packet parsing, ensuring the car responds instantly to stop commands (<50ms).
  - Safety Features: Includes a hardware slide switch to cut power to the motor driver for safe maintenance.


🛠 Hardware Components
  - Microcontroller: STM32L432KC.
  - Motor Driver: Adafruit TB6612 1.2A DC Motor Driver (Dual Channel).
  - Bluetooth Module: Adafruit Bluefruit LE UART Friend.
  - Power: External battery pack with a integrated slide switch.
  - App: Bluefruit LE Connect (Control Pad mode).


💻 Control Logic & PWM The motors are controlled using a combination of GPIO for direction and PWM for speed management.
  - Timer Period: 1000.
  - Full Speed: 70% Duty Cycle (700/1000).
  - Turning Speed: 20% - 40% Duty Cycle depending on turn type.


🔧 Challenges & Solutions
  - Flow Control Issue: Initially, the Bluetooth module would not transmit data because the CTS (Clear to Send) pin was floating. *Solution: Grounded the CTS pin to enable transmission.
  - Pin Conflict: Initial UART pins (A2/A7) interfered with the on-board USB debugger. *Solution: Switched to dedicated hardware serial pins D0 and D1.
  - Motor Phase: One wheel initially spun in the wrong direction. *Solution: Swapped motor wires at the terminal block to match firmware logic.


💡Future Improvements
  - PWM Ramping: Adding a "soft start" to increment speed over 200ms to prevent wheel slip and current spikes.
  - Obstacle Avoidance: Integration of an HC-SR04 ultrasonic sensor to automatically stop if an object is within 20cm.
  - Battery Monitoring: Using the STM32 ADC to monitor voltage and alert the user when batteries are low.
