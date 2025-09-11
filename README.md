**Project: Gesture-Controlled Robotic Arm** 

_using Raspberry Pi 5, EK-TM4C123GH6PM development kit and Booster pack
ARM tool chain used: gcc-arm-none-eabi-10.3-2021.10-win32.exe
Version: 1.0
Date: September 6, 2025_

**1. Introduction** 
This project implements a gesture-controlled robotic arm where a user’s hand gestures, detected via a USB webcam and processed by an AI model on the Raspberry Pi 5, are translated into motion commands. 
These commands are transmitted via Bluetooth to a TM4C123GH6PM microcontroller, which controls the 4 DOF wooden robotic arm through a PCA9685 PWM driver board. 
The system demonstrates edge AI inference on embedded Linux, wireless communication, and real-time robotic actuation.

**2. System Overview** 
  1. The Logitech USB webcam captures live video of the user’s hand.
  2. The Raspberry Pi 5 processes the video feed:
    - Extracts hand landmarks using TensorFlow Lite (and optionally Edge Impulse for model development).
    - Classifies gestures (e.g., fist, open palm, point left, point right).
  3. The Pi sends a corresponding command string via Bluetooth to the TM4C123GH6PM.
  4. The TM4C123 decodes the command and generates PWM signals using the PCA9685 driver board, which controls the 4 servo motors (base, shoulder, elbow, claw) on the robotic arm.
  5. The arm responds to the detected gesture in near real time.

**3. Hardware Components** 
  - Raspberry Pi 5: Edge AI inference, gesture recognition, Bluetooth communication.
  - Logitech USB Webcam: Captures hand gesture video.
  - TM4C123GH6PM: Local controller for servos, receives Pi commands via Bluetooth.
  - PCA9685 Servo Driver: Generates stable PWM signals, controlled by TM4C123 over I²C.
  - Wooden 4 DOF Robotic Arm Kit: Servos at base, shoulder, elbow, claw.
  - Servo Motors (x4): Arm movement, requires external power.
  - Bluetooth Modules (HC-05/HC-06, x2): Wireless communication between Pi and TM4C123.
  - Power Supply (5–6V, 2–3A): External supply for servos.
  - Wiring &amp; Connectors: Electrical connections, common ground required.

**4. Hardware Connections**
  - Servo Control Path: Servos → PCA9685 → TM4C123 (via I²C).
  - Communication Path: Raspberry Pi 5 ↔ Bluetooth ↔ TM4C123 (UART).
  - Camera Path: Logitech Webcam → Raspberry Pi 5 (USB).

**5. Software Architecture**
Raspberry Pi 5:
  - OS: Ubuntu 24.04 / Raspberry Pi OS
  - Libraries: OpenCV, TensorFlow Lite, Edge Impulse SDK, PyBluez/pyserial
  - Process: Capture video → preprocess → TFLite inference → gesture → command mapping → send via Bluetooth
    TM4C123GH6PM:
  - Firmware: C (Keil/Code Composer Studio)
  - Tasks: Initialize UART, parse command, control PCA9685 via I²C, generate PWM for servos
    PCA9685 Driver:
  - I²C slave to TM4C123, generates PWM outputs for servos

**6. Functional Requirements**
  1. Detect and classify at least 4 distinct hand gestures.
  2. Arm response latency &lt; 300 ms.
  3. Reliable Raspberry Pi ↔ TM4C123 Bluetooth communication.
  4. TM4C123 maps commands to servo motion sequences.
  5. Servos powered by external supply, not Pi directly.

**7. Constraints &amp; Considerations**
  - Power Isolation: Separate 5–6V, 2–3A supply for servos.
  - Bluetooth Range: ~10m with HC-05 modules.
  - Model Efficiency: TFLite model &lt;5MB for real-time Pi 5 execution.
  - Heat Dissipation: Continuous servo use may need cooling.

**9. Future Extensions**
  - Integrate ROS 2 for modularity.
  - Add gesture + voice multimodal control.
  - Enable remote monitoring via MQTT/Node-RED.
  - Implement gesture mirroring mode (arm mimics hand pose).
