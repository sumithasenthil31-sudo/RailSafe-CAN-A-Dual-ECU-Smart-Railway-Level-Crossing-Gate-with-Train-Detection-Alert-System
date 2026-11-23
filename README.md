 RailSafe-CAN-A-Dual-ECU-Smart-Railway-Level-Crossing-Gate-with-Train-Detection-Alert-System

 RailSafe-CAN-A-Dual-ECU-Smart-Railway-Level-Crossing-Gate-with-Train-Detection-Alert-System

This project develops an automatic railway level crossing gate using two ECUs communicating through a CAN bus.
ECU1 detects train entry and exit using IR sensors, monitors train status, and sends information to ECU2.
ECU2 controls the gate, traffic lights, and buzzer, ensuring safe operation and automatically closing the gate if CAN communication fails.

FEATURES:
Automatic train detection using IR sensors.
Dual-ECU communication using the CAN bus protocol.
Automatic railway gate operation using a servo motor.
Warning system with red/green signal lights and a buzzer.
Live status monitoring, train counting and fail-safe protection during CAN communication failure.

HARDWARE COMPONENTS:
LPC2129 ARM7 Microcontroller (2) – Controls the two ECUs.
IR Sensors (2) – Detect train entry and exit.
Servo Motor (SG90) – Controls automatic gate opening and closing.
16×2 LCD Display (1) – Displays live train and gate status.
Red and Green LEDs – Indicate railway crossing signals.
Buzzer (1) – Provides an audible warning alert.

SOFTWARE AND TOOLS:
Embedded C
Keil uVision (Compiler and IDE)
Flash Magic (flashing the hex file)

FOLDER STRUCTURE
RailSafe_CAN/
│
├── ECU1_Train_Detection/
│   │
│   ├── ecu1_main.c
│   ├── can_driver.h
│   └── header.h
│
├── ECU2_Gate_Control/
│   │
│   ├── ecu2_main.c
│   ├── ECU2_HEADER.H
│   └── ECU2_CANDRIVER.
│
└── README.md

