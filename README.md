🌿 Smart Greenhouse Controller – Environmental Regulation Using PIC18F45K22
A compact embedded system designed to automate environmental control in greenhouses by simulating and regulating temperature, humidity, and CO₂ levels using sensor inputs, actuator logic, and UART-based monitoring — ideal for closed-loop plant habitat management.

🛠️ Technologies Used
Microcontroller Unit (MCU): PIC18F45K22

Development Environment: MPLAB IDE (Legacy)

Programming Language: Embedded C

Debugging Tools: PICkit 3, MPLAB Simulator (Breakpoints, Watch Window)

Interfaces & Peripherals:

ADC for analog sensor input (3x 10k potentiometers)

GPIO for actuator control (Fan, Heater, Cooler, Sprinkler)

PWM & GPIO sequence for stepper motor

UART2 (9600 baud) for system monitoring

Timers & Interrupts for real-time control

🔩 Deliverables
1. 🌡️ Environmental Monitoring & Control
Sensor Simulation: 3x 10k potentiometers simulate temperature, humidity, and CO₂ levels.

Analog Data Acquisition: Sensor inputs are read through the ADC and compared against predefined setpoints.

Actuator Logic:

Fan, Heater, Cooler, and Sprinkler controlled based on threshold logic.

Supports both polling and interrupt-based control mechanisms.

2. 🔄 Stepper Motor-Based Damper Adjustment
Objective: Adjust air circulation using a damper mechanism driven by a bipolar stepper motor.

Trigger Logic: Motor activates when sensor readings cross defined upper or lower thresholds.

Control Method: GPIO pins sequence controlled via timer-based delay loops for precise angular steps.

3. 📡 UART Communication Protocol
Purpose: Enables system health reporting and feedback for real-time diagnostics.

UART2 Output: Transmits status messages in a defined sentence structure at 9600 baud.

Message Format:

php-template
Copy
Edit
$CONLIM,<TO>,<FROM>,<SENSOR>,<MODE>,<VALUE>,<CHECKSUM>
SENSOR: 0 = Temp, 1 = Humidity, 2 = CO₂

MODE: 0 = Upper Limit, 1 = Lower Limit

CHECKSUM: Simple XOR or modulo operation

Receiving Logic: Another PIC controller or UART monitor parses the sentence, validates checksum, and displays info.

4. 💡 System Features & Safety
Polling & Interrupt Handling: Efficient dual-mode design for sensor and actuator updates.

Debug Mode: Enabled via PICkit 3 for real-time breakpoint debugging and value tracing.

UART Logging: All events transmitted over UART for live tracking and closed-loop control potential.

✅ Project Status
Deliverable	Status
Sensor Simulation via Potentiometers	✅ Completed
Actuator Control Logic	✅ Completed
Stepper Motor Damper Mechanism	✅ Completed
UART Message Protocol Implementation	✅ Completed
Checksum & Sentence Parsing Logic	✅ Completed

🧪 Testing Highlights
ADC Readings: Calibrated and verified using multimeter-checked potentiometer values.

Actuator Control: LED indicators used to simulate actuator responses across all threshold scenarios.

Stepper Motor: Logic confirmed using LED sequencing and observed motor steps.

UART Messaging: Monitored and validated using Tera Term and PuTTY.

Checksum Logic: Tested with both valid and tampered inputs for robustness.

📩 Contact
Developer: Siddhant
📍 Embedded Systems Graduate Student | Fanshawe College
📧 Email
🔗 LinkedIn
