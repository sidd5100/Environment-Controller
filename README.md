# 🌿 Smart Greenhouse Controller – Environmental Regulation Using PIC18F45K22

A compact embedded system for greenhouse automation that simulates and regulates temperature, humidity, and CO₂ levels using sensor inputs, actuator control, and UART-based communication for system monitoring.

---

## 🛠️ Technologies Used

- **Microcontroller Unit (MCU):** PIC18F45K22  
- **Development Environment:** MPLAB IDE (Legacy)  
- **Programming Language:** Embedded C  
- **Debugging Tools:** PICkit 3, MPLAB Simulator (Breakpoints, Watch Window)  
- **Peripherals & Interfaces:**  
  - ADC for sensor input (3x 10k potentiometers)  
  - PWM for damper stepper motor control  
  - UART2 for communication (9600 baud)  
  - GPIO for actuator outputs (fan, heater, cooler, sprinkler)  
  - Timers & Interrupts for real-time event handling

---

## 🔩 Deliverables

### 1. 🌡️ Environmental Monitoring & Control

- **Simulated Sensors:**  
  3 x 10k potentiometers represent temperature, humidity, and CO₂ levels.  
- **Analog Sensing:**  
  Read via ADC channels and compared with predefined setpoints.  
- **Output Control:**  
  - Fan, Heater, Cooler, and Sprinkler are turned ON/OFF based on sensor thresholds.  
  - Control logic supports both **interrupt** and **polling** mechanisms.

---

### 2. 🔄 Stepper Motor-Based Damper Adjustment

- **Purpose:**  
  Adjusts air circulation dynamically using a stepper motor-driven damper.  
- **Trigger Condition:**  
  Activates when any sensor reading goes beyond or below set limits.  
- **Motor Control:**  
  Implemented via timed GPIO sequence to achieve precise positioning.

---

### 3. 📡 UART-Based Communication Protocol

- **System Health Feedback:**  
  Transmitted via UART2 at 9600 baud.  
- **Message Format:**

  ```
  $CONLIM,<TO>,<FROM>,<SENSOR>,<MODE>,<VALUE>,<CHECKSUM>
  ```

  **Example:**

  ```
  $CONLIM,1,977,1,0,45,85
  ```

  - `SENSOR`: Channel number (0 = Temp, 1 = Humidity, 2 = CO₂)  
  - `MODE`: 0 for Upperlimit, 1 for Lowerlimit  
  - `CHECKSUM`: Simple XOR or modulo-based integrity check  

- **Receiving Controller:**  
  Validates checksum, parses sentence, and displays info over UART1.

---

### 4. 💡 System Features & Safety

- **Polling & Interrupts:**  
  Efficient handling of analog inputs and actuator updates.  
- **Debug Mode:**  
  In-system debug via PICkit 3 with breakpoint support.  
- **UART Log:**  
  Real-time updates provide transparency and support for closed-loop integration.

---

## ✅ Project Status

| Deliverable                          | Status       |
|--------------------------------------|--------------|
| Sensor Simulation via Potentiometer  | ✅ Completed |
| Actuator Control Logic               | ✅ Completed |
| Damper Stepper Motor Control         | ✅ Completed |
| UART Sentence Formatting & Output    | ✅ Completed |
| Checksum Validation & Display        | ✅ Completed |

---

## 🧪 Testing Highlights

- **ADC calibration** tested using multimeter-verified potentiometer values.  
- **Output logic** verified through LED indicators on GPIO pins.  
- **Stepper motor** operation validated with LED sequence and test conditions.  
- **UART messages** logged and verified via Teraterm and PuTTY.  
- **Checksum validation** tested with both correct and altered data streams.

---

## 📩 Contact

**Developer**: Siddhant  
📍 Embedded Systems Graduate Student | Fanshawe College  
📧 [siddhant5100@gmail.com](mailto:siddhant5100@gmail.com)  
🔗 [LinkedIn](https://www.linkedin.com/in/siddhant-mahindrakar-362b761a2/)

---

> *Sustainable control meets embedded simplicity — optimizing greenhouse environments for a greener future.*
