# ⚙️ Smart Inventory Management System (Dual ARM-Based)

**Course:** 23ELC302 – Embedded Systems Design using ARM  
**Department:** Electrical and Computer Engineering  
**Institution:** Amrita School of Engineering, Coimbatore  
**Batch:** 8  

### 👩‍💻 Team Members
- CB.EN.U4ELC23019 – Indhu Prakash S  
- CB.EN.U4ELC23039 – Rahul K  
- CB.EN.U4ELC23040 – Rithvik Balaji Elangovan  
- CB.EN.U4ELC23054 – Shri Monesh V  

---

## 🧠 Project Overview

Traditional inventory systems are prone to human error, manual delays, and poor access control.  
This project presents a **Smart Inventory Management System** that automates the process using dual ARM microcontrollers (STM32F401RE) and an ESP8266 Wi-Fi module for real-time data synchronization.

The system ensures **secure, authenticated, and accurate inventory logging** through OTP-based user verification and weight-based item counting.

---

## 🧩 System Architecture

### 🔹 Authentication Station (Bare-Metal)
Handles user authentication and control logic.

**Peripherals:**
- RFID Reader (SPI)
- Keypad (GPIO)
- LCD Display (I2C)
- ESP8266 Wi-Fi Module (USART1)
- Communicates with Weighing Station (USART6)

### 🔹 Weighing Station (HAL)
Handles measurement and data response.

**Peripherals:**
- 2 × Load Cells + HX711 ADC (GPIO)
- LCD Display (I2C)
- Communicates with Authentication Station (USART6)

### 🔹 Communication Module (ESP8266)
- Acts as the **Wi-Fi interface**
- Connects to local server (XAMPP/PHP/MySQL)
- Handles OTP requests and inventory updates through HTTP APIs

---

## 🗂️ Workflow

1. **RFID Scan:** Authentication Station detects user RFID.  
2. **OTP Generation:** RFID UID sent to ESP8266 → Server → Generates OTP.  
3. **OTP Verification:** User enters OTP via keypad → Verified locally.  
4. **Weighing Request:** If valid, Authentication Station requests weight data from Weighing Station.  
5. **Data Update:** ESP8266 sends item count and weight to backend via HTTP.  
6. **Database Sync:** MySQL updates item count → Confirmation displayed on LCD.

---

## ⚙️ Hardware Components

| Component | Quantity | Interface / Protocol |
|------------|-----------|----------------------|
| STM32F401RE | 2 | HAL / Bare Metal |
| ESP8266 Wi-Fi Module | 1 | UART |
| RFID Reader (MFRC522) | 1 | SPI |
| 4x4 Matrix Keypad | 1 | GPIO |
| 16x2 LCD Display (I2C) | 2 | I2C |
| Load Cell (5kg) + HX711 ADC | 2 | GPIO |

---

## 💻 Software Stack

| Layer | Tools / Technologies |
|-------|----------------------|
| **Firmware** | STM32CubeIDE (HAL), Bare-Metal C |
| **Wi-Fi Controller** | Arduino IDE, ESP8266 Libraries |
| **Backend Server** | XAMPP (Apache, PHP, MySQL) |
| **Database** | MySQL |
| **Languages** | C, C++, PHP, SQL |

---

## 🧮 Communication Flow

| Communication | Protocol | Description |
|----------------|-----------|--------------|
| STM32 ↔ ESP8266 | UART | OTP requests & inventory updates |
| STM32 ↔ STM32 | USART6 | Weight data exchange |
| ESP8266 ↔ Server | HTTP (GET) | OTP and DB update requests |
| LCD, Keypad, RFID | I2C/SPI/GPIO | Local I/O interfaces |

---

## 🔐 Backend Overview

**Server APIs:**
- `get_otp.php` → Generates and returns OTP for a user RFID UID.  
- `update_inventory.php` → Updates inventory count for authenticated transactions.  
- `db_connect.php` → Handles MySQL connections.

**Database Tables:**
- `users` – stores RFID, OTP, and timestamp.  
- `inventory` – stores item names and counts.  
- `logs` – optional transaction history.

---

## 🧰 Implementation Highlights

- **Bare-Metal Firmware:** Direct register-level control for speed and minimal memory footprint.  
- **HAL Firmware:** Simplified driver-level access for fast prototyping.  
- **UART Communication:** Interrupt-driven design using HAL_UART_Receive_IT().  
- **Finite State Machine (FSM):** Manages authentication logic cleanly and predictably.  
- **JSON Parsing (ESP8266):** Uses ArduinoJson to extract OTP data.  
- **Secure Timestamp Handling:** Timezone set to `Asia/Kolkata` for accurate comparisons.

---

## 📊 Results

- **Accuracy:** ±2g up to 1kg (10-sample averaging).  
- **Latency:** <200ms end-to-end authentication and update (excluding user input).  
- **Performance:** Stable UART data transfer; zero packet loss under normal operation.
![WhatsApp Image 2025-10-14 at 08 55 53_fb88ddf9](https://github.com/user-attachments/assets/78421c2c-eb15-4c3d-ae13-84b2ba89e9ea)
![WhatsApp Image 2025-10-14 at 08 55 32_36788d4c](https://github.com/user-attachments/assets/7453712b-73f6-4757-89eb-40a6d0bace24)
![Uploading WhatsApp Image 2025-10-14 at 08.55.53_7d890845.jpg…]()

---

## 🧩 Analysis — HAL vs. Bare Metal

| Feature | HAL | Bare Metal |
|----------|-----|-------------|
| Development Speed | ✅ Fast setup | ❌ Slower |
| Code Size | ❌ Larger | ✅ Compact |
| Performance | Moderate | ✅ High |
| Control | Limited | ✅ Full hardware control |
| Learning Curve | Easy | Advanced |

---

## 🚀 Future Enhancements

- Deploy backend to **AWS / Heroku Cloud** for remote access.  
- Integrate **barcode scanner** for multi-item support.  
- Add **FreeRTOS** for task scheduling and concurrent operations.  
- Implement **checksum-based UART protocol** for error resilience.  
- Enable **HTTPS** with SSL encryption for secure data exchange.

---

## 🧾 References

1. Senanayake et al., *Design and Implementation of an IoT Based Smart Inventory Management System*, IEEE ICIAfS 2021.  
2. Khan et al., *IoT-Based Smart Weighing Machine for Inventory Management*, IEEE ICICCS 2020.  
3. Kassab et al., *A Secure RFID-based Access Control System*, IEEE ICCE 2019.  
4. Kumar et al., *IoT-Based Smart Warehouse Management System for Industry 4.0*, Springer, 2021.  
5. Rana et al., *RFID-Based Smart Inventory System using ESP8266 and Cloud*, Springer, 2021.

---

## 🏁 Conclusion

This project demonstrates a **fully functional, secure, and automated inventory management prototype** integrating embedded, IoT, and web technologies.  
It showcases efficient **ARM-based dual-controller communication**, **real-time backend integration**, and **multi-factor authentication**, forming a solid foundation for scalable, industry-grade inventory solutions.

---

📦 **Repository:** [https://github.com/SHRIMONESH/DBPS01](https://github.com/SHRIMONESH/DBPS01)  
📧 *Developed as part of Amrita V Semester ARM Course Project*
