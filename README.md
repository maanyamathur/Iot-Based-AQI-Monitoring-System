# 🌍 IoT-Based Air Quality Monitoring System

# 📖 Overview

The **IoT-Based Air Quality Monitoring System** is a smart environmental monitoring solution developed using **ESP32**, **Arduino Nano**, and multiple environmental sensors.

The system continuously monitors air quality and environmental parameters, processes the collected data, and displays it through a **Node-RED dashboard** as well as a **10-panel P10 LED display**. Communication between devices is achieved using the **MQTT protocol** with **HiveMQ**, enabling efficient real-time data transfer.

---

# ✨ Features

- 🌫️ Real-time Air Quality Index (AQI) Monitoring
- 🌬️ PM1.0, PM2.5 & PM10 Detection
- 🌡️ Temperature Monitoring
- 💧 Humidity Monitoring
- 📈 Atmospheric Pressure Monitoring
- 🌱 Gas Monitoring (CO₂, NH₃, VOC, NOx & CH₄)
- 🔊 Noise Level Monitoring
- 📊 Live Node-RED Dashboard
- 📺 P10 LED Display Output
- 📡 MQTT Communication
- 📶 Wi-Fi Configuration using WiFiManager
- 🔄 Over-The-Air (OTA) Firmware Updates
- 🛡️ Watchdog-based System Recovery

---

# 🛠 Hardware Components

| Component | Quantity |
|------------|---------:|
| ESP32 DevKit | 1 |
| Arduino Nano | 1 |
| PMS5003 Air Quality Sensor | 1 |
| BME680 Environmental Sensor | 1 |
| MQ135 Gas Sensor | 1 |
| MAX4466 Sound Sensor | 1 |
| P10 LED Display Panels | 10 |
| 5V 40A SMPS | 1 |

---

# 💻 Software & Technologies

- Arduino IDE
- Node-RED
- HiveMQ MQTT Broker
- WiFiManager
- ArduinoOTA
- MQTT Protocol

---

# 📡 System Architecture

```text
                +----------------------+
                |      Sensors         |
                |----------------------|
                | PMS5003              |
                | BME680               |
                | MQ135                |
                | MAX4466              |
                +----------+-----------+
                           |
                           |
                      ESP32 DevKit
                           |
             +-------------+--------------+
             |                            |
             | MQTT (HiveMQ)              |
             |                            |
             ▼                            ▼
      Node-RED Dashboard          Arduino Nano
                                       |
                                       |
                                10 × P10 Display
```

---

# 📂 Repository Structure

```text
IoT-Based-Air-Quality-Monitoring-System/
│
├── README.md
├── LICENSE
├── Libraries_Used.md
│
├── ESP32/
│   ├── esp32_testing_final.ino
│   └── README.md
│
├── Arduino_Nano/
│   ├── nano_testing_final.ino
│   └── README.md
│
├── Node-RED/
│   ├── flows(1).json
│   └── README.md
│
├── Documentation/
│   ├── internship project report(MAANYA MATHUR).pdf
│   └── README.md
│
├── Circuit_Diagram/
│   ├── AQI_Circuit Diagram.jpg
│   └── README.md
│
└── Images/
    ├── Dashboard
    ├── P10_Display
    └── README.md
```
# 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/IoT-Based-Air-Quality-Monitoring-System.git
```

### 2. Upload ESP32 Firmware

Open the code in the **ESP32** folder using the Arduino IDE and upload it to the ESP32.

### 3. Upload Arduino Nano Firmware

Upload the code inside the **Arduino_Nano** folder to the Arduino Nano.

### 4. Import Node-RED Flow

Import the `flow.json` file into Node-RED.

### 5. Configure Wi-Fi

Power on the ESP32 and configure Wi-Fi credentials using **WiFiManager**.

### 6. Start Monitoring

Open the Node-RED dashboard to view real-time environmental data.

---

# 📊 Parameters Monitored

- Air Quality Index (AQI)
- PM1.0
- PM2.5
- PM10
- Temperature
- Humidity
- Atmospheric Pressure
- CO₂
- NH₃
- VOC
- NOx
- CH₄
- Sound Level

---

# 📄 Documentation

The complete internship project report is available in the **Documentation** folder.

---

# 🔮 Future Scope

- ☁️ Cloud Integration
- 📱 Mobile Application
- 📍 GPS-based AQI Mapping
- 🤖 AI-based Air Quality Prediction
- 📈 Historical Data Analytics
- 📧 Email & SMS Alerts

---

# 👩‍💻 Author

**Maanya Mathur**

B.Tech – Computer Science & Engineering (AI & Data Science)

JECRC University

---

## ⭐ If you found this project interesting, please consider giving it a star!
