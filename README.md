[Uploading README.md…]()
# 🚁 Sky Tracker – AI‑Enhanced Drone Telemetry & Monitoring System

![Python](https://img.shields.io/badge/Python-3.9%2B-blue?style=for-the-badge&logo=python)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)
![AI](https://img.shields.io/badge/AI-Anomaly%20Detection-purple?style=for-the-badge)
![UI](https://img.shields.io/badge/UI-PyQt5-orange?style=for-the-badge)

> **A real-time, AI-assisted drone telemetry, visualization, and diagnostics platform.**

---

## 📖 Overview

Sky Tracker is a **real-time drone telemetry and monitoring system** built using **Python, PyQt5, and the MultiWii Serial Protocol (MSP)**.  
It communicates directly with a drone flight controller over serial, visualizes live flight data, and applies **Tier‑1 AI techniques** to enhance stability, safety, and diagnostics.

The project is designed to resemble **industry-grade ground control and monitoring tools**, making it highly relevant for **Technical Support Engineer, Product Engineer, and R&D roles**.

---

## 🚀 Key Features

### 🔹 Core Telemetry & Visualization
- Real-time **roll, pitch, and yaw** acquisition via MSP
- Robust **heading fallback logic** for stable yaw estimation
- Live attitude gauges for roll and pitch
- Interactive **3D drone visualization** using OpenGL (pyqtgraph)
- Orientation reset during flight
- Secure login with daily session caching

### 🔹 Recording & Diagnostics
- Start/stop telemetry recording
- Continuous tracking of **minimum and maximum values**
- Export diagnostics to **Excel** for offline analysis
- Real-time system and serial event logging

### 🔹 AI Enhancements (Tier‑1)
- **Kalman Filter–based sensor smoothing** for roll and pitch
- **Unsupervised anomaly detection** using Isolation Forest
- Detection of abnormal flight behavior and power irregularities
- **Online battery time estimation** using voltage trend analysis
- AI runs in real time without blocking UI or serial communication
- No labeled dataset required

---

## 🧠 System Architecture

Drone / Flight Controller  
|  
|  (MSP over Serial)  
|  
Serial Worker (Threaded)  
- Attitude parsing  
- Heading fallback  
- Analog data parsing  
|  
AI Processing Layer  
- Kalman filtering  
- Anomaly detection  
- Battery time estimation  
|  
UI Layer (PyQt5)  
- Live stats  
- Gauges  
- 3D visualization  
- Logs & alerts  

---

## 🛠️ Technology Stack

- Programming Language: Python  
- UI Framework: PyQt5  
- 3D Visualization: pyqtgraph + OpenGL  
- Communication: Serial (MSP)  
- Machine Learning: scikit-learn  
- Data Handling: NumPy, Pandas  

---

## 📂 Project Structure

Sky-Tracker/  
├── final_socketpatched_v2.py  
├── ai_modules.py  
├── drone3d_stable.py  
├── README.md  
└── requirements.txt  

---

## 💻 Installation & Usage

Prerequisites  
- Python 3.9 or higher  
- USB-connected drone flight controller  

Install dependencies  
pip install -r requirements.txt  

Run the application  
python final_socketpatched_v2.py  

---

## 📌 Use Cases

- Drone ground control and monitoring  
- AI-assisted flight diagnostics  
- Embedded systems telemetry visualization  
- Technical support and troubleshooting tools  

---

## 🔮 Future Scope

- Flight mode classification  
- Predictive failure detection  
- Advanced battery health analytics  
- Telemetry-based autotuning suggestions  

---

## 👤 Author

Mohammad Aneek Khan  
BTech Computer Science Engineering  
Central University of Kashmir  
