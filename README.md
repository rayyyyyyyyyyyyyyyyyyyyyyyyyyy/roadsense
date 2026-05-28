# RoadSense: Vehicle-Mounted Data Acquisition System for Road Damage Detection

RoadSense is an automated, low-cost data collection and monitoring system designed to detect and map road anomalies (such as potholes and surface defects) in real-time. The system integrates IoT hardware mounted on a vehicle with a centralized Python processing script and an interactive web dashboard for geospatial data visualization.

## Key Features
* **High-Frequency Sampling:** Monitors road surface vibrations at 20 Hz using an MPU6050 accelerometer.
* **Centimeter-Level Accuracy:** Tracks exact defect locations using a high-precision ZED-F9P RTK-GNSS module.
* **Dynamic Camera Synchronization:** Computes variable camera shutter delays based on live vehicle speed to capture evidence images at a fixed distance (5 meters) post-impact.
* **Data Debouncing Mechanism:** Implements a 1.5-second cooldown algorithm to filter vehicle suspension bouncing and eliminate redundant records.
* **Interactive Web Dashboard:** Features Google Maps API integration, severity threshold sliders (Critical, Urgent, Moderate), and drag-and-drop support for local CSV log files and images.

## Core Algorithms

### 1. Relative Vibration Filtering
To suppress static gravity noise and engine-induced vibrations, the system evaluates the absolute change in acceleration over consecutive discrete time steps:

$$\Delta G(t) = |G(t) - G(t-1)|$$

* An anomaly is registered when $\Delta G(t) \ge 0.70$.

### 2. Dynamic Shutter Delay
To capture photographic evidence exactly 5 meters behind the detected pothole, the shutter delay is calculated dynamically relative to the vehicle's real-time velocity:

$$\text{Delay (seconds)} = \frac{5\text{ meters}}{\text{Vehicle Speed}}$$

## System Architecture & Tech Stack
The project is divided into three main components:
* **Hardware (Firmware):** ESP32 Microcontroller programmed to interface with MPU6050 (via I2C) and ZED-F9P GNSS (via UART), streaming telemetry over USB Serial.
* **Central Processing (Logger):** A Python pipeline executing the debouncing logic, dynamic camera delay, and local CSV/image registry storage.
* **Frontend (Dashboard):** A web application utilizing HTML5/JavaScript and Google Maps API for interactive visualization and severity ranking.

## Repository Structure
```text
├── firmware_esp32/       # Source code for ESP32 data collection
├── logger_python/        # Core Python script for data processing and camera sync
└── web_dashboard/        # Frontend files for web visualization and data management

Author & Acknowledgements
Developed by Onuma DOKPIKUL during an international research internship at the National Institute of Technology, Okinawa College (NITOC), under the guidance of Professor Suriyon TANSURIYAVONG.
