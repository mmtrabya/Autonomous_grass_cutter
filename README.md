# 🤖 Autonomous Grass Care Robot 🌱  
**An eco-friendly, precision-guided robotic system for automating grass maintenance in large public spaces.**

---

## 🚀 Overview  
This project presents a high-precision autonomous robot designed to mow large green areas such as public parks, sports fields, and university campuses. It uses **RTK-GPS**, **IMU**, and **wheel encoders** for accurate localization, with a layered architecture built on **Raspberry Pi** and **Arduino** platforms.

Built for **municipalities**, **landscaping companies**, and **smart infrastructure planners**, the robot delivers a cost-effective and sustainable alternative to conventional grass maintenance.

---

## 📌 Key Features  
- ✅ **RTK-GPS-based Localization** – centimeter-level accuracy  
- ✅ **Systematic Path Planning** – no overlaps or missed areas  
- ✅ **Eco-Friendly** – fully electric operation with zero local emissions  
- ✅ **Obstacle Avoidance** – safe, autonomous movement  
- ✅ **Dual-Board Architecture** – Raspberry Pi + Arduino integration  
- ✅ **Smart City Ready** – designed for connected urban environments  

---

## 🧠 Technologies Used  
- **ROS 2 Foxy** (on Raspberry Pi 4)  
- **Arduino (Uno/Nano)** – motor control, encoder reading  
- **Python** + **C++** for ROS nodes  
- **QGIS / GeoJSON** for map zones  
- **Real-time sensor fusion** – GPS + IMU + Encoders  
- **Custom control logic** and safety protocols  

---

## 🛠️ Hardware Overview  

| Component          | Description                                   |
|--------------------|-----------------------------------------------|
| Raspberry Pi 4     | Runs ROS 2 nodes, localization, planning      |
| Arduino Uno/Nano   | Handles motor control, encoder inputs         |
| RTK-GPS (e.g. u-blox F9P) | Provides centimeter-level outdoor positioning |
| IMU (e.g. MPU6050) | Measures orientation and motion                |
| Wheel Encoders     | Tracks robot displacement                     |
| Motor Driver       | Dual-channel DC motor driver (e.g. L298N)     |
| 24V Battery Pack   | Powers motors and onboard systems             |
 
