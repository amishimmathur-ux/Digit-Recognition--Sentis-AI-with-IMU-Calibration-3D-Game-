# Integrating Motion Sensors (MEMS) with Unity Framework for 3D Game Simulation

<div align="center">

## TechBlazers 2.0

### AI Powered Gesture-Based 3D Game Control using STM32 MEMS Sensors and Unity Sentis

**Mentor:** Mr. Saurabh Rawat

**Developed by**
- Amishi Mathur
- Lakshveer Singh

</div>

---

# Overview

This project demonstrates how an embedded MEMS-based motion sensing system can be integrated with a Unity 3D application to provide intuitive gesture-driven game interaction.

Instead of relying solely on traditional keyboard and mouse input, an STM32 development board equipped with an inertial measurement unit (IMU) captures real-time hand motion. The captured sensor data is processed, calibrated, transmitted to a host computer, and interpreted by an AI-based digit recognition model built using the Unity Sentis inference engine.

The project combines:

- Embedded Systems
- MEMS Sensors
- AI Inference
- Unity 3D
- Human Computer Interaction
- Real-time Sensor Processing

---

# Objectives

- Develop a gesture-based game interaction system.
- Integrate STM32 MEMS sensors with Unity.
- Perform AI inference using Unity Sentis.
- Replace conventional input devices with motion-based interaction.
- Demonstrate low-latency real-time gameplay.

---

# System Architecture

```
Physical Hand Motion
        │
        ▼
STM32 IMU Sensors
        │
        ▼
MotionAC Calibration
        │
        ▼
6-Axis Motion Vector
        │
        ▼
UART Communication
        │
        ▼
Host PC
        │
        ▼
Preprocessing
        │
        ▼
Unity Sentis AI Model
        │
        ▼
Gesture Prediction
        │
        ▼
Unity Input System
        │
        ▼
Game Movement
```

---

# Hardware Used

| Component | Purpose |
|------------|----------|
| STM32 Development Board | Main embedded controller |
| ISM330DHCX IMU | Accelerometer + Gyroscope |
| USB UART Interface | Communication |
| PC/Laptop | AI Inference & Unity |
| Mouse Buttons | Drawing / Selection |

---

# Software Stack

- Unity 6
- Unity Sentis
- C#
- STM32CubeIDE
- STM32 HAL
- MotionAC Library
- Python (UART communication)
- Visual Studio Code

---

# MEMS Sensor Pipeline

The STM32 continuously reads data from the onboard IMU.

The pipeline consists of:

### 1. Sensor Acquisition

- 3-axis Accelerometer
- 3-axis Gyroscope

Outputs:

```
ax
ay
az

gx
gy
gz
```

---

### 2. Raw Data Collection

The STM32 continuously samples sensor registers and stores measurements into a buffer.

---

### 3. MotionAC Calibration

The MotionAC middleware performs

- Zero-offset compensation
- Thermal drift correction
- Sensor calibration
- Noise reduction

---

### 4. 6-Axis Motion Vector

The calibrated motion vector becomes

```
(ax, ay, az, gx, gy, gz)
```

---

### 5. Data Transmission

The processed motion data is transmitted through UART to the host computer.

---

# AI Processing Pipeline

Once received on the host PC:

### Preprocessing

- Normalization
- Buffering
- Formatting

↓

### Unity Sentis Model

The trained AI model performs inference on the incoming motion data.

↓

### Prediction

Possible outputs include

- Forward
- Backward
- Left
- Right
- Rotate
- Stop
- Drawing Digits

↓

### Unity Input Mapping

Predicted actions are converted into Unity game commands.

---

# Game Execution Pipeline

```
STM32 Motion

      ↓

Sensor Reading

      ↓

Motion Calibration

      ↓

UART

      ↓

Host PC

      ↓

Sentis AI Model

      ↓

Gesture Prediction

      ↓

Unity Input Manager

      ↓

Player Controller

      ↓

Character Movement

      ↓

Real-Time Rendering
```

---

# Features

- Real-time gesture recognition
- AI-based inference
- Low latency
- MEMS sensor integration
- Unity Sentis inference
- UART communication
- Motion calibration
- Interactive gameplay

---

# AI Model

Framework:

- Unity Sentis

Model Input:

- Motion vectors

Model Output:

- Predicted gesture
- Confidence score

---

# Motion Calibration

The project uses MotionAC middleware to improve sensor quality.

Calibration includes:

- Zero-g correction
- Bias compensation
- Thermal drift removal
- Sensor fusion preparation

---

# Communication

```
STM32

↓

UART

↓

Python Host

↓

Unity

↓

Sentis Model

↓

Game
```

---

# Project Workflow

```
User performs hand motion

↓

MEMS sensors detect movement

↓

STM32 reads sensor values

↓

MotionAC calibration

↓

6-axis motion vector generated

↓

UART transmission

↓

Host receives data

↓

Data preprocessing

↓

Unity Sentis inference

↓

Gesture prediction

↓

Unity Input Mapping

↓

Player movement inside game
```

---

# Challenges Faced

- Unity package compatibility
- Unity version migration
- Sensor drift
- Motion calibration
- Gesture stability
- UART synchronization
- AI inference latency
- Threshold tuning
- Cursor smoothing

---

# Learning Outcomes

## Embedded Systems

- STM32 HAL
- UART
- MEMS Sensors
- MotionAC

## Unity

- Scene management
- Input system
- Sentis
- C#

## Artificial Intelligence

- Model deployment
- Real-time inference
- Gesture recognition

## Signal Processing

- Filtering
- Calibration
- Sensor fusion
- Noise suppression

---

# Team Contributions

## Amishi Mathur

- Unity Sentis integration
- AI inference pipeline
- Gesture recognition implementation
- Unity game development
- AR/VR exploration

## Lakshveer Singh

- STM32 firmware
- Motion sensor processing
- UART communication
- Python host bridge
- Gyroscope-based cursor control
- Motion calibration

---

# Future Improvements

- Meta Quest Pro support
- Hand tracking
- Full VR integration
- Wireless BLE communication
- Larger gesture vocabulary
- Custom AI model training
- Multiplayer support
- Edge AI inference on embedded hardware

---

# Applications

- VR Gaming
- AR Interaction
- Rehabilitation
- Human Computer Interaction
- Smart Controllers
- Motion-controlled interfaces
- Robotics
- Educational simulations

---

# Acknowledgements

We sincerely thank **Mr. Saurabh Rawat** for his continuous guidance and mentorship throughout this project.

---

# Authors

**Amishi Mathur**

**Lakshveer Singh**

TechBlazers 2.0