# Integrating Motion Sensors (MEMS) with Unity Framework for 3D Game Simulation

<div align="center">

## TechBlazers 2.0

### AI Powered Gesture-Based 3D Game Control using STM32 MEMS Sensors and Unity Sentis

**Mentor:** Mr. Saurabh Rawat

**Developed by:**
 Amishi Mathur,
 Lakshveer Singh

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
STM32 IMU Sensors (ISM330DHCX)
        │
        ▼
Gyro EMA Filter (α = 0.4, 500 mdps dead-zone)
        │
        ▼
6-Axis Motion Vector
        │
        ▼
UART Communication (115200 baud)
        │
        ▼
Host PC (Python Bridge)
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
| STM32 B-U585I-IOT02A | Main embedded controller |
| ISM330DHCX IMU | 6-axis Accelerometer + Gyroscope |
| USB UART Interface | Communication (115200 baud) |
| PC/Laptop | AI Inference & Unity |
| Mouse Buttons | Drawing / Selection |

---

# Software Stack

- Unity 6
- Unity Sentis
- C#
- STM32CubeIDE
- STM32 HAL
- ISM330DHCX Component Driver
- Python (UART communication)
- pynput / pyserial
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

# Gesture Controller (Lakshveer Singh)

The STM32 firmware functions as a game controller, translating IMU sensor data into keyboard and mouse inputs via UART. The Python host script bridges these commands to the Windows input system.

## Control Scheme

The controller operates in two modes, toggled by double-tapping the user button.

### Normal Mode (Default)

| Action | Gesture |
|--------|---------|
| Move forward | Hold user button (maps to W key) |
| Look around | Tilt or rotate the board (maps to mouse movement) |

### Drawing Mode

| Action | Gesture |
|--------|---------|
| Draw | Hold user button (maps to left mouse button) |
| Move cursor | Tilt or rotate the board (maps to mouse movement) |
| Exit drawing mode | Double-tap user button (maps to spacebar) |

### Mode Toggle

Double-tapping the user button sends a spacebar tap to the host and switches between modes. The green LED indicates the current state:

- **Off** — Normal mode (W key active)
- **On** — Drawing mode (left click active)

> **Note:** The first tap of a double-tap produces a brief key press (W or left click) before the mode toggles. This is intentional — it eliminates added latency on real single presses, which is critical for responsive forward movement.

## UART Command Protocol

| Command | Trigger |
|---------|---------|
| `W_DOWN` | Button pressed in normal mode |
| `W_UP` | Button released in normal mode |
| `LCLICK_DOWN` | Button pressed in drawing mode |
| `LCLICK_UP` | Button released in drawing mode |
| `SPACE_TAP` | Double-tap detected (mode toggle) |
| `MOUSE_MOVE:dx,dy` | Continuous board tilt (20 ms interval) |

## Mouse-Look Filter

Gyroscope readings are processed through a single exponential moving average (EMA) filter with α = 0.4:

- **Gyro Z-axis** (yaw) → horizontal cursor movement
- **Gyro X-axis** (pitch) → vertical cursor movement
- **Dead-zone:** 500 mdps — eliminates tremor and noise
- **Update rate:** 20 ms (50 Hz)
- **Sensitivity:** 22 pixels per (mdps × iteration)

Cursor deltas are sent as `MOUSE_MOVE:dx,dy` commands over UART. The Python host uses the Windows `SendInput` API for relative mouse movement, which game engines correctly interpret as raw mouse deltas (unlike `SetCursorPos` which most games ignore).

## Button Handling

The user button is debounced (50 ms) and supports two actions:

- **Single press:** maps to W key (normal mode) or left mouse button (drawing mode)
- **Double-tap** (within 400 ms): toggles drawing mode and sends a spacebar tap

## LED Indicators

| LED | Behaviour |
|-----|-----------|
| Green | Off in normal mode; on in drawing mode |
| Red | Blinks briefly each time a UART command is transmitted |

## Python Host Script: linker_script_digit_recognition.py

The host script reads UART commands from the serial port and maps them to keyboard and mouse actions on the laptop.

### Dependencies

```bash
pip install pyserial pynput
```

### Usage

```bash
python linker_script_digit_recognition.py                # interactive COM port selection
python linker_script_digit_recognition.py --port COM3    # specify port directly
```

### Command Mapping

| UART Command | Host Action |
|-------------|-------------|
| `W_DOWN` | W key pressed and held |
| `W_UP` | W key released |
| `LCLICK_DOWN` | Left mouse button pressed and held |
| `LCLICK_UP` | Left mouse button released |
| `SPACE_TAP` | Spacebar tapped (press + release) |
| `MOUSE_MOVE:dx,dy` | Cursor moved by (dx, dy) pixels (relative) |

### Safety Features

- **Auto-reconnect:** The script automatically reconnects if the serial connection is interrupted.
- **Release on disconnect:** If the board disconnects while a key or mouse button is held, the script releases it to prevent stuck inputs.
- **Release on exit:** Pressing Ctrl+C releases all held keys and mouse buttons.

---

# AI Processing Pipeline

Once received on the host PC:

### Preprocessing

- Normalization
- Buffering
- Formatting

### Unity Sentis Model

The trained AI model performs inference on the incoming motion data.

### Prediction

Possible outputs include

- Forward
- Backward
- Left
- Right
- Rotate
- Stop
- Drawing Digits

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
- Dual-mode control (navigation + drawing)
- Gyroscope-based cursor with EMA filtering

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

**Gyro-side filtering (Gesture Controller):**

The gesture controller firmware applies its own signal conditioning on the gyroscope data before transmission:

- **EMA filter** (α = 0.4) smooths raw gyro readings to reduce jitter
- **Dead-zone** (500 mdps) suppresses noise and hand tremor when the board is held still
- **Sensitivity scaling** converts filtered angular velocity into pixel deltas

---

# Communication

```
STM32

      ↓

UART (115200 baud)

      ↓

Python Host (linker_script_digit_recognition.py)

      ↓

Windows SendInput API

      ↓

Unity / Sentis Model

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

- STM32 firmware (ISM330DHCX IMU driver, sensor pipeline, main loop)
- Dual-mode control system (Normal Mode + Drawing Mode with double-tap toggle)
- Gyroscope-based mouse-look cursor with EMA filtering (α = 0.4, 500 mdps dead-zone)
- UART command protocol design (W_DOWN/UP, LCLICK_DOWN/UP, SPACE_TAP, MOUSE_MOVE)
- Button debounce (50 ms) and double-tap detection (400 ms window)
- Python host bridge (linker_script_digit_recognition.py) — UART-to-keyboard/mouse translation
- Windows SendInput API integration for relative mouse movement
- Auto-reconnect and safety release mechanisms in host script
- LED status indicators (green = drawing mode, red = UART TX)

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
