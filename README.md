# RHEWI-CPSP: Rocketpy Hardware-in-the-Loop EKF Waypoint Interception

**RHEWI-CPSP** is a high-fidelity 6-DOF simulation environment and flight software suite designed for testing Guidance, Navigation, and Control (GNC) algorithms. This project integrates a **Python-based physics engine** with an **embedded C flight computer** (Arduino Nano ESP32) to perform Hardware-in-the-Loop (HIL) testing.

## Overview

The system allows developers to bridge the gap between simulation and real-world deployment. In **HIL mode**, the Python simulator acts as the environment, streaming noisy sensor data to the Arduino Nano ESP32. The ESP32 processes this data through its internal **16-state Extended Kalman Filter (EKF)** and control loops, then sends actuator commands back to the simulator to influence the flight path.

### Key Features

* **6-DOF Simulation**: Accurate rocket dynamics using `rocketpy`, featuring variable thrust and Mach-dependent drag.
* **Dual-Mode Firmware**:
* `APP_MODE_HIL`: Communicates via USB CDC with the Python simulator.
* `APP_MODE_REAL`: Interfaces with physical sensors (**MS8607** barometer and **BNO055** IMU) for actual flight.


* **Advanced State Estimation**: 16-state EKF implemented in high-performance C, estimating position, velocity, acceleration, orientation (quaternions), and angular rates.
* **Active Control**:
  * **Guidance**: Proportional navigation logic to steer toward a target coordinate.
  * **Roll Stabilization**: PD control for roll damping using aero-fins.



---

## Repository Structure

### Python Simulator (`/`)

* `FlightSim.py`: The main simulation entry point. Manages the 6-DOF integration, sensor noise injection, and the serial HIL bridge.
* `Cesaroni_8187M1545_P.csv`: Thrust curve for the rocket motor.
* `Nemesis150_v4.0_...csv`: Aerodynamic drag coefficients (Cd) for power-on and power-off phases.

### 🔌 Embedded Firmware (`/src` or your Arduino folder)

* `main.c`: Entry point using FreeRTOS tasks.
* `ekf.c / .h`: High-efficiency C implementation of the 16-state Kalman Filter.
* `app_hil.c / .h`: Manages the TinyUSB CDC communication and packet parsing.
* `app_context.h`: Defines the global application state and HIL command constants.
* `control_task.c`: Manages system status and RGB LED feedback.

---

## Hardware & LED Indicators

The firmware uses the **Arduino Nano ESP32** onboard RGB LED to signal the internal state:

| LED Color | Pattern | Meaning |
| --- | --- | --- |
| **Blinking Green** | Slow | **Idle (HIL Mode)**: Waiting for connection from Python. |
| **Blinking Red** | Fast | **Active (HIL Mode)**: Flight loop is running/receiving data. |
| **Blinking Blue** | Fast | **Flight Mode (Real)**: Reading from physical sensors. |

---

## Setup & Usage

### 1. Requirements

* **Python**: `rocketpy`, `numpy`, `pyserial`, `matplotlib`
* **Hardware**: Arduino Nano ESP32
* **Toolchain**: ESP-IDF or Arduino IDE (with ESP32 support and TinyUSB library)

### 2. Running a HIL Simulation

1. **Flash the Firmware**: Upload the C code to your Arduino Nano ESP32. Ensure `APP_MODE` is set to `APP_MODE_HIL` in `app_context.h`.
2. **Configure Python**: In `FlightSim.py`, set:
```python
HIL_ENABLED = True
HIL_PORT = 'COM_YOUR_PORT' # (e.g., 'COM3' or '/dev/ttyACM0')

```


3. **Execute**:
```bash
python FlightSim.py

```


The simulator will perform a handshake (`0x01` sync) and wait for the Arduino's `0x88` acknowledgement before launching.

---

## Technical Details

### The 16-State EKF

The filter estimates the following state vector $x$:


$$x = [pos_{x,y,z}, vel_{x,y,z}, accel_{x,y,z}, q_{0,1,2,3}, \omega_{x,y,z}]^T$$


It fuses 11 measurements:

* $1 \times$ Barometric Altitude
* $3 \times$ Accelerometer ($x, y, z$)
* $4 \times$ Quaternion Orientation (from IMU fusion)
* $3 \times$ Gyroscope ($\omega_x, \omega_y, \omega_z$)

### HIL Communication Protocol (Serial)

Communication is handled via binary `struct` packets for low latency:

* **Python → Arduino (`0x03`)**: Sends 11 floats (Sensors) + 1 float (Timestamp).
* **Arduino → Python**: Returns 19 floats (States + Actuator commands).

---

## ⚖️ License

This project is provided for educational and research purposes in experimental rocketry. Use at your own risk.
