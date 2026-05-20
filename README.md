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
* **Toolchain**: ESP-IDF

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


### End-to-End Reproducibility Checklist (New Machine)

Follow this checklist strictly to reproduce the HIL experiment:

#### 1. Environment Setup (Python)
- [ ] Install required Python packages:
  ```bash
  pip install rocketpy numpy pyserial matplotlib


* [ ] Verify that `matplotlib` backend works (`TkAgg` is used in code)

#### 2. Firmware Configuration

* [ ] Open `app_context.h`
* [ ] Ensure HIL mode is enabled:
  ```c
  #define APP_MODE APP_MODE_HIL
  ```
* [ ] Flash firmware to Arduino Nano ESP32

#### 3. Serial Communication Setup

* [ ] Connect board via USB

* [ ] Identify serial port:
  * Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`
  * Windows: `COMx`

* [ ] Set port in `FlightSim.py`:
  ```python
  HIL_ENABLED = True
  HIL_PORT = "YOUR_PORT"
  HIL_BAUD = 115200
  ```

#### 4. Timing Consistency

* [ ] Ensure simulator runs at:
  ```python
  SAMPLING_RATE = 50  # Hz
  ```
* [ ] This must match firmware:
  ```c
  #define HIL_FREQ_HZ 50.0f
  ```

#### 5. Run the Experiment

* [ ] Power the board

* [ ] Run:
  ```bash
  python FlightSim.py
  ```

* [ ] Verify handshake:
  * Python sends `0x01`
  * Firmware responds (acknowledgment)

#### 6. Validate Active HIL Loop

* [ ] LED turns **blinking red**
* [ ] Continuous data exchange at \~50 Hz
* [ ] No serial timeout (timeout < 0.02 s)

#### 7. Validate Outputs

* [ ] Simulation plots are generated
* [ ] EKF states evolve over time
* [ ] Rocket trajectory changes (closed-loop behavior)

If any step fails, the HIL experiment is not correctly reproduced.


## Sensor Acquisition Experiment (REAL Mode) — Reproducibility Checklist

This section describes how to reproduce **real sensor readings + Teleplot visualization** (no HIL).

Follow this checklist exactly on a new machine.

---

### 1. Firmware Configuration (REAL Mode)

- [ ] Open `app_context.h`
- [ ] Set the application mode to REAL:

```c
#define APP_MODE APP_MODE_REAL
````

* [ ] This enables:
  * Physical sensor acquisition (`app_sensors_init()`)
  * Sensor loop execution (`app_sensors_loop()`)

***

### 2. Hardware Setup

* [ ] Connect the following sensors to the Arduino Nano ESP32:
  * **MS8607** (barometer)
  * **BNO055** (IMU)

* [ ] Ensure sensors are correctly wired on I²C
<img width="1376" height="868" alt="WhatsApp Image 2026-05-12 at 11 47 30" src="https://github.com/user-attachments/assets/3d131c22-ae8f-4505-b73b-5510337770ee" />


***

### 3. Flash the Firmware

* [ ] Build and flash using ESP-IDF (VSCode or CLI)
* [ ] After flashing, power the board via USB

***

### 4. Verify System Is Running

* [ ] Observe onboard LED:
  * **Blinking Blue (fast)** → REAL mode active

* This indicates:
  * Sensor loop is running
  * FreeRTOS tasks are active 

***

### 5. Open Serial Monitor

* [ ] Open a serial monitor (VSCode / ESP-IDF Monitor / PuTTY):
  * Baud rate: **115200**

* [ ] You should see continuous output in the following format:

```
>BARO_z:XXX m
>RAW_roll:XXX rad
>RAW_pitch:XXX rad
>RAW_yaw:XXX rad
>EKF_z:XXX m
>EKF_roll:XXX rad
>EKF_pitch:XXX rad
>EKF_yaw:XXX rad
```

* These values are printed using Teleplot-compatible syntax

***

### 6. Enable Teleplot in VSCode

* [ ] Install **Teleplot extension** in VSCode

* [ ] Open Command Palette:
  ```
  Teleplot: Connect to Serial
  ```

* [ ] Select the correct serial port

* Teleplot will automatically parse variables formatted as:
  ```
  >variable:value unit
  ```

***

### 7. Verify Sensor Data

Check that:

* [ ] **BARO\_z** changes with altitude/pressure
* [ ] **RAW\_roll/pitch/yaw** respond to board motion
* [ ] **EKF\_values** evolve consistently with raw data
* [ ] Data updates continuously (\~50 Hz)

Sampling frequency is defined as:

````c
#define SENSORS_SAMPLE_FREQ_HZ 50.0f
````

---

### 8. Validate EKF Behavior

- [ ] Compare:
  - RAW attitude (IMU)
  - EKF attitude (filtered)

- Expected:
  - EKF signals are smoother than raw measurements
  - No discontinuities in angles (angle unwrapping applied)

---

### 9. Minimal Success Criteria

The experiment is successful if:

- [ ] Continuous Teleplot streams are visible
- [ ] Sensor values react to motion
- [ ] EKF outputs are stable and smooth
- [ ] No missing or corrupted serial data

---

## Work Division and Contributions

This project was developed as a collaborative effort, with a clear division of responsibilities that reflects both system-level design thinking and low-level embedded implementation.

### Luca Di Giorgio

Luca Di Giorgio was responsible for the **embedded software development on the Arduino Nano ESP32**, covering the complete low-level and real-time implementation of the flight computer. His contributions include:

- Development of **hardware drivers** for onboard sensors:
  - MS8607 barometer
  - BNO055 IMU
- Full implementation of the **16-state Extended Kalman Filter (EKF)** in C, optimized for real-time execution on a microcontroller
- Implementation of the **Guidance, Navigation, and Control (GNC)** stack on the embedded side:
  - Guidance logic
  - Roll stabilization control
- Integration of all subsystems within a **FreeRTOS-based architecture**
- Implementation of the **sensor acquisition pipeline** and Teleplot-compatible visualization output

Overall, Luca engineered the **complete embedded flight software**, ensuring deterministic timing, efficient computation, and correct hardware interfacing.

---

### Filippo Galavotti

Filippo Galavotti was responsible for the **high-level simulation environment and system integration**, enabling the Hardware-in-the-Loop (HIL) validation framework. His contributions include:

- Development of the **Python-based 6-DOF flight simulator** using `rocketpy`
- Design and implementation of the **serial communication protocol** between simulator and embedded system
- High-level design and implementation of:
  - Extended Kalman Filter (EKF)
  - Guidance algorithms
  - Roll control logic

- Integration of:
  - Sensor noise and bias models
  - Realistic flight dynamics and environmental conditions
- Implementation of **data visualization and post-processing tools**

Filippo effectively built the **digital twin of the physical system**, allowing rigorous validation of embedded algorithms before deployment.

---

### Overall Assessment

The project demonstrates a **well-balanced division between embedded systems engineering and high-level simulation**, closely resembling real aerospace GNC development workflows.

- The embedded implementation ensures **real-time feasibility and hardware correctness**
- The simulation layer ensures **repeatability, observability, and validation**

This separation of concerns, combined with a fully functional HIL loop, represents a **complete and coherent experimental framework**, suitable for advanced research and prototyping in guidance and control systems.


## ⚖️ License

This project is provided for educational and research purposes in experimental rocketry. Use at your own risk.
