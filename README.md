# In-HomeMobilityAssist: Closed-Loop Parallel Kinematics Control System

A real-time, parallel-kinematics tracking and control system built for the Raspberry Pi Pico SDK. This firmware implements multi-axis stepper motor actuation dynamically co-regulated by real-time magnetic rotary encoder feedback across a shared I2C bus to assist with localized indoor mobility tracking.

## 🚀 System Architecture

The core firmware handles high-frequency sensory acquisition, geometric state translation, and closed-loop motor correction within a strict **10ms control loop interval (BTI)**.

### Hardware Mapping
*   **Microcontroller:** Raspberry Pi Pico (RP2040)
*   **Sensors:** 4x AS5600 Magnetic Rotary Encoders (12-bit resolution via I2C)
*   **Multiplexer:** TCA9548A 8-Channel I2C Switch (Address `0x70`)
*   **Actuators:** Dual Translational Stepper Motors & Single Rotational Stepper Motor via Dedicated Pulse/Direction Drivers

---

## 🛠️ Pin Configuration

### I2C Interfaces
| Signal | Pico GPIO | Target Component |
|--------|-----------|------------------|
| **SDA**| GPIO 4    | TCA9548A Data    |
| **SCL**| GPIO 5    | TCA9548A Clock   |

### TCA9548A Multiplexer Bus Assignments
*   **Bus 0:** Angular Target ($r$) Sensor (`bus_num_r`)
*   **Bus 1:** Angular Target ($t$) Sensor (`bus_num_t`)
*   **Bus 2:** Rotational Hub Sensor (`bus_num_rot`)
*   **Bus 3:** Translational Hub Sensor (`bus_num_tran`)

### Stepper Motor Drivers
| Driver Target | PUL (Pulse) | DIR (Direction) | ENA (Enable) |
|---------------|-------------|-----------------|--------------|
| **Translational Motor 1 (T)**  | GPIO 0 | GPIO 1 | GPIO 4 |
| **Translational Motor 2 (T2)** | GPIO 2 | GPIO 3 | N/A    |
| **Rotational Motor (R)**       | GPIO 16| GPIO 17| N/A    |

---

## 📐 Kinematics & Control Strategy

The firmware processes mechanical expansion and alignment tracking states using closed-form parallel kinematics configurations:

1. **Sensory Acquisition:** The RP2040 queries individual AS5600 registers through mapped multiplexer channels, accounting for raw 12-bit transformations, mechanical zero-offsets, and phase rollovers ($> \pm180^\circ$).
2. **Geometric Calculation:** Tracks spatial deviations ($\Delta X, \Delta Y$) using fixed design constants ($Z = 33\text{mm}$ ground clearance, $R = 40\text{mm}$ arm length).
3. **PI Regulation:** Spatial errors ($y_{\text{hub}}$ and $\Delta\theta_{\text{hub}}$) drive independent Proportional-Integral (PI) control loops.
4. **Dynamic Pulse Interleaving:** The loop subdivides the 10ms window into 2000 micro-steps, checking division intervals on the fly via `actuate_motors()` to smoothly scale stepper frequencies based on instantaneous error magnitude.

---

## 📂 Project Repository Structure

*   `In-HomeMobilityAssist.c` — Main C firmware execution loop, kinematics computation, and PI stepper scheduling.
*   `CMakeLists.txt` — Build system configuration file for the Pico SDK toolchain.
*   `ReadSerial.py` — Python utility script for streaming, parsing, and logging live tracking diagnostics from the Pico's UART serial interface.
*   `Data Processing` — Data analysis scripts/notes handling log files for system characterization.

---

## 🔨 Build & Flash Instructions

1. Ensure the Raspberry Pi Pico SDK is correctly installed and configured in your environment path.
2. Initialize the build directory:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. Boot the Raspberry Pi Pico in `BOOTSEL` mode.
4. Copy the compiled `In-HomeMobilityAssist.uf2` binary file directly onto the mounted Pico mass-storage volume.
