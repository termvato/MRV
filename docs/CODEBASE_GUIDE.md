# MRV Robot Controller — Full Codebase Guide

This document explains every file, concept, and decision in the MRV firmware from the ground up. It is written for someone who understands basic programming but may be new to embedded systems, robotics maths, or C++.

---

## Table of Contents

1. [What this project does](#1-what-this-project-does)
2. [Hardware overview](#2-hardware-overview)
3. [Project structure](#3-project-structure)
4. [The build system — PlatformIO](#4-the-build-system--platformio)
5. [Terminology and shorthands](#5-terminology-and-shorthands)
6. [File-by-file walkthrough](#6-file-by-file-walkthrough)
   - [config/config.h](#configconfigh)
   - [hardware/pinout.h](#hardwarepinouth)
   - [hardware/imu.h and imu.cpp](#hardwareimuhandimucpp)
   - [hardware/encoder.h and encoder.cpp](#hardwareencoderhandencordercpp)
   - [hardware/motor_driver.h and motor_driver.cpp](#hardwaremotor_driverhandmotor_drivercpp)
   - [math/quaternion.h and quaternion.cpp](#mathquaternionhandquaternioncpp)
   - [math/filters.h and filters.cpp](#mathfiltershandfilterscpp)
   - [state/robot_state.h and robot_state.cpp](#staterobot_statehrobot_statecpp)
   - [state/state_estimator.h and state_estimator.cpp](#statestate_estimatorhstate_estimatorcpp)
   - [comms/serial_reporter.h and serial_reporter.cpp](#commsserial_reporterhserial_reportercpp)
   - [main.cpp](#maincpp)
7. [Visualisation tools](#7-visualisation-tools)
   - [tools/visualise_imu.py](#toolsvisualise_imupy)
   - [tools/visualise_imu.html](#toolsvisualise_imuhtml)
8. [Data flow — end to end](#8-data-flow--end-to-end)
9. [Serial output format](#9-serial-output-format)
10. [What is stubbed / not yet built](#10-what-is-stubbed--not-yet-built)
11. [Libraries reference](#11-libraries-reference)

---

## 1. What this project does

The MRV robot has a physical body with moving arms and joints. This firmware runs on a small computer (a **Teensy 4.0** microcontroller) that is mounted on the robot. Its current jobs are:

1. Read the **IMU** (inertial measurement unit) 120 times per second to find out how the robot body is oriented in space — is it tilted? Rotated? Upright?
2. Express that orientation as a **quaternion** (a compact mathematical description of rotation — explained below).
3. Derive human-readable **roll, pitch, and yaw** angles from the quaternion using standard ZYX Euler conversion.
4. Read the **ARM0** (pins 22/23) and **ARM1** (pins 20/21) **joint encoders** and convert quadrature counts to degrees, angular velocity (deg/s), and angular acceleration (deg/s²).
5. Read two **speed encoders** (pins 14/15 and 16/17) and report shaft RPM.
6. Drive **arm motors** (TB6612FNG H-bridge, pins 8/9/10/11/12) and a **spring motor** (pins 2/7) via serial commands from the visualiser.
7. Run a **safety cutoff ISR** at 1 kHz that kills all motors if roll or yaw exceeds 45° — independent of the main loop.
8. Send all sensor data over **USB serial** to a connected computer 10 times per second.
9. Accept incoming serial commands (`ARM:`, `SPR:`, `SAFE_CLR`) from the visualiser to control motors.
10. Display the live orientation and arm angles in a **3D visualiser** (either Python or web browser).

Remaining future work: wire the LEG encoder (joint index 2).

---

## 2. Hardware overview

| Component | Part | Role |
|-----------|------|------|
| Microcontroller | **Teensy 4.0** | Runs the firmware. ARM Cortex-M7 CPU at 600 MHz. |
| IMU | **LSM6DSV16X** | 6-axis IMU (accelerometer + gyroscope). Has a built-in fusion processor. |
| IMU breakout | SparkFun Micro 6DoF (Qwiic) | Breakout board for the LSM6DSV16X, communicates via I²C. |
| ARM0 encoder | PIM604 quadrature encoder on pins 22/23 | Measures the angle of the left arm joint in degrees. |
| ARM1 encoder | PIM604 quadrature encoder on pins 20/21 | Measures the angle of the right arm joint in degrees. |
| Speed encoders | PIM604 on pins 14/15 and 16/17 | Measure shaft RPM for two drive motors. |
| Arm motor driver | **TB6612FNG** dual H-bridge | Drives two arm motors (A and B) via pins 8/9/10/11/12. STBY on pin 10 = safety cutoff. |
| Spring motor driver | **TB6612FNG** (channels paralleled) | Drives the spring winding motor via pins 2/7. STBY permanently HIGH. |
| LEG encoder | Not yet wired (joint index 2) | Will measure vertical slide position. |

**IMU** stands for *Inertial Measurement Unit*. It measures two things:
- **Accelerometer**: linear acceleration in three axes (X, Y, Z). At rest it measures gravity.
- **Gyroscope**: angular velocity (rotation speed) in three axes.

The LSM6DSV16X combines both measurements inside the chip using a **SFLP** (Sensor Fusion Low Power) engine to produce a smooth orientation estimate, output as a quaternion.

---

## 3. Project structure

```
MRV_code/
│
├── platformio.ini          ← Build configuration (compiler, libraries, ports)
│
├── models/
│   └── model.obj           ← 3D model of the robot (used only by the visualiser)
│
├── src/                    ← All firmware source code
│   ├── main.cpp            ← Entry point (setup + loop)
│   │
│   ├── config/
│   │   └── config.h        ← Every tunable constant in one place
│   │
│   ├── hardware/
│   │   ├── pinout.h           ← Every pin number in one place
│   │   ├── imu.h/.cpp         ← IMU driver (fully implemented)
│   │   ├── encoder.h/.cpp     ← Joint encoder driver (ARM0+ARM1 active, speed encoders, kinematics)
│   │   └── motor_driver.h/.cpp ← TB6612 H-bridge driver with safety cutoff
│   │
│   ├── math/
│   │   ├── quaternion.h/.cpp ← Quaternion maths
│   │   └── filters.h/.cpp    ← Signal filters (stub — not yet needed)
│   │
│   ├── state/
│   │   ├── robot_state.h/.cpp       ← The data structure holding current robot state
│   │   └── state_estimator.h/.cpp   ← Runs the sampling loop, populates state
│   │
│   └── comms/
│       └── serial_reporter.h/.cpp   ← Sends state to the computer over USB
│
└── tools/                  ← PC-side tools (not part of the firmware)
    ├── visualise_imu.py    ← Python 3D visualiser
    └── visualise_imu.html  ← Web browser 3D visualiser (uses WebSerial + Three.js)
```

**Key rule: one responsibility per file.** Every file has exactly one job. The IMU file knows nothing about serial output. The serial file knows nothing about quaternion maths. This makes each file easier to understand, test, and replace.

---

## 4. The build system — PlatformIO

### What is a build system?

A build system automates the process of turning your source code (`.cpp`, `.h` files) into a binary that can run on the Teensy. It handles:
- Compiling each source file
- Linking them together
- Downloading the result to the device

### PlatformIO

PlatformIO is a build system and package manager for embedded projects. It is an extension for VS Code. The configuration lives in **`platformio.ini`**.

### `platformio.ini` explained

```ini
[env:teensy40]
platform       = teensy          ← Use the Teensy compiler toolchain
board          = teensy40        ← Specific board: Teensy 4.0
framework      = arduino         ← Use the Arduino API (setup/loop, Serial, etc.)

lib_deps =
    stm32duino/STM32duino LSM6DSV16X  ← Auto-download the IMU library
    paulstoffregen/Encoder            ← Auto-download the quadrature encoder library

build_flags =
    -std=gnu++17   ← Use C++17 standard (more modern C++ features)
    -I src         ← Add src/ to the include search path
    -Wall          ← Enable common compiler warnings
    -Wextra        ← Enable extra compiler warnings

upload_protocol = teensy-cli             ← Use teensy-cli tool to upload
upload_port     = /dev/cu.usbmodem...    ← The USB port the Teensy is connected to
monitor_speed   = 115200                 ← Baud rate for the serial monitor
```

**Baud rate**: the speed of serial communication, in bits per second. Both sides (Teensy and computer) must agree on the same number. 115200 is a common fast rate.

**`-I src`**: This tells the compiler that when code writes `#include "config/config.h"`, it should look in the `src/` directory first. Without this flag, the path would need to be written differently.

---

## 5. Terminology and shorthands

### C++ basics

| Term | Meaning |
|------|---------|
| `#pragma once` | Tells the compiler to include this header file at most once per compilation unit. Prevents duplicate definitions. |
| `#include "file.h"` | Paste the contents of `file.h` here at compile time. Quotes = local project file. |
| `#include <library.h>` | Same but angle-brackets = system or external library. |
| `constexpr` | A constant whose value is known at compile time. Computed once; zero runtime cost. Safer than `#define`. |
| `static` (inside a file) | The variable or function is only visible in this file. Other files cannot accidentally use it. |
| `volatile` | Tells the compiler this variable can change at any time (e.g., from an interrupt), so it must never cache it in a register. |
| `uint8_t`, `uint16_t`, `uint32_t` | Unsigned integers of exactly 8, 16, or 32 bits. The `u` suffix on a number literal (e.g., `120u`) makes it unsigned. |
| `float` | 32-bit floating-point number. Used for all sensor maths. |
| `const` | A value that cannot be changed after it is set. |
| `static constexpr` | A compile-time constant that belongs to a class but is shared across all instances (not per-object). |
| `F("string")` | Arduino macro. Stores the string in flash memory instead of RAM. Important on chips with limited RAM (less relevant on Teensy 4.0 but good practice). |

### C++ concepts

| Term | Meaning |
|------|---------|
| **Header file** (`.h`) | Declares *what* exists: class names, function signatures, constants. Shared between files. |
| **Source file** (`.cpp`) | Defines *how* it works: the actual function bodies. |
| **Class** | A blueprint for an object. Bundles data (fields) and behaviour (methods) together. |
| **Constructor** | A special method called automatically when an object is created. Sets initial values. |
| **`begin()`** | A common pattern in Arduino code: initialise hardware once in `setup()`. |
| **`update()`** / **`tick()`** | Methods called repeatedly in `loop()` to do ongoing work. |
| **`private`** | Fields or methods only accessible from inside the class. |
| **`public`** | Fields or methods accessible from outside the class. |
| **`const` method** | A method that promises not to modify the object. Written as `void foo() const;`. |
| **Reference** (`&`) | An alias for another variable. Avoids copying large objects. `const Quaternion& q` means "a reference to a Quaternion I promise not to modify." |
| **`explicit` constructor** | Prevents accidental automatic conversions. E.g., `explicit IMU()` means you must write `IMU()` explicitly and the compiler won't silently convert other types to an IMU. |
| **ISR** | Interrupt Service Routine. A function that runs automatically when a hardware event fires (e.g., a timer). Must be very fast and avoid slow operations like I²C. |
| **Heap** | Dynamic memory allocated with `new`/`malloc`. Fragmentation is dangerous on embedded systems. This project avoids the heap. |
| **Stack** | Automatic memory for local variables in a function. Cleaned up when the function returns. |

### Embedded / hardware terms

| Term | Meaning |
|------|---------|
| **I²C** (I2C / IIC) | Inter-Integrated Circuit. A two-wire bus (SDA + SCL) for communicating with sensors at low speed. This project uses I²C for the IMU. |
| **SPI** | Serial Peripheral Interface. A four-wire bus, faster than I²C. The IMU can optionally use SPI (not the default in this project). |
| **FIFO** | First-In, First-Out buffer. The IMU stores new quaternion samples in a FIFO queue on-chip. The firmware reads and drains it. |
| **ODR** | Output Data Rate. How many times per second the sensor produces a new reading. |
| **FS / FSR** | Full-Scale Range. The maximum value the sensor can measure. ±2g for the accelerometer means it saturates at twice Earth's gravity. ±250 dps for the gyroscope means it saturates at 250 degrees per second. Smaller range = finer resolution. |
| **IntervalTimer** | A Teensy hardware timer that calls a function at a precise interval. Teensy 4.0 has four of them. |
| **micros()** | Arduino function. Returns the number of microseconds since the program started (as a 32-bit unsigned integer, wraps after ~70 minutes). |
| **millis()** | Same but in milliseconds. |
| **Baud rate** | Serial communication speed in bits per second. |
| **WHO_AM_I** | A standard register on I²C/SPI sensors that returns the chip's ID. Used to verify the sensor is connected and responding correctly. |
| **SA0** | A hardware pin on the LSM6DSV16X that selects one of two I²C addresses. Tied HIGH on the SparkFun board → address 0x6B. |
| **Quadrature encoder** | A rotary position sensor with two output signals (A and B) that are 90° out of phase. The phase difference reveals direction; counting transitions reveals position. |
| **PPR** | Pulses Per Revolution. The number of complete A/B cycles an encoder produces per shaft revolution. In full quadrature (4× counting), 1 PPR = 4 raw counts. |
| **CPR / counts per rev** | Total quadrature counts per revolution = 4 × PPR. This is the number you configure in `ENC_ARM1_COUNTS_PER_REV`. |
| **Gear ratio** | How many encoder shaft turns per one joint output turn. If the encoder is geared 5:1 down to the arm, one full arm rotation = 5 encoder revolutions. |

### Sensor fusion terms

| Term | Meaning |
|------|---------|
| **SFLP** | Sensor Fusion Low Power. ST's proprietary algorithm running inside the LSM6DSV16X chip, combining accelerometer and gyroscope data to produce a stable orientation estimate. |
| **Game rotation vector** | The specific SFLP output used here: a quaternion representing 3-DOF orientation (no magnetometer). Yaw is relative to power-on, not magnetic north. |
| **Quaternion** | A 4-number representation of a 3D rotation. More compact and numerically stable than Euler angles. Described in detail below. |
| **Euler angles** | Roll, pitch, yaw — three angles describing rotation about three axes. Intuitive but suffer from gimbal lock. |
| **Roll** | Rotation about the front-to-back (X) axis. Like tilting your head to the side. |
| **Pitch** | Rotation about the left-to-right (Y) axis. Like nodding. |
| **Yaw** | Rotation about the vertical (Z) axis. Like shaking your head "no". |
| **Gimbal lock** | A situation where two rotation axes align, causing the loss of one degree of freedom. Occurs when pitch reaches ±90°. Quaternions avoid this. |
| **Drift** | Slow accumulation of error in the yaw angle over time, because the SFLP engine has no magnetometer to give an absolute yaw reference. Roll and pitch drift very little because gravity provides an absolute reference. |

### Three.js / visualiser terms

| Term | Meaning |
|------|---------|
| **Three.js** | A JavaScript library that makes 3D graphics in a web browser easy. Wraps the browser's WebGL API. |
| **WebGL** | The browser's 3D graphics API, based on OpenGL. Three.js sits on top of it. |
| **WebSerial** | A browser API (Chrome and Edge only) that allows a web page to talk directly to a USB serial port. |
| **OBJ / MTL** | Common 3D model file formats. `.obj` stores geometry (vertices, faces). `.mtl` stores materials (colours, textures). |
| **Slerp** | Spherical Linear Interpolation. Smoothly interpolates between two quaternion orientations. Used to add visual damping so the 3D model doesn't jitter. |
| **AxesHelper** | A Three.js object that draws three coloured arrows: Red = X, Green = Y, Blue = Z. |
| **OrbitControls** | A Three.js add-on that lets you drag to rotate the camera, scroll to zoom, and right-drag to pan. |
| **Pivot** | The point about which the 3D model rotates. Set to the bottom of the robot's foot so the robot appears to stand on the ground. |
| **Z-up vs Y-up** | Three.js uses Y as the "up" axis. Autodesk (which exported the OBJ) uses Z as "up". A 90° rotation corrects this. |
| **Conjugation** (quaternion) | R·q·R⁻¹ — a way to change a quaternion's reference frame. Used to apply the IMU's axis corrections without breaking continuous rotation. |
| **IMU_OFFSET_INV** | A correction quaternion that cancels the 45° angle at which the IMU board is physically mounted on the robot. |

---

## 6. File-by-file walkthrough

---

### `config/config.h`

**Purpose:** Single source of truth for every number in the project. No magic numbers appear anywhere else.

**Why this matters:** If you want to change the sample rate, you change it in one place. Every file that needs it just reads this header.

```cpp
constexpr uint16_t IMU_SAMPLE_RATE_HZ = 120u;
```
The firmware asks the IMU for a new orientation reading 120 times per second. This matches the IMU chip's maximum fusion rate (SFLP_ODR_HZ).

```cpp
constexpr uint16_t SERIAL_REPORT_RATE_HZ = 10u;
```
The firmware only prints to the serial port 10 times per second. Reading 120 Hz and printing 10 Hz avoids flooding the USB connection while still getting smooth sensor data internally.

```cpp
constexpr float IMU_ACCEL_ODR_HZ = 120.0f;
constexpr uint8_t  IMU_ACCEL_FS_G  = 2u;
```
The accelerometer is configured to measure 120 times per second, with a ±2g full-scale range. ±2g means if the robot accelerates faster than twice Earth's gravity (unlikely), readings will saturate. The smaller the range, the more detail you get within it.

```cpp
constexpr float    IMU_GYRO_ODR_HZ  = 120.0f;
constexpr uint16_t IMU_GYRO_FS_DPS  = 250u;
```
The gyroscope: 120 Hz, ±250 degrees per second. The robot moves slowly, so this range gives good resolution.

```cpp
constexpr float SFLP_ODR_HZ      = 120.0f;
constexpr uint8_t SFLP_ENABLE_GBIAS = 1u;
```
The SFLP fusion engine runs at 120 Hz. `GBIAS` (gyroscope bias estimation) is enabled — the chip tracks slow drift in the gyroscope and corrects for it automatically over ~30 seconds of use.

```cpp
constexpr uint16_t IMU_FIFO_WATERMARK = 1u;
```
The IMU's internal FIFO buffer. A watermark of 1 means: as soon as one sample is ready, the firmware reads it. Keeps latency minimal.

```cpp
constexpr float ENC_ARM1_COUNTS_PER_REV = 12.0f;  // DO NOT TOUCH
constexpr float ENC_ARM1_GEAR_RATIO     = 50.0f;  // DO NOT TOUCH
```
ARM1 encoder calibration for the **Pimoroni PIM604** Rotary Encoder Breakout. Empirically calibrated — the effective counts-per-rev is 12 (not the nominal 96) due to the encoder's output characteristics. The ARM1 joint has a **50:1 gear reduction** between the encoder shaft and the arm output. ARM0 uses identical constants (`ENC_ARM0_COUNTS_PER_REV = 12.0f`, `ENC_ARM0_GEAR_RATIO = 50.0f`).

```cpp
constexpr float ENC_ARM0_INIT_DEG = 0.0f;
constexpr float ENC_ARM1_INIT_DEG = 90.0f;
```
Calibration offsets: the physical rest position in degrees at power-on. ARM1 starts at 90° because of its mounting orientation.

```cpp
constexpr uint8_t NUM_SPEED_ENCODERS = 2u;
constexpr float ENC_SPEED_COUNTS_PER_REV = 96.0f;
```
Two speed-only encoders (pins 14/15 and 16/17) report shaft RPM but not absolute position.

```cpp
constexpr uint8_t ENC_VEL_WINDOW = 10u;
constexpr float   ENC_VEL_ALPHA  = 0.10f;
```
Encoder kinematics filter: velocity is computed over a sliding window of 10 ticks (~83 ms at 120 Hz). An EMA filter with alpha=0.10 smooths the velocity before acceleration is derived.

```cpp
constexpr float ACCEL_TILT_ALPHA = 0.15f;
```
EMA smoothing factor for raw accelerometer tilt computation (reserved for future use — currently not called in the main path).

```cpp
constexpr uint32_t SAFETY_CHECK_RATE_HZ = 1000u;
constexpr float    SAFETY_TILT_DEG      = 45.0f;
```
The safety ISR runs at 1 kHz in its own IntervalTimer. If |roll| or |yaw| exceeds 45°, the arm motor driver asserts STBY LOW and all motors stop. Motors can only be re-enabled via the `SAFE_CLR` serial command once tilt is back within limits.

```cpp
constexpr uint8_t IMU_I2C_SA0 = 1u;
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000u;
```
`SA0 = 1` selects I²C address 0x6B (the SparkFun breakout board default). 400,000 Hz = 400 kHz = "Fast Mode" I²C — four times faster than the 100 kHz default.

---

### `hardware/pinout.h`

**Purpose:** Every Teensy pin number in one place. Hardware wiring changes only require editing this file.

```cpp
// IMU
constexpr uint8_t PIN_IMU_INT1   = 2u;   // sensor INT1 (unused — firmware polls)
constexpr uint8_t PIN_IMU_SPI_CS = 10u;  // SPI chip-select (unused — I2C mode)

// Joint encoders
constexpr uint8_t PIN_ENC_ARM0_A = 22u;  // ARM0 (left arm) encoder channel A
constexpr uint8_t PIN_ENC_ARM0_B = 23u;  // ARM0 encoder channel B
constexpr uint8_t PIN_ENC_ARM1_A = 20u;  // ARM1 (right arm) encoder channel A
constexpr uint8_t PIN_ENC_ARM1_B = 21u;  // ARM1 encoder channel B

// Speed encoders
constexpr uint8_t PIN_ENC_SPD2_A = 14u;  // Speed encoder 2 channel A
constexpr uint8_t PIN_ENC_SPD2_B = 15u;  // Speed encoder 2 channel B
constexpr uint8_t PIN_ENC_SPD3_A = 16u;  // Speed encoder 3 channel A
constexpr uint8_t PIN_ENC_SPD3_B = 17u;  // Speed encoder 3 channel B

// Arm motors (TB6612FNG)
constexpr uint8_t PIN_ARM_AIN2 = 8u;   // Motor A direction 2
constexpr uint8_t PIN_ARM_AIN1 = 9u;   // Motor A direction 1 / PWM
constexpr uint8_t PIN_ARM_STBY = 10u;  // Standby — LOW kills both arm motors (safety cutoff)
constexpr uint8_t PIN_ARM_BIN1 = 11u;  // Motor B direction 1 / PWM
constexpr uint8_t PIN_ARM_BIN2 = 12u;  // Motor B direction 2

// Spring motor (TB6612FNG, channels paralleled)
constexpr uint8_t PIN_SPR_IN1 = 7u;   // Spring forward direction / PWM
constexpr uint8_t PIN_SPR_IN2 = 2u;   // Spring reverse direction / PWM

// Status LED
constexpr uint8_t PIN_LED = 13u;
```

**Pin sharing notes:** `PIN_IMU_SPI_CS` (pin 10) and `PIN_ARM_STBY` (pin 10) share the same physical pin — this is safe because SPI mode is disabled (I2C is used). Similarly, `PIN_IMU_INT1` (pin 2) and `PIN_SPR_IN2` (pin 2) share because the INT1 interrupt is unused (firmware polls the IMU).

---

### `hardware/imu.h` and `imu.cpp`

**Purpose:** Talk to the LSM6DSV16X chip and deliver a clean quaternion. Nothing outside this file needs to know how the sensor works.

#### The class interface (`imu.h`)

The `IMU` class has seven public methods:

| Method | When to call | What it does |
|--------|-------------|-------------|
| `IMU()` | Automatically on creation | Sets quaternion to identity, marks as uninitialised |
| `begin()` | Once in `setup()` | Starts I²C, configures chip, starts FIFO |
| `update()` | Each sample tick | Drains the FIFO, stores the latest quaternion |
| `getQuaternion()` | After `update()` | Returns the stored quaternion |
| `calibrate()` | On demand | Resets the SFLP engine to re-zero orientation |
| `getRawAccel(ax,ay,az)` | On demand | Reads the accelerometer output in g (polled, not FIFO) |
| `getStatusString()` | After any method | Returns a human-readable status/error string |

#### What `begin()` does step by step (`imu.cpp`)

1. **Start the I²C bus** at 400 kHz
2. **WHO_AM_I check** — call `_sensorPtr->begin()`. The library reads the sensor's ID register. If it doesn't match the expected value, the sensor is not connected or is at the wrong address.
3. **Enable the accelerometer** (`Enable_X()`) and set its ODR and full-scale range
4. **Enable the gyroscope** (`Enable_G()`) and set its ODR and full-scale range
5. **Set SFLP ODR** — how fast the fusion engine runs
6. **Enable the rotation vector** — turns on SFLP and tells it to write quaternions to the FIFO
7. **Set FIFO watermark** to 1
8. **Set FIFO mode** to stream (continuous) — when the FIFO is full, oldest samples are overwritten, so you always get the freshest reading

#### What `update()` does

The FIFO may contain several samples if the loop is slightly late. The function reads *all* of them but keeps only the last one:

```
for each sample in FIFO:
    read its tag (what type of data is it?)
    if tag == kTagGameRv (0x13):
        read the quaternion data
        store it (do NOT stop — keep reading to get the freshest)
    else:
        consume the 6 bytes anyway to advance the read pointer
```

The tag `0x13` (`kTagGameRv`) identifies SFLP game rotation vector data. The chip's C header defines this as an anonymous enum value, which is inaccessible from C++ by name, so the raw hex value is used directly.

#### Module-static sensor object

```cpp
static LSM6DSV16XSensor s_sensor(&Wire, LSM6DSV16X_I2C_ADD_H);
```

`static` here means this variable lives only in `imu.cpp` — no other file can see or accidentally modify it. The `IMU` class holds a pointer to it (`_sensorPtr`). This avoids heap allocation while still allowing the class to be used normally.

---

### `hardware/encoder.h` and `encoder.cpp`

**Purpose:** Read quadrature encoders on each robot joint and convert counts to degrees, angular velocity, and angular acceleration. ARM0 (joint index 0, pins 22/23) and ARM1 (joint index 1, pins 20/21) are fully implemented. Two speed-only encoders (pins 14/15 and 16/17) report shaft RPM.

The class is named **`JointEncoders`** (not `Encoder`) to avoid a naming collision with the `Encoder` class from the paulstoffregen library, which is used internally inside `encoder.cpp`.

#### Quadrature encoding

A rotary encoder outputs two square waves called **channel A** and **channel B**. The waves are 90° out of phase with each other. By watching which channel transitions first, you can tell:
- **Direction**: if A rises before B, it's spinning clockwise; if B rises before A, it's counter-clockwise.
- **Position**: each full cycle of the two channels = 4 counts. The PIM604 is a 24 PPR encoder, so it nominally produces 4 × 24 = 96 quadrature counts per encoder shaft revolution.

The paulstoffregen/Encoder library handles all of this using hardware interrupts — the CPU is notified every time a pin changes, counts are kept automatically, and you just call `read()` to get the current count.

#### Counts-to-degrees conversion

```
degrees = count × 360 / (ENC_ARM1_COUNTS_PER_REV × ENC_ARM1_GEAR_RATIO)
```

- `ENC_ARM1_COUNTS_PER_REV` — empirically calibrated to 12.0 (not the nominal 96).
- `ENC_ARM1_GEAR_RATIO` — 50.0 (50:1 gear reduction).
- ARM0 uses identical constants.

#### Kinematics (velocity and acceleration)

Angular velocity is computed over a sliding window of `ENC_VEL_WINDOW` (10) past angle samples — total angle change divided by total elapsed time. This averages away quantisation noise. The velocity is then smoothed with an EMA filter (`ENC_VEL_ALPHA = 0.10`), and acceleration is derived as the rate of change of the smoothed velocity.

#### `begin()`

Resets counts to zero and applies calibration offsets (`ENC_ARM0_INIT_DEG`, `ENC_ARM1_INIT_DEG`) so the power-on position is defined correctly. ARM1 starts at 90° because of its mounting orientation.

#### Joint index mapping

| Index | Joint | Status |
|-------|-------|--------|
| 0 | ARM (left) | **active — pins 22/23** |
| 1 | ARM(1) (right) | **active — pins 20/21** |
| 2 | LEG (vertical slide) | stub — 0° (not yet wired) |

#### Speed encoders

Two additional PIM604 encoders (96 CPR) on pins 14/15 and 16/17 report shaft RPM only — no position tracking. RPM is computed from count differences between successive `update()` calls.

---

### `hardware/motor_driver.h` and `motor_driver.cpp`

**Purpose:** Controls two TB6612FNG dual H-bridge breakout boards for arm and spring motors, with a hardware safety cutoff.

#### Two motor boards

| Board | Motors | STBY | Control pins |
|-------|--------|------|-------------|
| Arm board | Motor A (pins 9/8) + Motor B (pins 11/12) | Pin 10 (software-controlled) | Sign-magnitude PWM on IN pins |
| Spring board | Channels A+B paralleled (pins 7/2) | Permanently HIGH (3.3V) | Same PWM scheme |

#### Speed control

Uses sign-magnitude PWM with PWMA/PWMB assumed tied HIGH:
- **Forward**: `analogWrite(IN1, duty)`, `digitalWrite(IN2, LOW)`
- **Reverse**: `digitalWrite(IN1, LOW)`, `analogWrite(IN2, duty)`
- **Coast**: both LOW

#### Safety cutoff

`safetyStop()` is called from the safety ISR (interrupt context) when tilt exceeds the threshold. It:
1. Asserts `PIN_ARM_STBY` LOW — instantly kills both arm motors at the hardware level
2. Coasts the spring motor (both IN pins LOW)
3. Latches the `_safetyTripped` flag (volatile, ISR-safe)

All subsequent `setArmA()`, `setArmB()`, `setSpring()` calls are silently ignored while the latch is set. `clearSafety()` re-enables the driver (STBY HIGH) and clears the latch, but motor outputs remain at zero until the next explicit command.

---

### `math/quaternion.h` and `quaternion.cpp`

**Purpose:** All quaternion mathematics. No hardware or Arduino dependencies — pure maths.

#### What is a quaternion?

A quaternion is a way to represent a 3D rotation using four numbers: `(x, y, z, w)`.

- `w` is the **scalar** part. For a rotation of angle θ around an axis, `w = cos(θ/2)`.
- `x, y, z` are the **vector** part. They encode the axis of rotation, scaled by `sin(θ/2)`.
- A **unit quaternion** (one where x²+y²+z²+w²=1) always represents a valid pure rotation.

Why use quaternions instead of roll/pitch/yaw?
- They avoid **gimbal lock** (when pitch reaches ±90°, yaw and roll collapse into the same thing)
- They are compact (4 numbers vs. a 3×3 rotation matrix's 9 numbers)
- They compose (multiply) smoothly and numerically stably
- They interpolate naturally via **slerp**

The identity quaternion `(0, 0, 0, 1)` means "no rotation" — the object is in its reference orientation.

#### Key operations

**Normalize:**
```
|q|² = x²+y²+z²+w²
q_normalised = q / sqrt(|q|²)
```
Keeps the quaternion on the unit sphere after floating-point arithmetic may have drifted it slightly off.

**Conjugate:**
```
q* = (-x, -y, -z, w)
```
For a unit quaternion, the conjugate is the inverse (undoes the rotation).

**Hamilton product (multiplication):**
Quaternion multiplication is not commutative: `p×q ≠ q×p`. It composes two rotations: "first rotate by q, then rotate by p" = `p×q`.

**ZYX Euler conversion:**
The firmware converts quaternions to roll/pitch/yaw for human-readable output. The ZYX convention means: first yaw (rotate around Z), then pitch (around Y), then roll (around X). This is the standard robotics convention.

The maths uses `atan2f`, `asinf` — the `f` suffix means single-precision float, using the Teensy's hardware floating-point unit (FPU) for speed.

**Gimbal lock handling:** When `|sinp| >= 1` (pitch is exactly ±90°), `asinf` would produce `NaN`. The code clamps `sinp` to [-1, 1] before calling `asinf`, returning ±90° safely.

---

### `math/filters.h` and `filters.cpp`

**Purpose:** Placeholder for software signal filters (e.g., Kalman, complementary, low-pass).

Currently empty stubs. Because the LSM6DSV16X SFLP engine does its own high-quality sensor fusion in hardware, software filters are not needed for orientation. This module exists for future use if, for example, encoder readings need filtering.

---

### `state/robot_state.h` and `robot_state.cpp`

**Purpose:** The central data structure that holds a complete snapshot of the robot's state at one instant in time.

```cpp
struct RobotState {
    Quaternion imuQuaternion;                     // SFLP orientation (x, y, z, w)
    float roll, pitch, yaw;                       // ZYX Euler angles in degrees
    float jointAngles[NUM_ENCODER_JOINTS];        // [0]=ARM0, [1]=ARM1, [2]=LEG
    float jointAngularVelDps[NUM_ENCODER_JOINTS]; // deg/s per joint
    float jointAngularAccDps2[NUM_ENCODER_JOINTS];// deg/s² per joint
    float encoderSpeedRPM[NUM_SPEED_ENCODERS];    // [0]=pins 14/15, [1]=pins 16/17
    uint32_t timestampUs;                         // when this snapshot was taken
};
```

This is a **plain data struct** — it has no logic, no hardware access, just data. Think of it as a form that gets filled in by `StateEstimator` and read by `SerialReporter` and the safety ISR.

Methods:
- `reset()` — sets all fields back to zero / identity defaults
- `copyFrom(src)` — copies all fields from another `RobotState`
- `print()` — prints a debug summary to Serial (for development use)

---

### `state/state_estimator.h` and `state_estimator.cpp`

**Purpose:** Runs the sampling loop. Decides *when* to read the IMU, calls it, and populates `RobotState`.

#### The IntervalTimer pattern

The Teensy 4.0 has four hardware **IntervalTimers**. A timer fires a callback function at a precise interval regardless of what the main loop is doing.

However: the callback runs in **interrupt context** — it interrupts whatever the CPU was doing. I²C (the bus used to talk to the IMU) cannot safely be used from inside an interrupt, because the I²C library uses internal state that could be partially modified when interrupted.

The solution is a two-stage pattern:

```
Timer fires every 1/120th second
  → ISR: _samplePending = true     ← ONLY this, very fast

Main loop runs continuously
  → tick(): if _samplePending:
       _samplePending = false
       read IMU over I²C             ← safe here
       update RobotState
```

The `volatile` keyword on `_samplePending` is essential. Without it, the compiler might optimise away the check (thinking: "nothing in this function changes `_samplePending`, so I'll just cache it as false"). `volatile` forces the compiler to re-read the variable from memory every time.

#### `tick()` step by step

1. If `_samplePending` is false: return immediately (nothing to do)
2. Clear the flag *before* the I²C read (if the I²C takes a long time, a new timer tick might fire — clearing early means that flag is not lost)
3. Call `_encoders.update()` — snapshots the latest quadrature counts for all active joints
4. Copy the resulting angles, velocities, accelerations, and speed RPMs into `_state`
5. Call `_imu.update()` — drains the FIFO, returns true if a quaternion was found
6. If a quaternion was found:
   - Store it in `_state.imuQuaternion`
   - Convert to Euler: `q.toEuler(_state.roll, _state.pitch, _state.yaw)` (standard ZYX)
   - Record the current timestamp: `_state.timestampUs = micros()`
7. If `update()` returns false (FIFO was empty or there was a bus error): leave the quaternion unchanged — consumers always have the last valid reading

Encoders are read on every tick regardless of whether the IMU FIFO had new data, so joint angles and kinematics stay current at 120 Hz.

---

### `comms/serial_reporter.h` and `serial_reporter.cpp`

**Purpose:** Sends the current `RobotState` to the connected computer over USB serial, at a controlled rate.

#### Rate limiting

The firmware samples the IMU at 120 Hz but only needs to print at 10 Hz. Instead of using a second timer, `SerialReporter` checks the elapsed time every time `update()` is called:

```cpp
const uint32_t now = micros();
if ((now - _lastReportUs) < kReportIntervalUs) {
    return;  // too soon — do nothing
}
_lastReportUs = now;
// ... print ...
```

Unsigned subtraction (`now - _lastReportUs`) handles the 70-minute wraparound of `micros()` correctly: if `now` has wrapped around to a small value, the subtraction still gives the correct elapsed time.

`kReportIntervalUs = 1,000,000 / 10 = 100,000 microseconds = 100 ms` between prints.

#### Output format

Each line looks like:
```
T:12345678 | Q:0.01234,0.00123,-0.00456,0.99990 | RPY:2.34,1.23,-0.56 | J:12.3,88.6,0.0 | K:5.40,12.30 | S:120.5,0.0
```

| Field | Meaning |
|-------|---------|
| `T:<us>` | Teensy timestamp in microseconds |
| `Q:<x>,<y>,<z>,<w>` | IMU quaternion (5 decimal places) |
| `RPY:<r>,<p>,<y>` | Roll, pitch, yaw in degrees (2 decimal places) |
| `J:<j0>,<j1>,<j2>` | Three joint angles in degrees (ARM0, ARM1, LEG) |
| `K:<vel>,<acc>` | ARM1 angular velocity (deg/s) and acceleration (deg/s²) |
| `S:<rpm0>,<rpm1>` | Speed encoder RPMs (pins 14/15 and 16/17) |

The `F("literal")` macro stores string constants in flash memory instead of RAM — a good habit from Arduino days when RAM was scarce.

---

### `main.cpp`

**Purpose:** Entry point. Wires the subsystems together and manages the safety cutoff and serial command parser.

#### Two Arduino functions

Every Arduino/Teensy program has exactly two mandatory functions:
- `setup()` — called once at startup
- `loop()` — called repeatedly forever after

```cpp
// Five global objects, created at program start:
static IMU            imu;
static JointEncoders  encoders;
static StateEstimator stateEstimator(imu, encoders);
static SerialReporter reporter;
static MotorDriver    motors;
```

`static` here means file-scope — these live for the entire program lifetime and are not accessible from other files.

#### Safety ISR

A dedicated IntervalTimer runs `safetyISR()` at 1 kHz — independently of the main loop and I2C traffic. It reads `roll` and `yaw` from `RobotState` and, if either exceeds `SAFETY_TILT_DEG` (45°), calls `motors.safetyStop()` which asserts STBY LOW (arm motors off) and coasts the spring motor. The latched flag blocks all motor commands until `SAFE_CLR` is received over serial.

#### Serial command parser

The firmware accepts newline-terminated commands from the visualiser:

| Command | Action |
|---------|--------|
| `ARM:<a>,<b>` | Set arm motor A and B speeds (-255 to 255) |
| `SPR:<speed>` | Set spring motor speed (-255 to 255) |
| `SAFE_CLR` | Re-enable motors (only if tilt is back within limits) |

#### `setup()` sequence

1. Configure LED pin as an output, turn it off
2. Open serial at 115200 baud; wait up to 3 seconds for a USB connection
3. `motors.begin()` — configures all motor pins, STBY HIGH, outputs zeroed
4. `encoders.begin()` — resets counts and applies calibration offsets; prints a warning if it fails but does **not** halt
5. `imu.begin()` — if this fails, call `faultHalt()`: blink the LED rapidly and print the error forever
6. `stateEstimator.begin()` — starts the 120 Hz IntervalTimer; if no timer is available, blink at medium speed and halt
7. Start the safety IntervalTimer at 1 kHz
8. Turn LED on solid (= all OK)

#### `faultHalt()`

If a critical initialisation step fails, there is no useful work the firmware can do. `faultHalt()` enters an infinite loop that blinks the LED at the given period. Different blink speeds indicate different faults:
- 200 ms (fast): IMU failure — check wiring and I²C address
- 500 ms (medium): IntervalTimer resource exhausted

#### `loop()`

```cpp
void loop() {
    readSerialCommands();                       // parse motor/safety commands
    stateEstimator.tick();                      // process one IMU sample if ready
    if (s_safetyJustTripped) { ... }            // relay "SAFE:1" to serial
    reporter.update(stateEstimator.getState()); // print if interval elapsed
}
```

The loop runs thousands of times per second. Most `tick()` calls return immediately (no sample pending). Most `reporter.update()` calls return immediately (not yet time to print). This is correct and efficient — the CPU is idle most of the time.

---

## 7. Visualisation tools

These tools run on your computer (not on the Teensy) and read the serial output.

---

### `tools/visualise_imu.py`

**Language:** Python 3
**Libraries:** `pyserial`, `numpy`, `matplotlib`

A quick-start visualiser. Opens the serial port, reads quaternion data, and displays three animated arrows representing the robot's X, Y, Z axes.

#### Running it

```bash
# Activate your Python virtual environment first, then:
pip install pyserial numpy matplotlib
python tools/visualise_imu.py /dev/cu.usbmodem187379501
```

#### How it works

- A **background thread** (`_serial_thread`) reads from the serial port continuously. It parses `Q:x,y,z,w` and `RPY:r,p,y` from each line and writes them into shared variables (`_quat`, `_rpy`) protected by a **threading lock**.
- A **threading lock** (`_lock`) prevents the display thread from reading a quaternion while the serial thread is in the middle of writing it (which would give corrupted half-old, half-new values).
- The **main thread** runs `matplotlib`'s animation loop, which calls `_update()` 20 times per second.
- `_update()` converts the quaternion to a **3×3 rotation matrix** using the Hamilton formula, then redraws three arrows (quivers) along the three columns of that matrix.

#### Quaternion → rotation matrix

```
R = | 1-2(y²+z²)   2(xy-zw)   2(xz+yw) |
    | 2(xy+zw)   1-2(x²+z²)   2(yz-xw) |
    | 2(xz-yw)     2(yz+xw) 1-2(x²+y²) |
```

The columns of R are where the X, Y, Z axes point after the rotation.

---

### `tools/visualise_imu.html`

**Language:** HTML + JavaScript (ES modules)
**Libraries:** Three.js (loaded from CDN), WebSerial API (built into Chrome/Edge)

The main visualiser. Loads the actual 3D robot model and shows it rotating in real time. Also controls arm and spring motors via serial commands and displays RPY calibration offsets, arm encoder angles, and live telemetry graphs.

#### Running it

```bash
# From the project root:
python3 -m http.server 8080
# Then open Chrome or Edge and go to http://localhost:8080/tools/visualise_imu.html
```

A local HTTP server is required because browsers block loading local files for security reasons (CORS policy). The visualiser needs to load `models/model.obj` and `models/model.mtl`.

**WebSerial requires Chrome or Edge.** Safari does not support it.

#### Three.js scene structure

```
Scene
├── GridHelper         ← grey floor grid
├── AmbientLight       ← fills shadows
├── DirectionalLight   ← main sun-like light
├── DirectionalLight   ← fill light (blue tint)
└── pivot (Object3D)   ← quaternion rotation applied here
    ├── AxesHelper     ← Red=X, Green=Y, Blue=Z arrows
    └── wrapper (Group)
        ├── rotation.x = -π/2   ← converts Z-up OBJ to Y-up Three.js
        ├── scale = 20/maxDim   ← normalises model size
        └── obj (the robot model, positioned so foot sits at wrapper origin)
```

The key insight: the pivot's origin is at the foot bottom. When the quaternion rotates the pivot, the whole robot rotates *about its foot*, which is what you see physically.

ARM0 and ARM1 meshes have their own rotation pivots built using `buildArmPivot()`, which creates a parent group centred at the mesh's bounding-box centroid. Arm angles from the `J:` serial field drive `arm0Mesh.rotation.y` and `arm1Mesh.rotation.x` respectively, with calibration offsets applied when "Calibrate" is pressed.

#### How the serial data is applied

```
Raw quaternion (x,y,z,w)
    ↓ multiply by IMU_OFFSET_INV        (cancel 45° mount angle)
    ↓ premultiply by refQuatInv         (cancel starting orientation — calibration)
    ↓ premultiply _axisFlip, multiply _axisFlip  (correct X/Y axis inversion)
    ↓ slerp toward this over multiple frames     (smooth out jitter)
    ↓ apply to pivot.quaternion
```

**IMU_OFFSET_INV** — The IMU board is physically mounted at 45° relative to the robot's forward axis. This correction quaternion (a -45° rotation around Y) cancels that offset.

**refQuatInv** — When you click "Calibrate", the firmware captures the current corrected orientation and inverts it. All future orientations are expressed relative to that reference, so the robot appears in its neutral position.

**Axis flip (conjugation by R_z(π))** — The IMU's X and Y axes are physically wired opposite to the model's frame. A conjugation `R·q·R⁻¹` by a 180° rotation around Z maps X→-X and Y→-Y while leaving Z unchanged. This is the correct way to flip axes for *all* orientations including continuous rotation (a simple sign flip on components would only work for small angles around the identity).

**Slerp** — The smoothing slider controls alpha (0.05–1.0). At alpha=1.0, the model snaps instantly to each new reading. At alpha=0.05, it moves very slowly toward the new orientation. Default 0.25 gives a small amount of damping.

**Pivot Z slider** — Controls the `PIVOT_Z` global variable (OBJ-space Z coordinate). This determines *which point on the 3D model* sits at the scene origin and therefore which point the robot appears to rotate about. Range: −10 (below foot) to +26 (above top). Default −8.5 = foot bottom. Dragging this slider up moves the pivot toward the hip or body, which changes the apparent rotation centre without affecting the quaternion data at all. The slider only changes `obj.position.z = -PIVOT_Z`; the X and Y offset remain fixed at the original foot-bottom values.

#### Model loading details

The OBJ model was exported from Autodesk with Z as the up-axis. Three.js uses Y as up. The `wrapper` object applies a -90° rotation around X to convert between these conventions, so the robot's leg (OBJ Z axis) appears vertical (Three.js Y axis).

The foot bottom in OBJ local coordinates is `(-0.079, 0.048, -8.507)`. The OBJ model is offset so this point sits at the wrapper's origin, which in turn sits at the scene's origin (0,0,0). So the robot pivots about its foot.

---

## 8. Data flow — end to end

```
[Physical world]
      │
      ├──────────────────────────────┬──────────────────────────────────┐
      ▼                              ▼                                  ▼
LSM6DSV16X chip              ARM0/ARM1 encoders              Speed encoders
  Accel + Gyro → SFLP          (pins 22/23, 20/21)            (pins 14/15, 16/17)
  → quaternion → FIFO          → A/B pulse waves               → A/B pulse waves
      │                        → interrupt-counted              → interrupt-counted
      ▼                              ▼                                  ▼
imu.update()                 encoders.update()
  → drains FIFO over I²C      → reads counts, converts to degrees, vel, acc, RPM
  → stores freshest quat       → stores in _angles[], _angularVelDps[], _speedRPM[]
      │                              │
      └──────────────┬───────────────┘
                     ▼
          StateEstimator::tick()  [120 Hz]
            → encoder data    → _state.jointAngles/vel/acc/speedRPM
            → IMU quaternion  → _state.imuQuaternion
            → q.toEuler()    → _state.roll/pitch/yaw (ZYX Euler)
            → micros()       → _state.timestampUs
                     │
          ┌──────────┼──────────────────────┐
          ▼          ▼                      ▼
   safetyISR()    SerialReporter       MotorDriver
   [1 kHz ISR]    [10 Hz print]       [command-driven]
   reads RPY      T:|Q:|RPY:|J:|K:|S:  ARM:/SPR:/SAFE_CLR
   |roll|>45°?         │                    ▲
   → STBY LOW          ▼  (USB 115200)     │
   → SAFE:1       [Computer]           [Computer]
                       │                    │
                       ▼                    │
           visualise_imu.html ──────────────┘
             → parses Q:, RPY:, J: from each line
             → sends ARM:/SPR:/SAFE_CLR commands back
             → applies axis corrections + calibration offset
             → slerps toward corrected quaternion
             → renders the 3D robot with arm rotation
```

---

## 9. Serial output format

The firmware prints one line every 100 ms (10 Hz). Example:

```
T:1234567 | Q:0.00123,0.00456,-0.00012,0.99998 | RPY:0.14,0.52,-0.03 | J:12.3,88.6,0.0 | K:5.40,12.30 | S:120.5,0.0
```

| Part | Example value | Notes |
|------|---------------|-------|
| `T:` | `1234567` | Teensy timestamp in microseconds |
| `Q:` | `0.00123,0.00456,-0.00012,0.99998` | Quaternion x,y,z,w. Near identity when upright. 5 decimal places. |
| `RPY:` | `0.14,0.52,-0.03` | Roll, pitch, yaw in degrees (ZYX Euler). 2 decimal places. |
| `J:` | `12.3,88.6,0.0` | Three joint angles: ARM0, ARM1, LEG. 1 decimal place. |
| `K:` | `5.40,12.30` | ARM1 angular velocity (deg/s) and acceleration (deg/s²). 2 decimal places. |
| `S:` | `120.5,0.0` | Speed encoder RPMs (pins 14/15 and 16/17). 1 decimal place. |

Safety events are printed on separate lines: `SAFE:1` (motors cut) or `SAFE:0` (motors re-enabled via `SAFE_CLR`).

A header is printed on startup:
```
=== MRV Firmware — Serial Reporter ===
Sample rate : 120 Hz
Report rate : 10 Hz
Format: T:<us> | Q:<x>,<y>,<z>,<w> | RPY:<r>,<p>,<y> deg | J:<j0>,<j1>,<j2> deg | S:<rpm0>,<rpm1> | K:<vel_dps>,<acc_dps2>
----------------------------------------------------------------------
```

---

## 10. What is stubbed / not yet built

| Module | Status | What is needed to activate it |
|--------|--------|-------------------------------|
| Encoder — ARM0 (index 0) | **Active** — pins 22/23 | PIM604, 50:1 gear ratio. Calibration constants may need empirical tuning. |
| Encoder — ARM1 (index 1) | **Active** — pins 20/21 | PIM604, 50:1 gear ratio. Empirically calibrated (`ENC_ARM1_COUNTS_PER_REV = 12.0f`). |
| Encoder — LEG (index 2) | Stub — 0° | Add pin assignments to `pinout.h`, add constants to `config.h`, add `Encoder` library object and conversion logic in `encoder.cpp`. |
| Speed encoders (2 and 3) | **Active** — pins 14/15, 16/17 | PIM604 (96 CPR). Report shaft RPM. |
| Motor driver (arm + spring) | **Active** | TB6612FNG on pins 8/9/10/11/12 (arm) and 2/7 (spring). Safety cutoff via STBY. |
| Safety ISR | **Active** — 1 kHz | Checks roll/yaw against 45° threshold. Cuts motors via STBY LOW. |
| Filter module (`filters.cpp`) | Empty stub | Not needed yet (SFLP provides filtered output). Reserved for future encoder angle filtering. |
| `getRawAccel()` in IMU | **Implemented** but unused | Reads accelerometer in g. Available for future accel-based tilt computation if needed. |
| INT1 interrupt (`PIN_IMU_INT1`) | Pin shared with spring motor | Interrupt approach abandoned — pin 2 is used for the spring motor. IMU is polled. |

---

## 11. Libraries reference

### STM32duino LSM6DSV16X

**PlatformIO name:** `stm32duino/STM32duino LSM6DSV16X`
**What it does:** Provides a C++ class (`LSM6DSV16XSensor`) that wraps all the low-level I²C/SPI register reads and writes for the ST LSM6DSV16X chip.

Key methods used:
| Method | What it does |
|--------|-------------|
| `begin()` | Reads WHO_AM_I register to verify chip identity |
| `Enable_X()` | Turns on the accelerometer |
| `Enable_G()` | Turns on the gyroscope |
| `Set_X_ODR(hz)` | Sets accelerometer output data rate |
| `Set_X_FS(g)` | Sets accelerometer full-scale range |
| `Set_G_ODR(hz)` | Sets gyroscope output data rate |
| `Set_G_FS(dps)` | Sets gyroscope full-scale range |
| `Set_SFLP_ODR(hz)` | Sets SFLP fusion engine output rate |
| `Enable_Rotation_Vector()` | Starts SFLP and enables game-rotation-vector output to FIFO |
| `FIFO_Set_Watermark_Level(n)` | FIFO threshold level |
| `FIFO_Set_Mode(mode)` | Set FIFO operating mode (stream = continuous) |
| `FIFO_Get_Num_Samples(&n)` | How many samples are in the FIFO |
| `FIFO_Get_Tag(&tag)` | Read the type tag of the next FIFO entry |
| `FIFO_Get_Rotation_Vector(float[4])` | Read quaternion from FIFO: [x, y, z, w] |
| `FIFO_Get_Data(buf)` | Read and discard a FIFO entry of unknown type |
| `Reset_SFLP()` | Reset the fusion engine (use for recalibration) |
| `Get_X_Axes(int32_t[3])` | Read current accelerometer output in mg (milligravity) |
| `Enable_Rotation_Vector()` | Start SFLP game-rotation-vector output to FIFO |

Return code: `LSM6DSV16X_OK = 0` on success, nonzero on error.

---

### paulstoffregen/Encoder

**PlatformIO name:** `paulstoffregen/Encoder`
**What it does:** Reads quadrature encoders using hardware interrupts, giving accurate counts without the CPU having to continuously poll the pins. Heavily optimised for Teensy.

Key methods used:

| Method | What it does |
|--------|-------------|
| `Encoder enc(pinA, pinB)` | Constructor — attaches interrupt handlers to both pins immediately |
| `enc.read()` | Returns the current signed count as `int32_t`. Counts up or down depending on direction. |
| `enc.write(n)` | Sets the current count to `n`. Used in `begin()` to zero the angle at startup. |

**Important:** The library's class is also called `Encoder`. To avoid a name collision with the project's own encoder wrapper, the wrapper class is named `JointEncoders`. Inside `encoder.cpp`, `Encoder` always refers to the library class.

The firmware creates four `Encoder` objects: `s_arm0` (pins 22/23), `s_arm1` (pins 20/21), `s_speed2` (pins 14/15), and `s_speed3` (pins 16/17).

---

### Arduino / Teensyduino core

Built in to every Arduino/Teensy project. Used functions:

| Function / class | Meaning |
|-----------------|---------|
| `setup()`, `loop()` | Entry points for every Arduino sketch |
| `pinMode(pin, mode)` | Configure a pin as INPUT or OUTPUT |
| `digitalWrite(pin, value)` | Set a digital pin HIGH (3.3V) or LOW (0V) |
| `delay(ms)` | Block the CPU for ms milliseconds |
| `millis()` | Milliseconds since power-on (32-bit, wraps after ~49 days) |
| `micros()` | Microseconds since power-on (32-bit, wraps after ~70 minutes) |
| `Serial.begin(baud)` | Open the USB serial port |
| `Serial.print(x)` | Print a value (no newline) |
| `Serial.println(x)` | Print a value and newline |
| `Serial.flush()` | Wait until all data has been sent |
| `Wire` | The I²C bus object (from `<Wire.h>`) |
| `Wire.begin()` | Start the I²C bus |
| `Wire.setClock(hz)` | Set I²C bus frequency |
| `IntervalTimer` | Teensy hardware timer (4 available, from `<IntervalTimer.h>`) |
| `IntervalTimer::begin(fn, us)` | Call `fn` every `us` microseconds |
| `F("string")` | Store a string literal in flash memory |
| `String` | Arduino string class (heap-allocated) |

---

### Three.js (browser visualiser)

Loaded via importmap CDN — no installation needed. Version 0.164.1.

| Import | What it provides |
|--------|-----------------|
| `three` | Core 3D engine: Scene, Camera, Renderer, Quaternion, etc. |
| `three/addons/loaders/OBJLoader.js` | Loads Wavefront `.obj` files |
| `three/addons/loaders/MTLLoader.js` | Loads `.mtl` material files for the OBJ |
| `three/addons/controls/OrbitControls.js` | Mouse-driven camera controls |

---

### Python visualiser dependencies

| Package | What it does |
|---------|-------------|
| `pyserial` | Reads from USB serial ports |
| `numpy` | Fast numerical arrays; used for quaternion→matrix conversion |
| `matplotlib` | Plots and animation |
| `mpl_toolkits.mplot3d` | 3D axes for matplotlib |
| `threading` | Runs the serial reader in a background thread |

Install: `pip install pyserial numpy matplotlib`

---

*This README is kept up to date as the project evolves. If you notice anything that is out of date or unclear, edit it alongside the code change that prompted the confusion.*
