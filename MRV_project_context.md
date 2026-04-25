# MRV Project — Context for New Chat

## What this is
Teensy 4.0 embedded firmware for a robotic system, plus a browser-based 3D visualiser.
Build system: PlatformIO. Project root: `/Users/vato/Documents/MRV/MRV_code`.

---

## The OBJ file — `models/model.obj`

**Format:** ASCII WaveFront OBJ, exported from Autodesk Fusion (ATF exporter). 408,289 lines.
Companion material file: `models/model.mtl`.

**Coordinate system:** Z-up (Autodesk convention). The Three.js visualiser applies
`wrapper.rotation.x = -Math.PI / 2` to convert Z-up → Y-up (Three.js convention).
After this rotation, the robot's vertical axis is Three.js Y (green).

**Overall Z extent in OBJ space:** −8.54 (foot bottom) to ~25.7 (top of ARM(1)). Total height ~34 units.

**Pivot point used in visualiser:** foot bottom at OBJ coordinates `(-0.079, 0.048, -8.507)`.
This is negated and applied as `obj.position` so the foot sits at the wrapper origin — the robot
rotates about its ground contact point. A slider controls the Z component live.

**Materials used:**
- `Steel_-_Satin` — structural metal parts
- `Brass_-_Matte` — gears
- `ABS_(White)` — plastic body panels
- `Nylon_6-6_(White)` — bearings/joints
- `EPX_86FR_(with_Carbon_M2_M3_L1_3D_Printers)` — carbon composite parts
- `Glass_(Blue)` — transparent cover
- `Paint_-_Metallic_(Black)`, `Paint_-_Metallic_(Dark_Grey)` — painted parts
- `Opaque(203,210,239)` — PCB

---

## All 32 mesh groups

| OBJ group name       | Role                                          | Centroid (x, y, z)     | Z range        |
|----------------------|-----------------------------------------------|------------------------|----------------|
| `Body1`              | Main body panel (right side)                  | (2.24, −1.59, 11.76)   | 11.26–12.26    |
| `Body1:1`            | Body sub-part                                 | (−4.58, −1.63, 18.18)  | 17.72–18.64    |
| `Body1:2`            | Body sub-part (main lower)                    | (0, 0, 9.16)           | 7.76–10.56     |
| `Body1:3`            | Body sub-part                                 | (0, 5.08, 17.06)       | 16.56–17.56    |
| `Body1:4`            | Body sub-part                                 | (5.08, 0, 17.06)       | 16.56–17.56    |
| `Body1:5`            | Main body panel (left side)                   | (−2.24, −1.59, 11.76)  | 11.26–12.26    |
| `Body1:6`            | Body sub-part                                 | (0, −3.56, 14.21)      | 12.59–15.84    |
| `Body2`              | Main chassis — largest part (15k verts)        | (0, 0, −0.21)          | −8.54–8.11     |
| `ThinBearing`        | Thin bearing race                             | (−1.19, −1.30, 14.79)  | 14.44–15.14    |
| `ARM`                | **Left arm** — encoder not yet wired          | (8.49, 0, 17.06)       | 16.31–17.81    |
| `ARM (1)`            | **Right arm** — active encoder on pins 20/21  | (0, 8.49, 17.06)       | 8.38–25.74     |
| `WEIGHT`             | Counterweight on ARM (left)                   | (10.09, −7.95, 17.06)  | 15.63–18.49    |
| `WEIGHT2`            | Counterweight on ARM (left)                   | (10.09, 7.95, 17.06)   | 15.63–18.49    |
| `WEIGHT2 (1)`        | Counterweight on ARM (1) (right)              | (0, 10.09, 9.11)       | 8.59–9.64      |
| `WEIGHT (2)`         | Counterweight on ARM (1) (right)              | (0, 10.09, 25.01)      | 24.49–25.54    |
| `ConRod L`           | Left connecting rod                           | (−1.56, −0.76, 13.26)  | 11.48–15.04    |
| `ConRod R`           | Right connecting rod                          | (1.56, −0.76, 13.26)   | 11.48–15.04    |
| `ConRodwSpring L`    | Left connecting rod with spring               | (−1.68, 0.81, 13.13)   | 11.23–15.04    |
| `ConRodwSpring R`    | Right connecting rod with spring              | (1.68, 0.81, 13.13)    | 11.23–15.04    |
| `DriverGear (1)`     | Drive gear (right)                            | (2.24, 0, 11.76)       | 10.17–13.36    |
| `DriverGear (1) (1)` | Drive gear (left)                             | (−2.24, 0, 11.76)      | 10.17–13.36    |
| `DrivenGear`         | Driven gear                                   | (1.99, −0.01, 14.72)   | 13.12–16.32    |
| `DrivenGear (1)`     | Driven gear (second)                          | (−1.99, −0.01, 14.72)  | 13.12–16.32    |
| `CROWN`              | Crown gear (large, 3361 verts)                | (0, 0.53, 14.02)       | 10.33–17.70    |
| `DriveMotorMount`    | Motor mount bracket                           | (0, −2.17, 11.48)      | 10.56–12.40    |
| `Struct`             | Main structural cross-member                  | (0, 0.78, 9.77)        | 7.53–12.01     |
| `LEG`                | Vertical slide / leg (14k verts, full Z span) | (0, 0, 5.78)           | −7.61–19.17    |
| `Foot`               | Foot platform                                 | (0, 0, −8.01)          | −8.51–7.51     |
| `BALL (1)`           | Ball foot contact                             | (0, 0, −8.34)          | −8.89–7.80     |
| `DriveMotorMount`    | Motor mount bracket                           | (0, −2.17, 11.48)      | 10.56–12.40    |
| `PCB`                | Circuit board                                 | (−3.34, −2.52, 15.58)  | 12.55–18.60    |
| `PCB_Mount`          | PCB mount bracket                             | (−4.08, −1.14, 18.14)  | 17.70–18.57    |
| `PCB_Mount2`         | PCB mount bracket (second)                    | (−1.91, −3.31, 18.14)  | 17.70–18.57    |

**Critical naming gotcha:** Autodesk exports group names with spaces before parentheses.
In Three.js `traverse()` you must match `child.name === 'ARM (1)'` (with a space), not `'ARM(1)'`.
Same applies to `DriverGear (1)`, `WEIGHT2 (1)`, `WEIGHT (2)`, `BALL (1)`, etc.

---

## Arm rotation — how it works in the visualiser

**ARM (left arm)** rotates about its own bounding-box centroid. Implementation:
```javascript
const armBox    = new THREE.Box3().setFromObject(arm1Mesh);
const armCenter = armBox.getCenter(new THREE.Vector3());
const armPivot  = new THREE.Group();
armPivot.position.copy(armCenter);
obj.add(armPivot);
[arm1Mesh, ...weightMeshes].forEach(mesh => {
  mesh.position.sub(armCenter);  // keep visual position unchanged
  armPivot.add(mesh);
});
arm1Mesh = armPivot;  // animate the pivot
```

- **WEIGHT** and **WEIGHT2** follow ARM's rotation (arm counterweights).
- **WEIGHT2 (1)** and **WEIGHT (2)** belong to ARM (1) — not yet animated.
- Rotation applied in render loop:
  `arm1Mesh.rotation.x = -(arm1AngleDeg + arm1CalibOffset) * Math.PI / 180`
- Negated because the encoder direction and model direction are opposite.
- `arm1CalibOffset` is set on Calibrate: `arm1CalibOffset = ARM1_INIT_DEG - arm1AngleDeg`

---

## Firmware — file structure

```
platformio.ini
src/
  main.cpp                    # setup()/loop() only — no hardware access here
  config/config.h             # ALL constants (constexpr only, no #define magic numbers)
  hardware/
    pinout.h                  # ALL pin numbers — nowhere else
    imu.h / imu.cpp           # LSM6DSV16X SFLP driver
    encoder.h / encoder.cpp   # Joint + speed encoder driver
  math/
    quaternion.h / .cpp       # Quaternion struct + ZYX Euler decomposition
    filters.h / .cpp          # stub (unused)
  state/
    robot_state.h / .cpp      # RobotState struct — data contract between all modules
    state_estimator.h / .cpp  # IntervalTimer sets flag; tick() does I2C in main loop
  comms/
    serial_reporter.h / .cpp  # Rate-limited Serial output
tools/
  visualise_imu.html          # Single-file Three.js visualiser (no build step)
models/
  model.obj / model.mtl       # Robot 3D model
```

---

## Key design decisions

- **IMU:** LSM6DSV16X hardware SFLP fusion — no software Madgwick/Mahony.
  PlatformIO lib: `stm32duino/LSM6DSV16X`.
  I2C address `0x6B` (SA0=1, SparkFun Qwiic breakout default).
  FIFO drains to the most-recent sample. `IntervalTimer` sets a `volatile bool` flag only;
  actual I2C read happens in main loop (never in ISR).
- **All constants:** `constexpr` in `config.h`. All pins in `pinout.h`. No magic numbers elsewhere.
- **Bus selection:** `// #define USE_IMU_SPI` in config.h (commented = I2C).
- **No heap allocation:** all drivers use module-static objects (`static Encoder s_arm1(...)`).
- **State flow:** `StateEstimator::tick()` reads IMU + encoders → populates `RobotState` →
  `SerialReporter::update()` prints it at the configured rate.

---

## Encoder hardware and config

### ARM (1) — right arm, ACTIVE

- Hardware: Pimoroni PIM604 Rotary Encoder Breakout
- Pins: 20 (A), 21 (B)
- Library: paulstoffregen/Encoder (hardware quadrature interrupts on Teensy 4.0)
- **DO NOT change these without re-tuning:**
  - `ENC_ARM1_COUNTS_PER_REV = 12.0f` — nominal 96 CPR but empirically only 12 effective
    counts/rev (possibly single-channel wiring or non-standard PPR)
  - `ENC_ARM1_GEAR_RATIO = 50.0f`
  - Scalar `359.32f` in the angle formula — manually tuned
  - Formula: `degrees = count × 359.32 / (ENC_ARM1_COUNTS_PER_REV × ENC_ARM1_GEAR_RATIO)`
- Initial calibration position: **90°** (`ENC_ARM1_INIT_DEG = 90.0f`)

### ARM — left arm, NOT YET WIRED

- Guarded by `// #define USE_ARM0_ENCODER` in config.h (commented out)
- Pins: `PIN_ENC_ARM0_A/B` — left as `???` in pinout.h, fill in when wired
- Placeholder constants (tune empirically when wired):
  - `ENC_ARM0_COUNTS_PER_REV = 12.0f`
  - `ENC_ARM0_GEAR_RATIO = 50.0f`
  - `ENC_ARM0_SCALAR = 360.0f`
- Initial position: **0°** (`ENC_ARM0_INIT_DEG = 0.0f`)

### Speed encoders — shaft RPM only, no position tracking

- `s_spd2`: pins 14 (A) / 15 (B)
- `s_spd3`: pins 16 (A) / 17 (B)
- Both PIM604, `ENC_SPEED_COUNTS_PER_REV = 96.0f`
- Formula: `RPM = (delta_count / 96) × (60 / dt_seconds)`

---

## Velocity/acceleration computation (encoder kinematics)

Uses a **sliding-window velocity estimator** + **EMA filter** in `JointEncoders::update()`:

1. A ring buffer (`_angleHistory[joint][ENC_VEL_WINDOW]`, `_timeHistory[ENC_VEL_WINDOW]`)
   stores the last `ENC_VEL_WINDOW = 10` **raw (unwrapped)** angles and timestamps.
2. `rawVel = (angle_now − angle_oldest) / window_dt` — averages quantisation noise over ~83 ms at 120 Hz.
3. `smoothVel = ENC_VEL_ALPHA × rawVel + (1 − α) × smoothVel_prev` — EMA, α = 0.10.
4. `acc = (smoothVel_now − smoothVel_prev) / one_tick_dt`

**Important:** The ring buffer stores the **unwrapped** (continuously accumulating) angle so that
velocity across a 0°/360° boundary computes correctly. The **reported** `_angles[1]` should be
wrapped to `[0, 360)` for display only. Kinematics must always use the unwrapped value.

Minimum detectable speed at 120 Hz with a 10-tick window: ~7 deg/s.

---

## Serial output format

Rate: `SERIAL_REPORT_RATE_HZ = 10` Hz (≤ `IMU_SAMPLE_RATE_HZ = 120`). Baud: 115200.

```
T:<us> | Q:<x>,<y>,<z>,<w> | RPY:<r>,<p>,<y> deg | J:<j0>,<j1>,<j2> deg | S:<rpm0>,<rpm1> | K:<vel_dps>,<acc_dps2>
```

| Field | Content |
|-------|---------|
| `T`   | `micros()` timestamp — wraps at ~70 min, use difference arithmetic only |
| `Q`   | Raw unit quaternion from SFLP engine, 5 decimal places |
| `RPY` | ZYX Euler angles in degrees derived from Q |
| `J`   | Joint angles: [0]=ARM, [1]=ARM(1), [2]=LEG |
| `S`   | Shaft RPM: [0]=spd2 (pins 14/15), [1]=spd3 (pins 16/17) |
| `K`   | ARM(1) kinematics: smoothed angular velocity (deg/s), acceleration (deg/s²) |

---

## Visualiser — `tools/visualise_imu.html`

Single-file Three.js app, no build step. Open in Chrome or Edge (WebSerial API required).

- **Libs:** Three.js r0.164.1 via CDN importmap; OBJLoader, MTLLoader, OrbitControls
- **Serial:** WebSerial API, 115200 baud, line-buffered `parseLine()`
- **IMU rendering:** target quaternion slerp'd toward display quaternion each frame.
  Corrections applied: 45° yaw offset (IMU board mounting angle) + X/Y axis flip
  (`R_z(π)·q·R_z(π)` conjugation).
- **Calibration button:** zeroes IMU orientation AND resets arm visual to 90°.
- **Graphs:** two `<canvas>` elements fixed at bottom, declared before the overlay `<div>`
  in DOM order (controls z-stacking without explicit z-index):
  - Left 50%: ARM(1) angular velocity (red) — autoscaled y-axis
  - Right 50%: IMU Roll (red) / Pitch (green) / Yaw (blue) — fixed ±180°
  - 300-sample ring buffers ≈ 30 s of history at 10 Hz report rate
- **Sliders:**
  - Smoothing — quaternion slerp alpha
  - Pivot Z — which OBJ Z coordinate sits at the ground pivot (default −8.5 = foot bottom)

---

## What's confirmed working on hardware

- IMU quaternion → RPY at 120 Hz, displayed correctly in 3D
- ARM(1) encoder angle (manually tuned 359.32f scalar confirmed correct)
- Arm rotation direction correct in visualiser (encoder direction is negated)
- Calibrate button resets both IMU zero point and arm visual to 90°
- Speed encoders reading RPM on pins 14/15 and 16/17

## What's not yet wired / implemented

- ARM (left arm) encoder — `USE_ARM0_ENCODER` compile guard exists, pins TBD
- LEG encoder (joint index 2) — always 0° in firmware
- `WEIGHT2 (1)` and `WEIGHT (2)` mesh animation — belong to ARM(1), no encoder yet
- ConRod encoders (indices 3–5) — not in firmware
- IMU INT1 interrupt (Teensy pin 2) — unused, IMU is polled not interrupt-driven
