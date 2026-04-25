# Monopedal Reconnaissance Vehicle (MRV)

![MRV](/MRV.jpg)

A spring-loaded monopedal hopping robot stabilised by dual-axis reaction wheels, designed as a proof of concept for low-gravity exploration.

Developed as a BEng Mechanical Engineering thesis at the University of Warwick (ES327), supervised by Dr. Chris Purssell. Inspired by UC Berkeley's [SALTO](https://doi.org/10.1126/scirobotics.aag2048).

---

## Specifications

| Parameter | Value |
|---|---|
| Mass | 360 g |
| Jump height | 41.1 cm ± 1.2 cm |
| Release efficiency | 67.8% |
| Max recovery angle | 12.7° (bang-bang, full extension) |
| Energy recovery | 26–41% (height dependent) |
| Spring | 324 N/m, 11.5 cm stroke |
| MCU | Teensy 4.0 (600 MHz ARM Cortex-M7) |
| IMU | LSM6DSV16X (SFLP hardware fusion, 120 Hz) |
| Motors (jump) | 2× 298:1 geared DC |
| Motors (stabilisation) | 2× 50:1 geared DC |
| Battery | 2× 3.8V LiHV (7.6V) |
| Structure | SLS Nylon 12 |
| Counterweights | Lead-free pewter, cast in PPS-CF moulds |

---

## How It Works

The robot stores elastic potential energy in a compression spring via a rack-and-pinion driven by two geared DC motors. A servo-actuated cam releases the driven gear from the rack, allowing the spring to extend and launch the robot. Sawtooth teeth on the rack enable passive ratcheting on landing, recovering kinetic energy without motor power.

Two orthogonal reaction wheel arms provide pitch and roll correction during flight. Each arm carries a 42 g pewter counterweight at 79 mm radius, driven by a 50:1 geared DC motor. A 1 kHz safety ISR disengages the arms if attitude exceeds 45°.

---

## Test Results

| Test | Result |
|---|---|
| Jump height (mean of 3 valid trials) | 41.1 cm ± 1.2 cm |
| Theoretical max jump height | 60.6 cm |
| Release efficiency | 67.8% |
| Jump reliability | 4/7 (57%) |
| Launch tilt | 32° in 0.28 s |
| Recovery angle (bang-bang) | 6.8°, 10.7°, 12.7° ✓ / 14° ✗ |
| Energy recovery (50 cm drop) | 26.1% |
| Energy recovery (100 cm drop) | 40.9% |
| Unsymmetric wedge improvement | +38% stored energy |

---

## CAD

Full Fusion 360 project with all components: [Autodesk Fusion link](YOUR_FUSION_LINK_HERE)

---

## Hardware

| Component | Part | Role |
|-----------|------|------|
| Microcontroller | Teensy 4.0 | ARM Cortex-M7 at 600 MHz |
| IMU | LSM6DSV16X (SparkFun Micro 6DoF) | 6-axis, SFLP hardware fusion, I²C |
| Arm encoders | PIM604 quadrature (×2) | Arm angle, velocity, acceleration |
| Speed encoders | PIM604 quadrature (×2) | Shaft RPM for drive motors |
| Arm motor driver | TB6612FNG dual H-bridge | Drives two arm motors. STBY = safety cutoff |
| Spring motor driver | TB6612FNG (channels paralleled) | Drives spring winding motor |
| PCB | Custom 4-layer (JLCPCB) | 30 mil traces per IPC-2221A |

---

## Repository Structure

```
MRV/
├── README.md
├── LICENSE
├── platformio.ini
├── src/
│   ├── main.cpp
│   ├── config/
│   │   └── config.h
│   ├── hardware/
│   │   ├── pinout.h
│   │   ├── imu.h / imu.cpp
│   │   ├── encoder.h / encoder.cpp
│   │   └── motor_driver.h / motor_driver.cpp
│   ├── math/
│   │   ├── quaternion.h / quaternion.cpp
│   │   └── filters.h / filters.cpp
│   ├── state/
│   │   ├── robot_state.h / robot_state.cpp
│   │   └── state_estimator.h / state_estimator.cpp
│   └── comms/
│       └── serial_reporter.h / serial_reporter.cpp
├── tools/
│   ├── visualise_imu.py
│   └── visualise_imu.html
├── models/
│   └── model.obj
├── PCB/
│   ├── schematic/
│   └── gerber/
└── docs/
    ├── BOM.csv
    ├── CODEBASE_GUIDE.md
    └── photos/
```

One responsibility per file. The IMU file knows nothing about serial output. The serial file knows nothing about quaternion maths.

---

## Getting Started

### Flash the firmware
1. Install [PlatformIO](https://platformio.org/) in VS Code.
2. Clone this repo and open the project folder.
3. Connect Teensy 4.0 via USB.
4. Build and upload:
   ```
   pio run --target upload
   ```
5. Open serial monitor at 115200 baud:
   ```
   pio device monitor
   ```

### Run the visualiser
**Browser (recommended):** Open `tools/visualise_imu.html` → click Connect → select the Teensy serial port. Requires a Chromium-based browser with WebSerial support.

**Python:** Install dependencies and run:
```
pip install pyserial numpy matplotlib
python tools/visualise_imu.py
```

---

## Serial Output Format

One line every 100 ms (10 Hz):

```
T:1234567 | Q:0.00123,0.00456,-0.00012,0.99998 | RPY:0.14,0.52,-0.03 | J:12.3,88.6,0.0 | K:5.40,12.30 | S:120.5,0.0
```

| Field | Description |
|-------|-------------|
| `T:` | Timestamp (µs) |
| `Q:` | Quaternion x,y,z,w |
| `RPY:` | Roll, pitch, yaw (degrees, ZYX Euler) |
| `J:` | Joint angles: ARM0, ARM1, LEG (degrees) |
| `K:` | ARM1 angular velocity (°/s) and acceleration (°/s²) |
| `S:` | Speed encoder RPMs |

Safety events: `SAFE:1` (motors cut) / `SAFE:0` (cleared via `SAFE_CLR`).

---

## Commands

Send via serial from the visualiser or terminal:

| Command | Action |
|---------|--------|
| `ARM:127` | Set arm motor speed (-255 to 255) |
| `SPR:200` | Set spring motor speed (-255 to 255) |
| `SAFE_CLR` | Re-enable motors after safety cutoff |

---

## Detailed Codebase Guide

For a complete file-by-file walkthrough of every module, data structure and design decision, see [`docs/CODEBASE_GUIDE.md`](docs/CODEBASE_GUIDE.md).

---

## Inspired By

- [SALTO](https://doi.org/10.1126/scirobotics.aag2048) — UC Berkeley Biomimetic Millisystems Lab
- [Jumper](https://doi.org/10.1038/s41586-022-04606-3) — UC Santa Barbara
- [CUBLI](https://doi.org/10.1109/pedestrians.2013.6496012) — ETH Zurich

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Author

**Vakhtang Inasaridze** — University of Warwick, 2026

Continuing at UC Berkeley MEng Mechanical Engineering.
