/**
 * @file    config.h
 * @brief   Global compile-time configuration constants for the MRV firmware.
 * @details Single source of truth for every tunable parameter in the project.
 *          No magic numbers belong anywhere else in the codebase.
 *          All values are constexpr — zero runtime cost, full type safety.
 * @inputs  None — this file has no dependencies.
 * @outputs constexpr constants consumed by all other modules.
 * @deps    None.
 */

#pragma once

#include <stdint.h>

// =============================================================================
// Sample and report rates
// =============================================================================

/// State estimator sample rate in Hz.
/// Drives the IntervalTimer period inside StateEstimator.
/// Must not exceed the configured SFLP_ODR_HZ.
constexpr uint16_t IMU_SAMPLE_RATE_HZ = 120u;

/// Serial reporter output rate in Hz.
/// Must be <= IMU_SAMPLE_RATE_HZ.  Controls how often SerialReporter prints.
constexpr uint16_t SERIAL_REPORT_RATE_HZ = 10u;

// =============================================================================
// Accelerometer / gyroscope ODR and full-scale range
// =============================================================================

/// Accelerometer output data rate in Hz.
/// Should match SFLP_ODR_HZ so the fusion engine is never starved.
constexpr float IMU_ACCEL_ODR_HZ = 120.0f;

/// Accelerometer full-scale range in ±g.  Valid: 2, 4, 8, 16.
/// ±2g gives the best resolution for a slow robot.
constexpr uint8_t IMU_ACCEL_FS_G = 2u;

/// Gyroscope output data rate in Hz.
/// Should match SFLP_ODR_HZ.
constexpr float IMU_GYRO_ODR_HZ = 120.0f;

/// Gyroscope full-scale range in ±dps.  Valid: 125, 250, 500, 1000, 2000, 4000.
/// ±250 dps balances resolution with headroom for fast motions.
/// Lower values improve bias estimation and reduce drift.
constexpr uint16_t IMU_GYRO_FS_DPS = 250u;

// =============================================================================
// SFLP (Sensor Fusion Low Power) engine configuration
// =============================================================================

/// SFLP output data rate in Hz.
/// Valid values for LSM6DSV16X: 15, 30, 60, 120.
/// Set to the maximum supported rate to minimise fusion latency.
constexpr float SFLP_ODR_HZ = 120.0f;

/// Enable onboard gyroscope bias estimation within the SFLP engine.
/// 1 = enabled (recommended for long-run stability), 0 = disabled.
constexpr uint8_t SFLP_ENABLE_GBIAS = 1u;

/// FIFO watermark level — number of SFLP game-rotation-vector samples to
/// accumulate before the firmware reads the FIFO.
/// 1 minimises latency; increase only if I2C bus overhead is a concern.
constexpr uint16_t IMU_FIFO_WATERMARK = 1u;

// =============================================================================
// Encoder / joint configuration
// =============================================================================

/// Number of independently encoded robot joints.
///   0 — ARM     (left arm)
///   1 — ARM(1)  (right arm)  ← active encoder, pins 20/21
///   2 — LEG     (vertical slide)
constexpr uint8_t NUM_ENCODER_JOINTS = 3u;

/// ARM1 (joint index 1) encoder — quadrature counts per full encoder shaft revolution.
/// Empirically calibrated: with a confirmed 50:1 gear ratio, one full arm revolution
/// measured 22°, implying the encoder produces only ~12 counts/rev rather than the
/// nominal 96 (24 PPR × 4).  Possible causes: encoder outputs non-quadrature signal,
/// channel B wiring issue, or non-standard PPR.  Adjust until one arm rev = 360°.
constexpr float ENC_ARM1_COUNTS_PER_REV = 12.0f; //DO NOT TOUCH

/// ARM1 gear ratio: encoder shaft revolutions per joint output revolution.
/// Confirmed by user as 50:1.
constexpr float ENC_ARM1_GEAR_RATIO = 50.0f; //DO NOT TOUCH

/// ARM (joint 0) calibration angle at begin() — physical rest position in degrees.
constexpr float ENC_ARM0_INIT_DEG = 0.0f;

/// ARM(1) (joint 1) calibration angle at begin() — physical rest position in degrees.
constexpr float ENC_ARM1_INIT_DEG = 90.0f;

// =============================================================================
// ARM (joint 0) encoder — enable once wired
// =============================================================================

/// Uncomment to activate the ARM (joint 0) encoder driver.
/// Also set PIN_ENC_ARM0_A/B in pinout.h before enabling.
#define USE_ARM0_ENCODER

/// Counts per encoder shaft revolution — same model as ARM(1), placeholder value.
/// Tune empirically once wired (like ENC_ARM1_COUNTS_PER_REV).
constexpr float ENC_ARM0_COUNTS_PER_REV = 12.0f;  // tune when wired

/// Gear ratio — placeholder, confirm with hardware.
constexpr float ENC_ARM0_GEAR_RATIO = 50.0f;  // tune when wired

/// Angle-to-count scalar — analogous to the 359.32f in the ARM(1) formula.
/// Tune manually until one full arm revolution reads exactly 360°.
constexpr float ENC_ARM0_SCALAR = 360.0f;  // tune when wired

// =============================================================================
// Speed encoder configuration
// =============================================================================

/// Number of arm-shaft RPM channels (one per arm motor).
/// RPM is derived from joint angular velocity — no separate encoder objects.
///   [0] = ARM1 / Motor B    [1] = ARM0 / Motor A
constexpr uint8_t NUM_SPEED_ENCODERS = 2u;

// =============================================================================
// Encoder kinematics filter
// =============================================================================

/// Sliding-window length for the velocity estimator (number of update() ticks).
/// Velocity is computed as total_angle_change / total_elapsed_time over this
/// window, which averages away quantisation spikes.
/// At 120 Hz: 10 ticks → 83 ms window; minimum detectable speed ≈ 7 deg/s.
/// Increase for smoother readings at very slow speeds; decrease for faster
/// response to sudden starts/stops.
constexpr uint8_t ENC_VEL_WINDOW = 10u;

/// EMA smoothing factor applied to the windowed velocity before acceleration
/// is computed.  Range (0, 1]: lower = heavier smoothing / more lag.
constexpr float ENC_VEL_ALPHA = 0.10f;

// =============================================================================
// Accelerometer tilt filter
// =============================================================================

/// EMA smoothing factor for raw accelerometer tilt computation.
/// Range (0, 1]: lower = heavier smoothing / more lag.
/// At 120 Hz, 0.15 gives a ~55 ms time constant.
constexpr float ACCEL_TILT_ALPHA = 0.15f;

// =============================================================================
// IMU bus selection
// =============================================================================

/// Define USE_IMU_SPI to select SPI transport; leave commented for I2C.
// #define USE_IMU_SPI

/// LSM6DSV16X SA0 pin state that selects the I2C address.
///   0 → LSM6DSV16X_I2C_ADD_L (0x6A, SA0 tied to GND)
///   1 → LSM6DSV16X_I2C_ADD_H (0x6B, SA0 tied to VDD)
/// SparkFun Micro 6DoF IMU Breakout (Qwiic) default: SA0 HIGH → 0x6B.
/// Cut the ADR jumper on the back of the board to switch to 0x6A.
constexpr uint8_t IMU_I2C_SA0 = 1u;

/// I2C bus clock speed in Hz.  400 kHz (Fast Mode) is safe for LSM6DSV16X.
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000u;

// =============================================================================
// Motor PWM cap and PID defaults
// =============================================================================

/// Hard duty-cycle cap for arm motors.
/// 196 / 255 = 76.9 % ≈ 77 % → ≤ 6 V from a 7.6 V supply.  DO NOT CHANGE.
constexpr int16_t MAX_ARM_PWM = 196;

/// Default PID gains for the IMU attitude control loop (both arm motors).
/// Tune interactively via PIDA:/PIDB: serial commands in the visualiser.
constexpr float PID_ATT_KP = 2.0f;
constexpr float PID_ATT_KI = 0.05f;
constexpr float PID_ATT_KD = 0.3f;

/// Default proportional gain for the per-motor speed PID used in revolution-
/// sync mode.  ki and kd are left at 0 for a simple P-only speed regulator.
constexpr float PID_RPM_KP = 0.5f;

// =============================================================================
// Motor safety cutoff thresholds
// =============================================================================

/// Safety monitor poll rate in Hz.  Runs in its own IntervalTimer ISR,
/// independently of the main loop and I2C traffic, so the hardware cutoff
/// fires within one ISR period of the tilt threshold being exceeded.
/// 1000 Hz → worst-case latency ≈ 1 ms (vs. ~8 ms at 120 Hz IMU rate).
constexpr uint32_t SAFETY_CHECK_RATE_HZ = 1000u;

/// Maximum tilt from vertical (degrees) allowed before motors are cut.
/// Applied independently to roll and pitch.  Yaw is a free-spinning axis
/// (robot rotates around it) and is not checked.
/// If |roll| > SAFETY_TILT_DEG OR |pitch| > SAFETY_TILT_DEG, the arm driver
/// asserts STBY LOW and all motors stop.  Motors can only be re-enabled
/// (via SAFE_CLR command) once both axes are back within this limit.
constexpr float SAFETY_TILT_DEG = 45.0f;
