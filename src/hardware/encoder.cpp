/**
 * @file    encoder.cpp
 * @brief   Joint encoder driver implementation using paulstoffregen/Encoder.
 * @details ARM0 (joint 0, pins 17/16) and ARM1 (joint 1, pins 14/15) are both
 *          decoded in full 4× quadrature mode, reporting position (degrees),
 *          angular velocity (deg/s), and angular acceleration (deg/s²).
 *
 *          encoderSpeedRPM[] is derived from joint angular velocity — no separate
 *          encoder objects are needed for RPM since the joint encoders already
 *          compute smoothed velocity:
 *            RPM = angularVelDps / 6.0  (= deg/s ÷ 360 × 60)
 *            speedRPM[0] ← ARM1 arm-shaft RPM  (Motor B feedback)
 *            speedRPM[1] ← ARM0 arm-shaft RPM  (Motor A feedback)
 *
 *          Counts-to-degrees for ARM(1):
 *            degrees = count × 359.32 / (ENC_ARM1_COUNTS_PER_REV × ENC_ARM1_GEAR_RATIO)
 *
 *          Angular velocity (sliding window + EMA):
 *            rawVel = (angle_now − angle[ENC_VEL_WINDOW ticks ago]) / window_dt
 *            vel    = ENC_VEL_ALPHA × rawVel + (1 − α) × vel_prev
 *
 *          Angular acceleration:
 *            acc (deg/s²) = (vel_now − vel_prev) / one_tick_dt
 *
 * @inputs  ARM0:  A/B on PIN_ENC_ARM0_A (17) and PIN_ENC_ARM0_B (16).
 *          ARM1:  A/B on PIN_ENC_ARM1_A (14) and PIN_ENC_ARM1_B (15).
 * @outputs Angles + kinematics via getAngle(), getAngularVelDps(), getAngularAccDps2().
 *          Arm-shaft RPM via getSpeedRPM().
 * @deps    hardware/encoder.h, hardware/pinout.h, config/config.h,
 *          <Encoder.h> (paulstoffregen/Encoder library)
 */

#include <Encoder.h>                 // paulstoffregen's quadrature library

#include "hardware/encoder.h"
#include "hardware/pinout.h"
#include "config/config.h"

// =============================================================================
// Module-level encoder pointers (constructed in begin() after USB init)
// =============================================================================

// Encoder objects are heap-constructed in begin() rather than at static init
// time.  Static construction of Encoder objects runs before Teensyduino's USB
// initialisation and was found to crash the firmware (8-blink Teensyduino
// fault pattern) on Teensy 4.0.  Deferring to begin() avoids this.

static Encoder* s_arm1 = nullptr;

#ifdef USE_ARM0_ENCODER
static Encoder* s_arm0 = nullptr;
#endif

// =============================================================================
// Constructor
// =============================================================================

JointEncoders::JointEncoders()
    : _prevSpeedUs(0u),
      _histIdx(0u), _histFull(false)
{
    for (uint8_t i = 0u; i < NUM_ENCODER_JOINTS; ++i) {
        _angles[i]         = 0.0f;
        _angularVelDps[i]  = 0.0f;
        _angularAccDps2[i] = 0.0f;
    }
    for (uint8_t s = 0u; s < ENC_VEL_WINDOW; ++s) {
        _timeHistory[s] = 0u;
        for (uint8_t j = 0u; j < NUM_ENCODER_JOINTS; ++j) {
            _angleHistory[j][s] = 0.0f;
        }
    }
    for (uint8_t i = 0u; i < NUM_SPEED_ENCODERS; ++i) {
        _speedRPM[i] = 0.0f;
    }
}

// =============================================================================
// JointEncoders::begin()
// =============================================================================

bool JointEncoders::begin()
{
    // ── Construct encoder objects here (deferred from static init) ────────────
    // Constructing Encoder objects at static init time caused a Teensyduino
    // hard-fault (8-blink) on Teensy 4.0 because the interrupt infrastructure
    // is not yet fully initialised before setup() runs.
    s_arm1 = new Encoder(PIN_ENC_ARM1_A, PIN_ENC_ARM1_B);

#ifdef USE_ARM0_ENCODER
    s_arm0 = new Encoder(PIN_ENC_ARM0_A, PIN_ENC_ARM0_B);
#endif

    // ── ARM(1) — set count to match physical rest angle ───────────────────────
    // Derived by inverting: angle = count × 359.32 / (CPR × ratio)
    const int32_t arm1InitCount = static_cast<int32_t>(
        ENC_ARM1_INIT_DEG * ENC_ARM1_COUNTS_PER_REV * ENC_ARM1_GEAR_RATIO / 359.32f
    );
    s_arm1->write(arm1InitCount);

#ifdef USE_ARM0_ENCODER
    // ── ARM — set count to match physical rest angle ──────────────────────────
    const int32_t arm0InitCount = static_cast<int32_t>(
        ENC_ARM0_INIT_DEG * ENC_ARM0_COUNTS_PER_REV * ENC_ARM0_GEAR_RATIO / ENC_ARM0_SCALAR
    );
    s_arm0->write(arm0InitCount);
#endif

    // Initialise stored state from calibrated starting positions.
    for (uint8_t i = 0u; i < NUM_ENCODER_JOINTS; ++i) {
        _angles[i]         = 0.0f;
        _angularVelDps[i]  = 0.0f;
        _angularAccDps2[i] = 0.0f;
    }
    _angles[0] = ENC_ARM0_INIT_DEG;
    _angles[1] = ENC_ARM1_INIT_DEG;

    // Pre-fill ring buffer with the calibrated angles so the first update()
    // computes a valid (zero) velocity immediately rather than waiting for
    // ENC_VEL_WINDOW ticks.  All slots share the same timestamp — the winDt
    // guard in update() prevents a divide-by-zero on the first real tick.
    const uint32_t t0 = micros();
    for (uint8_t s = 0u; s < ENC_VEL_WINDOW; ++s) {
        _timeHistory[s]    = t0;
        _angleHistory[0][s] = ENC_ARM0_INIT_DEG;
        _angleHistory[1][s] = ENC_ARM1_INIT_DEG;
    }
    _histIdx  = 0u;
    _histFull = true;

    for (uint8_t i = 0u; i < NUM_SPEED_ENCODERS; ++i) {
        _speedRPM[i] = 0.0f;
    }

    _prevSpeedUs = micros();

    return true;
}

// =============================================================================
// JointEncoders::update()
// =============================================================================

void JointEncoders::update()
{
    // ── Elapsed time ──────────────────────────────────────────────────────────
    const uint32_t now = micros();
    const float    dt  = static_cast<float>(now - _prevSpeedUs) * 1e-6f;
    _prevSpeedUs = now;

    // ── ARM(1) position ───────────────────────────────────────────────────────
    const int32_t count1 = s_arm1->read();

    // degrees = count × 359.32 / (COUNTS_PER_REV × GEAR_RATIO)
    _angles[1] = static_cast<float>(count1) * 359.32f //CLAUDE DO NOT TOUCH, I MANUALLY TUNED THIS
                 / (static_cast<float>(ENC_ARM1_COUNTS_PER_REV)
                    * ENC_ARM1_GEAR_RATIO);

    // ── Ring buffer: record current time and arm(1) angle ─────────────────────
    _timeHistory[_histIdx]     = now;
    _angleHistory[1][_histIdx] = _angles[1];

    // ── ARM(1) kinematics (sliding window + EMA) ──────────────────────────────
    if (_histFull && dt > 0.0f) {
        const uint8_t oldIdx = (_histIdx + 1u) % ENC_VEL_WINDOW;
        const float   winDt  = static_cast<float>(now - _timeHistory[oldIdx]) * 1e-6f;
        if (winDt > 0.0f) {
            const float rawVel1    = (_angles[1] - _angleHistory[1][oldIdx]) / winDt;
            const float smoothVel1 = ENC_VEL_ALPHA * rawVel1
                                     + (1.0f - ENC_VEL_ALPHA) * _angularVelDps[1];
            _angularAccDps2[1]     = (smoothVel1 - _angularVelDps[1]) / dt;
            _angularVelDps[1]      = smoothVel1;
        }
    }

#ifdef USE_ARM0_ENCODER
    // ── ARM position ──────────────────────────────────────────────────────────
    const int32_t count0 = s_arm0->read();

    _angles[0] = static_cast<float>(count0) * ENC_ARM0_SCALAR  // tune when wired
                 / (static_cast<float>(ENC_ARM0_COUNTS_PER_REV)
                    * ENC_ARM0_GEAR_RATIO);

    _angleHistory[0][_histIdx] = _angles[0];

    // ── ARM kinematics (sliding window + EMA) ─────────────────────────────────
    if (_histFull && dt > 0.0f) {
        const uint8_t oldIdx = (_histIdx + 1u) % ENC_VEL_WINDOW;
        const float   winDt  = static_cast<float>(now - _timeHistory[oldIdx]) * 1e-6f;
        if (winDt > 0.0f) {
            const float rawVel0    = (_angles[0] - _angleHistory[0][oldIdx]) / winDt;
            const float smoothVel0 = ENC_VEL_ALPHA * rawVel0
                                     + (1.0f - ENC_VEL_ALPHA) * _angularVelDps[0];
            _angularAccDps2[0]     = (smoothVel0 - _angularVelDps[0]) / dt;
            _angularVelDps[0]      = smoothVel0;
        }
    }
#endif

    // ── Advance ring buffer ───────────────────────────────────────────────────
    _histIdx = (_histIdx + 1u) % ENC_VEL_WINDOW;
    if (_histIdx == 0u) _histFull = true;

    // ── Arm-shaft RPM — derived from joint angular velocity ───────────────────
    // RPM = deg/s ÷ 360 × 60 = deg/s ÷ 6
    // Index mapping matches original motor_controller expectation:
    //   [0] = ARM1 (Motor B)    [1] = ARM0 (Motor A)
    _speedRPM[0] = _angularVelDps[1] / 6.0f;
    _speedRPM[1] = _angularVelDps[0] / 6.0f;
}

// =============================================================================
// Accessors
// =============================================================================

float JointEncoders::getAngle(uint8_t jointIndex) const
{
    if (jointIndex >= NUM_ENCODER_JOINTS) return 0.0f;
    return _angles[jointIndex];
}

float JointEncoders::getAngularVelDps(uint8_t jointIndex) const
{
    if (jointIndex >= NUM_ENCODER_JOINTS) return 0.0f;
    return _angularVelDps[jointIndex];
}

float JointEncoders::getAngularAccDps2(uint8_t jointIndex) const
{
    if (jointIndex >= NUM_ENCODER_JOINTS) return 0.0f;
    return _angularAccDps2[jointIndex];
}

float JointEncoders::getSpeedRPM(uint8_t index) const
{
    if (index >= NUM_SPEED_ENCODERS) return 0.0f;
    return _speedRPM[index];
}
