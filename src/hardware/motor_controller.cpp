/**
 * @file    motor_controller.cpp
 * @brief   MotorController implementation.
 * @details See motor_controller.h for full design rationale and hardware mapping.
 *
 *          Speed-encoder index mapping:
 *            Motor A (ARM0, AIN1=8/AIN2=9/PWMA=5)  → encoderSpeedRPM[1]  (ARM0 angular vel)
 *            Motor B (ARM1, BIN1=11/BIN2=12/PWMB=6) → encoderSpeedRPM[0]  (ARM1 angular vel)
 *
 *          Revolution counting uses RPM integration:
 *            revs += |RPM| / 60.0f * dt
 *          This avoids needing raw accumulated counts and works correctly for
 *          any speed profile, including ramp-up.
 *
 * @inputs  RobotState via tick(), mode/gain setters via serial commands.
 * @outputs MotorDriver::setArmA() / setArmB() at IMU_SAMPLE_RATE_HZ.
 * @deps    hardware/motor_controller.h, <Arduino.h>
 */

#include "hardware/motor_controller.h"

// =============================================================================
// MotorController::begin()
// =============================================================================

void MotorController::begin(MotorDriver& drv, JointEncoders& enc)
{
    _drv = &drv;
    _enc = &enc;

    // Attitude PID — start at zero gains; set via serial before activating
    _pidA.kp = 0.0f;  _pidA.ki = 0.0f;  _pidA.kd = 0.0f;
    _pidB.kp = 0.0f;  _pidB.ki = 0.0f;  _pidB.kd = 0.0f;
    _pidA.integralMax = static_cast<float>(kMaxDuty);
    _pidB.integralMax = static_cast<float>(kMaxDuty);

    _rpmPidA.kp = 0.0f;
    _rpmPidA.integralMax = static_cast<float>(kMaxDuty);
    _rpmPidB.kp = 0.0f;
    _rpmPidB.integralMax = static_cast<float>(kMaxDuty);

    // Ensure motors are off at startup
    _drv->setArmA(0);
    _drv->setArmB(0);
}

// =============================================================================
// MotorController::tick()
// =============================================================================

void MotorController::tick(const RobotState& st, float dtSec)
{
    if (_drv == nullptr) return;
    if (_drv->isSafetyTripped()) return;

    switch (_mode) {

    // ── MANUAL: nothing to do — manual calls go directly via setManualA/B ────
    case CtrlMode::MANUAL:
        break;

    // ── ATTITUDE_PID: IMU tilt feedback on Motor B only ───────────────────────
    case CtrlMode::ATTITUDE_PID: {
        const float measurement = axisAngle(st);
        const float error       = _attSetpoint - measurement;
        applyA(0.0f);
        applyB(_pidB.compute(error, dtSec));
        break;
    }

    // ── REV_SYNC: per-motor speed PID + revolution accumulation ──────────────
    case CtrlMode::REV_SYNC: {
        // Motor A (ARM0) — arm-shaft RPM from encoderSpeedRPM[1]
        const float rpmA  = st.encoderSpeedRPM[1];
        const float errA  = _targetRPM - fabsf(rpmA);
        _revCountA += fabsf(rpmA) / 60.0f * dtSec;

        // Motor B (ARM1) — arm-shaft RPM from encoderSpeedRPM[0]
        const float rpmB  = st.encoderSpeedRPM[0];
        const float errB  = _targetRPM - fabsf(rpmB);
        _revCountB += fabsf(rpmB) / 60.0f * dtSec;

        // Stop both motors as soon as either has reached the target revolutions
        if (_revCountA >= _targetRevs || _revCountB >= _targetRevs) {
            applyA(0.0f);
            applyB(0.0f);
            _rpmPidA.reset();
            _rpmPidB.reset();
            _mode = CtrlMode::MANUAL;   // fall back to manual so ARM: commands work
            break;
        }

        applyA(_rpmPidA.compute(errA, dtSec));
        applyB(_rpmPidB.compute(errB, dtSec));
        break;
    }
    }
}

// =============================================================================
// Mode
// =============================================================================

void MotorController::setMode(CtrlMode m)
{
    if (m == _mode) return;
    _mode = m;

    // Reset integrators and counters on mode transition
    _pidA.reset();
    _pidB.reset();
    _rpmPidA.reset();
    _rpmPidB.reset();
    _revCountA = 0.0f;
    _revCountB = 0.0f;

    // Coast motors on transition to MANUAL
    if (m == CtrlMode::MANUAL && _drv) {
        _drv->setArmA(0);
        _drv->setArmB(0);
    }
}

// =============================================================================
// Manual mode
// =============================================================================

void MotorController::setManualA(int16_t spd)
{
    if (_mode != CtrlMode::MANUAL) return;
    if (_drv) applyA(static_cast<float>(spd));
}

void MotorController::setManualB(int16_t spd)
{
    if (_mode != CtrlMode::MANUAL) return;
    if (_drv) applyB(static_cast<float>(spd));
}

// =============================================================================
// Attitude PID
// =============================================================================

void MotorController::setPIDAxis(uint8_t axis)
{
    _pidAxis = (axis <= 2u) ? axis : 0u;
}

void MotorController::setAttitudeSetpoint(float deg)
{
    _attSetpoint = deg;
}

void MotorController::setGainsA(float kp, float ki, float kd)
{
    _pidA.kp = kp;
    _pidA.ki = ki;
    _pidA.kd = kd;
    _pidA.reset();
}

void MotorController::setGainsB(float kp, float ki, float kd)
{
    _pidB.kp = kp;
    _pidB.ki = ki;
    _pidB.kd = kd;
    _pidB.reset();
}

// =============================================================================
// Revolution sync
// =============================================================================

void MotorController::startRevSync(float targetRPM, float nRevolutions)
{
    if (nRevolutions <= 0.0f || targetRPM <= 0.0f) return;

    _targetRPM  = targetRPM;
    _targetRevs = nRevolutions;
    _revCountA  = 0.0f;
    _revCountB  = 0.0f;
    _rpmPidA.reset();
    _rpmPidB.reset();
    _mode = CtrlMode::REV_SYNC;
}

// =============================================================================
// Direction flip
// =============================================================================

void MotorController::setDirFlipA(bool flip)
{
    _dirFlipA = flip;
}

void MotorController::setDirFlipB(bool flip)
{
    _dirFlipB = flip;
}

// =============================================================================
// Safety passthrough
// =============================================================================

void MotorController::safetyStop()
{
    if (_drv) _drv->safetyStop();
}

void MotorController::clearSafety()
{
    if (_drv) _drv->clearSafety();
}

bool MotorController::isSafetyTripped() const
{
    return _drv ? _drv->isSafetyTripped() : false;
}

// =============================================================================
// Private helpers
// =============================================================================

void MotorController::applyA(float output)
{
    int16_t duty = static_cast<int16_t>(constrain(output, -kMaxDuty, kMaxDuty));
    if (_dirFlipA) duty = -duty;
    _drv->setArmA(duty);
}

void MotorController::applyB(float output)
{
    int16_t duty = static_cast<int16_t>(constrain(output, -kMaxDuty, kMaxDuty));
    if (_dirFlipB) duty = -duty;
    _drv->setArmB(duty);
}

float MotorController::axisAngle(const RobotState& st) const
{
    switch (_pidAxis) {
        case 1u: return st.pitch;
        case 2u: return st.yaw;
        default: return st.roll;   // 0 = roll
    }
}
