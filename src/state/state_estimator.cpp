/**
 * @file    state_estimator.cpp
 * @brief   StateEstimator implementation.
 */

#include "state/state_estimator.h"

volatile bool StateEstimator::_samplePending = false;

StateEstimator::StateEstimator(IMU& imu, JointEncoders& encoders)
    : _imu(imu), _encoders(encoders)
{
    _state.reset();
}

bool StateEstimator::begin()
{
    _state.reset();
    const float intervalUs = 1000000.0f / static_cast<float>(IMU_SAMPLE_RATE_HZ);
    return _timer.begin(StateEstimator::onTimer, intervalUs);
}

void StateEstimator::onTimer()
{
    _samplePending = true;
}

void StateEstimator::tick()
{
    if (!_samplePending) {
        return;
    }
    _samplePending = false;

    // ── Encoders ──────────────────────────────────────────────────────────────
    _encoders.update();
    for (uint8_t i = 0u; i < NUM_ENCODER_JOINTS; ++i) {
        _state.jointAngles[i]         = _encoders.getAngle(i);
        _state.jointAngularVelDps[i]  = _encoders.getAngularVelDps(i);
        _state.jointAngularAccDps2[i] = _encoders.getAngularAccDps2(i);
    }
    for (uint8_t i = 0u; i < NUM_SPEED_ENCODERS; ++i) {
        _state.encoderSpeedRPM[i] = _encoders.getSpeedRPM(i);
    }

    // ── IMU ───────────────────────────────────────────────────────────────────
    if (_imu.update()) {
        const Quaternion q = _imu.getQuaternion();
        _state.imuQuaternion = q;
        q.toEuler(_state.roll, _state.pitch, _state.yaw);
        _state.timestampUs = micros();
    }
}

const RobotState& StateEstimator::getState() const
{
    return _state;
}
