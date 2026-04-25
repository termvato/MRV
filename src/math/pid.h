/**
 * @file    pid.h
 * @brief   Header-only PID controller with integral anti-windup.
 * @details Computes P + I + D output from a signed error signal and a time
 *          step.  The integral accumulator is clamped to ±integralMax so a
 *          sustained error cannot wind the integrator beyond the actuator
 *          saturation limit.  Derivative is computed on the error (not the
 *          measurement), which is adequate for setpoint-tracking loops where
 *          the setpoint changes infrequently.
 *
 *          Typical usage:
 *            PID pid;
 *            pid.kp = 2.0f; pid.ki = 0.05f; pid.kd = 0.3f;
 *            float out = pid.compute(setpoint - measurement, dt_sec);
 *
 * @inputs  error (float), dt in seconds (float).
 * @outputs Signed control output (float), magnitude determined by gains.
 * @deps    <Arduino.h>  (for constrain())
 */

#pragma once

#include <Arduino.h>

// =============================================================================
// PID struct
// =============================================================================

/**
 * @struct PID
 * @brief  Proportional-integral-derivative controller with integral clamping.
 *
 * All members are public so gains can be set directly without accessors.
 * State (_integral, _prevError) is prefixed with underscore as a convention
 * to indicate "don't touch externally".
 */
struct PID {

    // ── Gains ──────────────────────────────────────────────────────────────────
    float kp = 0.0f;   ///< Proportional gain.
    float ki = 0.0f;   ///< Integral gain.
    float kd = 0.0f;   ///< Derivative gain.

    /// Integral accumulator clamp magnitude.  Output of compute() can still
    /// exceed this due to the P and D terms; set to MAX_ARM_PWM so the
    /// integral alone cannot saturate the actuator.
    float integralMax = 196.0f;

    // ── Internal state (reset via reset()) ─────────────────────────────────────
    float _integral  = 0.0f;   ///< Accumulated integral (clamped).
    float _prevError = 0.0f;   ///< Previous error for derivative calculation.

    // ── Methods ────────────────────────────────────────────────────────────────

    /**
     * @brief  Advances the PID by one time step.
     * @param  error  Signed error: setpoint − measurement.
     * @param  dt     Time since the previous call in seconds.  Must be > 0.
     *                If dt ≤ 0 the derivative term is zeroed for that step.
     * @return Signed control output = kp*e + ki*∫e·dt + kd*(de/dt).
     */
    float compute(float error, float dt)
    {
        _integral = constrain(_integral + error * dt, -integralMax, integralMax);
        const float deriv = (dt > 0.0f) ? (error - _prevError) / dt : 0.0f;
        _prevError = error;
        return kp * error + ki * _integral + kd * deriv;
    }

    /**
     * @brief  Clears accumulated integral and previous-error state.
     *         Call when switching modes or enabling a motor after a stop.
     */
    void reset()
    {
        _integral  = 0.0f;
        _prevError = 0.0f;
    }
};
