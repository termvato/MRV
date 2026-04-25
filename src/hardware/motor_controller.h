/**
 * @file    motor_controller.h
 * @brief   High-level arm motor controller: manual pass-through, IMU attitude
 *          PID, and synchronised shaft-revolution mode.
 * @details Wraps MotorDriver and JointEncoders to provide three operating modes:
 *
 *            MANUAL    — Raw PWM commands forwarded directly to MotorDriver.
 *                        Arms controlled by ARM: serial commands from visualiser.
 *
 *            ATTITUDE_PID — Closed-loop attitude control using IMU roll/pitch/yaw.
 *                        Both motors receive the same PID output so the arm pair
 *                        acts as a symmetric actuator.  Gains are tunable live via
 *                        PIDA:/PIDB: serial commands.
 *
 *            REV_SYNC  — Both motors spin at a user-specified RPM until each has
 *                        completed a requested number of shaft revolutions, then
 *                        both stop simultaneously.  Per-motor speed PID tracks the
 *                        target RPM using speed encoder feedback.
 *
 *          Hardware mapping (user-confirmed):
 *            Motor A (ARM0)  AIN1=8, AIN2=9, PWMA=5  → encoderSpeedRPM[1] (ARM0 vel)
 *            Motor B (ARM1)  BIN1=11, BIN2=12, PWMB=6 → encoderSpeedRPM[0] (ARM1 vel)
 *
 *          PWM cap: all motor outputs are clamped to ±MAX_ARM_PWM (196 / 77 %)
 *          before being forwarded to MotorDriver.  This is enforced even in
 *          MANUAL mode.
 *
 * @inputs  RobotState (attitude + speed RPM) passed to tick().
 *          Serial commands interpreted by main.cpp and forwarded via setters.
 * @outputs MotorDriver::setArmA() / setArmB() at 120 Hz.
 * @deps    hardware/motor_driver.h, hardware/encoder.h, math/pid.h,
 *          state/robot_state.h, config/config.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>

#include "config/config.h"
#include "math/pid.h"
#include "hardware/motor_driver.h"
#include "hardware/encoder.h"
#include "state/robot_state.h"

// =============================================================================
// CtrlMode enum
// =============================================================================

/**
 * @enum  CtrlMode
 * @brief Selects the active motor control strategy.
 */
enum class CtrlMode : uint8_t {
    MANUAL       = 0,   ///< Raw PWM pass-through from serial commands.
    ATTITUDE_PID = 1,   ///< IMU tilt feedback PID (roll/pitch/yaw selectable).
    REV_SYNC     = 2,   ///< Synchronised shaft-revolution mode.
};

// =============================================================================
// MotorController class
// =============================================================================

/**
 * @class  MotorController
 * @brief  Mode-switching arm motor controller with PID and revolution tracking.
 *
 * Lifecycle
 * ---------
 *   1. Construct — no hardware access.
 *   2. Call begin(drv, enc) once in setup() after MotorDriver and JointEncoders
 *      are initialised.
 *   3. Call tick(state, dt) every IMU sample period (120 Hz) in loop().
 *   4. Use set*() methods to change mode and parameters at runtime.
 */
class MotorController {
public:

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @brief  Stores references to the motor driver and encoder objects.
     *         Initialises PID gains from config defaults.
     *         Motor outputs are zeroed; mode is MANUAL.
     */
    void begin(MotorDriver& drv, JointEncoders& enc);

    /**
     * @brief  Called at IMU_SAMPLE_RATE_HZ by loop().  Executes the active
     *         control mode and updates motor outputs via MotorDriver.
     * @param  st    Current robot state (attitude + speed RPM).
     * @param  dtSec Time since the previous tick in seconds.
     */
    void tick(const RobotState& st, float dtSec);

    // ── Mode ──────────────────────────────────────────────────────────────────

    /**
     * @brief  Switches the active control mode.
     *         Resets all PID accumulators and revolution counters on transition.
     *         In MANUAL mode the motors are coasted (set to 0) immediately.
     */
    void setMode(CtrlMode m);

    /** @return Current control mode. */
    CtrlMode getMode() const { return _mode; }

    // ── Manual mode ───────────────────────────────────────────────────────────

    /**
     * @brief  Sets arm Motor A to the given speed.
     *         Clamped to ±MAX_ARM_PWM even in MANUAL mode.
     *         Silently ignored when mode is not MANUAL.
     */
    void setManualA(int16_t spd);

    /**
     * @brief  Sets arm Motor B to the given speed.  Same semantics as setManualA().
     */
    void setManualB(int16_t spd);

    // ── Attitude PID ──────────────────────────────────────────────────────────

    /**
     * @brief  Selects which IMU axis is used as the PID feedback signal.
     * @param  axis  0 = roll, 1 = pitch, 2 = yaw.  Clamped to [0, 2].
     */
    void setPIDAxis(uint8_t axis);

    /**
     * @brief  Sets the desired attitude angle in degrees for the attitude PID.
     */
    void setAttitudeSetpoint(float deg);

    /**
     * @brief  Updates Motor A PID gains.  Takes effect immediately.
     */
    void setGainsA(float kp, float ki, float kd);

    /**
     * @brief  Updates Motor B PID gains.  Takes effect immediately.
     */
    void setGainsB(float kp, float ki, float kd);

    // ── Revolution sync mode ──────────────────────────────────────────────────

    /**
     * @brief  Arms revolution-sync mode and starts both motors.
     *         The motors will spin at targetRPM until each has accumulated
     *         nRevolutions shaft revolutions, then stop.
     * @param  targetRPM     Desired shaft speed in RPM (magnitude; direction
     *                       is set by the per-motor direction-flip flag).
     * @param  nRevolutions  Number of shaft revolutions to execute (> 0).
     */
    void startRevSync(float targetRPM, float nRevolutions);

    // ── Direction flip ────────────────────────────────────────────────────────

    /**
     * @brief  Reverses the effective polarity of Motor A by swapping the
     *         forward and reverse PWM pins in software.
     */
    void setDirFlipA(bool flip);

    /** @brief  Reverses the effective polarity of Motor B. */
    void setDirFlipB(bool flip);

    // ── Safety passthrough ────────────────────────────────────────────────────

    /**
     * @brief  Immediately stops all motor output and latches safety.
     *         Delegates to MotorDriver::safetyStop(); ISR-safe.
     */
    void safetyStop();

    /**
     * @brief  Re-enables motor output and clears the safety latch.
     *         Delegates to MotorDriver::clearSafety().
     */
    void clearSafety();

    /** @return true while the safety latch is set. */
    bool isSafetyTripped() const;

    // ── Status ────────────────────────────────────────────────────────────────

    /** @return Accumulated shaft revolutions for Motor A since last REV_SYNC start. */
    float getRevCountA() const { return _revCountA; }

    /** @return Accumulated shaft revolutions for Motor B since last REV_SYNC start. */
    float getRevCountB() const { return _revCountB; }

private:

    // ── 77 % PWM hard cap ─────────────────────────────────────────────────────
    static constexpr int16_t kMaxDuty = MAX_ARM_PWM;  // 196 — DO NOT CHANGE

    // ── Dependencies ──────────────────────────────────────────────────────────
    MotorDriver*   _drv = nullptr;
    JointEncoders* _enc = nullptr;

    // ── Mode ──────────────────────────────────────────────────────────────────
    CtrlMode _mode = CtrlMode::MANUAL;

    // ── Attitude PID ──────────────────────────────────────────────────────────
    PID     _pidA;
    PID     _pidB;
    uint8_t _pidAxis     = 0u;      ///< 0=roll, 1=pitch, 2=yaw
    float   _attSetpoint = 0.0f;   ///< Target attitude angle in degrees

    // ── Revolution sync ───────────────────────────────────────────────────────
    float _targetRPM  = 0.0f;
    float _targetRevs = 0.0f;
    float _revCountA  = 0.0f;  ///< Accumulated shaft revolutions, Motor A
    float _revCountB  = 0.0f;  ///< Accumulated shaft revolutions, Motor B
    PID   _rpmPidA;             ///< Speed PID for Motor A in REV_SYNC
    PID   _rpmPidB;             ///< Speed PID for Motor B in REV_SYNC

    // ── Direction flip ────────────────────────────────────────────────────────
    bool _dirFlipA = false;
    bool _dirFlipB = false;

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * @brief  Clamps output to ±kMaxDuty, applies direction flip for Motor A,
     *         then calls _drv->setArmA().
     */
    void applyA(float output);

    /**
     * @brief  Same as applyA() but for Motor B.
     */
    void applyB(float output);

    /**
     * @brief  Extracts the attitude angle selected by _pidAxis from the state.
     * @return roll, pitch, or yaw in degrees depending on _pidAxis.
     */
    float axisAngle(const RobotState& st) const;
};
