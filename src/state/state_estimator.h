/**
 * @file    state_estimator.h
 * @brief   StateEstimator class interface.
 * @details StateEstimator owns the periodic sampling of hardware drivers and
 *          the population of RobotState.  It uses a Teensy IntervalTimer to
 *          schedule samples at a precise rate defined by IMU_SAMPLE_RATE_HZ
 *          in config.h.
 *
 *          The IntervalTimer callback runs in interrupt context and only sets
 *          a volatile flag.  Actual sensor reads and state computation happen
 *          in tick(), which must be called from the main loop.  This keeps
 *          I2C/SPI bus access out of interrupt context.
 *
 *          StateEstimator knows nothing about the internal implementation of
 *          the IMU or encoder drivers — it only calls their public interfaces.
 *
 * @inputs  IMU& reference and JointEncoders& reference (provided at construction).
 * @outputs Populated RobotState accessible via getState().
 * @deps    state/robot_state.h, hardware/imu.h, hardware/encoder.h, config/config.h,
 *          <IntervalTimer.h> (Teensy core), <Arduino.h>
 */

#pragma once

#include <Arduino.h>
#include <IntervalTimer.h>

#include "state/robot_state.h"
#include "hardware/imu.h"
#include "hardware/encoder.h"
#include "config/config.h"

// =============================================================================
// StateEstimator class
// =============================================================================

/**
 * @class  StateEstimator
 * @brief  Orchestrates periodic hardware reads and populates RobotState.
 *
 * Usage
 * -----
 *   StateEstimator estimator(imu, encoders);
 *   estimator.begin();            // starts IntervalTimer
 *   // in loop():
 *   estimator.tick();             // processes one pending sample if available
 *   const RobotState& s = estimator.getState();
 */
class StateEstimator {
public:

    /**
     * @brief  Constructor.  Stores references to hardware drivers.
     *         Does not start the timer or access hardware.
     * @param  imu       Reference to an initialised IMU object.
     * @param  encoders  Reference to an initialised JointEncoders object.
     *                   Both must remain valid for the lifetime of this object.
     */
    StateEstimator(IMU& imu, JointEncoders& encoders);

    /**
     * @brief  Starts the IntervalTimer at IMU_SAMPLE_RATE_HZ.
     *         Also resets the internal RobotState to its zero defaults.
     * @return true  if the IntervalTimer was successfully started;
     *         false if no timer resource was available (Teensy 4.0 has 4).
     */
    bool begin();

    /**
     * @brief  Processes one pending sample if the IntervalTimer has fired.
     *         Must be called from the main loop() — not from an interrupt.
     */
    void tick();

    /**
     * @brief  Returns a const reference to the most recently populated state.
     *         The reference remains valid for the lifetime of this object.
     */
    const RobotState& getState() const;

private:

    static void onTimer();
    static volatile bool _samplePending;

    IMU&           _imu;
    JointEncoders& _encoders;
    RobotState     _state;
    IntervalTimer  _timer;
};
