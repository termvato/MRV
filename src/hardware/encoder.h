/**
 * @file    encoder.h
 * @brief   Joint encoder driver interface.
 * @details Wraps the paulstoffregen/Encoder library for quadrature decoding.
 *
 *          Joint index mapping (based on the OBJ model mesh groups):
 *            0 — ARM0    (left)   ← active encoder, pins 17/16
 *            1 — ARM1    (right)  ← active encoder, pins 14/15
 *            2 — LEG     (vertical slide)  ← no encoder
 *
 *          The class is named JointEncoders (not Encoder) to avoid a name
 *          collision with the paulstoffregen/Encoder library's Encoder class,
 *          which is included inside encoder.cpp.
 *
 * @inputs  Quadrature signals on pins defined in hardware/pinout.h.
 * @outputs Joint angle in degrees via getAngle(jointIndex).
 * @deps    config/config.h, hardware/pinout.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>
#include "config/config.h"

// =============================================================================
// JointEncoders class
// =============================================================================

/**
 * @class  JointEncoders
 * @brief  Reads quadrature encoders for each robot arm joint.
 *
 * Lifecycle
 * ---------
 *   1. Construct — attaches interrupts on ARM1 pins immediately.
 *   2. Call begin() once in setup() to verify readiness.
 *   3. Call update() each sample tick to snapshot current counts.
 *   4. Call getAngle(i) to retrieve the latest angle for joint i.
 */
class JointEncoders {
public:

    /**
     * @brief  Constructor.  Initialises the ARM0 and ARM1 encoder library
     *         objects, which attach interrupt handlers to their respective pins.
     *         No other hardware access occurs here.
     */
    JointEncoders();

    /**
     * @brief  Verifies encoder hardware is ready.
     *         Resets all stored joint angles to zero.
     * @return true  — ARM1 encoder is ready.
     *         false — encoder not yet wired or pin conflict detected.
     */
    bool begin();

    /**
     * @brief  Reads the current count from every active encoder and converts
     *         it to degrees.  Call from the main loop (not from an ISR).
     */
    void update();

    /**
     * @brief  Returns the latest joint angle in degrees for the given index.
     * @param  jointIndex  Index in [0, NUM_ENCODER_JOINTS).
     * @return Angle in degrees.  0.0f for joints without a wired encoder.
     */
    float getAngle(uint8_t jointIndex) const;

    /**
     * @brief  Returns the latest arm-shaft RPM for the given motor.
     * @param  index  0 = ARM1 / Motor B,  1 = ARM0 / Motor A.
     *                Derived from joint angular velocity — no separate encoder needed.
     * @return Arm-shaft RPM (positive = forward, negative = reverse).  0.0f if index
     *         is out of range.
     */
    float getSpeedRPM(uint8_t index) const;

    /**
     * @brief  Returns the latest angular velocity in degrees/second for the given joint.
     * @param  jointIndex  Index in [0, NUM_ENCODER_JOINTS).
     * @return Deg/s (positive = forward).  0.0f for unimplemented joints.
     */
    float getAngularVelDps(uint8_t jointIndex) const;

    /**
     * @brief  Returns the latest angular acceleration in degrees/second² for the given joint.
     * @param  jointIndex  Index in [0, NUM_ENCODER_JOINTS).
     * @return Deg/s² (raw, unfiltered).  0.0f for unimplemented joints.
     */
    float getAngularAccDps2(uint8_t jointIndex) const;

private:

    /// Stored joint angles in degrees, updated by update().
    float _angles[NUM_ENCODER_JOINTS];

    /// Angular velocity in deg/s per joint, updated by update().
    float _angularVelDps[NUM_ENCODER_JOINTS];

    /// Angular acceleration in deg/s² per joint, updated by update().
    float _angularAccDps2[NUM_ENCODER_JOINTS];

    /// Sliding-window ring buffer of past joint angles (degrees).
    /// Indexed [jointIndex][slotIndex].  Newest sample is at _histIdx.
    float _angleHistory[NUM_ENCODER_JOINTS][ENC_VEL_WINDOW];

    /// micros() timestamps corresponding to each ring-buffer slot.
    uint32_t _timeHistory[ENC_VEL_WINDOW];

    /// Next write position in the ring buffers (wraps modulo ENC_VEL_WINDOW).
    uint8_t _histIdx;

    /// True once every slot in the ring buffers has been written at least once.
    bool _histFull;

    /// Arm-shaft RPM per motor, derived from joint angular velocity in update().
    /// [0] = ARM1 / Motor B,  [1] = ARM0 / Motor A.
    float _speedRPM[NUM_SPEED_ENCODERS];

    /// micros() timestamp at the last update() call — used for kinematics dt.
    uint32_t _prevSpeedUs;
};
