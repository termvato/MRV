/**
 * @file    robot_state.h
 * @brief   Central RobotState struct — the canonical description of the robot's
 *          pose and sensor readings at a single instant in time.
 * @details RobotState is the data contract between the hardware drivers,
 *          the state estimator, and the communication layer.  It is written
 *          by StateEstimator and read by SerialReporter (and later by any
 *          control layer).  No module that reads this struct needs to know
 *          how it was produced.
 *
 *          Fields are grouped by source:
 *            - IMU quaternion + derived Euler angles (populated by StateEstimator)
 *            - Encoder joint angles (stubbed — populated once encoder layer exists)
 *            - Timestamp in microseconds (populated by StateEstimator)
 *
 * @inputs  Data written by StateEstimator.
 * @outputs Struct consumed by SerialReporter and future control modules.
 * @deps    math/quaternion.h, config/config.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>

#include "math/quaternion.h"
#include "config/config.h"

// =============================================================================
// RobotState struct
// =============================================================================

/**
 * @struct  RobotState
 * @brief   Complete robot pose snapshot at a single sample instant.
 *
 * All fields are value types (no pointers) so the struct can be safely copied
 * between contexts.  Use the provided helper methods instead of direct field
 * manipulation where possible.
 */
struct RobotState {

    // ── IMU orientation ───────────────────────────────────────────────────────

    /// Raw unit quaternion from the SFLP engine: q = w + xi + yj + zk.
    /// Represents the orientation of the robot body in world space.
    Quaternion imuQuaternion;

    /// Tilt around body X, derived via gravity-vector projection.
    /// Range: [-180, 180].  Gravity-stable; no heading drift.
    float roll;

    /// Free-spinning axis angle.  Range: [-180, 180].
    /// May drift or spin freely — do not rely on its absolute value.
    float pitch;

    /// Total tilt from vertical.  Range: [0, 180].
    /// 0° = perfectly upright, 180° = fully inverted.
    /// Heading-free; used for the safety cutoff.
    float yaw;

    // ── Joint encoder angles  (stubbed — all zeros until encoder layer) ───────

    /// Joint angles in degrees for each encoded robot joint.
    /// Index mapping (based on OBJ model groups):
    ///   [0] ARM (left)  [1] ARM(1) (right, active encoder)  [2] LEG
    float jointAngles[NUM_ENCODER_JOINTS];

    /// Angular velocity in deg/s per joint.  0.0f for unimplemented joints.
    float jointAngularVelDps[NUM_ENCODER_JOINTS];

    /// Angular acceleration in deg/s² per joint (raw/unfiltered).  0.0f for unimplemented.
    float jointAngularAccDps2[NUM_ENCODER_JOINTS];

    // ── Speed encoder readings ─────────────────────────────────────────────────

    /// Shaft speeds in RPM for the two speed-only encoders (PIM604, 96 CPR).
    ///   [0] pins 14/15   [1] pins 16/17
    float encoderSpeedRPM[NUM_SPEED_ENCODERS];

    // ── Timing ────────────────────────────────────────────────────────────────

    /// Teensy micros() timestamp at the moment StateEstimator populated this
    /// struct.  Wraps around after ~70 minutes; use difference arithmetic only.
    uint32_t timestampUs;

    // -------------------------------------------------------------------------
    // Helper methods
    // -------------------------------------------------------------------------

    /**
     * @brief  Resets all fields to their zero/identity defaults:
     *           - imuQuaternion → identity (0, 0, 0, 1)
     *           - roll, pitch, yaw → 0.0f
     *           - jointAngles[] → all 0.0f
     *           - timestampUs → 0
     */
    void reset();

    /**
     * @brief  Copies all fields from src into this struct.
     *         Equivalent to a memberwise assignment; provided as a named
     *         method to make copy intent explicit at call sites.
     * @param  src  Source state to copy from.
     */
    void copyFrom(const RobotState& src);

    /**
     * @brief  Prints a single-line human-readable summary of this state
     *         to Serial.  Format:
     *           [RobotState] t=<us> q=(<x>,<y>,<z>,<w>) rpy=(<r>,<p>,<y>)
     * @note   For use during development only.  High-rate calls will flood
     *         the Serial buffer; use SerialReporter for rate-limited output.
     */
    void print() const;
};
