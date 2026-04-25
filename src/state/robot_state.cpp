/**
 * @file    robot_state.cpp
 * @brief   Helper method implementations for the RobotState struct.
 * @details Provides reset, copy, and Serial print utilities.
 *          No hardware access — this file only manipulates data and writes
 *          to the Arduino Serial object for debug output.
 * @inputs  RobotState field values.
 * @outputs Reset/zeroed struct fields; formatted Serial output.
 * @deps    state/robot_state.h, <Arduino.h>
 */

#include "state/robot_state.h"

// =============================================================================
// RobotState::reset()
// =============================================================================

/**
 * @brief Zeroes all fields.  imuQuaternion becomes the identity (0,0,0,1).
 */
void RobotState::reset()
{
    imuQuaternion = Quaternion();  // identity: x=0, y=0, z=0, w=1

    roll  = 0.0f;
    pitch = 0.0f;
    yaw   = 0.0f;

    for (uint8_t i = 0u; i < NUM_ENCODER_JOINTS; ++i) {
        jointAngles[i]         = 0.0f;
        jointAngularVelDps[i]  = 0.0f;
        jointAngularAccDps2[i] = 0.0f;
    }

    for (uint8_t i = 0u; i < NUM_SPEED_ENCODERS; ++i) {
        encoderSpeedRPM[i] = 0.0f;
    }

    timestampUs = 0u;
}

// =============================================================================
// RobotState::copyFrom()
// =============================================================================

/**
 * @brief Memberwise copy from src.  All fields are plain value types so the
 *        default assignment is safe and complete.
 */
void RobotState::copyFrom(const RobotState& src)
{
    *this = src;
}

// =============================================================================
// RobotState::print()
// =============================================================================

/**
 * @brief Writes a single formatted debug line to Serial.
 *        Example output:
 *          [RobotState] t=123456us q=(0.0012,-0.0034,0.7071,0.7071) rpy=(0.39,-0.27,89.94)
 */
void RobotState::print() const
{
    Serial.print(F("[RobotState] t="));
    Serial.print(timestampUs);
    Serial.print(F("us  q=("));
    Serial.print(imuQuaternion.x, 4);
    Serial.print(',');
    Serial.print(imuQuaternion.y, 4);
    Serial.print(',');
    Serial.print(imuQuaternion.z, 4);
    Serial.print(',');
    Serial.print(imuQuaternion.w, 4);
    Serial.print(F(")  rpy=("));
    Serial.print(roll,  2);
    Serial.print(',');
    Serial.print(pitch, 2);
    Serial.print(',');
    Serial.print(yaw,   2);
    Serial.println(')');
}
