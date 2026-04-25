/**
 * @file    serial_reporter.cpp
 * @brief   SerialReporter implementation.
 * @details Prints timestamped, labelled RobotState snapshots to Serial at a
 *          configurable rate.  Uses elapsed-micros gating — no extra timer
 *          resource required.  All output uses F() macro to keep string
 *          literals in flash rather than SRAM.
 * @inputs  RobotState passed to update(); elapsed micros() for rate gating.
 * @outputs Formatted ASCII lines on Serial.
 * @deps    comms/serial_reporter.h, state/robot_state.h, config/config.h,
 *          <Arduino.h>
 */

#include "comms/serial_reporter.h"
#include "hardware/motor_controller.h"
#include "hardware/pinout.h"

// =============================================================================
// Constructor
// =============================================================================

/**
 * @brief Initialises the last-report timestamp to zero so the first update()
 *        prints immediately (before one full interval has elapsed).
 */
SerialReporter::SerialReporter()
    : _lastReportUs(0u)
{}

// =============================================================================
// SerialReporter::begin()
// =============================================================================

/**
 * @brief Opens the Serial port and prints a column-header line.
 */
void SerialReporter::begin(uint32_t baudRate)
{
    Serial.begin(baudRate);

    // Print a fixed header so log files are self-describing.
    Serial.println(F("=== MRV Firmware — Serial Reporter ==="));
    Serial.print(F("Sample rate : "));
    Serial.print(IMU_SAMPLE_RATE_HZ);
    Serial.println(F(" Hz"));
    Serial.print(F("Report rate : "));
    Serial.print(SERIAL_REPORT_RATE_HZ);
    Serial.println(F(" Hz"));
    Serial.println(F("Format: T:<us> | Q:<x>,<y>,<z>,<w> | RPY:<r>,<p>,<y> deg | J:<j0>,<j1>,<j2> deg | S:<rpm0>,<rpm1> | K:<vel_dps>,<acc_dps2> | M:<mode>,<revA>,<revB> | V:<volts>"));
    Serial.println(F("----------------------------------------------------------------------"));
}

// =============================================================================
// SerialReporter::update()
// =============================================================================

/**
 * @brief Rate-gated print of the current RobotState.
 *
 * Unsigned subtraction handles micros() wraparound correctly (after ~70 min).
 * Serial.print() calls are sequenced to keep each field labelled and aligned.
 */
void SerialReporter::update(const RobotState& state, const MotorController& ctrl)
{
    const uint32_t now = micros();
    if ((now - _lastReportUs) < kReportIntervalUs) {
        return;
    }
    _lastReportUs = now;

    // ── Timestamp ─────────────────────────────────────────────────────────────
    Serial.print(F("T:"));
    Serial.print(state.timestampUs);

    // ── Quaternion ────────────────────────────────────────────────────────────
    Serial.print(F(" | Q:"));
    Serial.print(state.imuQuaternion.x, 5);
    Serial.print(',');
    Serial.print(state.imuQuaternion.y, 5);
    Serial.print(',');
    Serial.print(state.imuQuaternion.z, 5);
    Serial.print(',');
    Serial.print(state.imuQuaternion.w, 5);

    // ── Euler angles ──────────────────────────────────────────────────────────
    Serial.print(F(" | RPY:"));
    Serial.print(state.roll,  2);
    Serial.print(',');
    Serial.print(state.pitch, 2);
    Serial.print(',');
    Serial.print(state.yaw,   2);

    // ── Joint angles ──────────────────────────────────────────────────────────
    Serial.print(F(" | J:"));
    for (uint8_t i = 0u; i < NUM_ENCODER_JOINTS; ++i) {
        Serial.print(state.jointAngles[i], 1);
        if (i < (NUM_ENCODER_JOINTS - 1u)) {
            Serial.print(',');
        }
    }

    // ── ARM(1) kinematics — angular velocity and acceleration ─────────────────
    // Index 1 = ARM(1), the only joint with a position encoder currently.
    Serial.print(F(" | K:"));
    Serial.print(state.jointAngularVelDps[1],  2);
    Serial.print(',');
    Serial.print(state.jointAngularAccDps2[1], 2);

    // ── Speed encoder RPM ─────────────────────────────────────────────────────
    Serial.print(F(" | S:"));
    for (uint8_t i = 0u; i < NUM_SPEED_ENCODERS; ++i) {
        Serial.print(state.encoderSpeedRPM[i], 1);
        if (i < (NUM_SPEED_ENCODERS - 1u)) {
            Serial.print(',');
        }
    }

    // ── Motor controller status ────────────────────────────────────────────────
    Serial.print(F(" | M:"));
    Serial.print(static_cast<uint8_t>(ctrl.getMode()));
    Serial.print(',');
    Serial.print(ctrl.getRevCountA(), 2);
    Serial.print(',');
    Serial.print(ctrl.getRevCountB(), 2);

    // ── Voltage (pin 26, A12) ──────────────────────────────────────────────────
    // Voltage divider: 24 kΩ top (two 12 kΩ in series) + 12 kΩ bottom, tap at bottom.
    // V_pin = V_batt * 12 / 36 = V_batt / 3  →  V_batt = V_pin * 3
    // At 8.4 V: V_pin = 2.8 V — safely within the 3.3 V Teensy ADC limit.
    // ADC: 10-bit (0–1023), 3.3 V reference.
    const float voltageV = static_cast<float>(analogRead(PIN_VOLTAGE)) * (9.9f / 1023.0f);
    Serial.print(F(" | V:"));
    Serial.print(voltageV, 3);

    Serial.println();
}
