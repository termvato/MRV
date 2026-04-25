/**
 * @file    serial_reporter.h
 * @brief   SerialReporter class interface.
 * @details Outputs structured, human-readable Serial text describing the
 *          current RobotState.  The output rate is decoupled from the sample
 *          rate: the reporter checks elapsed time on each update() call and
 *          prints only when the configured interval has passed.
 *
 *          Both rates are defined in config/config.h:
 *            IMU_SAMPLE_RATE_HZ   — how often StateEstimator updates state
 *            SERIAL_REPORT_RATE_HZ — how often SerialReporter prints
 *
 *          The reporter knows nothing about hardware.  It reads a RobotState
 *          and a MotorController (for mode + revolution counters) and writes
 *          to Serial.
 *
 * @inputs  const RobotState& and const MotorController& passed to update().
 * @outputs Formatted text lines on the Arduino Serial port.
 * @deps    state/robot_state.h, config/config.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>

#include "state/robot_state.h"
#include "config/config.h"

// Forward declaration — avoids pulling all motor headers into every translation
// unit that includes serial_reporter.h.  The full definition is included in
// serial_reporter.cpp where update() is implemented.
class MotorController;

// =============================================================================
// SerialReporter class
// =============================================================================

/**
 * @class  SerialReporter
 * @brief  Rate-limited Serial output of RobotState.
 *
 * Output format (one line per report interval):
 *   T:<us> | Q: <x> <y> <z> <w> | RPY: <roll> <pitch> <yaw> | J: <j0>..<j5>
 *
 * Usage
 * -----
 *   SerialReporter reporter;
 *   reporter.begin(115200);        // in setup()
 *   reporter.update(state);        // in loop() — rate-limited internally
 */
class SerialReporter {
public:

    /**
     * @brief  Default constructor.  Sets the last-report timestamp to zero
     *         so the first update() call always prints immediately.
     */
    SerialReporter();

    /**
     * @brief  Initialises the Serial port at the given baud rate and prints
     *         a header line identifying the output fields.
     * @param  baudRate  Baud rate to pass to Serial.begin().
     *                   Should match monitor_speed in platformio.ini.
     */
    void begin(uint32_t baudRate);

    /**
     * @brief  Prints a formatted RobotState snapshot plus motor-controller
     *         status if the report interval has elapsed.  Safe to call every
     *         loop() iteration.
     * @param  state  The current robot state to report.
     * @param  ctrl   MotorController to read mode and revolution counters from.
     *                Appends "| M:<mode>,<revA>,<revB>" to the output line.
     */
    void update(const RobotState& state, const MotorController& ctrl);

private:

    /// Microsecond timestamp of the most recent Serial print.
    uint32_t _lastReportUs;

    /// Report interval in microseconds, derived from SERIAL_REPORT_RATE_HZ.
    /// constexpr so the division is resolved at compile time.
    static constexpr uint32_t kReportIntervalUs =
        1000000u / static_cast<uint32_t>(SERIAL_REPORT_RATE_HZ);
};
