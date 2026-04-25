/**
 * @file    main.cpp
 * @brief   Firmware entry point for the MRV robot controller (Teensy 4.0).
 * @details setup() and loop() only — no hardware access, no logic.
 *          All work is delegated to the appropriate subsystem objects:
 *            - IMU          reads the LSM6DSV16X SFLP quaternion.
 *            - StateEstimator  runs at IMU_SAMPLE_RATE_HZ via IntervalTimer,
 *                              populates RobotState each tick.
 *            - SerialReporter  prints RobotState at SERIAL_REPORT_RATE_HZ.
 *            - MotorDriver     drives arm and spring TB6612 H-bridges with
 *                              a safety cutoff triggered by tilt (roll / yaw).
 *            - safetyISR()     IntervalTimer ISR at SAFETY_CHECK_RATE_HZ that
 *                              cuts motors immediately on tilt — independent of
 *                              I2C / serial load in the main loop.
 *
 *          Fault handling: if begin() fails on any subsystem, the firmware
 *          halts and blinks PIN_LED with a distinct pattern so the failure is
 *          visible without a debugger.
 *
 *          Incoming serial commands (from visualiser):
 *            ARM:<a>,<b>\n          — arm motor speeds in MANUAL mode (-255…255)
 *            SPR:<speed>\n          — spring motor speed (-255…255)
 *            SAFE_CLR\n             — re-enable motors after safety trip
 *            CTRL:<0/1/2>\n         — switch mode: 0=Manual, 1=AttitudePID, 2=RevSync
 *            PIDA:<kp>,<ki>,<kd>\n  — Motor A PID gains
 *            PIDB:<kp>,<ki>,<kd>\n  — Motor B PID gains
 *            AXIS:<0/1/2>\n         — PID feedback axis: 0=roll, 1=pitch, 2=yaw
 *            SETP:<deg>\n           — attitude PID setpoint in degrees
 *            REV:<n>,<rpm>\n        — start revolution sync (N revs at RPM)
 *            DIRA:<0/1>\n           — Motor A direction flip
 *            DIRB:<0/1>\n           — Motor B direction flip
 *
 * @inputs  None — all hardware is abstracted behind the subsystem classes.
 * @outputs SerialReporter + safety-event lines "SAFE:1\n" / "SAFE:0\n".
 * @deps    hardware/imu.h, hardware/encoder.h, hardware/motor_driver.h,
 *          hardware/motor_controller.h, hardware/pinout.h,
 *          state/state_estimator.h, comms/serial_reporter.h,
 *          config/config.h, <Arduino.h>
 */

#include <Arduino.h>
#include <Servo.h>

#include "config/config.h"
#include "hardware/pinout.h"
#include "hardware/imu.h"
#include "hardware/encoder.h"
#include "hardware/motor_driver.h"
#include "hardware/motor_controller.h"
#include "state/state_estimator.h"
#include "comms/serial_reporter.h"

// =============================================================================
// Subsystem objects  (static storage — constructed before setup())
// =============================================================================

static IMU             imu;
static JointEncoders   encoders;
static StateEstimator  stateEstimator(imu, encoders);
static SerialReporter  reporter;
static MotorDriver     motors;
static MotorController motorCtrl;

// =============================================================================
// Servo — manually triggered via TAP serial command
// =============================================================================

static Servo s_servo;

/// How long to wait for the servo to physically reach 0° before snapping back (ms).
static constexpr uint32_t kServoTransitMs = 800u;

static bool     s_servoActive  = false;
static uint32_t s_servoTimerMs = 0u;

// =============================================================================
// Safety IntervalTimer — runs at SAFETY_CHECK_RATE_HZ independently of loop()
// =============================================================================

static IntervalTimer     s_safetyTimer;
static volatile bool     s_safetyJustTripped = false;

static void safetyISR()
{
    const RobotState& st = stateEstimator.getState();
    // Trip on physical tilt axes: roll (left/right) and pitch (forward/back).
    // In R/Y/P display mode these show as displayed Roll and displayed Yaw.
    if (fabsf(st.roll) > SAFETY_TILT_DEG || fabsf(st.pitch) > SAFETY_TILT_DEG) {
        if (!motorCtrl.isSafetyTripped()) {
            motorCtrl.safetyStop();
            s_safetyJustTripped = true;
        }
    }
}

// =============================================================================
// Incoming serial command parser
// =============================================================================

namespace {
    char    s_cmdBuf[32];
    uint8_t s_cmdIdx = 0u;

    void parseCommand(const char* cmd)
    {
        // ── ARM: manual arm PWM (only active in MANUAL mode) ──────────────────
        if (strncmp(cmd, "ARM:", 4) == 0) {
            int16_t a = 0, b = 0;
            if (sscanf(cmd + 4, "%hd,%hd", &a, &b) == 2) {
                motorCtrl.setManualA(a);
                motorCtrl.setManualB(b);
            }

        // ── SPR: spring motor (unchanged) ─────────────────────────────────────
        } else if (strncmp(cmd, "SPR:", 4) == 0) {
            int16_t s = 0;
            if (sscanf(cmd + 4, "%hd", &s) == 1) {
                motors.setSpring(s);
            }

        // ── CTRL: switch control mode (0=Manual, 1=AttitudePID, 2=RevSync) ───
        } else if (strncmp(cmd, "CTRL:", 5) == 0) {
            int m = 0;
            if (sscanf(cmd + 5, "%d", &m) == 1) {
                motorCtrl.setMode(static_cast<CtrlMode>(m));
            }

        // ── PIDA: Motor A PID gains <kp>,<ki>,<kd> ────────────────────────────
        } else if (strncmp(cmd, "PIDA:", 5) == 0) {
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            if (sscanf(cmd + 5, "%f,%f,%f", &kp, &ki, &kd) == 3) {
                motorCtrl.setGainsA(kp, ki, kd);
            }

        // ── PIDB: Motor B PID gains <kp>,<ki>,<kd> ────────────────────────────
        } else if (strncmp(cmd, "PIDB:", 5) == 0) {
            float kp = 0.0f, ki = 0.0f, kd = 0.0f;
            if (sscanf(cmd + 5, "%f,%f,%f", &kp, &ki, &kd) == 3) {
                motorCtrl.setGainsB(kp, ki, kd);
            }

        // ── AXIS: PID feedback axis (0=roll, 1=pitch, 2=yaw) ─────────────────
        } else if (strncmp(cmd, "AXIS:", 5) == 0) {
            int ax = 0;
            if (sscanf(cmd + 5, "%d", &ax) == 1) {
                motorCtrl.setPIDAxis(static_cast<uint8_t>(ax));
            }

        // ── SETP: attitude PID setpoint in degrees ────────────────────────────
        } else if (strncmp(cmd, "SETP:", 5) == 0) {
            float deg = 0.0f;
            if (sscanf(cmd + 5, "%f", &deg) == 1) {
                motorCtrl.setAttitudeSetpoint(deg);
            }

        // ── REV: start revolution sync <n_revs>,<rpm> ────────────────────────
        } else if (strncmp(cmd, "REV:", 4) == 0) {
            float nRevs = 0.0f, rpm = 0.0f;
            if (sscanf(cmd + 4, "%f,%f", &nRevs, &rpm) == 2) {
                motorCtrl.startRevSync(rpm, nRevs);
            }

        // ── DIRA / DIRB: per-motor direction flip (0=normal, 1=flipped) ──────
        } else if (strncmp(cmd, "DIRA:", 5) == 0) {
            int f = 0;
            if (sscanf(cmd + 5, "%d", &f) == 1) {
                motorCtrl.setDirFlipA(f != 0);
            }
        } else if (strncmp(cmd, "DIRB:", 5) == 0) {
            int f = 0;
            if (sscanf(cmd + 5, "%d", &f) == 1) {
                motorCtrl.setDirFlipB(f != 0);
            }

        // ── SAFE_CLR: re-enable motors after safety trip ──────────────────────
        } else if (strcmp(cmd, "SAFE_CLR") == 0) {
            motorCtrl.clearSafety();
            Serial.println(F("SAFE:0"));

        // ── TAP: manually trigger servo ───────────────────────────────────────
        } else if (strcmp(cmd, "TAP") == 0) {
            if (!s_servoActive) {
                s_servoActive  = true;
                s_servoTimerMs = millis();
                s_servo.write(0);
                Serial.println(F("TAP: servo -> 0 deg"));
            }
        }
    }

    void readSerialCommands()
    {
        while (Serial.available()) {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r') {
                if (s_cmdIdx > 0u) {
                    s_cmdBuf[s_cmdIdx] = '\0';
                    parseCommand(s_cmdBuf);
                    s_cmdIdx = 0u;
                }
            } else if (s_cmdIdx < static_cast<uint8_t>(sizeof(s_cmdBuf) - 1u)) {
                s_cmdBuf[s_cmdIdx++] = c;
            }
        }
    }
}


// =============================================================================
// Internal helpers
// =============================================================================

/**
 * @brief  Infinite fault loop — blinks PIN_LED at the given period and
 *         repeatedly prints msg to Serial.
 *         Halts all other firmware activity.
 * @param  msg         Error description to print on Serial.
 * @param  blinkMs     Half-period of the LED blink in milliseconds.
 */
static void faultHalt(const char* msg, uint32_t blinkMs)
{
    // Serial may not be open yet; flush whatever we can.
    Serial.println(msg);
    Serial.flush();

    while (true) {
        digitalWrite(PIN_LED, HIGH);
        delay(blinkMs);
        digitalWrite(PIN_LED, LOW);
        delay(blinkMs);
    }
}

// =============================================================================
// setup()
// =============================================================================

void setup()
{
    // ── Status LED ────────────────────────────────────────────────────────────
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // ── Serial ────────────────────────────────────────────────────────────────
    // reporter.begin() calls Serial.begin(); call it before any Serial prints.
    reporter.begin(115200u);

    // Wait up to 3 s for a USB serial connection (useful during development).
    // Remove this block for standalone (non-tethered) deployment.
    const uint32_t serialWaitStart = millis();
    while (!Serial && (millis() - serialWaitStart) < 3000u) { /* wait */ }

    Serial.println(F("MRV firmware starting…"));

    // ── Motor driver ─────────────────────────────────────────────────────────
    motors.begin();
    motorCtrl.begin(motors, encoders);
    Serial.println(F("Motors: OK"));

    // ── Encoders ──────────────────────────────────────────────────────────────
    // Encoder begin() resets the count to zero (power-on position = 0°).
    // It does not halt on failure — the robot can still run with IMU only.
    if (encoders.begin()) {
        Serial.println(F("Encoders: ARM1 active on pins 20/21"));
    } else {
        Serial.println(F("Encoders: begin() failed — joint angles will read 0°"));
    }

    // ── IMU ───────────────────────────────────────────────────────────────────
    if (!imu.begin()) {
        // Short rapid blink (200 ms) = IMU fault.
        faultHalt("FAULT: IMU begin() failed — check wiring and config.h",
                  200u);
    }
    Serial.print(F("IMU: "));
    Serial.println(imu.getStatusString());

    // ── State estimator ───────────────────────────────────────────────────────
    if (!stateEstimator.begin()) {
        // Medium blink (500 ms) = timer resource exhausted.
        faultHalt("FAULT: StateEstimator begin() failed — no IntervalTimer available",
                  500u);
    }
    Serial.print(F("StateEstimator: running at "));
    Serial.print(IMU_SAMPLE_RATE_HZ);
    Serial.println(F(" Hz"));


    // ── Safety monitor ────────────────────────────────────────────────────────
    s_safetyTimer.begin(safetyISR, 1000000u / SAFETY_CHECK_RATE_HZ);
    Serial.println(F("Safety: armed (roll/yaw tilt check at 1 kHz)"));

    // ── Servo ─────────────────────────────────────────────────────────────────
    s_servo.attach(PIN_SERVO);
    s_servo.write(90);
    Serial.println(F("Servo: attached on pin 28, parked at 90 deg"));

    // ── Ready ─────────────────────────────────────────────────────────────────
    digitalWrite(PIN_LED, HIGH);
    Serial.println(F("Setup complete — entering main loop."));
}

// =============================================================================
// loop()
// =============================================================================

void loop()
{
    // ── Parse any incoming motor/safety commands from the visualiser ──────────
    readSerialCommands();

    // ── Notify visualiser of safety trip (deferred from ISR) ─────────────────
    if (s_safetyJustTripped) {
        s_safetyJustTripped = false;
        Serial.println(F("SAFE:1"));
    }

    // ── Process one pending IMU sample (if the IntervalTimer has fired) ───────
    stateEstimator.tick();

    // ── Servo snap-back ───────────────────────────────────────────────────────
    if (s_servoActive && (millis() - s_servoTimerMs >= kServoTransitMs)) {
        s_servo.write(90);
        s_servoActive = false;
        Serial.println(F("TAP: servo -> 90 deg (snap back)"));
    }

    // ── Print the current state to Serial if the report interval has elapsed ──
    const RobotState& st = stateEstimator.getState();
    motorCtrl.tick(st, 1.0f / static_cast<float>(IMU_SAMPLE_RATE_HZ));
    reporter.update(st, motorCtrl);
}
