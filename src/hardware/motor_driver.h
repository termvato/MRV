/**
 * @file    motor_driver.h
 * @brief   TB6612FNG dual H-bridge driver for arm and spring motors.
 * @details Controls two TB6612 breakout boards:
 *
 *          Arm board (STBY = pin 10):
 *            ARM0 — AIN1=8, AIN2=9, PWMA=5
 *            ARM1 — BIN1=11, BIN2=12, PWMB=6
 *
 *          Spring board (STBY tied to 3.3 V — always on):
 *            SpringA — AIN1=7, AIN2=2, PWMA=3
 *            SpringB — BIN1=2, BIN2=7, PWMB=4  (IN pins tied to SpringA)
 *
 *          Each channel: direction set by IN1/IN2, speed by PWM pin.
 *            forward:  IN1=HIGH, IN2=LOW,  analogWrite(PWM, duty)
 *            reverse:  IN1=LOW,  IN2=HIGH, analogWrite(PWM, duty)
 *            stop:     IN1=LOW,  IN2=LOW,  analogWrite(PWM, 0)
 *
 *          Safety cutoff:
 *            safetyStop() asserts PIN_ARM_STBY LOW (killing arm motors),
 *            coasts the spring motors, and latches the tripped flag so all
 *            subsequent set*() calls are ignored until clearSafety() is called.
 *
 * @inputs  Commands via setArmA(), setArmB(), setSpring().
 * @outputs PWM signals on motor pins; STBY for arm cutoff.
 * @deps    hardware/pinout.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>
#include "hardware/pinout.h"

class MotorDriver {
public:

    /**
     * @brief  Configures all motor pins as outputs, ensures STBY is HIGH
     *         (arm motors enabled), and zeros all motor outputs.
     */
    void begin();

    /**
     * @brief  Sets ARM0 speed and direction.
     * @param  speed  -255 (full reverse) … 0 (coast) … +255 (full forward).
     *                Has no effect while safety is tripped.
     */
    void setArmA(int16_t speed);

    /**
     * @brief  Sets ARM1 speed and direction.  Same semantics as setArmA().
     */
    void setArmB(int16_t speed);

    /**
     * @brief  Sets spring motor speed and direction (SpringA + SpringB together).
     *         Spring board STBY is permanently HIGH so this is always available,
     *         but safetyStop() will coast the spring and block future commands
     *         until clearSafety() is called.
     */
    void setSpring(int16_t speed);

    /**
     * @brief  Cuts all arm drive (STBY LOW), coasts the spring motor, and
     *         latches the safety-tripped flag.
     *         Call clearSafety() to re-enable.
     */
    void safetyStop();

    /**
     * @brief  Re-enables the arm driver (STBY HIGH) and clears the latch.
     *         Motor outputs remain at zero until the next set*() call.
     */
    void clearSafety();

    /** @return true while the safety latch is set. */
    bool isSafetyTripped() const { return _safetyTripped; }

private:

    /**
     * @brief  Drives one H-bridge channel.
     * @param  pinFwd  IN1 pin (HIGH = forward direction).
     * @param  pinRev  IN2 pin (HIGH = reverse direction).
     * @param  pinPWM  Dedicated PWM speed pin.
     * @param  speed   Signed duty cycle [-255, 255].
     */
    static void driveH(uint8_t pinFwd, uint8_t pinRev, uint8_t pinPWM, int16_t speed);

    volatile bool _safetyTripped = false;
};
