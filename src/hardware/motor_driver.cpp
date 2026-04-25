/**
 * @file    motor_driver.cpp
 * @brief   TB6612FNG H-bridge driver implementation.
 * @deps    hardware/motor_driver.h, hardware/pinout.h, <Arduino.h>
 */

#include "hardware/motor_driver.h"
#include "hardware/pinout.h"

// =============================================================================
// MotorDriver::begin()
// =============================================================================

void MotorDriver::begin()
{
    // ARM0 pins
    pinMode(PIN_ARM0_AIN1, OUTPUT);
    pinMode(PIN_ARM0_AIN2, OUTPUT);
    pinMode(PIN_ARM0_PWMA, OUTPUT);

    // ARM1 pins
    pinMode(PIN_ARM1_BIN1, OUTPUT);
    pinMode(PIN_ARM1_BIN2, OUTPUT);
    pinMode(PIN_ARM1_PWMB, OUTPUT);

    // Shared arm STBY
    pinMode(PIN_ARM_STBY, OUTPUT);

    // Spring direction pins (SpringA AIN1/AIN2; SpringB BIN1/BIN2 are tied)
    pinMode(PIN_SPRA_AIN1, OUTPUT);
    pinMode(PIN_SPRA_AIN2, OUTPUT);

    // Spring PWM pins (separate per channel)
    pinMode(PIN_SPRA_PWMA, OUTPUT);
    pinMode(PIN_SPRB_PWMB, OUTPUT);

    // Start all motors stopped, STBY high (arm driver active)
    digitalWrite(PIN_ARM0_AIN1, LOW);
    digitalWrite(PIN_ARM0_AIN2, LOW);
    analogWrite (PIN_ARM0_PWMA, 0);

    digitalWrite(PIN_ARM1_BIN1, LOW);
    digitalWrite(PIN_ARM1_BIN2, LOW);
    analogWrite (PIN_ARM1_PWMB, 0);

    digitalWrite(PIN_ARM_STBY, HIGH);

    digitalWrite(PIN_SPRA_AIN1, LOW);
    digitalWrite(PIN_SPRA_AIN2, LOW);
    analogWrite (PIN_SPRA_PWMA, 0);
    analogWrite (PIN_SPRB_PWMB, 0);

    _safetyTripped = false;
}

// =============================================================================
// Private helper — direction + PWM on one H-bridge channel
// =============================================================================

void MotorDriver::driveH(uint8_t pinFwd, uint8_t pinRev, uint8_t pinPWM, int16_t speed)
{
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(pinFwd, HIGH);
        digitalWrite(pinRev, LOW);
        analogWrite (pinPWM, static_cast<uint8_t>(speed));
    } else if (speed < 0) {
        digitalWrite(pinFwd, LOW);
        digitalWrite(pinRev, HIGH);
        analogWrite (pinPWM, static_cast<uint8_t>(-speed));
    } else {
        // Brake: both IN pins HIGH = active short-circuit stop (TB6612 brake mode)
        digitalWrite(pinFwd, HIGH);
        digitalWrite(pinRev, HIGH);
        analogWrite (pinPWM, 0);
    }
}

// =============================================================================
// Public motor setters
// =============================================================================

void MotorDriver::setArmA(int16_t speed)
{
    if (_safetyTripped) return;
    driveH(PIN_ARM0_AIN1, PIN_ARM0_AIN2, PIN_ARM0_PWMA, speed);
}

void MotorDriver::setArmB(int16_t speed)
{
    if (_safetyTripped) return;
    driveH(PIN_ARM1_BIN1, PIN_ARM1_BIN2, PIN_ARM1_PWMB, speed);
}

void MotorDriver::setSpring(int16_t speed)
{
    if (_safetyTripped) return;
    // IN pins are tied: AIN1=BIN2=7, AIN2=BIN1=2.
    // Set direction once via SpringA pins, then drive both PWM channels.
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_SPRA_AIN1, HIGH);
        digitalWrite(PIN_SPRA_AIN2, LOW);
        analogWrite (PIN_SPRA_PWMA, static_cast<uint8_t>(speed));
        analogWrite (PIN_SPRB_PWMB, static_cast<uint8_t>(speed));
    } else if (speed < 0) {
        digitalWrite(PIN_SPRA_AIN1, LOW);
        digitalWrite(PIN_SPRA_AIN2, HIGH);
        analogWrite (PIN_SPRA_PWMA, static_cast<uint8_t>(-speed));
        analogWrite (PIN_SPRB_PWMB, static_cast<uint8_t>(-speed));
    } else {
        digitalWrite(PIN_SPRA_AIN1, LOW);
        digitalWrite(PIN_SPRA_AIN2, LOW);
        analogWrite (PIN_SPRA_PWMA, 0);
        analogWrite (PIN_SPRB_PWMB, 0);
    }
}

// =============================================================================
// Safety cutoff
// =============================================================================

void MotorDriver::safetyStop()
{
    _safetyTripped = true;
    digitalWrite(PIN_ARM_STBY, LOW);   // kills both arm channels instantly
    // Brake spring motors
    digitalWrite(PIN_SPRA_AIN1, HIGH);
    digitalWrite(PIN_SPRA_AIN2, HIGH);
    analogWrite (PIN_SPRA_PWMA, 0);
    analogWrite (PIN_SPRB_PWMB, 0);
}

void MotorDriver::clearSafety()
{
    _safetyTripped = false;
    digitalWrite(PIN_ARM_STBY, HIGH);  // re-enable arm driver; outputs stay zero
}
