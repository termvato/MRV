/**
 * @file    pinout.h
 * @brief   All Teensy 4.0 pin assignments for the MRV project.
 * @details Single source of truth for every physical pin number.
 *          No pin numbers appear anywhere else in the codebase —
 *          every driver must import this header instead of using literals.
 * @inputs  None.
 * @outputs constexpr pin-number constants consumed by hardware drivers.
 * @deps    <Arduino.h>  (for uint8_t in the Arduino build environment)
 */

#pragma once

#include <Arduino.h>

// =============================================================================
// IMU — LSM6DSV16X
// =============================================================================

/// INT1 output from the LSM6DSV16X (unused — firmware polls FIFO).
constexpr uint8_t PIN_IMU_INT1   = 2u;

/// SPI chip-select (unused — firmware uses I2C).
constexpr uint8_t PIN_IMU_SPI_CS = 10u;

// =============================================================================
// ARM0 motor — TB6612FNG channel A
// =============================================================================

constexpr uint8_t PIN_ARM0_AIN1 = 8u;   ///< ARM0 direction 1  (TB6612 AIN1)
constexpr uint8_t PIN_ARM0_AIN2 = 9u;   ///< ARM0 direction 2  (TB6612 AIN2)
constexpr uint8_t PIN_ARM0_PWMA = 5u;   ///< ARM0 speed        (TB6612 PWMA)

// =============================================================================
// ARM1 motor — TB6612FNG channel B  (shared STBY with ARM0)
// =============================================================================

constexpr uint8_t PIN_ARM1_BIN1 = 11u;  ///< ARM1 direction 1  (TB6612 BIN1)
constexpr uint8_t PIN_ARM1_BIN2 = 12u;  ///< ARM1 direction 2  (TB6612 BIN2)
constexpr uint8_t PIN_ARM1_PWMB = 6u;   ///< ARM1 speed        (TB6612 PWMB)

constexpr uint8_t PIN_ARM_STBY  = 10u;  ///< Standby shared by ARM0 + ARM1 — LOW disables both

// =============================================================================
// Spring motor A — TB6612FNG channel A
// =============================================================================

constexpr uint8_t PIN_SPRA_AIN1 = 7u;   ///< SpringA direction 1  (TB6612 AIN1)
constexpr uint8_t PIN_SPRA_AIN2 = 2u;   ///< SpringA direction 2  (TB6612 AIN2)
constexpr uint8_t PIN_SPRA_PWMA = 3u;   ///< SpringA speed        (TB6612 PWMA)

// =============================================================================
// Spring motor B — TB6612FNG channel B  (IN pins tied to SpringA)
// =============================================================================

constexpr uint8_t PIN_SPRB_BIN1 = 2u;   ///< SpringB direction 1  (TB6612 BIN1) — tied to PIN_SPRA_AIN2
constexpr uint8_t PIN_SPRB_BIN2 = 7u;   ///< SpringB direction 2  (TB6612 BIN2) — tied to PIN_SPRA_AIN1
constexpr uint8_t PIN_SPRB_PWMB = 4u;   ///< SpringB speed        (TB6612 PWMB)

// =============================================================================
// Joint encoders — quadrature A/B channels
// =============================================================================

constexpr uint8_t PIN_ENC_ARM0_A = 17u;  ///< ARM0 encoder channel A
constexpr uint8_t PIN_ENC_ARM0_B = 16u;  ///< ARM0 encoder channel B

constexpr uint8_t PIN_ENC_ARM1_A = 14u;  ///< ARM1 encoder channel A
constexpr uint8_t PIN_ENC_ARM1_B = 15u;  ///< ARM1 encoder channel B

constexpr uint8_t PIN_ENC_SPRA_A = 20u;  ///< SpringA encoder channel A
constexpr uint8_t PIN_ENC_SPRA_B = 21u;  ///< SpringA encoder channel B

constexpr uint8_t PIN_ENC_SPRB_A = 22u;  ///< SpringB encoder channel A
constexpr uint8_t PIN_ENC_SPRB_B = 23u;  ///< SpringB encoder channel B

// =============================================================================
// General / status
// =============================================================================

constexpr uint8_t PIN_LED = 13u;  ///< Teensy 4.0 onboard LED — startup + fault blink

// =============================================================================
// Servo
// =============================================================================

constexpr uint8_t PIN_SERVO = 28u;  ///< Tap-triggered servo signal output

// =============================================================================
// Voltage sensor — analogue input
// =============================================================================

/// Voltage divider: 24 kΩ top (two 12 kΩ) + 12 kΩ bottom across battery.
/// V_batt = V_pin × 3.  At 8.4 V: V_pin = 2.8 V — within 3.3 V ADC limit.
constexpr uint8_t PIN_VOLTAGE = 26u;
