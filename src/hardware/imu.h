/**
 * @file    imu.h
 * @brief   IMU driver interface for the LSM6DSV16X.
 * @details Wraps the STM32duino LSM6DSV16XSensor Arduino library.
 *          Responsible solely for delivering a normalised quaternion
 *          representing the orientation of the main robot body in world space,
 *          as computed by the chip's onboard SFLP (Sensor Fusion Low Power)
 *          engine.  Knows nothing about robot state, kinematics, or joints.
 * @inputs  I2C bus (Wire) or SPI bus (SPI), selected by USE_IMU_SPI in
 *          config.h.  Physical wiring defined in hardware/pinout.h.
 * @outputs Quaternion (x, y, z, w) via getQuaternion().
 * @deps    STM32duino LSM6DSV16XSensor library (LSM6DSV16XSensor.h),
 *          math/quaternion.h, config/config.h, hardware/pinout.h, <Arduino.h>
 */

#pragma once

#include <Arduino.h>
#include <LSM6DSV16XSensor.h>

#include "math/quaternion.h"
#include "config/config.h"

// =============================================================================
// IMU class
// =============================================================================

/**
 * @class  IMU
 * @brief  Driver for the LSM6DSV16X IMU using the onboard SFLP fusion engine.
 *
 * Lifecycle
 * ---------
 *   1. Construct (does not touch hardware).
 *   2. Call begin() once in setup().
 *   3. Call update() at IMU_SAMPLE_RATE_HZ from the main loop.
 *   4. Retrieve orientation with getQuaternion().
 *
 * Interrupt safety
 * ----------------
 *   No method is ISR-safe — call from main context only.
 */
class IMU {
public:

    /**
     * @brief  Default constructor.  Does not access hardware.
     *         Quaternion initialised to identity (0, 0, 0, 1).
     */
    IMU();

    /**
     * @brief  Initialises the sensor, enables accelerometer and gyroscope,
     *         configures the SFLP engine, and starts FIFO streaming.
     * @return true  on success; false if any configuration step fails.
     *         Call getStatusString() for the specific failure message.
     */
    bool begin();

    /**
     * @brief  Drains the FIFO and stores the most recent game-rotation-vector.
     *         Must be called from the main loop (not from an ISR).
     * @return true  if at least one valid quaternion was read; false otherwise.
     */
    bool update();

    /**
     * @brief  Returns the most recently latched quaternion.
     *         Returns identity if begin() or update() have not succeeded.
     * @return Last valid Quaternion from the SFLP engine.
     */
    Quaternion getQuaternion() const;

    /**
     * @brief  Returns true if the FIFO holds at least one unread sample.
     */
    bool isReady();

    /**
     * @brief  Resets the SFLP engine and re-enables the rotation vector.
     *         Hold the robot stationary for ~1 second after calling.
     *         Blocks for ~1 s.  Do not call from an ISR.
     */
    void calibrate();

    /**
     * @brief  Enables hardware double-tap detection on the accelerometer.
     *         Changes accel ODR to 480 Hz and full-scale to ±8 g (required by
     *         the tap engine).  Call once after begin() in setup().
     * @return true on success.
     */
    bool enableDoubleTap();

    /**
     * @brief  Returns true once when a double-tap event has been detected since
     *         the last call.  Reads and clears the hardware event flag — call
     *         every main loop iteration.
     * @return true if a double tap occurred; false otherwise.
     */
    bool hasDoubleTap();

    /**
     * @brief  Reads the current accelerometer output (polled, not from FIFO).
     *         Returns acceleration in units of g (1 g ≈ 9.81 m/s²).
     * @param  ax  [out] Acceleration along body X axis in g.
     * @param  ay  [out] Acceleration along body Y axis in g.
     * @param  az  [out] Acceleration along body Z axis in g.
     * @return true on success; false if the sensor is not initialised.
     */
    bool getRawAccel(float& ax, float& ay, float& az);

    /**
     * @brief  Returns a human-readable status/error string for Serial debug.
     * @return Descriptive status String.
     */
    String getStatusString() const;

private:

    /// Pointer to the module-static sensor instance (allocated in imu.cpp).
    LSM6DSV16XSensor* _sensorPtr;

    /// Most recently read orientation quaternion.
    Quaternion _quaternion;

    /// True after a successful begin().
    bool _initialized;

    /// Human-readable status updated by every public method.
    String _statusMsg;
};
