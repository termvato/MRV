/**
 * @file    imu.cpp
 * @brief   IMU driver implementation using the STM32duino LSM6DSV16XSensor library.
 * @details Configures the LSM6DSV16X SFLP engine for game-rotation-vector output
 *          via the FIFO, and exposes a clean Quaternion interface.
 *
 *          API notes (verified against LSM6DSV16XSensor.h v2.0.3):
 *            - Class:    LSM6DSV16XSensor
 *            - SFLP on:  Enable_Rotation_Vector()  (also sets FIFO batch flag)
 *            - SFLP off: Disable_Rotation_Vector()
 *            - Reset:    Reset_SFLP()
 *            - FIFO read: FIFO_Get_Rotation_Vector(float[4])
 *                         rvec[0]=x, rvec[1]=y, rvec[2]=z, rvec[3]=w
 *                         (w is computed inside the library via sflp2q)
 *            - FIFO tag:  LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG (0x13)
 *            - FIFO mode: LSM6DSV16X_STREAM_MODE passed as uint8_t
 *            - Watermark: FIFO_Set_Watermark_Level(uint8_t)
 *
 * @inputs  I2C bus (Wire) or SPI (SPI) per USE_IMU_SPI in config.h.
 * @outputs Quaternion via IMU::getQuaternion().
 * @deps    hardware/imu.h, hardware/pinout.h, config/config.h, math/quaternion.h,
 *          LSM6DSV16XSensor library, <Wire.h> or <SPI.h>
 */

#include "hardware/imu.h"
#include "hardware/pinout.h"
#include "config/config.h"

// =============================================================================
// Module-level sensor instance  (static storage — no heap allocation)
// =============================================================================

#ifdef USE_IMU_SPI
    #include <SPI.h>
    static LSM6DSV16XSensor s_sensor(&SPI, PIN_IMU_SPI_CS);
#else
    #include <Wire.h>
    // IMU_I2C_SA0 = 0 → LSM6DSV16X_I2C_ADD_L (0x6A)
    // IMU_I2C_SA0 = 1 → LSM6DSV16X_I2C_ADD_H (0x6B)  ← SparkFun board default
    static LSM6DSV16XSensor s_sensor(
        &Wire,
        (IMU_I2C_SA0 == 0u) ? LSM6DSV16X_I2C_ADD_L : LSM6DSV16X_I2C_ADD_H
    );
#endif

// FIFO tag for SFLP game rotation vector.
// Value 0x13 is defined as LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG inside
// an anonymous enum within a C typedef struct in lsm6dsv16x_reg.h.  That enum
// is not accessible from C++ code by name, so we use the raw value directly.
static constexpr uint8_t kTagGameRv = 0x13u;

// =============================================================================
// Constructor
// =============================================================================

IMU::IMU()
    : _sensorPtr(&s_sensor),
      _quaternion(),              // identity: (0, 0, 0, 1)
      _initialized(false),
      _statusMsg(F("Not initialised — call begin()"))
{}

// =============================================================================
// IMU::begin()
// =============================================================================

bool IMU::begin()
{
    _initialized = false;

    // ── 1. Start the bus ─────────────────────────────────────────────────────
#ifndef USE_IMU_SPI
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);
#endif

    // ── 2. Verify sensor connection (WHO_AM_I check) ─────────────────────────
    if (_sensorPtr->begin() != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: begin() failed — check wiring and I2C address (SA0)");
        return false;
    }

    // ── 3. Enable accelerometer and gyroscope (required by SFLP) ─────────────
    if (_sensorPtr->Enable_X() != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Enable_X() failed");
        return false;
    }
    if (_sensorPtr->Set_X_ODR(IMU_ACCEL_ODR_HZ) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Set_X_ODR() failed");
        return false;
    }
    if (_sensorPtr->Set_X_FS(static_cast<int32_t>(IMU_ACCEL_FS_G)) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Set_X_FS() failed");
        return false;
    }
    if (_sensorPtr->Enable_G() != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Enable_G() failed");
        return false;
    }
    if (_sensorPtr->Set_G_ODR(IMU_GYRO_ODR_HZ) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Set_G_ODR() failed");
        return false;
    }
    if (_sensorPtr->Set_G_FS(static_cast<int32_t>(IMU_GYRO_FS_DPS)) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Set_G_FS() failed");
        return false;
    }

    // ── 4. Set SFLP output data rate ─────────────────────────────────────────
    if (_sensorPtr->Set_SFLP_ODR(SFLP_ODR_HZ) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Set_SFLP_ODR() failed — verify SFLP_ODR_HZ is 15/30/60/120");
        return false;
    }

    // ── 5. Enable SFLP rotation vector ───────────────────────────────────────
    // Enable_Rotation_Vector() does two things in one call:
    //   a) Sets fifo_sflp.game_rotation = 1  → batch quaternion into FIFO
    //   b) Calls lsm6dsv16x_sflp_game_rotation_set(ENABLE) → starts the engine
    if (_sensorPtr->Enable_Rotation_Vector() != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: Enable_Rotation_Vector() failed");
        return false;
    }

    // ── 6. Set FIFO watermark (uint8_t) ──────────────────────────────────────
    if (_sensorPtr->FIFO_Set_Watermark_Level(
            static_cast<uint8_t>(IMU_FIFO_WATERMARK)) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: FIFO_Set_Watermark_Level() failed");
        return false;
    }

    // ── 7. Start FIFO in continuous (stream) mode ─────────────────────────────
    // LSM6DSV16X_STREAM_MODE = 6: overwrites oldest sample when full,
    // so getQuaternion() always returns the freshest orientation.
    if (_sensorPtr->FIFO_Set_Mode(
            static_cast<uint8_t>(LSM6DSV16X_STREAM_MODE)) != LSM6DSV16X_OK) {
        _statusMsg = F("ERROR: FIFO_Set_Mode() failed");
        return false;
    }

    _initialized = true;
    _statusMsg   = "OK: LSM6DSV16X SFLP active at "
                   + String(static_cast<int>(SFLP_ODR_HZ)) + " Hz";
    return true;
}

// =============================================================================
// IMU::update()
// =============================================================================

bool IMU::update()
{
    if (!_initialized) {
        return false;
    }

    uint16_t numSamples = 0u;
    if (_sensorPtr->FIFO_Get_Num_Samples(&numSamples) != LSM6DSV16X_OK) {
        return false;
    }
    if (numSamples == 0u) {
        return false;
    }

    bool gotQuaternion = false;

    for (uint16_t i = 0u; i < numSamples; ++i) {

        uint8_t tag = 0u;
        if (_sensorPtr->FIFO_Get_Tag(&tag) != LSM6DSV16X_OK) {
            break;
        }

        if (tag == kTagGameRv) {
            // FIFO_Get_Rotation_Vector reads 6 bytes and calls sflp2q() internally:
            //   rvec[0]=x, rvec[1]=y, rvec[2]=z, rvec[3]=w  (w already computed)
            float rvec[4] = {0.0f, 0.0f, 0.0f, 1.0f};
            if (_sensorPtr->FIFO_Get_Rotation_Vector(rvec) == LSM6DSV16X_OK) {
                _quaternion   = Quaternion(rvec[0], rvec[1], rvec[2], rvec[3]);
                gotQuaternion = true;
                // Do NOT break — drain to the last (freshest) sample.
            }
        } else {
            // Unknown tag: consume the 6-byte payload to advance the FIFO pointer.
            uint8_t dummy[6];
            _sensorPtr->FIFO_Get_Data(dummy);
        }
    }

    return gotQuaternion;
}

// =============================================================================
// IMU::getQuaternion()
// =============================================================================

Quaternion IMU::getQuaternion() const
{
    return _quaternion;
}

// =============================================================================
// IMU::isReady()
// =============================================================================

bool IMU::isReady()
{
    if (!_initialized) {
        return false;
    }
    uint16_t numSamples = 0u;
    return (_sensorPtr->FIFO_Get_Num_Samples(&numSamples) == LSM6DSV16X_OK)
           && (numSamples > 0u);
}

// =============================================================================
// IMU::calibrate()
// =============================================================================

void IMU::calibrate()
{
    if (!_initialized) {
        return;
    }
    // Reset_SFLP() writes to the embedded function init register, resetting
    // the bias integrator.  Re-enabling the rotation vector restarts the engine.
    _statusMsg = F("Calibrating — hold robot stationary for 1 s…");
    _sensorPtr->Reset_SFLP();
    delay(50u);
    _sensorPtr->Enable_Rotation_Vector();
    delay(1000u);
    _statusMsg = F("OK: SFLP reset and re-initialised");
}

// =============================================================================
// IMU::enableDoubleTap()
// =============================================================================

bool IMU::enableDoubleTap()
{
    if (!_initialized) {
        return false;
    }
    // Enable_Double_Tap_Detection() sets accel ODR to 480 Hz and FS to ±8 g
    // internally — these are required by the hardware tap engine.
    // Route event to INT1 pin (wired to pin 2); we poll status so the pin
    // itself is not strictly required.
    return _sensorPtr->Enable_Double_Tap_Detection(LSM6DSV16X_INT1_PIN)
           == LSM6DSV16X_OK;
}

// =============================================================================
// IMU::hasDoubleTap()
// =============================================================================

bool IMU::hasDoubleTap()
{
    if (!_initialized) {
        return false;
    }
    LSM6DSV16X_Event_Status_t status;
    if (_sensorPtr->Get_X_Event_Status(&status) != LSM6DSV16X_OK) {
        return false;
    }
    return status.DoubleTapStatus != 0u;
}

// =============================================================================
// IMU::getRawAccel()
// =============================================================================

bool IMU::getRawAccel(float& ax, float& ay, float& az)
{
    if (!_initialized) {
        return false;
    }
    // Get_X_Axes() returns acceleration in mg (milligravity).
    int32_t accelData[3] = {0, 0, 0};
    if (_sensorPtr->Get_X_Axes(accelData) != LSM6DSV16X_OK) {
        return false;
    }
    ax = static_cast<float>(accelData[0]) * 0.001f;   // mg → g
    ay = static_cast<float>(accelData[1]) * 0.001f;
    az = static_cast<float>(accelData[2]) * 0.001f;
    return true;
}

// =============================================================================
// IMU::getStatusString()
// =============================================================================

String IMU::getStatusString() const
{
    return _statusMsg;
}
