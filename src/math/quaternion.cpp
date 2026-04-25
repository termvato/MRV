/**
 * @file    quaternion.cpp
 * @brief   Quaternion math implementations.
 * @details Implements the methods declared in quaternion.h.
 *          Uses single-precision intrinsics (sqrtf, atan2f, asinf, fabsf,
 *          copysignf) to take advantage of the Cortex-M7 FPU on Teensy 4.0.
 *          No heap allocation — all operations are pure stack computation.
 * @inputs  Quaternion component values (float).
 * @outputs Modified or derived Quaternion values; Euler angles in degrees.
 * @deps    math/quaternion.h, <math.h>
 */

#include "math/quaternion.h"

// =============================================================================
// Module-private constants
// =============================================================================

/// Minimum squared magnitude below which normalisation is skipped.
/// Prevents division by zero for degenerate (all-zero) quaternions.
static constexpr float kNormEpsilonSq = 1.0e-10f;

/// Conversion factor from radians to degrees.
static constexpr float kRadToDeg = 57.29577951308232f;  // 180 / π

// =============================================================================
// Constructors
// =============================================================================

Quaternion::Quaternion()
    : x(0.0f), y(0.0f), z(0.0f), w(1.0f)
{}

Quaternion::Quaternion(float x_, float y_, float z_, float w_)
    : x(x_), y(y_), z(z_), w(w_)
{}

// =============================================================================
// Mutating operations
// =============================================================================

/**
 * @brief Normalises this quaternion in-place.
 *        Computes |q|² first; skips if below kNormEpsilonSq to avoid NaN.
 */
void Quaternion::normalize()
{
    const float magSq = x*x + y*y + z*z + w*w;
    if (magSq < kNormEpsilonSq) {
        return;  // degenerate — leave unchanged rather than produce NaN
    }
    const float invMag = 1.0f / sqrtf(magSq);
    x *= invMag;
    y *= invMag;
    z *= invMag;
    w *= invMag;
}

// =============================================================================
// Non-mutating operations
// =============================================================================

/**
 * @brief Returns the conjugate  q* = (w, -x, -y, -z).
 */
Quaternion Quaternion::conjugate() const
{
    return Quaternion(-x, -y, -z, w);
}

/**
 * @brief Returns the Hamilton product  p * q.
 *        Formula:  (p.w·q.x + p.x·q.w + p.y·q.z - p.z·q.y,
 *                   p.w·q.y - p.x·q.z + p.y·q.w + p.z·q.x,
 *                   p.w·q.z + p.x·q.y - p.y·q.x + p.z·q.w,
 *                   p.w·q.w - p.x·q.x - p.y·q.y - p.z·q.z)
 */
Quaternion Quaternion::multiply(const Quaternion& rhs) const
{
    return Quaternion(
        w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y,  // i
        w*rhs.y - x*rhs.z + y*rhs.w + z*rhs.x,  // j
        w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w,  // k
        w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z   // scalar
    );
}

// =============================================================================
// Conversion
// =============================================================================

/**
 * @brief Converts this unit quaternion to ZYX intrinsic Euler angles.
 *
 * Standard aerospace / Tait-Bryan convention:
 *   roll  = rotation about X  (bank)
 *   pitch = rotation about Y  (elevation)
 *   yaw   = rotation about Z  (heading)
 *
 * Range: roll [-180,180], pitch [-90,90], yaw [-180,180].
 * Gimbal lock at pitch = ±90°.
 */
void Quaternion::toEuler(float& roll, float& pitch, float& yaw) const
{
    // Roll (X)
    const float sinr_cosp = 2.0f * (w*x + y*z);
    const float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    roll = atan2f(sinr_cosp, cosr_cosp) * kRadToDeg;

    // Pitch (Y) — clamped to avoid NaN from asinf
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    pitch = asinf(sinp) * kRadToDeg;

    // Yaw (Z)
    const float siny_cosp = 2.0f * (w*z + x*y);
    const float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    yaw = atan2f(siny_cosp, cosy_cosp) * kRadToDeg;
}
