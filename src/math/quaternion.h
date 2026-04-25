/**
 * @file    quaternion.h
 * @brief   Quaternion struct and associated math helper functions.
 * @details Provides a unit-quaternion representation in scalar-last form
 *          (x, y, z, w), along with normalisation, conjugation, Hamilton
 *          multiplication, and conversion to ZYX Euler angles.
 *          All arithmetic uses single-precision float to match the format
 *          output by the LSM6DSV16X SFLP engine.
 *          This module is pure math — it has no hardware or Arduino dependencies.
 * @inputs  Quaternion component values (float).
 * @outputs Derived Quaternion values; Euler angles in degrees (float).
 * @deps    <math.h>
 */

#pragma once

#include <math.h>

// =============================================================================
// Quaternion struct
// =============================================================================

/**
 * @struct  Quaternion
 * @brief   Unit quaternion  q = w + xi + yj + zk  in scalar-last storage.
 *
 * Convention
 * ----------
 * Scalar-last: fields are ordered (x, y, z, w).
 * The LSM6DSV16X SFLP engine outputs a "game rotation vector" — three
 * half-precision floats (i, j, k).  The scalar component w is NOT stored
 * on-chip and must be recovered as  w = sqrt(max(0, 1 - x² - y² - z²)).
 * The IMU driver performs this recovery before populating this struct.
 *
 * Tilt convention
 * ---------------
 * toEuler() uses a gravity-vector projection, not ZYX Euler decomposition.
 * This eliminates heading drift in yaw and gimbal lock at ±90° pitch.
 * See toEuler() docs for the full definition of each output angle.
 */
struct Quaternion {

    float x;  ///< i (vector) component
    float y;  ///< j (vector) component
    float z;  ///< k (vector) component
    float w;  ///< scalar component

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /**
     * @brief  Default constructor.  Initialises to the identity quaternion
     *         (no rotation): x=0, y=0, z=0, w=1.
     */
    Quaternion();

    /**
     * @brief  Parameterised constructor.
     * @param  x_  i component
     * @param  y_  j component
     * @param  z_  k component
     * @param  w_  scalar component
     */
    Quaternion(float x_, float y_, float z_, float w_);

    // -------------------------------------------------------------------------
    // Mutating operations
    // -------------------------------------------------------------------------

    /**
     * @brief  Normalises this quaternion in-place so that |q| = 1.
     * @note   No-op if the squared magnitude is below the internal epsilon
     *         (degenerate quaternion).  Should be called after any arithmetic
     *         that may accumulate floating-point drift.
     */
    void normalize();

    // -------------------------------------------------------------------------
    // Non-mutating operations (return new Quaternion)
    // -------------------------------------------------------------------------

    /**
     * @brief  Returns the conjugate  q* = w - xi - yj - zk.
     *         For a unit quaternion, the conjugate equals the inverse.
     * @return Conjugate quaternion.
     */
    Quaternion conjugate() const;

    /**
     * @brief  Returns the Hamilton product  (this) * rhs.
     *         Multiplication is non-commutative: p * q ≠ q * p in general.
     * @param  rhs  Right-hand operand.
     * @return Product quaternion (not automatically normalised).
     */
    Quaternion multiply(const Quaternion& rhs) const;

    // -------------------------------------------------------------------------
    // Conversion
    // -------------------------------------------------------------------------

    /**
     * @brief  Converts this unit quaternion to gravity-referenced tilt angles.
     *         Uses a gravity-vector projection — no ZYX decomposition, no gimbal
     *         lock, no heading drift.
     *
     * @param  roll   [out] Tilt around body X, degrees, range [-180, 180].
     *                      Gravity-stable; does not drift.
     * @param  pitch  [out] Free-spinning axis angle, degrees, range [-180, 180].
     *                      May drift or spin freely; do not rely on its absolute value.
     * @param  yaw    [out] Total tilt from vertical, degrees, range [0, 180].
     *                      0° = perfectly upright, 180° = fully inverted.
     *                      Heading-free; used for the safety cutoff.
     *
     * @note   Assumes a normalised input quaternion.
     */
    void toEuler(float& roll, float& pitch, float& yaw) const;
};
