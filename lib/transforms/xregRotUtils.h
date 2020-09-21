/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef XREGROTUTILS_H_
#define XREGROTUTILS_H_

/// \file
/// \brief Utilities for dealing with rotations (SO(3)) and rigid body (SE(3)) transformations.

#include "xregCommon.h"

namespace xreg
{

/**
 * @brief Computes a skew matrix given a vector.
 *
 * \f[
    \textrm{skew}(\alpha) = \begin{pmatrix}
                              0 & -\alpha_3 &  \alpha_2 \\
                       \alpha_3 &         0 & -\alpha_1 \\
                      -\alpha_2 &  \alpha_1 &         0
                            \end{pmatrix}
   \f]
 * @param v The input vector of length 3
 * @param m The output 3x3 matrix (previously allocated)
 **/
Mat3x3 SkewMatrix(const Pt3& v);

/**
 * @brief Computes several skew matrices given a set of vectors.
 *
 * If a multi-threading library is available, this executes each
 * skew matrix computation in parallel.
 * @param pts The list of input points
 * @param mats The list of output matrices
 *             (already set to have the appropriate length)
 * @see SkewMatrix
 **/
Mat3x3List SkewMatrices(const Pt3List& pts);

/// \brief Retrieves the vector in R^3 that generates a 3x3 skew-symmetric matrix
///
/// The hat is commonly used on a vector in R^3 to indicate the creation of a
/// skew symmetric matrix, and therefore the "inverse" is the wedge (upside-down hat)
Pt3 WedgeSkew(const Mat3x3& W);

/// \brief Compute the matrix exponential of a point that generates a skew symmetric matrix
///
/// The result is a rotation matrix in SO(3)
Mat3x3 ExpSO3(const Pt3& x);

/// \brief Compute the matrix exponential of a 3x3 skew symmetric matrix
///
/// The result is a rotation matrix in SO(3)
Mat3x3 ExpSO3(const Mat3x3& W);

/// \brief Compute the matrix logarithm from a rotation matrix
///
/// The log here is a point in R^3 (that would generate a skew symmetric matrix in the Lie Algebra)
Pt3 LogSO3ToPt(const Mat3x3& R);

/// \brief SO(3) -> so(3) logarithm, output is 3x3 skew symmetric
Mat3x3 LogSO3ToSkew(const Mat3x3& R);

/// \brief Compute the geodesic length between two rotation matrices
///
/// This is computed as the angle of rotation matrix R1^T * R2
CoordScalar GeodesicLengthRots3x3(const Mat3x3& R1, const Mat3x3& R2);

/// \brief Converts a unit quaternion into a rotation matrix
Mat3x3 QuatToRotMat(const Pt4& q);

/** \brief Rotation matrix representing a rotation about the X-Axis.
 *
 * This calls the Eigen Axis/Angle representation with the axis equal to the X-axis.
 *  \f[
 *     R_x = \begin{pmatrix}
 *            1 &        0       &      0 \           \
 *            0 &  \cos \theta_x & -\sin \theta_x \   \
 *            0 &  \sin \theta_x &  \cos \theta_x
 *           \end{pmatrix}
 *  \f]
 */
Mat3x3 EulerRotX(const CoordScalar theta_rad);

/** \brief Rotation matrix representing a rotation about the Y-Axis.
 *
 * This calls the Eigen Axis/Angle representation with the axis equal to the Y-axis.
 *  \f[
 *     R_y = \begin{pmatrix}
 *           \cos \theta_y  & 0 & \sin \theta_y \ \
 *           0              & 1 & 0 \             \
 *           -\sin \theta_y & 0 & \cos \theta_y
 *         \end{pmatrix}
 *  \f]
 */
Mat3x3 EulerRotY(const CoordScalar theta_rad);

/** \brief Rotation matrix representing a rotation about the Z-Axis.
 *
 * This calls the Eigen Axis/Angle representation with the axis equal to the Z-axis.
 *  \f[
 *  R_z = \begin{pmatrix}
 *          \cos \theta_z & -\sin \theta_z & 0 \  \
 *          \sin \theta_z & \cos \theta_z & 0 \   \
 *               0        &       0       & 1
 *        \end{pmatrix}
 *    \f]
 */
Mat3x3 EulerRotZ(const CoordScalar theta_rad);

/// \brief Rotation matrix representing a series of extrinsic rotations, first
///        about the X-Axis, followed by the Y-Axis, and finally the Z-Axis.
///
/// Returns EulerRotZ(theta_z_rad) * EulerRotY(theta_y_rad) * EulerRotX(theta_x_rad)
/// \see EulerRotX
/// \see EulerRotY
/// \see EulerRotZ
Mat3x3 EulerRotXYZ(const CoordScalar theta_x_rad,
                   const CoordScalar theta_y_rad,
                   const CoordScalar theta_z_rad);

/**
 * @brief Given a rotation matrix, calculate a valid set of X, Y, Z Euler
 * axis rotation angles.
 *
 * @param R The 3x3 rotation matrix
 * @param theta_x The rotation angle about the X-Axis in radians
 * @param theta_y The rotation angle about the Y-Axis in radians
 * @param theta_z The rotation angle about the Z-Axis in radians
 * @see EulerXYZRotMatrix
 **/
std::tuple<CoordScalar,CoordScalar,CoordScalar> RotMatrixToEulerXYZ(const Mat3x3& R);

/// \brief Computes the Frechet variance of a collection of rotation matrices
///        at a reference rotation matrix.
CoordScalar SO3FrechetVar(const Mat3x3List& rots, const Mat3x3& p);

/// \brief Computes an estimate of the Frechet mean rotation matrix from a collection of rotation matrices
///
/// The mean is computed by iteratively projecting onto the tangent space, computing the
/// mean element in the tangent space, exponentiating back to the manifold, etc.
/// When elements in the tangent space are to close to, or on, the cut locus, they
/// are not used to calculate the next mean estimate.
/// \param init_ref_idx indicates the index of the rotation to be used for the initial mean,
///                     when -1, the initial mean is identity, when -2, the initial mean
///                     is the matrix (in rots) that has the smallest total angle
///                     difference between all other matrices in rots (Frechet variance).
Mat3x3 SO3FrechetMean(const Mat3x3List& rots,
                      const int init_ref_idx = 0,
                      const unsigned long max_num_its = 0);

/// \brief Computes a mean of rotation matrices using Quaternions.
///
/// This is done by converting each rotation to Quaternion form, computing the
/// mean quaternion (component-wise under addtion), re-normalizing to have unit
/// norm, and converting back into a rotation matrix.
/// Appears to yield a rotation within 0.1 degrees of the rotation computed
/// with manifold operations, and with approximately the same Frechet variance.
Mat3x3 SO3MeanQuat(const Mat3x3List& rots);

/// \brief Given a general 3x3 matrix, find the closest rotation matrix to it. SVD approach.
Mat3x3 FindClosestRot(const Mat3x3& A);

}  // xreg

#endif
