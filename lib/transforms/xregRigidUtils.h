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

#ifndef XREGRIGIDUTILS_H_
#define XREGRIGIDUTILS_H_

#include "xregCommon.h"

namespace xreg
{

/// \brief Inverse of SE(3) (rigid) transformation without using matrix inverse
///
/// Uses the fact that R^-1 = R^T
Mat4x4 SE3Inv(const Mat4x4& T);

/// \brief se(3) -> SE(3) exponential
///
/// Maps 4x4 matrix with 4th row of zeros and left 3x3 block skew symmetric from
/// the Lie algebra into the Lie group.
Mat4x4 ExpSE3(const Mat4x4& M);

/// \brief se(3) 6D vector -> SE(3) exponential
///
/// First maps the 6D point to a 4x4 matrix in the se(3) Lie Algebra and then
/// to the corresponding element in the SE(3) Lie Group.
Mat4x4 ExpSE3(const Pt6& x);

/// \brief SE(3) -> se(3) logarithm
///
/// Maps 4x4 rigid transform matrices, (4th row equal to 0 0 0 1, and the left
/// 3x3 block a rotation matrix), from the Lie group into the Lie algebra.
Mat4x4 LogSE3ToMat4x4(const Mat4x4& T);

/// \brief Computes a weighted distance between two elements in SE(3)
///
/// Computes delta T as T1^-1 * T2, and let alpha be the rotation angle of
/// delta T, and x be delta T's translation. The distance is computed as
/// w_R * alpha + w_t * ||x||_2
CoordScalar WeightedLengthSE3Mats(const Mat4x4& T1, const Mat4x4& T2,
                                  const CoordScalar w_R = 1, const CoordScalar w_t = 1);

/// \brief Compute the Frechet variance at a point in SE(3) relative to a
///        collection of other SE(3) objects using a weighted distance metric
///        on SE(3).
///
/// \see WeightedLengthSE3Mats
CoordScalar SE3FrechetVarWeightedLength(const Mat4x4List& mats, const Mat4x4& m);

/// \brief Computes an estimate of the Frechet mean of SE(3) elements.
///
/// Starting from a fixed reference point on SE(3) this iteratively computes the
/// mean in the tangent space (se(3)), exponentiates the tangent space mean back
/// to the manifold and uses it as the new reference point on SE(3).
/// Similar to computation of a mean rotation, this will ignore elements too
/// close, or on, the cut locus.
/// The init_ref_idx argument is behaves exactly the same as that from
/// MeanRotMatrixManifold.
Mat4x4 SE3FrechetMean(const Mat4x4List& xforms,
                      const int init_ref_idx = 0,
                      const unsigned long max_num_its = 0);

/// \brief Retrieve the rotation angle and and translation magnitude from a
///        rigid transformation.
std::tuple<CoordScalar,CoordScalar>
ComputeRotAngTransMag(const FrameTransform& xform);

/// \brief Compute the angular and translational components of a difference between two frames.
///
/// The measurements are with respect to the space that both frames map to.
/// e.g. computes the rotation angle and translation magnitude of f1 * inv(f2)
std::tuple<CoordScalar,CoordScalar>
FrameDiffRotAngTransMag(const FrameTransform& f1, const FrameTransform& f2);

/// \brief Compute 4x4 homogenous transform matrix consisting of only a translation along the x-axis
Mat4x4 TransX4x4(const CoordScalar x);

/// \brief Compute 4x4 homogenous transform matrix consisting of only a translation along the y-axis
Mat4x4 TransY4x4(const CoordScalar x);

/// \brief Compute 4x4 homogenous transform matrix consisting of only a translation along the z-axis
Mat4x4 TransZ4x4(const CoordScalar x);

/// \brief Compute 4x4 homogenous transform matrix consisting of only a translation
Mat4x4 TransXYZ4x4(const Pt3& x);

/// \brief Compute 4x4 homogenous transform matrix consisting of only a rotation about the x-axis
Mat4x4 EulerRotX4x4(const CoordScalar theta_rad);


/// \brief Compute 4x4 homogenous transform matrix consisting of only a rotation about the y-axis
Mat4x4 EulerRotY4x4(const CoordScalar theta_rad);


/// \brief Compute 4x4 homogenous transform matrix consisting of only a rotation about the z-axis
Mat4x4 EulerRotZ4x4(const CoordScalar theta_rad);


/// \brief Homogeneous 4x4 (rigid) transformation matrix (in SE(3)) representing
///        a series of extrinsic Euler rotations, followed by a translation.
///
/// Returns [R t; 0 1] where R = EulerRotXYZ(theta_x_rad, theta_y_rad, theta_z_rad) and is in SO(3),
/// and t = [trans_x; trans_y; trans_z]
Mat4x4 EulerRotXYZTransXYZ(const CoordScalar theta_x_rad,
                           const CoordScalar theta_y_rad,
                           const CoordScalar theta_z_rad,
                           const CoordScalar trans_x,
                           const CoordScalar trans_y,
                           const CoordScalar trans_z);

/// \brief Rigid transformation object (in SE(3)) representing a series of
///        extrinsic Euler rotations, followed by a translation.
///
/// Returns the transformation object that encapsulates the 4x4 matrix returned
/// from EulerRotXYZTransXYZ(theta_x_rad, theta_y_rad, theta_z_rad, trans_x, trans_y, trans_z).
FrameTransform EulerRotXYZTransXYZFrame(const CoordScalar theta_x_rad,
                                        const CoordScalar theta_y_rad,
                                        const CoordScalar theta_z_rad,
                                        const CoordScalar trans_x,
                                        const CoordScalar trans_y,
                                        const CoordScalar trans_z);


/// \brief Computes the frame transformation object representing a rotation about
///        the X-Axis.
FrameTransform EulerRotXFrame(const CoordScalar rot_ang_rad);

/// \brief Computes the frame transformation object representing a rotation about
///        the Y-Axis.
FrameTransform EulerRotYFrame(const CoordScalar rot_ang_rad);

/// \brief Computes the frame transformation object representing a rotation about
///        the Z-Axis.
FrameTransform EulerRotZFrame(const CoordScalar rot_ang_rad);

/**
 * @brief Given a rigid transformation object, compute the X, Y, Z Euler axis
 *        rotation agnles and translation componenets.
 *
 * @param xform The existing rigid transformation object
 * @param theta_x The rotation angle about the X-Axis in radians to populate
 * @param theta_y The rotation angle about the Y-Axis in radians to populate
 * @param theta_z The rotation angle about the Z-Axis in radians to populate
 * @param trans_x The translation vector component parallel to the X-Axis to populate
 * @param trans_y The translation vector component parallel to the Y-Axis to populate
 * @param trans_z The translation vector component parallel to the Z-Axis to populate
 **/
std::tuple<CoordScalar,CoordScalar,CoordScalar,
           CoordScalar,CoordScalar,CoordScalar>
RigidXformToEulerXYZAndTrans(const FrameTransform& xform);

}  // xreg

#endif

