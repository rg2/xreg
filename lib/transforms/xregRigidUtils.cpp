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

#include "xregRigidUtils.h"

#include "xregRotUtils.h"

xreg::Mat4x4 xreg::SE3Inv(const Mat4x4& T)
{
  Mat4x4 T_inv = Mat4x4::Identity();
  
  T_inv.block(0,0,3,3) = T.block(0,0,3,3).transpose();
  
  T_inv.block(0,3,3,1) = -1 * T_inv.block(0,0,3,3) * T.block(0,3,3,1);

  return T_inv;
}

xreg::Mat4x4 xreg::ExpSE3(const Mat4x4& M)
{
  Mat4x4 T = Mat4x4::Identity();

  const Mat3x3 W = M.block(0,0,3,3);
  const Pt3    v = M.block(0,3,3,1);

  const CoordScalar theta = WedgeSkew(W).norm();
  if (theta > 1.0e-14)
  {
    // first exponentiate to get the rotation component
    T.block(0,0,3,3) = ExpSO3(W);

    // next, we need to do some work to transform the translation part of the
    // Lie algebra to get the translation component in SE(3)
    const CoordScalar theta_sq = theta * theta;

    Mat3x3 A = Mat3x3::Identity();
    A += ((1 - std::cos(theta)) / theta_sq) * W;
    A += ((theta - std::sin(theta)) / (theta * theta_sq)) * (W * W);

    T.block(0,3,3,1) = A * v;
  }
  else
  {
    // rotation component is identity

    // no special processing on the translation part in the Lie algebra
    T.block(0,3,3,1) = v;
  }
  
  return T;
}

xreg::Mat4x4 xreg::ExpSE3(const Pt6& x)
{
  Mat4x4 M = Mat4x4::Zero();
  
  M.block(0,0,3,3) = SkewMatrix(x.head(3));

  M(0,3) = x(3);
  M(1,3) = x(4);
  M(2,3) = x(5);

  return ExpSE3(M);
}

xreg::Mat4x4 xreg::LogSE3ToMat4x4(const Mat4x4& T)
{
  Mat4x4 M = Mat4x4::Zero();

  const Pt3    v = LogSO3ToPt(T.block(0,0,3,3));
  const Mat3x3 W = SkewMatrix(v);
  
  M.block(0,0,3,3) = W;

  const Pt3 trans = T.block(0,3,3,1);

  const CoordScalar theta = v.norm();
  if (theta > 1.0e-14)
  {
    // compute the appropriate translation component of the Lie Algebra

    const CoordScalar sin_theta = std::sin(theta);

    Mat3x3 A_inv = Mat3x3::Identity();
    A_inv += -0.5 * W;
    A_inv += (((2 * sin_theta) - (theta * (1 + std::cos(theta)))) / (2 * theta * theta * sin_theta)) * (W * W);

    M.block(0,3,3,1) = A_inv * trans;
  }
  else
  {
    // no rotation, no special processing to get the translation parameters in
    // the Lie algebra
    M.block(0,3,3,1) = trans;
  }
  
  return M;
}

xreg::CoordScalar xreg::WeightedLengthSE3Mats(const Mat4x4& T1, const Mat4x4& T2,
                                              const CoordScalar w_R, const CoordScalar w_t)
{
  const Mat4x4 T_diff = SE3Inv(T1) * T2;

  return (w_R * LogSO3ToPt(T_diff.block(0,0,3,3)).norm()) + (w_t * T_diff.block(0,3,3,1).norm());
}

xreg::CoordScalar xreg::SE3FrechetVarWeightedLength(const Mat4x4List& mats, const Mat4x4& m)
{
  const size_type num_mats = mats.size();

  CoordScalar v = 0;

  for (size_type i = 0; i < num_mats; ++i)
  {
    const CoordScalar d = WeightedLengthSE3Mats(m, mats[i]);
    v += d * d;
  }

  return v;
}

xreg::Mat4x4 xreg::SE3FrechetMean(const Mat4x4List& xforms,
                                  const int init_ref_idx,
                                  const unsigned long max_num_its)
{
  const CoordScalar kPI = 3.141592653589793;

  Mat4x4 cur_mean = Mat4x4::Identity();
  Mat4x4 cur_mean_inv;
  Mat4x4 prev_mean;

  const size_type num_xforms = xforms.size();

  if (init_ref_idx >= 0)
  {
    cur_mean = xforms[init_ref_idx];
  }
  else if (init_ref_idx == -2)
  {
    size_type min_frech_var_idx = 0;
    CoordScalar min_frech_var = std::numeric_limits<CoordScalar>::max();

    CoordScalar cur_frech_var;

    for (size_type cur_mean_idx = 0; cur_mean_idx < num_xforms; ++cur_mean_idx)
    {
      cur_frech_var = SE3FrechetVarWeightedLength(xforms, xforms[cur_mean_idx]);

      if (cur_frech_var < min_frech_var)
      {
        min_frech_var     = cur_frech_var;
        min_frech_var_idx = cur_mean_idx;
      }
    }

    cur_mean = xforms[min_frech_var_idx];
  }

  // Tangent space are 4x4 matrices M:
  // [ W v ]
  // [ 0 0 ]
  // Where W is a 3x3 skew symmetric matrix, such that exp(W) generates the rotation
  // component, and v is a 3x1 matrix and there is some V so that the translation
  // component is equal to V * v
  // This is derived by computing exp(M)

  Mat4x4 mean_tangent_space;
  Mat4x4 tmp_tangent_space;

  // will store an exponentiated tangent space element
  Mat4x4 tmp_exp;

  bool should_stop = false;

  unsigned long num_its = 0;
  while (!should_stop)
  {
    prev_mean    = cur_mean;
    cur_mean_inv = SE3Inv(cur_mean);

    mean_tangent_space.setZero();

    for (size_type i = 0; i < num_xforms; ++i)
    {
      tmp_tangent_space = LogSE3ToMat4x4(cur_mean_inv * xforms[i]);

      // if the current rotation lies on, or close to, the cut locus, don't let
      // it influence the computation as the element at the opposite side of
      // the cut locus is identical.
      if ((kPI - WedgeSkew(tmp_tangent_space.block(0,0,3,3)).norm()) < 1.0e-3)
      {
        tmp_tangent_space.setZero();
      }

      mean_tangent_space += tmp_tangent_space;
    }

    mean_tangent_space /= static_cast<CoordScalar>(num_xforms);

    // check to see if the rotational component or translation component will
    // result in a change
    if ((WedgeSkew(mean_tangent_space.block(0,0,3,3)).norm() > 1.0e-6) ||
        (mean_tangent_space.block(0,3,3,1).norm() > 1.0e-6))
    {
      tmp_exp = ExpSE3(mean_tangent_space);
      cur_mean = prev_mean * tmp_exp;

      // we have not converged, so only stop if we've reached the maximum number
      // of iterations
      should_stop = max_num_its && (num_its >= max_num_its);
    }
    else
    {
      should_stop = true;
    }

    ++num_its;
  }
  //std::cout << "its = " << num_its << std::endl;

  return cur_mean;
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar>
xreg::ComputeRotAngTransMag(const FrameTransform& xform)
{
  return std::make_tuple(LogSO3ToPt(xform.matrix().block(0,0,3,3)).norm(),
                         xform.matrix().block(0,3,3,1).norm());
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar>
xreg::FrameDiffRotAngTransMag(const FrameTransform& f1, const FrameTransform& f2)
{
  return ComputeRotAngTransMag(f1 * f2.inverse());
}

xreg::Mat4x4 xreg::TransX4x4(const CoordScalar x)
{
  Mat4x4 H = Mat4x4::Identity();
  H(0,3) = x;

  return H;
}

xreg::Mat4x4 xreg::TransY4x4(const CoordScalar x)
{
  Mat4x4 H = Mat4x4::Identity();
  H(1,3) = x;

  return H;
}

xreg::Mat4x4 xreg::TransZ4x4(const CoordScalar x)
{
  Mat4x4 H = Mat4x4::Identity();
  H(2,3) = x;

  return H;
}

xreg::Mat4x4 xreg::TransXYZ4x4(const Pt3& x)
{
  Mat4x4 H = Mat4x4::Identity();
  H.block(0,3,3,1) = x;

  return H;
}

xreg::Mat4x4 xreg::EulerRotX4x4(const CoordScalar theta_rad)
{
  Mat4x4 H = Mat4x4::Identity();

  H.block(0,0,3,3) = EulerRotX(theta_rad);

  return H;
}

xreg::Mat4x4 xreg::EulerRotY4x4(const CoordScalar theta_rad)
{
  Mat4x4 H = Mat4x4::Identity();

  H.block(0,0,3,3) = EulerRotY(theta_rad);

  return H;
}

xreg::Mat4x4 xreg::EulerRotZ4x4(const CoordScalar theta_rad)
{
  Mat4x4 H = Mat4x4::Identity();

  H.block(0,0,3,3) = EulerRotZ(theta_rad);

  return H;
}

xreg::Mat4x4 xreg::EulerRotXYZTransXYZ(const CoordScalar theta_x_rad,
                                       const CoordScalar theta_y_rad,
                                       const CoordScalar theta_z_rad,
                                       const CoordScalar trans_x,
                                       const CoordScalar trans_y,
                                       const CoordScalar trans_z)
{
  Mat4x4 H = Mat4x4::Identity();

  H.block(0,0,3,3) = EulerRotXYZ(theta_x_rad, theta_y_rad, theta_z_rad);

  H(0,3) = trans_x;
  H(1,3) = trans_y;
  H(2,3) = trans_z;

  return H;
}

xreg::FrameTransform xreg::EulerRotXYZTransXYZFrame(const CoordScalar theta_x_rad,
                                                    const CoordScalar theta_y_rad,
                                                    const CoordScalar theta_z_rad,
                                                    const CoordScalar trans_x,
                                                    const CoordScalar trans_y,
                                                    const CoordScalar trans_z)
{
  return FrameTransform(EulerRotXYZTransXYZ(theta_x_rad, theta_y_rad, theta_z_rad,
                                            trans_x, trans_y, trans_z));
}

xreg::FrameTransform xreg::EulerRotXFrame(const CoordScalar rot_ang_rad)
{
  return FrameTransform(EulerRotX4x4(rot_ang_rad));
}

xreg::FrameTransform xreg::EulerRotYFrame(const CoordScalar rot_ang_rad)
{
  return FrameTransform(EulerRotY4x4(rot_ang_rad));
}

xreg::FrameTransform xreg::EulerRotZFrame(const CoordScalar rot_ang_rad)
{
  return FrameTransform(EulerRotZ4x4(rot_ang_rad));
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar,xreg::CoordScalar,
           xreg::CoordScalar,xreg::CoordScalar,xreg::CoordScalar>
xreg::RigidXformToEulerXYZAndTrans(const FrameTransform& xform)
{
  const auto rot_angs = RotMatrixToEulerXYZ(xform.matrix().block(0,0,3,3));

  return std::make_tuple(std::get<0>(rot_angs), std::get<1>(rot_angs), std::get<2>(rot_angs),
                         xform.matrix()(0,3), xform.matrix()(1,3), xform.matrix()(2,3));
}

