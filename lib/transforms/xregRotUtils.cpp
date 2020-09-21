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

#include "xregRotUtils.h"

#include <limits>
#include <cmath>
//#include <iostream>

#include "xregTBBUtils.h"

xreg::Mat3x3 xreg::SkewMatrix(const Pt3& v)
{
  Mat3x3 m = Mat3x3::Zero();

  m(0,1) = -v[2];
  m(0,2) =  v[1];
  m(1,0) =  v[2];
  m(1,2) = -v[0];
  m(2,0) = -v[1];
  m(2,1) =  v[0];

  return m;
}

xreg::Mat3x3List xreg::SkewMatrices(const Pt3List& pts)
{
  const size_type num_pts = pts.size();

  Mat3x3List skews(num_pts);

  auto skew_fn = [&pts, &skews] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      skews[i] = SkewMatrix(pts[i]);
    }
  };
  
  ParallelFor(skew_fn, RangeType(0, num_pts));
  
  return skews;
}

xreg::Pt3 xreg::WedgeSkew(const Mat3x3& W)
{
  Pt3 w;
  w(0) = W(2,1);
  w(1) = W(0,2);
  w(2) = W(1,0);

  return w;
}

xreg::Mat3x3 xreg::ExpSO3(const Pt3& x)
{
  return ExpSO3(SkewMatrix(x));
}

xreg::Mat3x3 xreg::ExpSO3(const Mat3x3& W)
{
  // verify the input is skew-symmetric
  xregASSERT((W + W.transpose()).norm() < 1.0e-4);
  
  Mat3x3 R = Mat3x3::Identity();

  const CoordScalar theta = WedgeSkew(W).norm();
  if (theta > 1.0e-14)
  {
    const Mat3x3 W_unit = W / theta;

    R += (std::sin(theta) * W_unit) + ((1 - std::cos(theta)) * W_unit * W_unit);
    // May want to do some small angle approximations: e.g.
    //const CoordScalar pi_minus_theta = kPI - theta;
    //R += (pi_minus_theta * W_unit) + ((2 - (0.5 * pi_minus_theta * pi_minus_theta)) * W_unit * W_unit);
  }

  // These are really just for debugging to make sure the answer is in the ball-park,
  // in reality various numerical issues can cause these to be slightly off.
  //xregASSERT(std::abs(R.determinant() - 1) < 1.0e-8);
  //xregASSERT(((R.transpose() * R) - Mat3x3::Identity()).norm() < 1.0e-8);
  
  return R;
}

xreg::Pt3 xreg::LogSO3ToPt(const Mat3x3& R)
{
  const CoordScalar theta = std::acos((R(0,0) + R(1,1) + R(2,2) - 1) / 2);

  Pt3 x;

  if (std::abs(theta) > 1.0e-14)
  {
    x(0) = R(2,1) - R(1,2);
    x(1) = R(0,2) - R(2,0);
    x(2) = R(1,0) - R(0,1);
    x *= theta / (2 * std::sin(theta));
  }
  else
  {
    x.setZero();
  }
  
  return x;
}

xreg::Mat3x3 xreg::LogSO3ToSkew(const Mat3x3& R)
{
  return SkewMatrix(LogSO3ToPt(R));
}

xreg::CoordScalar xreg::GeodesicLengthRots3x3(const Mat3x3& R1, const Mat3x3& R2)
{
  return LogSO3ToPt(R1.transpose() * R2).norm();
}

xreg::Mat3x3 xreg::QuatToRotMat(const Pt4& q)
{
  const CoordScalar q_0_sq = q[0] * q[0];
  const CoordScalar q_x_sq = q[1] * q[1];
  const CoordScalar q_y_sq = q[2] * q[2];
  const CoordScalar q_z_sq = q[3] * q[3];

  const CoordScalar q_0_q_x = q[0] * q[1];
  const CoordScalar q_0_q_y = q[0] * q[2];
  const CoordScalar q_0_q_z = q[0] * q[3];

  const CoordScalar q_x_q_y = q[1] * q[2];
  const CoordScalar q_x_q_z = q[1] * q[3];

  const CoordScalar q_y_q_z = q[2] * q[3];

  Mat3x3 r;
  r(0,0) = q_0_sq + q_x_sq - q_y_sq - q_z_sq;
  r(0,1) = 2 * (q_x_q_y - q_0_q_z);
  r(0,2) = 2 * (q_x_q_z + q_0_q_y);
  r(1,0) = 2 * (q_x_q_y + q_0_q_z);
  r(1,1) = q_0_sq - q_x_sq + q_y_sq - q_z_sq;
  r(1,2) = 2 * (q_y_q_z - q_0_q_x);
  r(2,0) = 2 * (q_x_q_z - q_0_q_y);
  r(2,1) = 2 * (q_y_q_z + q_0_q_x);
  r(2,2) = q_0_sq - q_x_sq - q_y_sq + q_z_sq;
  
  return r;
}

xreg::Mat3x3 xreg::EulerRotX(const CoordScalar theta_rad)
{
  return Eigen::AngleAxis<CoordScalar>(theta_rad, Pt3::UnitX()).matrix();
}

xreg::Mat3x3 xreg::EulerRotY(const CoordScalar theta_rad)
{
  return Eigen::AngleAxis<CoordScalar>(theta_rad, Pt3::UnitY()).matrix();
}

xreg::Mat3x3 xreg::EulerRotZ(const CoordScalar theta_rad)
{
  return Eigen::AngleAxis<CoordScalar>(theta_rad, Pt3::UnitZ()).matrix();
}

xreg::Mat3x3 xreg::EulerRotXYZ(const CoordScalar theta_x_rad,
                               const CoordScalar theta_y_rad,
                               const CoordScalar theta_z_rad)
{
  return EulerRotZ(theta_z_rad) * EulerRotY(theta_y_rad) * EulerRotX(theta_x_rad);
}

std::tuple<xreg::CoordScalar,xreg::CoordScalar,xreg::CoordScalar>
xreg::RotMatrixToEulerXYZ(const Mat3x3& R)
{
  CoordScalar theta_x = 0;
  CoordScalar theta_y = 0;
  CoordScalar theta_z = 0;

  const CoordScalar& R_20 = R(2,0);

  const bool R_20_is_one       = R_20 == CoordScalar(1);
  const bool R_20_is_minus_one = R_20 == CoordScalar(-1);

  if (!R_20_is_one && !R_20_is_minus_one)
  {
    theta_y = -std::asin(R_20);

    const double cos_theta_y = std::cos(theta_y);

    theta_x = std::atan2(R(2,1) / cos_theta_y, R(2,2) / cos_theta_y);

    theta_z = std::atan2(R(1,0) / cos_theta_y, R(0,0) / cos_theta_y);
  }
  else
  {
    const double kPI_OVER_TWO = 3.141592653589793 / 2.0;

    // Theta z can really be chosen to be anything, but using zero makes it easy
    // to choose the appropriate theta x
    theta_z = 0;

    if (R_20_is_minus_one)
    {
      theta_y = kPI_OVER_TWO;
      theta_x = std::atan2(R(0,1), R(1,1));
    }
    else
    {
      theta_y = -kPI_OVER_TWO;
      theta_x = std::atan2(-R(0,1), R(1,1));
    }
  }
  
  return std::make_tuple(theta_x, theta_y, theta_z);
}

xreg::CoordScalar xreg::SO3FrechetVar(const Mat3x3List& rots, const Mat3x3& p)
{
  const size_type num_rots = rots.size();

  CoordScalar var = 0;

  for (size_type i = 0; i < num_rots; ++i)
  {
    var += GeodesicLengthRots3x3(p, rots[i]);
  }

  return var;
}

xreg::Mat3x3 xreg::SO3FrechetMean(const Mat3x3List& rots,
                                  const int init_ref_idx,
                                  const unsigned long max_num_its)
{
  const CoordScalar kPI = 3.141592653589793;

  Mat3x3 cur_mean = Mat3x3::Identity();
  Mat3x3 cur_mean_inv;
  Mat3x3 prev_mean;

  const size_type num_rots = rots.size();

  if (init_ref_idx >= 0)
  {
    cur_mean = rots[init_ref_idx];
  }
  else if (init_ref_idx == -2)
  {
    size_type min_frech_var_idx = 0;
    CoordScalar min_frech_var = std::numeric_limits<CoordScalar>::max();

    CoordScalar cur_frech_var;

    for (size_type cur_mean_idx = 0; cur_mean_idx < num_rots; ++cur_mean_idx)
    {
      cur_frech_var = SO3FrechetVar(rots, rots[cur_mean_idx]);

      if (cur_frech_var < min_frech_var)
      {
        min_frech_var     = cur_frech_var;
        min_frech_var_idx = cur_mean_idx;
      }
    }

    cur_mean = rots[min_frech_var_idx];
  }

  //std::cout << 0 << " Frechet var from mean: " << SO3FrechetVar(rots, cur_mean) << std::endl;

  // Tangent space are 3D vectors u, such that the rotation matrix on the manifold
  // is R = exp(||u|| * skew(u / ||u||)), where exp is the matrix exponential
  // and skew creates a 3x3 skew symmetric matrix from a 3D vector

  // This is on the tangent space
  Pt3 mean_vec;
  Pt3 tmp_vec;

  // will store an exponentiated tangent space element
  Mat3x3 tmp_exp;

  bool should_stop = false;

  unsigned long num_its = 0;
  while (!should_stop)
  {
    prev_mean    = cur_mean;
    cur_mean_inv = cur_mean.transpose();

    mean_vec.setZero();

    for (size_type i = 0; i < num_rots; ++i)
    {
      tmp_vec = LogSO3ToPt(cur_mean_inv * rots[i]);
      // || tmp_vec ||_2 is in [0,pi]

      // if the current rotation lies on, or close to, the cut locus, don't let
      // it influence the computation as the element at the opposite side of
      // the cut locus is identical.
      if ((kPI - tmp_vec.norm()) < 1.0e-3)
      {
        tmp_vec.setZero();
      }

      mean_vec += tmp_vec;
    }

    mean_vec /= static_cast<CoordScalar>(num_rots);
    //std::cout << "mean_vec =\n  " << mean_vec << std::endl;

    const CoordScalar mean_rot_ang = mean_vec.norm();

    if (mean_rot_ang > 1.0e-6)
    {
      tmp_exp = ExpSO3(mean_vec);
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

xreg::Mat3x3 xreg::SO3MeanQuat(const Mat3x3List& rots)
{
  using Quat = Eigen::Quaternion<CoordScalar>;

  const size_type num_rots = rots.size();

  Pt4 quat_coeffs = Pt4::Zero();

  for (size_type i = 0; i < num_rots; ++i)
  {
    Quat cur_quat(rots[i]);

    quat_coeffs[0] += cur_quat.w();
    quat_coeffs[1] += cur_quat.x();
    quat_coeffs[2] += cur_quat.y();
    quat_coeffs[3] += cur_quat.z();
  }

  quat_coeffs /= static_cast<CoordScalar>(num_rots);
  quat_coeffs.normalize();

  return Quat(quat_coeffs[0], quat_coeffs[1], quat_coeffs[2], quat_coeffs[3]).matrix();
}

xreg::Mat3x3 xreg::FindClosestRot(const Mat3x3& A)
{
  using SVD = Eigen::JacobiSVD<Eigen::Matrix<CoordScalar,Eigen::Dynamic,Eigen::Dynamic>>;

  SVD svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  const CoordScalar det_U = svd.matrixU().determinant();
  const CoordScalar det_V = svd.matrixV().determinant();

  CoordScalar sgn[3];

  CoordScalar min_abs_sv = std::numeric_limits<CoordScalar>::max();
  unsigned long min_abs_sv_idx = 0;
  
  for (unsigned long i = 0; i < 3; ++i)
  {
    const CoordScalar cur_sv = svd.singularValues()[i];

    const CoordScalar cur_abs_sv = std::abs(cur_sv);
    xregASSERT(cur_abs_sv > 1.0e-8);

    if (cur_abs_sv < min_abs_sv)
    {
      min_abs_sv = cur_abs_sv;
      min_abs_sv_idx = i;
    }

    sgn[i] = (cur_sv > 1.0e-8) ? CoordScalar(1) : CoordScalar(-1);
  }
 
  if ((det_U * det_V * sgn[0] * sgn[1] * sgn[2]) < -1.0e-8)
  {
    sgn[min_abs_sv_idx] *= CoordScalar(-1);
  }

  Mat3x3 new_sigma = Mat3x3::Zero();
  new_sigma(0,0) = sgn[0];
  new_sigma(1,1) = sgn[1];
  new_sigma(2,2) = sgn[2];

  return svd.matrixU() * new_sigma * svd.matrixV().transpose();
}

