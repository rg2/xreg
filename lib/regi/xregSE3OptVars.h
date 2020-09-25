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

#ifndef XREGSE3OPTVARS_H_
#define XREGSE3OPTVARS_H_

#include <array>

#include "xregCommon.h"
#include "xregPerspectiveXform.h"

namespace xreg
{

/// \brief Generic class for a parameterization of SE(3)
///
/// The primary purpose of this class is to run differing optimization strategies
/// without changing the optimization interface. E.g. one can perform in-plane
/// 2D/3D optimization with one derived class of SE3OptVars, out of plane
/// rotation with another derived class of SE3OptVars, and out of plane
/// translation with another derived class of SE3OptVars, but using the same
/// optimization interface.
class SE3OptVars
{
public:
  class UnsupportedOperation { };

  /// \brief Destructor
  virtual ~SE3OptVars() { }

  /// \brief Map from the parameterization to an SE(3) element
  ///
  /// Must be implemented by the derived class.
  virtual FrameTransform operator()(const PtN& x) const = 0;

  /// \brief The dimensionality of the parameterization.
  ///
  /// For example, for a parameterization over se(3) (Lie Algebra) this will be
  /// 6 and for rotation only about the x-axis this will be 1.
  /// Must be implemented by the derived class.
  virtual unsigned long num_params() const = 0;
};

/// \brief SE(3) parameterization by 3 Euler angles and 3 translation elements.
///
/// This allows for an arbitrary ordering of the specific transforms but the
/// default is T = TransXYZ * RotZ * RotY * RotX
/// TODO: Make this able to ignore a component by passing (-1)ul.
class SE3OptVarsEuler : public SE3OptVars
{
public:
  /// \brief Constructor - User can optionally specify the decomposition order
  ///        of each component.
  explicit
  SE3OptVarsEuler(const unsigned long rot_x_idx = 5, const unsigned long rot_y_idx = 4,
                  const unsigned long rot_z_idx = 3, const unsigned long trans_x_idx = 2,
                  const unsigned long trans_y_idx = 1, const unsigned long trans_z_idx = 0);

  /// \brief Map from the parameterization to an SE(3) element.
  ///
  /// This will compute the 4x4 homogeneous matrix for each component and
  /// multiply them in the user-specified order.
  FrameTransform operator()(const PtN& x) const override;

  /// \brief Parameterization dimensionality: 6
  unsigned long num_params() const override;

  /// \brief Retrieve the order (left to right zero-based) of the X Rotation
  unsigned long rot_x_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Y Rotation
  unsigned long rot_y_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Z Rotation
  unsigned long rot_z_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the X Translation
  unsigned long trans_x_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Y Translation
  unsigned long trans_y_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Z Translation
  unsigned long trans_z_order() const;

private:
  using HomogeneousMatXformFn = Mat4x4 (*) (const CoordScalar);

  unsigned long lookup_xform_idx(const unsigned long k) const;
  
  // Using std::array<HomogeneousMatXformFn,6> causes an ICE on VS 2013/2015
  HomogeneousMatXformFn xform_fns_[6];

  std::array<unsigned long,6> param_idx_;
};

/// \brief SE(3) parameterization using the se(3) Lie Algebra mapped to R^6
class SE3OptVarsLieAlg : public SE3OptVars
{
public:
  /// \brief Map from the parameterization to an SE(3) element.
  ///
  /// This performs the following computations:
  /// W = skew(x(0), x(1), x(2)), q = [x(3); x(4); x(5)]
  /// SE(3) element = exp([W, q; 0 0])
  FrameTransform operator()(const PtN& x) const override;

  /// \brief Parameterization dimensionality: 6
  unsigned long num_params() const override;
};

/// \brief Translation only parameterization of SE(3); X, Y, Z translations.
class SE3OptVarsTransOnly : public SE3OptVars
{
public:
  explicit
  SE3OptVarsTransOnly(const bool use_x = true, const bool use_y = true, const bool use_z = true);

  /// \brief Map from the parameterization to an SE(3) element with translation only.
  ///
  /// This just sets the translation component of a 4x4 homogeneous transform matrix,
  /// It must always be that the X component has a lower index than the Y component,
  /// and the Y component has a lower index than the Z component.
  FrameTransform operator()(const PtN& x) const override;

  /// \brief Parameterization dimensionality: 1-3
  unsigned long num_params() const override;

  bool use_x() const;

  bool use_y() const;

  bool use_z() const;

private:
  const unsigned long num_comps_;

  const bool use_x_;
  const bool use_y_;
  const bool use_z_;
};

/// \brief 1-D parameterization of SE(3) with translation along x-axis only
class SE3OptVarsTransXOnly : public SE3OptVarsTransOnly
{
public:
  SE3OptVarsTransXOnly();
};

/// \brief 1-D parameterization of SE(3) with translation along y-axis only
class SE3OptVarsTransYOnly : public SE3OptVarsTransOnly
{
public:
  SE3OptVarsTransYOnly();
};

/// \brief 1-D parameterization of SE(3) with translation along z-axis only
class SE3OptVarsTransZOnly : public SE3OptVarsTransOnly
{
public:
  SE3OptVarsTransZOnly();
};

/// \brief Parameterization of the rotational component of SE(3)
class SE3OptVarsRotOnly : public SE3OptVars
{ };

/// \brief SE(3) parameterization without translation using the so(3) Lie Algebra for mapping to SO(3)
class SO3OptVarsLieAlg : public SE3OptVarsRotOnly
{
public:
  /// \brief Map from the parameterization to an SE(3) element.
  ///
  /// This performs the following computations:
  /// W = skew(x(0), x(1), x(2))
  /// R = exp(W)
  /// SE(3) element = [R 0; 0 1])
  FrameTransform operator()(const PtN& x) const override;

  /// \brief Parameterization dimensionality: 3
  unsigned long num_params() const override;
};

/// \brief Parameterization of the rotational component of SE(3) using Euler Angles.
class SO3OptVarsEuler : public SE3OptVarsRotOnly
{
public:
  /// \brief Constructor - user may specify the order of the rotation matrices.
  ///
  /// Default rotation is RotZ * RotY * RotX
  explicit
  SO3OptVarsEuler(const unsigned long rot_x_idx = 2, const unsigned long rot_y_idx = 1,
                  const unsigned long rot_z_idx = 0);

  /// \brief Map from the parameterization to an SE(3) element with rotation only.
  FrameTransform operator()(const PtN& x) const override;

  /// \brief Parameterization dimensionality: 3
  unsigned long num_params() const override;
  
  /// \brief Retrieve the order (left to right zero-based) of the X Rotation
  unsigned long rot_x_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Y Rotation
  unsigned long rot_y_order() const;

  /// \brief Retrieve the order (left to right zero-based) of the Z Rotation
  unsigned long rot_z_order() const;

private:
  using HomogeneousMatXformFn = Mat4x4 (*) (const CoordScalar);
  
  unsigned long lookup_xform_idx(const unsigned long k) const;
  
  HomogeneousMatXformFn xform_fns_[3];

  std::array<unsigned long,3> param_idx_;
};

/// \brief Parameterization of rotation only, and only about the x axis.
class SO3OptVarsOnlyX : public SE3OptVarsRotOnly
{
public:
  FrameTransform operator()(const PtN& x) const override;

  unsigned long num_params() const override;
};

/// \brief Parameterization of rotation only, and only about the x axis.
class SO3OptVarsOnlyY : public SE3OptVarsRotOnly
{
public:
  FrameTransform operator()(const PtN& x) const override;

  unsigned long num_params() const override;
};


/// \brief Parameterization of rotation only, and only about the x axis.
class SO3OptVarsOnlyZ : public SE3OptVarsRotOnly
{
public:
  FrameTransform operator()(const PtN& x) const override;

  unsigned long num_params() const override;
};

class CamSourceObjPoseOptVars : public SE3OptVars
{
public:
  explicit CamSourceObjPoseOptVars(const CameraModel& ref_cam);

  FrameTransform obj_pose(const PtN& x) const;

  CameraModel cam(const PtN& x) const;

  FrameTransform operator()(const PtN& x) const override;

  unsigned long num_params() const override;

private:
  CameraModel ref_cam_;
};

}  // xreg

#endif
