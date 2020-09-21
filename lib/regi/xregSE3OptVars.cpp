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

#include "xregSE3OptVars.h"

#include "xregAssert.h"
#include "xregRigidUtils.h"
#include "xregRotUtils.h"

xreg::SE3OptVarsEuler::SE3OptVarsEuler(const unsigned long rot_x_idx,
                                       const unsigned long rot_y_idx,
                                       const unsigned long rot_z_idx,
                                       const unsigned long trans_x_idx,
                                       const unsigned long trans_y_idx,
                                       const unsigned long trans_z_idx)
{
  // check that all indices are in valid range
  xregASSERT((rot_x_idx < 6) && (rot_y_idx < 6) && (rot_z_idx < 6) &&
             (trans_x_idx < 6) && (trans_y_idx < 6) && (trans_z_idx < 6));
  // check that all indices are unique
  xregASSERT((rot_x_idx != rot_y_idx) && (rot_x_idx != rot_z_idx) &&
             (rot_x_idx != trans_x_idx) && (rot_x_idx != trans_y_idx) &&
             (rot_x_idx != trans_z_idx));
  xregASSERT((rot_y_idx != rot_z_idx) && (rot_y_idx != trans_x_idx) &&
             (rot_y_idx != trans_y_idx) && (rot_y_idx != trans_z_idx));
  xregASSERT((rot_z_idx != trans_x_idx) && (rot_z_idx != trans_y_idx) &&
             (rot_z_idx != trans_z_idx));
  xregASSERT((trans_x_idx != trans_y_idx) && (trans_x_idx != trans_z_idx));
  xregASSERT(trans_y_idx != trans_z_idx);

  xform_fns_[rot_x_idx] = EulerRotX4x4;
  xform_fns_[rot_y_idx] = EulerRotY4x4;
  xform_fns_[rot_z_idx] = EulerRotZ4x4;
  xform_fns_[trans_x_idx] = TransX4x4;
  xform_fns_[trans_y_idx] = TransY4x4;
  xform_fns_[trans_z_idx] = TransZ4x4;

  param_idx_[rot_x_idx] = 0;
  param_idx_[rot_y_idx] = 1;
  param_idx_[rot_z_idx] = 2;
  param_idx_[trans_x_idx] = 3;
  param_idx_[trans_y_idx] = 4;
  param_idx_[trans_z_idx] = 5;
}

xreg::FrameTransform
xreg::SE3OptVarsEuler::operator()(const PtN& x) const
{
  xregASSERT(x.size() == 6);

  FrameTransform T;

  T.matrix() = xform_fns_[0](x(param_idx_[0])) *
               xform_fns_[1](x(param_idx_[1])) *
               xform_fns_[2](x(param_idx_[2])) *
               xform_fns_[3](x(param_idx_[3])) *
               xform_fns_[4](x(param_idx_[4])) *
               xform_fns_[5](x(param_idx_[5]));

  return T;
}
  
unsigned long xreg::SE3OptVarsEuler::num_params() const
{
  return 6;
}

unsigned long xreg::SE3OptVarsEuler::rot_x_order() const
{
  return lookup_xform_idx(0);
}

unsigned long xreg::SE3OptVarsEuler::rot_y_order() const
{
  return lookup_xform_idx(1);
}

unsigned long xreg::SE3OptVarsEuler::rot_z_order() const
{
  return lookup_xform_idx(2);
}

unsigned long xreg::SE3OptVarsEuler::trans_x_order() const
{
  return lookup_xform_idx(3);
}

unsigned long xreg::SE3OptVarsEuler::trans_y_order() const
{
  return lookup_xform_idx(4);
}

unsigned long xreg::SE3OptVarsEuler::trans_z_order() const
{
  return lookup_xform_idx(5);
}

unsigned long xreg::SE3OptVarsEuler::lookup_xform_idx(const unsigned long k) const
{
  auto it = std::find(param_idx_.begin(), param_idx_.end(), k);
  xregASSERT(it != param_idx_.end());

  return it - param_idx_.begin();
}

xreg::FrameTransform
xreg::SE3OptVarsLieAlg::operator()(const PtN& x) const
{
  xregASSERT(x.size() == 6);

  FrameTransform T;

  T.matrix() = ExpSE3(Pt6(x));

  return T;
}
  
unsigned long xreg::SE3OptVarsLieAlg::num_params() const
{
  return 6;
}

xreg::SE3OptVarsTransOnly::SE3OptVarsTransOnly(const bool use_x, const bool use_y, const bool use_z)
  : num_comps_(static_cast<unsigned long>(use_x) +
               static_cast<unsigned long>(use_y) +
               static_cast<unsigned long>(use_z)),
    use_x_(use_x), use_y_(use_y), use_z_(use_z)
{
  xregASSERT(num_comps_);
}

xreg::FrameTransform
xreg::SE3OptVarsTransOnly::operator()(const PtN& x) const
{
  xregASSERT(x.size() == num_comps_);

  FrameTransform T = FrameTransform::Identity();

  unsigned long idx = 0;

  if (use_x_)
  {
    T.matrix()(0,3) = x(idx);
    ++idx;
  }

  if (use_y_)
  {
    T.matrix()(1,3) = x(idx);
    ++idx;
  }

  if (use_z_)
  {
    T.matrix()(2,3) = x(idx);
  }

  return T;
}
  
unsigned long xreg::SE3OptVarsTransOnly::num_params() const
{
  return num_comps_;
}

bool xreg::SE3OptVarsTransOnly::use_x() const
{
  return use_x_;
}

bool xreg::SE3OptVarsTransOnly::use_y() const
{
  return use_y_;
}

bool xreg::SE3OptVarsTransOnly::use_z() const
{
  return use_z_;
}
  
xreg::SE3OptVarsTransXOnly::SE3OptVarsTransXOnly()
  : SE3OptVarsTransOnly(true, false, false)
{ }
  
xreg::SE3OptVarsTransYOnly::SE3OptVarsTransYOnly()
  : SE3OptVarsTransOnly(false, true, false)
{ }
  
xreg::SE3OptVarsTransZOnly::SE3OptVarsTransZOnly()
  : SE3OptVarsTransOnly(false, false, true)
{ }

xreg::FrameTransform
xreg::SO3OptVarsLieAlg::operator()(const PtN& x) const
{
  xregASSERT(x.size() == 3);

  FrameTransform H = FrameTransform::Identity();

  H.matrix().block(0,0,3,3) = ExpSO3(Pt3(x));

  return H;
}
  
unsigned long xreg::SO3OptVarsLieAlg::num_params() const
{
  return 3;
}

xreg::SO3OptVarsEuler::SO3OptVarsEuler(const unsigned long rot_x_idx,
                                       const unsigned long rot_y_idx,
                                       const unsigned long rot_z_idx)
{
  // check that all indices are in valid range
  xregASSERT((rot_x_idx < 3) && (rot_y_idx < 3) && (rot_z_idx < 3));
  // check that all indices are unique
  xregASSERT((rot_x_idx != rot_y_idx) && (rot_x_idx != rot_z_idx));
  xregASSERT(rot_y_idx != rot_z_idx);

  xform_fns_[rot_x_idx] = EulerRotX4x4;
  xform_fns_[rot_y_idx] = EulerRotY4x4;
  xform_fns_[rot_z_idx] = EulerRotZ4x4;

  param_idx_[rot_x_idx] = 0;
  param_idx_[rot_y_idx] = 1;
  param_idx_[rot_z_idx] = 2;
}

xreg::FrameTransform
xreg::SO3OptVarsEuler::operator()(const PtN& x) const
{
  xregASSERT(x.size() == 3);

  FrameTransform T;

  T.matrix() = xform_fns_[0](x(param_idx_[0])) *
               xform_fns_[1](x(param_idx_[1])) *
               xform_fns_[2](x(param_idx_[2]));

  return T;
}

unsigned long xreg::SO3OptVarsEuler::num_params() const
{
  return 3;
}

unsigned long xreg::SO3OptVarsEuler::rot_x_order() const
{
  return lookup_xform_idx(0);
}

unsigned long xreg::SO3OptVarsEuler::rot_y_order() const
{
  return lookup_xform_idx(1);
}

unsigned long xreg::SO3OptVarsEuler::rot_z_order() const
{
  return lookup_xform_idx(2);
}

unsigned long xreg::SO3OptVarsEuler::lookup_xform_idx(const unsigned long k) const
{
  auto it = std::find(param_idx_.begin(), param_idx_.end(), k);
  xregASSERT(it != param_idx_.end());

  return it - param_idx_.begin();
}
  
xreg::FrameTransform xreg::SO3OptVarsOnlyX::operator()(const PtN& x) const
{
  xregASSERT(x.size() == 1);

  FrameTransform xform;
  xform.matrix() = EulerRotX4x4(x(0));
  return xform;
}

unsigned long xreg::SO3OptVarsOnlyX::num_params() const
{
  return 1;
}
  
xreg::FrameTransform xreg::SO3OptVarsOnlyY::operator()(const PtN& x) const
{
  FrameTransform xform;
  xform.matrix() = EulerRotY4x4(x(0));
  return xform;
}

unsigned long xreg::SO3OptVarsOnlyY::num_params() const
{
  return 1;
}
  
xreg::FrameTransform xreg::SO3OptVarsOnlyZ::operator()(const PtN& x) const
{
  FrameTransform xform;
  xform.matrix() = EulerRotZ4x4(x(0));
  return xform;
}

unsigned long xreg::SO3OptVarsOnlyZ::num_params() const
{
  return 1;
}

xreg::CamSourceObjPoseOptVars::CamSourceObjPoseOptVars(const CameraModel& ref_cam)
  : ref_cam_(ref_cam)
{ }

xreg::FrameTransform xreg::CamSourceObjPoseOptVars::obj_pose(const PtN& x) const
{
  xregASSERT(x.size() == 9);

  FrameTransform T;
  T.matrix() = ExpSE3(Pt6(x.tail(6)));
  
  return T;
}

xreg::CameraModel xreg::CamSourceObjPoseOptVars::cam(const PtN& x) const
{
  xregASSERT(x.size() >= 3);

  return MoveFocalPointUpdateCam(ref_cam_, x.head(3));
}

xreg::FrameTransform xreg::CamSourceObjPoseOptVars::operator()(const PtN& x) const
{
  return obj_pose(x);  
}

unsigned long xreg::CamSourceObjPoseOptVars::num_params() const
{
  return 9;
}

