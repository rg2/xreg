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

#include "xregLandmark2D3DRegiReprojDist.h"

#include "xregAssert.h"
#include "xregSE3OptVars.h"
 
void xreg::Landmark2D3DRegiReprojDist::set_se3_vars(SE3OptVarsPtr se3_vars)
{
  se3_vars_ = se3_vars;
}

void xreg::Landmark2D3DRegiReprojDist::set_reg_fn(const RegFn& reg_fn, const CoordScalar reg_coeff)
{
  reg_fn_    = reg_fn;
  reg_coeff_ = reg_coeff;
}

xreg::FrameTransform xreg::Landmark2D3DRegiReprojDist::cam_to_world(const PtN& x) const
{
  FrameTransform cam_to_world;
  
  const FrameTransform cur_opt = se3_vars_->operator()(x);

  if (this->ref_frame_maps_world_to_ref_)
  {
    cam_to_world = this->ref_frame_inv_ * cur_opt * this->ref_frame_ * this->init_cam_to_world_;
  }
  else
  {
    cam_to_world = this->init_cam_to_world_ * this->ref_frame_inv_ * cur_opt * this->ref_frame_;
  }
  
  return cam_to_world;
}

xreg::FrameTransform xreg::Landmark2D3DRegiReprojDist::world_to_cam(const PtN& x) const
{
  return cam_to_world(x).inverse();
}

xreg::CoordScalar xreg::Landmark2D3DRegiReprojDist::reproj_cost(const FrameTransform& cur_world_to_cam) const
{
  const size_type num_views = this->cams_.size();
  xregASSERT(this->inds_2d_.size() == num_views); 
  
  CoordScalar tot_err = 0;
  Pt3 tmp_pt3d;
    
  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    const auto& cur_cam = this->cams_[view_idx];

    const auto& cur_inds_2d = this->inds_2d_[view_idx];

    const size_type num_pts = cur_inds_2d.size();
    xregASSERT(this->world_pts_3d_.size() == num_pts);
  
    for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
    {
      tmp_pt3d = cur_cam.phys_pt_to_ind_pt(cur_world_to_cam * this->world_pts_3d_[pt_idx]); 

      tot_err += (tmp_pt3d.head(2) - cur_inds_2d[pt_idx]).squaredNorm();
    }
  }

  if (reg_fn_)
  {
    tot_err += reg_coeff_ * reg_fn_(cur_world_to_cam);
  }

  return tot_err;
}

bool xreg::Landmark2D3DRegiReprojDist::uses_ref_frame() const
{
  return true;
}

