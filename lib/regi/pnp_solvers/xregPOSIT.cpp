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

#include "xregPOSIT.h"

#include "xregAssert.h"
#include "xregLinAlgUtils.h"

void xreg::POSIT::run_impl()
{
  // POSIT is for a single view
  xregASSERT(this->cams_.size() == 1);
  xregASSERT(this->inds_2d_.size() == 1);

  const size_type num_pts = this->world_pts_3d_.size();
  xregASSERT(num_pts == this->inds_2d_[0].size());
  xregASSERT(num_pts > 3);
  // At least four points must not be coplanar.

  // investigate this case later
  xregASSERT(cams_[0].coord_frame_type != CameraModel::kORIGIN_ON_DETECTOR);
  
  if (ref_pt_idx_ < 0)
  {
    // recursively run posit with every point as the reference point and choose the
    // pose with minimum reprojection error; optionally enforce that the points should
    // lie between the origin and detector (nice for X-Ray)

    const auto& cam    = this->cams_[0];
    const auto& pts_2d = this->inds_2d_[0];
    const auto& pts_3d = this->world_pts_3d_;

    for (size_type new_ref_pt_idx = 0; new_ref_pt_idx < num_pts; ++new_ref_pt_idx)
    {
      std::vector<CoordScalar> reproj_errors(num_pts);
      std::vector<size_type> num_pts_between_origin_and_det(num_pts, 0);
      std::vector<FrameTransform> xforms(num_pts);

      for (new_ref_pt_idx = 0; new_ref_pt_idx < num_pts; ++new_ref_pt_idx)
      {
        const FrameTransform cur_xform = run_for_single_ref_pt(new_ref_pt_idx);
        const FrameTransform xform_world_to_cam_extrins = cur_xform.inverse();

        xforms[new_ref_pt_idx] = cur_xform;

        CoordScalar cur_reproj_error = 0;
        for (size_type i = 0; i < num_pts; ++i)
        {
          const Pt3 cur_pt_wrt_cam_extrins = xform_world_to_cam_extrins * pts_3d[i];
          
          cur_reproj_error += (pts_2d[i] - cam.phys_pt_to_ind_pt(cur_pt_wrt_cam_extrins).head(2)).norm();
          
          if (prefer_points_between_origin_and_det_)
          {
            const Pt3 cur_pt_wrt_cam = cam.extrins * cur_pt_wrt_cam_extrins;

            if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
            {
              if ((cur_pt_wrt_cam(2) < 0) && (cur_pt_wrt_cam(2) > -cam.focal_len))
              {
                ++num_pts_between_origin_and_det[new_ref_pt_idx];
              }
            }
            else if ((cur_pt_wrt_cam(2) > 0) && (cur_pt_wrt_cam(2) < cam.focal_len))
            {
              ++num_pts_between_origin_and_det[new_ref_pt_idx];
            }
          }
        }
      }
        
      const FrameTransform best_xform_min_reproj = xforms[std::min_element(reproj_errors.begin(), reproj_errors.end())
                                                      - reproj_errors.begin()];
      
      FrameTransform best_xform = best_xform_min_reproj;

      if (prefer_points_between_origin_and_det_)
      {
        using TmpInfo = std::tuple<CoordScalar,size_type,FrameTransform>;

        std::vector<TmpInfo> zipped_infos(num_pts);
        
        for (size_type i = 0; i < num_pts; ++i)
        {
          zipped_infos[i] = std::make_tuple(reproj_errors[i], num_pts_between_origin_and_det[i], xforms[i]);
        }

        std::sort(zipped_infos.begin(), zipped_infos.end(),
                  [] (const TmpInfo& l, const TmpInfo& r)
                  {
                    return std::get<0>(l) < std::get<0>(r);
                  });
        
        bool all_points_in_front = false;
        for (const auto& info : zipped_infos)
        {
          if (std::get<1>(info) == num_pts)
          {
            best_xform = std::get<2>(info);
            all_points_in_front = true;
            break;
          }
        }

        // this was for testing...
        //xregASSERT(all_points_in_front);
      }

      this->regi_cam_to_world_ = best_xform;
    }
  }
  else
  {
    this->regi_cam_to_world_ = run_for_single_ref_pt(static_cast<size_type>(ref_pt_idx_));
  }
}

bool xreg::POSIT::uses_ref_frame() const
{
  return false;
}

xreg::FrameTransform xreg::POSIT::run_for_single_ref_pt(const size_type ref_pt_idx) const
{
  using MatRowMaj = Eigen::Matrix<CoordScalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
  
  const auto& cam    = this->cams_[0];
  const auto& pts_2d = this->inds_2d_[0];
  const auto& pts_3d = this->world_pts_3d_;

  const size_type num_pts = this->world_pts_3d_.size();
  const size_type num_pts_minus_one = num_pts - 1;

  MatRowMaj eps(num_pts_minus_one, 1);
  eps.setZero();

  // map indices to physical points in camera frame
  Pt3List phys_pts_2d_wrt_cam(num_pts);
  for (size_type i = 0; i < num_pts; ++i)
  {
    phys_pts_2d_wrt_cam[i] = cam.extrins * cam.ind_pt_to_phys_det_pt(pts_2d[i]);
  }

  const Pt3 ref_pt_3d = pts_3d[ref_pt_idx];
  const Pt3 ref_pt_2d = phys_pts_2d_wrt_cam[ref_pt_idx]; 

  Pt3List pruned_3d_pts(num_pts_minus_one);
  Pt3List pruned_2d_pts(num_pts_minus_one);

  size_type new_idx = 0;
  for (size_type i = 0; i < num_pts; ++i)
  {
    if (i != ref_pt_idx)
    {
      pruned_3d_pts[new_idx] = pts_3d[i];
      pruned_2d_pts[new_idx] = phys_pts_2d_wrt_cam[i];

      ++new_idx;
    }
  }

  MatRowMaj A(num_pts_minus_one, 3);
  
  for (size_type i = 0; i < num_pts_minus_one; ++i)
  {
    A.row(i) = (pruned_3d_pts[i] - ref_pt_3d).transpose();
  }

  const MatRowMaj B = ComputePseudoInverse(A);

  MatRowMaj x_prime(num_pts_minus_one, 1);
  MatRowMaj y_prime(num_pts_minus_one, 1);

  Pt3 I;
  Pt3 J;

  // camera coordinate axes in the model frame
  Pt3 i_vec;
  Pt3 j_vec;
  Pt3 k_vec;

  CoordScalar s = 0;

  bool should_stop = false;

  // stop when much less than a pixel is changed
  const CoordScalar eps_thresh = 0.01 * std::min(cam.det_col_spacing, cam.det_row_spacing);

  constexpr size_type kMAX_NUM_ITS = 10;

  size_type num_its = 0;

  while (!should_stop)
  {
    ++num_its;

    // adjust points in the scaled ortho. proj.
    for (size_type i = 0; i < num_pts_minus_one; ++i)
    {
      x_prime(i) = (pruned_2d_pts[i](0) * (CoordScalar(1) + eps(i))) - ref_pt_2d(0);
      y_prime(i) = (pruned_2d_pts[i](1) * (CoordScalar(1) + eps(i))) - ref_pt_2d(1);
    }

    // find best fit scaled camera axis
    I = B * x_prime;
    J = B * y_prime;

    const CoordScalar s_1 = I.norm();
    const CoordScalar s_2 = J.norm();
    s = (s_1 + s_2) / 2;

    i_vec = I / s_1;
    j_vec = J / s_2;

    k_vec = i_vec.cross(j_vec);
  
    const CoordScalar Z_0 = (cam.focal_len / s) * ((cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
                                               ? CoordScalar(-1) : CoordScalar(1));

    should_stop = true;
    
    if (num_its < kMAX_NUM_ITS)
    {
      for (size_type i = 0; i < num_pts_minus_one; ++i)
      {
        const CoordScalar new_eps = A.row(i).dot(k_vec) / Z_0;
        if (std::abs(eps(i) - new_eps) > eps_thresh)
        {
          should_stop = false;
        }
        eps(i) = new_eps;
      }
    }
  }

  // make sure we have orthonormal basis
  k_vec /= k_vec.norm();
  j_vec = k_vec.cross(i_vec);

  FrameTransform posit_pose = FrameTransform::Identity();
  
  // Estimated transform from model relative frame to camera frame
  posit_pose.matrix().block(0,0,1,3) = i_vec.transpose();
  posit_pose.matrix().block(1,0,1,3) = j_vec.transpose();
  posit_pose.matrix().block(2,0,1,3) = k_vec.transpose();
  posit_pose.matrix().block(0,3,3,1) = ref_pt_2d / s;
  
  // Convert to transform from camera frame to model relative frame
  posit_pose = posit_pose.inverse();

  // We really want the transform from camera extrinsic frame to model frame
  FrameTransform cam_ext_wrt_world = posit_pose * cam.extrins;
  cam_ext_wrt_world.matrix()(0,3) += ref_pt_3d(0);
  cam_ext_wrt_world.matrix()(1,3) += ref_pt_3d(1);
  cam_ext_wrt_world.matrix()(2,3) += ref_pt_3d(2);

  return cam_ext_wrt_world;
}
  
void xreg::POSIT::set_ref_pt_idx(const int ref_pt_idx)
{
  ref_pt_idx_ = ref_pt_idx;
}

void xreg::POSIT::set_prefer_points_between_origin_and_det(const bool p)
{
  prefer_points_between_origin_and_det_ = p;
}
  
int xreg::POSIT::num_pts_required()
{
  return 4;
}

