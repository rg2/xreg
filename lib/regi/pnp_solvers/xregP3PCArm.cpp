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

#include "xregP3PCArm.h"

#include <array>

#include "xregAssert.h"
#include "xregPairedPointRegi3D3D.h"
#include "xregRigidUtils.h"

xreg::Pt3List
xreg::FindPossPtsAlongLineForMatch(const Pt3& ref_pt, const Pt3& src_pt, const Pt3& det_pt, const CoordScalar l,
                                   const CoordScalar dist_ratio_lower, const CoordScalar dist_ratio_upper)
{
	Pt3List pts;
  pts.reserve(3);

  const CoordScalar& p_x = ref_pt[0];
  const CoordScalar& p_y = ref_pt[1];
  const CoordScalar& p_z = ref_pt[2];

  const CoordScalar& s_x = src_pt[0];
  const CoordScalar& s_y = src_pt[1];
  const CoordScalar& s_z = src_pt[2];

  const CoordScalar& d_x = det_pt[0];
  const CoordScalar& d_y = det_pt[1];
  const CoordScalar& d_z = det_pt[2];

  // All Hail the MATLAB symbolic toolbox!

  // three critical points of the quadratic objective function

  const CoordScalar t1 = (d_x*p_x+d_y*p_y+d_z*p_z-d_x*s_x-d_y*s_y-d_z*s_z-p_x*s_x-p_y*s_y-p_z*s_z+sqrt(-(p_x*p_x)*(s_y*s_y)-(p_y*p_y)*(s_x*s_x)-(p_x*p_x)*(s_z*s_z)-(p_z*p_z)*(s_x*s_x)-(p_y*p_y)*(s_z*s_z)-(p_z*p_z)*(s_y*s_y)+(d_x*d_x)*(l*l)+(d_y*d_y)*(l*l)+(d_z*d_z)*(l*l)-(d_x*d_x)*(p_y*p_y)-(d_y*d_y)*(p_x*p_x)-(d_x*d_x)*(p_z*p_z)-(d_z*d_z)*(p_x*p_x)-(d_y*d_y)*(p_z*p_z)-(d_z*d_z)*(p_y*p_y)-(d_x*d_x)*(s_y*s_y)-(d_y*d_y)*(s_x*s_x)-(d_x*d_x)*(s_z*s_z)-(d_z*d_z)*(s_x*s_x)-(d_y*d_y)*(s_z*s_z)-(d_z*d_z)*(s_y*s_y)+(l*l)*(s_x*s_x)+(l*l)*(s_y*s_y)+(l*l)*(s_z*s_z)-d_x*(l*l)*s_x*2.0-d_y*(l*l)*s_y*2.0-d_z*(l*l)*s_z*2.0+d_x*p_x*(s_y*s_y)*2.0+d_x*(p_y*p_y)*s_x*2.0+(d_y*d_y)*p_x*s_x*2.0+d_x*p_x*(s_z*s_z)*2.0+d_x*(p_z*p_z)*s_x*2.0+d_y*p_y*(s_x*s_x)*2.0+d_y*(p_x*p_x)*s_y*2.0+(d_x*d_x)*p_y*s_y*2.0+(d_z*d_z)*p_x*s_x*2.0+d_y*p_y*(s_z*s_z)*2.0+d_y*(p_z*p_z)*s_y*2.0+d_z*p_z*(s_x*s_x)*2.0+d_z*(p_x*p_x)*s_z*2.0+(d_x*d_x)*p_z*s_z*2.0+(d_z*d_z)*p_y*s_y*2.0+d_z*p_z*(s_y*s_y)*2.0+d_z*(p_y*p_y)*s_z*2.0+(d_y*d_y)*p_z*s_z*2.0+d_x*d_y*p_x*p_y*2.0+d_x*d_z*p_x*p_z*2.0+d_y*d_z*p_y*p_z*2.0-d_x*d_y*p_x*s_y*2.0-d_x*d_y*p_y*s_x*2.0-d_x*d_z*p_x*s_z*2.0-d_x*d_z*p_z*s_x*2.0-d_y*d_z*p_y*s_z*2.0-d_y*d_z*p_z*s_y*2.0+d_x*d_y*s_x*s_y*2.0+d_x*d_z*s_x*s_z*2.0+d_y*d_z*s_y*s_z*2.0-d_x*p_x*p_y*s_y*2.0-d_y*p_x*p_y*s_x*2.0-d_x*p_x*p_z*s_z*2.0-d_z*p_x*p_z*s_x*2.0-d_y*p_y*p_z*s_z*2.0-d_z*p_y*p_z*s_y*2.0-d_x*p_y*s_x*s_y*2.0-d_y*p_x*s_x*s_y*2.0-d_x*p_z*s_x*s_z*2.0-d_z*p_x*s_x*s_z*2.0-d_y*p_z*s_y*s_z*2.0-d_z*p_y*s_y*s_z*2.0+p_x*p_y*s_x*s_y*2.0+p_x*p_z*s_x*s_z*2.0+p_y*p_z*s_y*s_z*2.0)+s_x*s_x+s_y*s_y+s_z*s_z)/(d_x*s_x*-2.0-d_y*s_y*2.0-d_z*s_z*2.0+d_x*d_x+d_y*d_y+d_z*d_z+s_x*s_x+s_y*s_y+s_z*s_z);

  const CoordScalar t2 = ((d_x-s_x)*(p_x-s_x)*2.0+(d_y-s_y)*(p_y-s_y)*2.0+(d_z-s_z)*(p_z-s_z)*2.0)/(pow(d_x-s_x,2.0)*2.0+pow(d_y-s_y,2.0)*2.0+pow(d_z-s_z,2.0)*2.0);

  const CoordScalar t3 = -(-d_x*p_x-d_y*p_y-d_z*p_z+d_x*s_x+d_y*s_y+d_z*s_z+p_x*s_x+p_y*s_y+p_z*s_z+sqrt(-(p_x*p_x)*(s_y*s_y)-(p_y*p_y)*(s_x*s_x)-(p_x*p_x)*(s_z*s_z)-(p_z*p_z)*(s_x*s_x)-(p_y*p_y)*(s_z*s_z)-(p_z*p_z)*(s_y*s_y)+(d_x*d_x)*(l*l)+(d_y*d_y)*(l*l)+(d_z*d_z)*(l*l)-(d_x*d_x)*(p_y*p_y)-(d_y*d_y)*(p_x*p_x)-(d_x*d_x)*(p_z*p_z)-(d_z*d_z)*(p_x*p_x)-(d_y*d_y)*(p_z*p_z)-(d_z*d_z)*(p_y*p_y)-(d_x*d_x)*(s_y*s_y)-(d_y*d_y)*(s_x*s_x)-(d_x*d_x)*(s_z*s_z)-(d_z*d_z)*(s_x*s_x)-(d_y*d_y)*(s_z*s_z)-(d_z*d_z)*(s_y*s_y)+(l*l)*(s_x*s_x)+(l*l)*(s_y*s_y)+(l*l)*(s_z*s_z)-d_x*(l*l)*s_x*2.0-d_y*(l*l)*s_y*2.0-d_z*(l*l)*s_z*2.0+d_x*p_x*(s_y*s_y)*2.0+d_x*(p_y*p_y)*s_x*2.0+(d_y*d_y)*p_x*s_x*2.0+d_x*p_x*(s_z*s_z)*2.0+d_x*(p_z*p_z)*s_x*2.0+d_y*p_y*(s_x*s_x)*2.0+d_y*(p_x*p_x)*s_y*2.0+(d_x*d_x)*p_y*s_y*2.0+(d_z*d_z)*p_x*s_x*2.0+d_y*p_y*(s_z*s_z)*2.0+d_y*(p_z*p_z)*s_y*2.0+d_z*p_z*(s_x*s_x)*2.0+d_z*(p_x*p_x)*s_z*2.0+(d_x*d_x)*p_z*s_z*2.0+(d_z*d_z)*p_y*s_y*2.0+d_z*p_z*(s_y*s_y)*2.0+d_z*(p_y*p_y)*s_z*2.0+(d_y*d_y)*p_z*s_z*2.0+d_x*d_y*p_x*p_y*2.0+d_x*d_z*p_x*p_z*2.0+d_y*d_z*p_y*p_z*2.0-d_x*d_y*p_x*s_y*2.0-d_x*d_y*p_y*s_x*2.0-d_x*d_z*p_x*s_z*2.0-d_x*d_z*p_z*s_x*2.0-d_y*d_z*p_y*s_z*2.0-d_y*d_z*p_z*s_y*2.0+d_x*d_y*s_x*s_y*2.0+d_x*d_z*s_x*s_z*2.0+d_y*d_z*s_y*s_z*2.0-d_x*p_x*p_y*s_y*2.0-d_y*p_x*p_y*s_x*2.0-d_x*p_x*p_z*s_z*2.0-d_z*p_x*p_z*s_x*2.0-d_y*p_y*p_z*s_z*2.0-d_z*p_y*p_z*s_y*2.0-d_x*p_y*s_x*s_y*2.0-d_y*p_x*s_x*s_y*2.0-d_x*p_z*s_x*s_z*2.0-d_z*p_x*s_x*s_z*2.0-d_y*p_z*s_y*s_z*2.0-d_z*p_y*s_y*s_z*2.0+p_x*p_y*s_x*s_y*2.0+p_x*p_z*s_x*s_z*2.0+p_y*p_z*s_y*s_z*2.0)-s_x*s_x-s_y*s_y-s_z*s_z)/(d_x*s_x*-2.0-d_y*s_y*2.0-d_z*s_z*2.0+d_x*d_x+d_y*d_y+d_z*d_z+s_x*s_x+s_y*s_y+s_z*s_z);

  const std::array<CoordScalar,3> ts = { t1, t2, t3 };

  for (const CoordScalar t : ts)
  {
    if (std::isfinite(t) && (0.6 < t) && (t < 1))
    {
      const Pt3 new_pt = src_pt + (t * (det_pt - src_pt));
      
      const CoordScalar l_ratio = (new_pt - ref_pt).norm() / l;
     
      // original distance should not change too much 
      if ((dist_ratio_lower < l_ratio) && (l_ratio < dist_ratio_upper))
      {
        const CoordScalar df2_dt2 = (pow(d_x-s_x,2.0)*2.0+pow(d_y-s_y,2.0)*2.0+pow(d_z-s_z,2.0)*2.0)*(pow(-p_x+s_x+t*(d_x-s_x),2.0)+pow(-p_y+s_y+t*(d_y-s_y),2.0)+pow(-p_z+s_z+t*(d_z-s_z),2.0)-l*l)*2.0+pow((d_x-s_x)*(-p_x+s_x+t*(d_x-s_x))*2.0+(d_y-s_y)*(-p_y+s_y+t*(d_y-s_y))*2.0+(d_z-s_z)*(-p_z+s_z+t*(d_z-s_z))*2.0,2.0)*2.0;
      
        if (df2_dt2 > 1.0e-6)
        {
          pts.push_back(new_pt);
        }
      }
    }
  }

  if (pts.size() > 2)
  {
    // I have seen a case where approximately the same point is returned 3 times
    // remove the third point
    // two BBs are never going to be injected this close and be useful
    if (((pts[2] - pts[1]).norm() < 0.5) || ((pts[2] - pts[0]).norm() < 0.5))
    {
      pts.resize(2);
    }
  }

  if (pts.size() > 1)
  {
    // should have already handled this case
    xregASSERT(pts.size() < 3);

    // Check if we have duplicate points, remove the second one if so
    // two BBs are never going to be injected this close and be useful
    if ((pts[0] - pts[1]).norm() < 0.5)
    {
      pts.resize(1);
    }
  }

  // should have a max of 2 points (2 minima of quadratic)
  xregASSERT(pts.size() < 3);

  return pts;
}

void xreg::CArmP3P::run_impl()
{
  xregASSERT(this->cams_.size() == 1);
  xregASSERT(this->inds_2d_.size() == 1);
  xregASSERT(this->inds_2d_[0].size() == 3);
  xregASSERT(this->world_pts_3d_.size() == 3);
  
  xregASSERT(!ref_pt_depths_.empty());
 
  xregASSERT(ref_idx_ < 3);

  const auto& cam     = this->cams_[0];
  const auto& pts_3d  = this->world_pts_3d_;
  const auto& inds_2d = this->inds_2d_[0];

  poss_poses_.clear();
  
  const CoordScalar ratio_lower = 1.0 - eps_;
  const CoordScalar ratio_upper = 1.0 + eps_;

  const Pt3& src_pt = cam.pinhole_pt;

  Mat3x3 inter_pt_dists = Mat3x3::Zero();
  inter_pt_dists(0,1) = (pts_3d[0] - pts_3d[1]).norm();
  inter_pt_dists(0,2) = (pts_3d[0] - pts_3d[2]).norm();
  inter_pt_dists(1,0) = inter_pt_dists(0,1);
  inter_pt_dists(1,2) = (pts_3d[1] - pts_3d[2]).norm();
  inter_pt_dists(2,0) = inter_pt_dists(0,2);
  inter_pt_dists(2,1) = inter_pt_dists(1,2);

  Pt3List tmp_pts_3d_world(3);
  Pt3List tmp_pts_3d_cam(3);

  const std::vector<int> ref_inds_to_use = (ref_idx_ < 0) ?
                                             std::vector<int>{ 0, 1, 2 } :
                                             std::vector<int>{ ref_idx_ };

  for (const int& cur_ref_idx : ref_inds_to_use)
  {
    xregASSERT((0 <= cur_ref_idx) && (cur_ref_idx < 3));

    const int cur_other_idx_1 = (ref_idx_ + 2) % 3;
    const int cur_other_idx_2 = (ref_idx_ + 1) % 3;

    const Pt3& cur_ref_pt_3d     = pts_3d[cur_ref_idx];
    const Pt3& cur_other_pt_3d_1 = pts_3d[cur_other_idx_1];
    const Pt3& cur_other_pt_3d_2 = pts_3d[cur_other_idx_2];

    const Pt2& cur_ref_ind_2d     = inds_2d[cur_ref_idx];
    const Pt2& cur_other_ind_2d_1 = inds_2d[cur_other_idx_1];
    const Pt2& cur_other_ind_2d_2 = inds_2d[cur_other_idx_2];
    
    tmp_pts_3d_world = { cur_ref_pt_3d, cur_other_pt_3d_1, cur_other_pt_3d_2 };

    for (const CoordScalar& cur_depth : ref_pt_depths_)
    {
      const Pt3 est_ref_pt_3d_wrt_cam = src_pt + (cur_depth * (cam.ind_pt_to_phys_det_pt(cur_ref_ind_2d) - src_pt));
     
      tmp_pts_3d_cam[0] = est_ref_pt_3d_wrt_cam;

      const auto est_other_pt_3d_1_wrt_cam = FindPossPtsAlongLineForMatch(est_ref_pt_3d_wrt_cam, src_pt,
                                                                       cam.ind_pt_to_phys_det_pt(cur_other_ind_2d_1),
                                                                       inter_pt_dists(cur_ref_idx, cur_other_idx_1),
                                                                          ratio_lower, ratio_upper);

      if (!est_other_pt_3d_1_wrt_cam.empty())
      {
        const auto est_other_pt_3d_2_wrt_cam = FindPossPtsAlongLineForMatch(est_ref_pt_3d_wrt_cam, src_pt,
                                                                         cam.ind_pt_to_phys_det_pt(cur_other_ind_2d_2),
                                                                        inter_pt_dists(cur_ref_idx, cur_other_idx_2),
                                                                            ratio_lower, ratio_upper);

        if (!est_other_pt_3d_2_wrt_cam.empty())
        {
          for (const auto& est_other_pt_3d_1 : est_other_pt_3d_1_wrt_cam)
          {
            for (const auto& est_other_pt_3d_2 : est_other_pt_3d_2_wrt_cam)
            {
              const CoordScalar est_ratio = (est_other_pt_3d_1 - est_other_pt_3d_2).norm()
                                                / inter_pt_dists(cur_other_idx_1, cur_other_idx_2);
              
              if ((ratio_lower < est_ratio) && (est_ratio < ratio_upper))
              {
                // solve 3D/3D problem
                
                tmp_pts_3d_cam[1] = est_other_pt_3d_1;
                tmp_pts_3d_cam[2] = est_other_pt_3d_2;

                const FrameTransform est_cam_to_world = PairedPointRegi3D3D(tmp_pts_3d_cam, tmp_pts_3d_world, 1);

                poss_poses_.push_back(est_cam_to_world);

                if (pose_found_call_back_fn_)
                {
                  pose_found_call_back_fn_(this, cur_ref_idx, est_cam_to_world, tmp_pts_3d_world, tmp_pts_3d_cam);
                }
              }
            }
          }
        }
      }
    }
  }

  if (cluster_poses_)
  {
    std::vector<FrameTransformList> clusters;

    CoordScalar cur_rot_diff   = 0;
    CoordScalar cur_trans_diff = 0;

    constexpr CoordScalar rot_thresh   = 1.0 * kDEG2RAD;
    constexpr CoordScalar trans_thresh = 1.0;

    for (const auto& p : poss_poses_)
    {
      bool added_to_existing_cluster = false;

      for (auto& cur_cluster : clusters)
      {
        for (const auto& cur_cluster_pose : cur_cluster)
        {
          std::tie(cur_rot_diff,cur_trans_diff) = FrameDiffRotAngTransMag(p, cur_cluster_pose);
          
          if ((cur_rot_diff < rot_thresh) && (cur_trans_diff < trans_thresh))
          {
            cur_cluster.push_back(p);
            
            added_to_existing_cluster = true;

            break;
          }
        }

        if (added_to_existing_cluster)
        {
          break;
        }
      }

      if (!added_to_existing_cluster)
      {
        clusters.push_back({ p });
      }
    }

    poss_poses_.clear();
      
    Mat4x4List tmp_mats;
    FrameTransform tmp_pose;
    
    for (const auto& cur_cluster : clusters)
    {
      const size_type num_poses = cur_cluster.size();

      if (num_poses > 1)
      {
        tmp_mats.clear();
        tmp_mats.reserve(num_poses);
        
        for (const auto& p : cur_cluster)
        {
          tmp_mats.push_back(p.matrix());
        }

        tmp_pose.matrix() = SE3FrechetMean(tmp_mats, -2, 20);

        poss_poses_.push_back(tmp_pose);
      }
      else
      {
        poss_poses_.push_back(cur_cluster[0]);
      }
    }
  }

  if (sort_reproj_ && (poss_poses_.size() > 1))
  {
    // sort by reprojection error
    
    std::vector<std::tuple<FrameTransform,CoordScalar>> poses_and_reprojs;
    poses_and_reprojs.reserve(poss_poses_.size());

    for (const auto& p : poss_poses_)
    {
      CoordScalar reproj_error = (cam.phys_pt_to_ind_pt(p * pts_3d[0]).head(2) - inds_2d[0]).squaredNorm() +
                                 (cam.phys_pt_to_ind_pt(p * pts_3d[1]).head(2) - inds_2d[1]).squaredNorm() +
                                 (cam.phys_pt_to_ind_pt(p * pts_3d[2]).head(2) - inds_2d[2]).squaredNorm();
      
     poses_and_reprojs.push_back(std::make_tuple(p, reproj_error)); 
    }
    
    std::sort(poses_and_reprojs.begin(), poses_and_reprojs.end(),
              [] (const std::tuple<FrameTransform,CoordScalar>& x, const std::tuple<FrameTransform,CoordScalar>& y)
              {
                return std::get<1>(x) < std::get<1>(y);
              });

    poss_poses_.clear();
    for (const auto& pe : poses_and_reprojs)
    {
      poss_poses_.push_back(std::get<0>(pe));
    }
  }

  this->regi_cam_to_world_ = !poss_poses_.empty() ? poss_poses_[0] : FrameTransform::Identity();
}
  
bool xreg::CArmP3P::uses_ref_frame() const
{
  return false;
}

void xreg::CArmP3P::set_ref_idx(const int ref_idx)
{
  ref_idx_ = ref_idx;
}

void xreg::CArmP3P::set_eps(const CoordScalar eps)
{
  eps_ = eps;
}

void xreg::CArmP3P::set_cluster_poses(const bool cluster_poses)
{
  cluster_poses_ = cluster_poses;
}

void xreg::CArmP3P::set_sort_reproj(const bool sort_reproj)
{
  sort_reproj_ = sort_reproj;
}

void xreg::CArmP3P::set_ref_pt_depths(const CoordScalarList& ref_pt_depths)
{
  ref_pt_depths_ = ref_pt_depths;
}

void xreg::CArmP3P::set_ref_pt_depths(const CoordScalar start,
                                      const CoordScalar stop,
                                      const CoordScalar inc)
{
  ref_pt_depths_ = ConstSpacedRange({ start, stop, inc }).vals();
}

void xreg::CArmP3P::set_pose_found_call_back_fn(PoseFoundCallBack& pose_found_call_back_fn)
{
  pose_found_call_back_fn_ = pose_found_call_back_fn;
}

const xreg::FrameTransformList& xreg::CArmP3P::poss_poses() const
{
  return poss_poses_;
}
  
int xreg::CArmP3P::num_pts_required()
{
  return 3;
}

