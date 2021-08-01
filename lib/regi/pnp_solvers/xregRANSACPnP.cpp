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

#include "xregRANSACPnP.h"

#include "xregAssert.h"
#include "xregSampleUtils.h"

void xreg::RANSACPnP::run_impl()
{
  xregASSERT(pnp_prop_ && pnp_);
  
  const int min_num_pts_for_prop = pnp_prop_->num_pts_required();
 
  // only supporting proposals using 3 or 4 points for now
  xregASSERT((min_num_pts_for_prop == 3) || (min_num_pts_for_prop == 4));

  // only supporting a single view for now
  xregASSERT(this->cams_.size() == 1);
  xregASSERT(this->inds_2d_.size() == 1);

  const auto& cam     = this->cams_[0];
  const auto& pts_3d  = this->world_pts_3d_;
  const auto& inds_2d = this->inds_2d_[0];

  pnp_prop_->set_cam(cam);
  pnp_prop_->set_init_cam_to_world(this->init_cam_to_world_);
  pnp_prop_->set_ref_frame(this->ref_frame_, this->ref_frame_maps_world_to_ref_);

  pnp_->set_cam(cam);
  pnp_->set_ref_frame(this->ref_frame_, this->ref_frame_maps_world_to_ref_);

  FrameTransform cur_best_pose;

  cur_best_mean_reproj_ = std::numeric_limits<CoordScalar>::max();
  
  cur_best_num_inliers_ = 0;
  
  const size_type num_pts = pts_3d.size();
  xregASSERT(num_pts == inds_2d.size());

  Pt3List tmp_pts_3d;
  Pt2List tmp_inds_2d;
  
  tmp_pts_3d.reserve(num_pts);
  tmp_inds_2d.reserve(num_pts);

  std::vector<std::vector<size_type>> combos = (min_num_pts_for_prop == 3) ?
                                                  BruteForce3Combos(num_pts) : BruteForce4Combos(num_pts);
  
  if ((num_proposals_ > 0) && (static_cast<size_type>(num_proposals_) < combos.size()))
  {
    std::mt19937 rng_eng;
    SeedRNGEngWithRandDev(&rng_eng);

    std::shuffle(combos.begin(), combos.end(), rng_eng);
    
    combos.resize(num_proposals_);
  }

  for (const auto& cur_combo : combos)
  {
    tmp_pts_3d.clear();
    tmp_inds_2d.clear();

    for (const size_type prop_idx : cur_combo)
    {
      tmp_pts_3d.push_back(pts_3d[prop_idx]);
      tmp_inds_2d.push_back(inds_2d[prop_idx]);
    }

    pnp_prop_->set_inds_2d(tmp_inds_2d);
    pnp_prop_->set_world_pts_3d(tmp_pts_3d);
    pnp_prop_->run();
    FrameTransform cur_pose = pnp_prop_->regi_cam_to_world();
 
    // pose maps camera to world, we want world to cam
    FrameTransform cur_pose_inv = cur_pose.inverse();

    tmp_pts_3d.clear();
    tmp_inds_2d.clear();

    CoordScalar tot_reproj = 0;

    for (size_type i = 0; i < num_pts; ++i)
    {
      const CoordScalar cur_reproj = (cam.phys_pt_to_ind_pt(cur_pose_inv * pts_3d[i]).head(2) - inds_2d[i]).norm();

      if (cur_reproj < inlier_reproj_thresh_pixels_)
      {
        tmp_pts_3d.push_back(pts_3d[i]);
        tmp_inds_2d.push_back(inds_2d[i]);
        
        tot_reproj += cur_reproj;
      }
    }

    const size_type cur_num_inliers = tmp_pts_3d.size();
    
    if (cur_num_inliers >= min_num_inliers_)
    {
      // solve the pnp prob with the inliers
      
      pnp_->set_inds_2d(tmp_inds_2d);
      pnp_->set_world_pts_3d(tmp_pts_3d);
      pnp_->set_init_cam_to_world(cur_pose);
      pnp_->run();
      cur_pose = pnp_->regi_cam_to_world();
      cur_pose_inv = cur_pose.inverse();
    
      tot_reproj = 0;

      for (size_type i = 0; i < cur_num_inliers; ++i)
      {
        tot_reproj += (cam.phys_pt_to_ind_pt(cur_pose_inv * tmp_pts_3d[i]).head(2) - tmp_inds_2d[i]).norm();
      }
    }

    tot_reproj /= cur_num_inliers;

    if ((cur_num_inliers > cur_best_num_inliers_) ||
        ((cur_num_inliers == cur_best_num_inliers_) && (tot_reproj < cur_best_mean_reproj_)))
    {
      // We have more inliers the previous best, or we have equal number with lower reproj. error.
      // Replace the previous best with current solution
      cur_best_num_inliers_ = cur_num_inliers;
      cur_best_mean_reproj_ = tot_reproj;
      cur_best_pose = cur_pose;
      
      //if ((cur_num_inliers == num_pts) || (cur_num_inliers >= 6))
      //{
      //  // we've got enough inliers to trust this result
      //  break;
      //}
    }  
  }

  this->regi_cam_to_world_ = cur_best_pose;
}

bool xreg::RANSACPnP::uses_ref_frame() const
{
  return pnp_prop_->uses_ref_frame() || pnp_->uses_ref_frame();
}

void xreg::RANSACPnP::set_pnp_prop(std::shared_ptr<Landmark2D3DRegi> pnp_prop)
{
  pnp_prop_ = pnp_prop;
}

void xreg::RANSACPnP::set_pnp(std::shared_ptr<Landmark2D3DRegi> pnp)
{
  pnp_ = pnp;
}

void xreg::RANSACPnP::set_num_proposals(const int num_proposals)
{
  num_proposals_ = num_proposals;
}

void xreg::RANSACPnP::set_inlier_reproj_thresh_pixels(const CoordScalar thresh)
{
  inlier_reproj_thresh_pixels_ = thresh;
}

void xreg::RANSACPnP::set_min_num_inliers(const size_type min_num_inliers)
{
  min_num_inliers_ = min_num_inliers;
}

xreg::CoordScalar xreg::RANSACPnP::best_mean_reproj() const
{
  return cur_best_mean_reproj_;
}

xreg::size_type xreg::RANSACPnP::best_num_inliers() const
{
  return cur_best_num_inliers_;
}

