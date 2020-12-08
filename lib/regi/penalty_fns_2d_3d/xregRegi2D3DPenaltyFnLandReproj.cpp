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

#include "xregRegi2D3DPenaltyFnLandReproj.h"

#include "xregLandmarkMapUtils.h"
#include "xregPerspectiveXform.h"
#include "xregTBBUtils.h"
#include "xregHDF5.h"

void xreg::Regi2D3DPenaltyFnLandReproj::set_lands(const LandMap3& lands_3d_map, const LandMap2& lands_2d_map)
{
  std::tie(lands_3d,lands_2d,std::ignore) = CreateCorrespondencePointLists(lands_3d_map, lands_2d_map);
  num_lands = lands_3d.size();
  xregASSERT(num_lands == lands_2d.size());
}
  
void xreg::Regi2D3DPenaltyFnLandReproj::setup()
{
  Regi2D3DPenaltyFn::setup();

  if (this->save_debug_info_)
  {
    debug_info_ = std::make_shared<LandDebugInfo>();
		debug_info_->vol_idx = vol_idx;
     
    debug_info_->lands_3d = lands_3d;
    debug_info_->lands_2d = lands_2d;
     
    debug_info_->std_dev = std_dev;
     
    debug_info_->use_outlier_det = use_outlier_det;

    debug_info_->outlier_thresh_num_stds = outlier_thresh_num_stds;
  }
  else
  {
    debug_info_ = nullptr;
  }
}
  
void xreg::Regi2D3DPenaltyFnLandReproj::compute(
               const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams, const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt)
{
  const CoordScalar std_dev_div = 2 * std_dev * std_dev;
  
  const FrameTransformList& cam_wrt_vols = cams_wrt_objs[vol_idx];

  xregASSERT(num_projs <= cam_wrt_vols.size());
  this->reg_vals_.resize(num_projs);

  if (use_outlier_det)
  {
    tmp_reproj_dists.resize(num_projs);
    
    if (this->compute_probs_)
    {
      tmp_num_inliers.resize(num_projs);
    }
  }

  auto compute_reg_for_projs_fn = [&] (const RangeType& r)
  {
    for (size_type i = r.begin(); i < r.end(); ++i)
    {
      const FrameTransform cur_vol_wrt_cam = cam_wrt_vols[i].inverse();
      
      CoordScalar sum_sq_dists = 0;
      
      if (use_outlier_det)
      {
        auto& cur_proj_reproj_dists = tmp_reproj_dists[i];
        
        cur_proj_reproj_dists.resize(num_lands);

        CoordScalar sum_dists = 0;

        for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
        {
          const CoordScalar cur_dist = (lands_2d[land_idx] -
                               cams[cam_assocs[i]].phys_pt_to_ind_pt(
                                   cur_vol_wrt_cam * lands_3d[land_idx]).head(2)).norm();
          
          cur_proj_reproj_dists[land_idx] = cur_dist;

          sum_dists += cur_dist;
        }

        const CoordScalar mean_dist = sum_dists / num_lands;

        sum_dists = 0;

        for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
        {
          // could cache this value for later threshold against std dev
          const CoordScalar diff_from_mean = cur_proj_reproj_dists[land_idx] - mean_dist;
          
          sum_dists += diff_from_mean * diff_from_mean;
        }

        const CoordScalar std_dev_dist = std::sqrt(sum_dists / (num_lands - 1));

        const CoordScalar inlier_thresh = (outlier_thresh_num_stds * std_dev_dist) + mean_dist;

        size_type num_inliers = 0;

        sum_sq_dists = 0;

        for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
        {
          const auto& cur_dist = cur_proj_reproj_dists[land_idx];

          if (cur_dist < inlier_thresh)
          {
            sum_sq_dists += cur_dist * cur_dist;

            ++num_inliers;
          }
        }

        if (this->compute_probs_)
        {
          tmp_num_inliers[i] = num_inliers;
        }
      }
      else
      {
        for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
        {
          sum_sq_dists += (lands_2d[land_idx] -
                           cams[cam_assocs[i]].phys_pt_to_ind_pt(
                               cur_vol_wrt_cam * lands_3d[land_idx]).head(2)
                          ).squaredNorm();
        }
      }

      this->reg_vals_[i] = sum_sq_dists / std_dev_div;
    }
  };

  ParallelFor(compute_reg_for_projs_fn, RangeType(0, num_projs));

  if (this->compute_probs_)
  {
    this->log_probs_.resize(num_projs);

    for (size_type i = 0; i < num_projs; ++i)
    {
      const CoordScalar log_norm_const = CoordScalar(use_outlier_det ? tmp_num_inliers[i] : num_lands) *
                                            std::log(CoordScalar(3.141592653589793) * std_dev_div);
      this->log_probs_[i] = -this->reg_vals_[i] - log_norm_const;
    }
  } 
}

std::shared_ptr<xreg::Regi2D3DPenaltyFn::DebugInfo> xreg::Regi2D3DPenaltyFnLandReproj::debug_info()
{
  return debug_info_;
}
    
void xreg::Regi2D3DPenaltyFnLandReproj::LandDebugInfo::write(H5::Group* h5)
{
  WriteStringH5("pen-fn-name", "lands-reproj", h5);

  WriteSingleScalarH5("vol-index", vol_idx, h5);

  WriteListOfPointsAsMatrixH5("inds-2d", lands_2d, h5);
  
  WriteListOfPointsAsMatrixH5("pts-3d-wrt-vol", lands_3d, h5);

  WriteSingleScalarH5("std-dev", std_dev, h5);

  WriteSingleScalarH5("use-outlier-det", use_outlier_det, h5);

  if (use_outlier_det)
  {
    WriteSingleScalarH5("outlier-thresh-std-devs", outlier_thresh_num_stds, h5);
  }
}
    
void xreg::Regi2D3DPenaltyFnLandReproj::LandDebugInfo::read(const H5::Group& h5)
{
  vol_idx = ReadSingleScalarH5ULong("vol-index", h5);

  lands_2d = ReadListOfPointsFromMatrixH5Pt2("inds-2d", h5);

  lands_3d = ReadListOfPointsFromMatrixH5Pt3("pts-3d-wrt-vol", h5);

  std_dev = ReadSingleScalarH5CoordScalar("std-dev", h5);

  use_outlier_det = ReadSingleScalarH5Bool("use-outlier-det", h5);

  if (use_outlier_det)
  {
    outlier_thresh_num_stds = ReadSingleScalarH5CoordScalar("outlier-thresh-std-devs", h5);
  }
}

