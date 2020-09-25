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

#include "xregImgSimMetric2DPatchCommon.h"

#include <numeric>

#include "xregITKOpenCVUtils.h"
#include "xregSampleUtils.h"
#include "xregAssert.h"
#include "xregTBBUtils.h"


std::array<xreg::size_type,2>
xreg::ImgSimMetric2DPatchCommon::PatchInfo::center_row_col() const
{
  return { start_row + ((stop_row - start_row + 1) / 2),
           start_col + ((stop_col - start_col + 1) / 2) };
}

cv::Rect
xreg::ImgSimMetric2DPatchCommon::PatchInfo::ocv_roi() const
{
  cv::Rect roi;
  
  roi.x = start_col;
  roi.y = start_row;

  roi.width  = stop_col - start_col + 1;
  roi.height = stop_row - start_row + 1;

  return roi;
}
  
xreg::size_type xreg::ImgSimMetric2DPatchCommon::patch_radius() const
{
  return patch_radius_;
}

void xreg::ImgSimMetric2DPatchCommon::set_patch_radius(const size_type r)
{
  xregASSERT(r > 0);
  patch_radius_ = r;
}

void xreg::ImgSimMetric2DPatchCommon::set_patch_stride(const size_type s)
{
  xregASSERT(s > 0);
  patch_stride_ = s;
}

xreg::size_type xreg::ImgSimMetric2DPatchCommon::patch_stride() const
{
  return patch_stride_;
}

void xreg::ImgSimMetric2DPatchCommon::set_compute_mean_of_patch_sims(const bool compute_mean)
{
  compute_mean_of_patch_sims_ = compute_mean;
}

bool xreg::ImgSimMetric2DPatchCommon::compute_mean_of_patch_sims() const
{
  return compute_mean_of_patch_sims_;
}

void xreg::ImgSimMetric2DPatchCommon::set_weight_patch_sims_in_combine(const bool use_wgt)
{
  weight_patch_sims_in_combine_ = use_wgt;
}

bool xreg::ImgSimMetric2DPatchCommon::weight_patch_sims_in_combine() const
{
  return weight_patch_sims_in_combine_;
}

void xreg::ImgSimMetric2DPatchCommon::set_use_mask_for_patch_weighting(const bool u)
{
  use_mask_for_weighting_ = u;
}

bool xreg::ImgSimMetric2DPatchCommon::use_mask_for_patch_weighting() const
{
  return use_mask_for_weighting_;
}

void xreg::ImgSimMetric2DPatchCommon::set_use_mask_for_patch_stats(const bool u)
{
  use_mask_for_patch_stats_ = u;
}

bool xreg::ImgSimMetric2DPatchCommon::use_mask_for_patch_stats() const
{
  return use_mask_for_patch_stats_;
}

const xreg::ImgSimMetric2DPatchCommon::ListOfSimScalarLists&
xreg::ImgSimMetric2DPatchCommon::sim_vals_for_each_patch() const
{
  return sim_vals_for_each_patch_;
}

void xreg::ImgSimMetric2DPatchCommon::set_save_all_per_patch_scores(const bool s)
{
  save_all_per_patch_scores_ = s;
}

bool xreg::ImgSimMetric2DPatchCommon::save_all_per_patch_scores() const
{
  return save_all_per_patch_scores_;
}

void xreg::ImgSimMetric2DPatchCommon::set_normalize_weights_as_prob(const bool norm)
{
  normalize_weights_as_prob_ = norm;
}

bool xreg::ImgSimMetric2DPatchCommon::normalize_weights_as_prob() const
{
  return normalize_weights_as_prob_;
}

void xreg::ImgSimMetric2DPatchCommon::set_choose_rand_patches(const bool choose_rand_patches)
{
  choose_rand_patches_ = choose_rand_patches;
}

bool xreg::ImgSimMetric2DPatchCommon::choose_rand_patches() const
{
  return choose_rand_patches_;
}

void xreg::ImgSimMetric2DPatchCommon::set_num_rand_patches(const size_type num_rand_patches)
{
  num_rand_patches_ = num_rand_patches;
}

xreg::size_type xreg::ImgSimMetric2DPatchCommon::num_rand_patches() const
{
  return num_rand_patches_;
}

void xreg::ImgSimMetric2DPatchCommon::set_rand_patch_min_pixels_sep(const double sep)
{
  rand_patch_min_pixels_sep_ = sep;
}

double xreg::ImgSimMetric2DPatchCommon::rand_patch_min_pixels_sep() const
{
  return rand_patch_min_pixels_sep_;
}

const xreg::ImgSimMetric2DPatchCommon::PatchInfoList&
xreg::ImgSimMetric2DPatchCommon::patch_infos() const
{
  return patch_infos_;
}

xreg::size_type xreg::ImgSimMetric2DPatchCommon::num_patches() const
{
  return do_not_update_patch_inds_to_use_ ? patch_inds_to_use_.size()
                                          : (choose_rand_patches_ ? num_rand_patches_ : patch_infos_.size());
}

void xreg::ImgSimMetric2DPatchCommon::set_from_other(const ImgSimMetric2DPatchCommon& other)
{
  patch_infos_  = other.patch_infos_;
  patch_radius_ = other.patch_radius_;
  patch_stride_ = other.patch_stride_;
  patch_diam_   = other.patch_diam_;

  compute_mean_of_patch_sims_ = other.compute_mean_of_patch_sims_;

  weight_patch_sims_in_combine_ = other.weight_patch_sims_in_combine_;

  use_mask_for_weighting_   = other.use_mask_for_weighting_;
  use_mask_for_patch_stats_ = other.use_mask_for_patch_stats_;

  save_all_per_patch_scores_ = other.save_all_per_patch_scores_;

  normalize_weights_as_prob_ = other.normalize_weights_as_prob_;

  choose_rand_patches_       = other.choose_rand_patches_;
  num_rand_patches_          = other.num_rand_patches_;
  rand_patch_min_pixels_sep_ = other.rand_patch_min_pixels_sep_;

  need_to_recompute_weights_ = other.need_to_recompute_weights_;
  patch_idx_dist_            = other.patch_idx_dist_;

  // should not need to copy this as the weights would have already been
  // included in the patch_infos_
  //wgt_img_ = other.wgt_img_;

  patches_setup_ = true;
}

void xreg::ImgSimMetric2DPatchCommon::set_weights_from_other(const ImgSimMetric2DPatchCommon& other)
{
  const size_type np = patch_infos_.size();
  xregASSERT(np == other.patch_infos_.size());

  for (size_type p = 0; p < np; ++p)
  {
    patch_infos_[p].weight = other.patch_infos_[p].weight;
  }

  patch_idx_dist_ = other.patch_idx_dist_;
}

void xreg::ImgSimMetric2DPatchCommon::set_patches_to_use(const PatchIndexList& patch_inds)
{
  patch_inds_to_use_ = patch_inds;
  do_not_update_patch_inds_to_use_ = true;
}

void xreg::ImgSimMetric2DPatchCommon::reset_patches_to_use()
{
  patch_inds_to_use_.clear();
  do_not_update_patch_inds_to_use_ = false;
}

void xreg::ImgSimMetric2DPatchCommon::set_wgt_img(WgtImgPtr wgt_img)
{
  wgt_img_ = wgt_img;

  need_to_recompute_weights_ = true;
}

xreg::ImgSimMetric2DPatchCommon::WgtImgPtr
xreg::ImgSimMetric2DPatchCommon::wgt_img() const
{
  return wgt_img_;
}
  
void xreg::ImgSimMetric2DPatchCommon::setup_patches(const size_type img_num_rows,
                                                    const size_type img_num_cols,
                                                    cv::Mat* mask,
                                                    const size_type num_mov_imgs,
                                                    const bool seed_rng)
{
  if (seed_rng && choose_rand_patches_)
  {
    SeedRNGEngWithRandDev(&rng_eng_);
  }
  
  if (!patches_setup_)
  {
    patch_diam_ = (2 * patch_radius_) + 1;

    xregASSERT(patch_diam_ <= img_num_rows);
    xregASSERT(patch_diam_ <= img_num_cols);

    const size_type end_row = img_num_rows - 1 - patch_radius_;
    const size_type end_col = img_num_cols - 1 - patch_radius_;

    for (size_type cur_patch_center_row = patch_radius_; cur_patch_center_row <= end_row;
         cur_patch_center_row += patch_stride_)
    {
      for (size_type cur_patch_center_col = patch_radius_; cur_patch_center_col <= end_col;
           cur_patch_center_col += patch_stride_)
      {
        PatchInfo cur_patch_info = { cur_patch_center_row - patch_radius_,
                                     cur_patch_center_col - patch_radius_,
                                     cur_patch_center_row + patch_radius_,
                                     cur_patch_center_col + patch_radius_,
                                     1
                                   };
          
        patch_infos_.push_back(cur_patch_info);
      }
    }

    compute_weights(mask);
    
    patches_setup_ = true;
  }
  
  if (save_all_per_patch_scores_)
  {
    xregASSERT(num_mov_imgs > 0);
    sim_vals_for_each_patch_.assign(num_patches(), std::vector<Scalar>(num_mov_imgs));
  }
}

bool xreg::ImgSimMetric2DPatchCommon::compute_weights(cv::Mat* mask)
{
  bool wgts_updated = false;

  if (need_to_recompute_weights_)
  {
    const size_type num_patches = patch_infos_.size();

    const bool use_mask_wgts = use_mask_for_weighting_ && mask;
    const bool use_img_wgts  = wgt_img_;

    if (use_img_wgts || use_mask_wgts)
    {
      // prefer to use image weights
      if (use_img_wgts)
      {
        const cv::Mat wgt_img_cv = ShallowCopyItkToOpenCV(wgt_img_.GetPointer());

        for (auto& p : patch_infos_)
        {
          const auto rc = p.center_row_col();
          p.weight = wgt_img_cv.at<Scalar>(rc[0],rc[1]);
        }

        if (use_mask_wgts)
        {
          // a mask is also specified to be used - zero out the weights that are masked
          for (auto& p : patch_infos_)
          {
            const auto rc = p.center_row_col();
            if (!mask->at<MaskScalar>(rc[0],rc[1]))
            {
              p.weight = 0;
            }
          }
        }
      }
      else  // if only use_mask_wgts
      {
        auto compute_wgts_by_mask = [&](const RangeType& r)
        {
          cv::Mat mask_roi;
          
          for (size_type patch_idx = r.begin(); patch_idx < r.end(); ++patch_idx)
          {
            PatchInfo& patch_info = patch_infos_[patch_idx];
            
            mask_roi = mask->operator()(patch_info.ocv_roi()); 
            
            size_type num_non_mask_pix = 0;

            for (int r = 0; r < static_cast<int>(patch_diam_); ++r)
            {
              for (int c = 0; c < static_cast<int>(patch_diam_); ++c)
              {
                if (mask_roi.at<MaskScalar>(r,c))
                {
                  ++num_non_mask_pix;
                }
              }
            }

            patch_info.weight = static_cast<Scalar>(num_non_mask_pix) /
                                    (patch_diam_ * patch_diam_);
          }
        };

        ParallelFor(compute_wgts_by_mask, RangeType(0, num_patches));
      }
   
      if (normalize_weights_as_prob_)
      {
        Scalar wgt_sum = 0;
        for (const auto& p : patch_infos_)
        {
          wgt_sum += p.weight;
        }

        for (auto& p : patch_infos_)
        {
          p.weight /= wgt_sum;
        }
      }
    }
    // otherwise keep weight at 1

    if (choose_rand_patches_)
    {
      std::vector<double> wgts;
      wgts.reserve(patch_infos_.size());
      
      for (const auto& p : patch_infos_)
      {
        wgts.push_back(p.weight);
      }
      
      patch_idx_dist_ = std::discrete_distribution<size_type>(wgts.begin(), wgts.end());
    }

    need_to_recompute_weights_ = false;
    wgts_updated = true;
  }

  return wgts_updated;
}

xreg::ImgSimMetric2DPatchCommon::PatchIndexList
xreg::ImgSimMetric2DPatchCommon::patch_indices_to_use()
{
  xregASSERT(patches_setup_);
  
  PatchIndexList inds;

  if (choose_rand_patches_)
  {
    xregASSERT(num_rand_patches_ < patch_infos_.size());
  
    inds.reserve(num_rand_patches_);

    const double min_sep_dist = (rand_patch_min_pixels_sep_ < 0) ?
                                  std::sqrt(2.0 * patch_radius_ * patch_radius_) :
                                  rand_patch_min_pixels_sep_;
    const bool check_min_sep_dist = min_sep_dist > 1.0e-8;

    // used to keep track of where each patch is and if we are trying to keep them
    // separate, then use this to reject newer patches that are too close to existing patches
    using Pt2 = Eigen::Matrix<double,2,1>;
    std::vector<Pt2> inds_center_pts;
    if (check_min_sep_dist)
    {
      inds_center_pts.reserve(num_rand_patches_);
    }

    for (size_type i = 0; i < num_rand_patches_;)
    {
      // random patch index
      const size_type candidate_idx = patch_idx_dist_(rng_eng_);

      if (check_min_sep_dist)
      {
        // verify that the candidate patch is sufficiently separated from existing patches
        // which have been chosen

        const auto cur_center_row_col = patch_infos_[candidate_idx].center_row_col();

        Pt2 candidate_center_pt;
        candidate_center_pt[0] = cur_center_row_col[0];
        candidate_center_pt[1] = cur_center_row_col[1];
       
        // for a small number of points, this linear search should be faster than
        // a spatial data structure 
        bool accept = true;
        for (const auto& existing_pt : inds_center_pts)
        {
          if ((existing_pt - candidate_center_pt).norm() < min_sep_dist)
          {
            // candidate patch is too close to an existing patch
            accept = false;
            break;
          }
        }

        if (accept)
        {
          // candidate patch is sufficiently different, keep track of where it is
          inds_center_pts.push_back(candidate_center_pt);
        }
        else
        {
          // candidate patch is too close, skip to the next loop iteration without
          // adding this patch and choose a new index.
          continue;
        }
      }

      inds.push_back(candidate_idx);
      ++i;
    }
  }
  else
  {
    inds.resize(patch_infos_.size());
    std::iota(inds.begin(), inds.end(), size_type(0));
  }

  xregASSERT(inds.size() == num_patches());
  return inds;
}
