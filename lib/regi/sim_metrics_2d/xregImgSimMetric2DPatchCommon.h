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

#ifndef XREGIMGSIMMETRIC2DPATCHCOMMON_H_
#define XREGIMGSIMMETRIC2DPATCHCOMMON_H_

#include <random>
#include <array>

#include <opencv2/core/core.hpp>

#include "xregImgSimMetric2D.h"

namespace xreg
{

class ImgSimMetric2DPatchCommon
{
public:
  using Scalar     = ImgSimMetric2D::Scalar;
  using MaskScalar = ImgSimMetric2D::MaskScalar;

  using ListOfSimScalarLists = std::vector<std::vector<Scalar>>;

  using WgtImg    = itk::Image<Scalar,2>;
  using WgtImgPtr = WgtImg::Pointer;

  struct PatchInfo
  {
    size_type start_row;
    size_type start_col;
    size_type stop_row;
    size_type stop_col;
  
    Scalar weight;

    std::array<size_type,2> center_row_col() const;

    cv::Rect ocv_roi() const;
  };

  using PatchInfoList = std::vector<PatchInfo>;

  using PatchIndexList = std::vector<size_type>;

  size_type patch_radius() const;

  void set_patch_radius(const size_type r);

  void set_patch_stride(const size_type s);

  size_type patch_stride() const;

  void set_compute_mean_of_patch_sims(const bool compute_mean);

  bool compute_mean_of_patch_sims() const;

  void set_weight_patch_sims_in_combine(const bool use_wgt);

  bool weight_patch_sims_in_combine() const;

  void set_use_mask_for_patch_weighting(const bool u);

  bool use_mask_for_patch_weighting() const;

  void set_use_mask_for_patch_stats(const bool u);

  bool use_mask_for_patch_stats() const;
  
  const ListOfSimScalarLists& sim_vals_for_each_patch() const;
  
  void set_save_all_per_patch_scores(const bool s);

  bool save_all_per_patch_scores() const;

  void set_normalize_weights_as_prob(const bool norm);

  bool normalize_weights_as_prob() const;

  void set_choose_rand_patches(const bool choose_rand_patches);

  bool choose_rand_patches() const;

  void set_num_rand_patches(const size_type num_rand_patches);

  size_type num_rand_patches() const;

  void set_rand_patch_min_pixels_sep(const double sep);

  double rand_patch_min_pixels_sep() const;

  const PatchInfoList& patch_infos() const;

  size_type num_patches() const;

  void set_from_other(const ImgSimMetric2DPatchCommon& other);

  void set_weights_from_other(const ImgSimMetric2DPatchCommon& other);

  void set_patches_to_use(const PatchIndexList& patch_inds);

  void reset_patches_to_use();

  void set_wgt_img(WgtImgPtr wgt_img);

  WgtImgPtr wgt_img() const;

protected:

  void setup_patches(const size_type img_num_rows, const size_type img_num_cols,
                     cv::Mat* mask, const size_type num_mov_imgs,
                     const bool seed_rng = true);
  
  bool compute_weights(cv::Mat* mask);

  PatchIndexList patch_indices_to_use();

  PatchInfoList patch_infos_;

  size_type patch_radius_ = 5;
 
  size_type patch_stride_ = 1; 

  // computed from patch_radius in setup()
  size_type patch_diam_ = 0;

  bool compute_mean_of_patch_sims_ = false;

  bool weight_patch_sims_in_combine_ = true;

  bool use_mask_for_weighting_    = true;
  bool use_mask_for_patch_stats_  = false;

  bool save_all_per_patch_scores_ = false;

  bool normalize_weights_as_prob_ = true;
  
  bool choose_rand_patches_ = false;

  size_type num_rand_patches_ = 100;

  //  < 0 --> use std::sqrt(2 * patch_radius_ * patch_radius_)
  // == 0 --> do not check for a minimum separation
  //  > 0 --> use this distance
  double rand_patch_min_pixels_sep_ = -1;

  bool patches_setup_ = false;

  ListOfSimScalarLists sim_vals_for_each_patch_;

  std::mt19937 rng_eng_;
  std::discrete_distribution<size_type> patch_idx_dist_;

  bool do_not_update_patch_inds_to_use_ = false;
  PatchIndexList patch_inds_to_use_;

  WgtImgPtr wgt_img_;
    
  bool need_to_recompute_weights_ = true;
};

}  // xreg

#endif

