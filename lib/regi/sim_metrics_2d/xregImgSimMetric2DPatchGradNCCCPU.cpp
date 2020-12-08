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

#include "xregImgSimMetric2DPatchGradNCCCPU.h"

#include "xregITKBasicImageUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregAssert.h"
#include "xregTBBUtils.h"
#include "xregHDF5.h"

void xreg::ImgSimMetric2DPatchGradNCCCPU::allocate_resources()
{
  ImgSimMetric2DGradImgCPU::allocate_resources();
  
  // cannot use both moving and fixed image patch variances as weights
  xregASSERT((int(use_mov_img_patch_variances_as_wgts_) +
              int(use_fixed_img_patch_variances_as_wgts_)) < 2);

  auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }
  this->setup_patches(itk_size[1], itk_size[0],
                      this->mask_ ? &ocv_mask : nullptr,
                      this->num_mov_imgs_);

  patch_ncc_x_.set_save_aux_info(this->save_aux_info_);
  patch_ncc_x_.set_num_moving_images(this->num_mov_imgs_);
  patch_ncc_x_.set_mov_imgs_host_buf(&this->grad_x_mov_imgs_buf_[0]);
  patch_ncc_x_.set_from_other(*this);
  patch_ncc_x_.set_fixed_image(ShallowCopyOpenCVToItk<Scalar>(this->fixed_grad_img_x_));
  patch_ncc_x_.set_mask(this->mask_);

  patch_ncc_y_.set_save_aux_info(this->save_aux_info_);
  patch_ncc_y_.set_num_moving_images(this->num_mov_imgs_);
  patch_ncc_y_.set_mov_imgs_host_buf(&this->grad_y_mov_imgs_buf_[0]);
  patch_ncc_y_.set_from_other(*this);
  patch_ncc_y_.set_fixed_image(ShallowCopyOpenCVToItk<Scalar>(this->fixed_grad_img_y_));
  patch_ncc_y_.set_mask(this->mask_);
  
  if (use_variances_in_grad_imgs_as_wgts_)
  {
    // if using variances as weights, this will use the variances in the gradient images separately in
    // each sub similarity metric
    
    // we've already asserted that both fixed and moving image patch variances are not simultaneously used

    patch_ncc_x_.set_use_fixed_img_patch_variances_as_wgts(use_fixed_img_patch_variances_as_wgts_);
    patch_ncc_y_.set_use_fixed_img_patch_variances_as_wgts(use_fixed_img_patch_variances_as_wgts_);
    
    patch_ncc_x_.set_use_mov_img_patch_variances_as_wgts(use_mov_img_patch_variances_as_wgts_);
    patch_ncc_y_.set_use_mov_img_patch_variances_as_wgts(use_mov_img_patch_variances_as_wgts_);
  }
  else if (use_mov_img_patch_variances_as_wgts_)
  {
    patch_ncc_x_.set_use_fixed_img_patch_variances_as_wgts(false);
    patch_ncc_y_.set_use_fixed_img_patch_variances_as_wgts(false);

    patch_ncc_x_.set_use_mov_img_patch_variances_as_wgts(true);
    patch_ncc_y_.set_use_mov_img_patch_variances_as_wgts(true);
    
    mov_img_patch_vars_.assign(this->num_mov_imgs_,
                               ScalarList(this->patch_infos_.size(), Scalar(0)));  
    
    patch_ncc_x_.set_other_mov_img_patch_vars(&mov_img_patch_vars_);
    patch_ncc_y_.set_other_mov_img_patch_vars(&mov_img_patch_vars_);
  }
  
  // default to using the scores from the sub objects  
  do_not_use_scores_from_sub_objs_.assign(this->num_mov_imgs_, false);

  this->process_updated_mask();

  patch_ncc_x_.allocate_resources();
  patch_ncc_y_.allocate_resources();
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::compute()
{
  this->pre_compute();
  this->compute_sobel_grads();
  
  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // This uses some state to determine if the weights actually need to be recomputed
  // returns true if the weights were updated, false otherwise
  if (this->compute_weights(this->mask_ ? &ocv_mask : nullptr))
  {
    // if we are using patch variances in the fixed images, then weights should not be updated
    xregASSERT(!use_fixed_img_patch_variances_as_wgts_);

    patch_ncc_x_.set_weights_from_other(*this);
    patch_ncc_y_.set_weights_from_other(*this);
  }
  
  if (enforce_same_patches_in_both_x_and_y_)
  {
    if (!this->do_not_update_patch_inds_to_use_)
    {
      this->patch_inds_to_use_ = this->patch_indices_to_use();
    }

    patch_ncc_x_.set_patches_to_use(this->patch_inds_to_use_);
    patch_ncc_y_.set_patches_to_use(this->patch_inds_to_use_);
  }
  
  const size_type num_patches = this->num_patches();
  
  if (use_mov_img_patch_variances_as_wgts_ && !use_variances_in_grad_imgs_as_wgts_)
  {
    // using variances computed directly from the moving images, not the gradient components
 
    std::fill(do_not_use_scores_from_sub_objs_.begin(), do_not_use_scores_from_sub_objs_.end(), false);

    const size_type num_rows = this->fixed_grad_img_x_.rows;
    const size_type num_cols = this->fixed_grad_img_x_.cols;
    const size_type num_pix  = num_rows * num_cols;


    for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
    {
      cv::Mat mov_ocv_img(num_rows, num_cols, this->fixed_grad_img_x_.type(),
                          this->mov_imgs_buf_ + (mov_idx * num_pix));

      auto& cur_mov_img_patch_vars = mov_img_patch_vars_[mov_idx];

      cur_mov_img_patch_vars.resize(num_patches);
      
      auto mov_patch_vars_fn = [&] (const RangeType& r)
      {
        Scalar tmp_std_dev = 0;

        for (size_type local_patch_idx = r.begin(); local_patch_idx < r.end(); ++local_patch_idx)
        {
          const auto& patch_info = this->patch_infos_[this->patch_inds_to_use_[local_patch_idx]];

          const auto patch_row_col = patch_info.center_row_col();

          if (!this->mask_ || ocv_mask.at<MaskScalar>(patch_row_col[0], patch_row_col[1]))
          {
            cv::Mat mov_roi = mov_ocv_img(patch_info.ocv_roi());
           
            std::tie(std::ignore, tmp_std_dev, std::ignore) =
                        detail::ComputePatchMeanStdDev(mov_roi, nullptr, false);
            
            cur_mov_img_patch_vars[local_patch_idx] = tmp_std_dev * tmp_std_dev;
          }
          else
          {
            cur_mov_img_patch_vars[local_patch_idx] = 0;
          } 
        }
      };

      ParallelFor(mov_patch_vars_fn, RangeType(0, num_patches));
    
      if (this->normalize_weights_as_prob_)
      {
        Scalar var_sum = 0;

        for (const auto& v : cur_mov_img_patch_vars)
        {
          var_sum += v;
        }

        if (var_sum > 1.0e-6)
        {
          for (auto& v : cur_mov_img_patch_vars)
          {
            v /= var_sum;
          }
        }
        else
        {
          do_not_use_scores_from_sub_objs_[mov_idx] = true;
          
          this->sim_vals_[mov_idx] = std::numeric_limits<Scalar>::max();
        
          // TODO: handle this->save_all_per_patch_scores_
        }
      }
    }
  }

  patch_ncc_x_.compute();
  patch_ncc_y_.compute();

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    if (!do_not_use_scores_from_sub_objs_[mov_idx])
    {
      this->sim_vals_[mov_idx] = 0.5 * (patch_ncc_x_.sim_val(mov_idx) + patch_ncc_y_.sim_val(mov_idx));
    }
  }

  if (this->save_all_per_patch_scores_)
  {
    for (size_type patch_idx = 0; patch_idx < num_patches; ++patch_idx)
    {
      auto& dst_patch_sim_vals = this->sim_vals_for_each_patch_[patch_idx];
      
      const auto& src_grad_x_patch_sim_vals = patch_ncc_x_.sim_vals_for_each_patch()[patch_idx];
      const auto& src_grad_y_patch_sim_vals = patch_ncc_y_.sim_vals_for_each_patch()[patch_idx];

      for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
      {
        if (!do_not_use_scores_from_sub_objs_[mov_idx])
        {
          dst_patch_sim_vals[mov_idx] = 0.5 * (src_grad_x_patch_sim_vals[mov_idx] +
                                               src_grad_y_patch_sim_vals[mov_idx]);
        }
      }
    }
  }
  
  if (this->save_aux_info_ && !sim_aux_)
  {
    sim_aux_ = std::make_shared<SimAux>();

    sim_aux_->sim_aux_x = patch_ncc_x_.aux_info();
    sim_aux_->sim_aux_y = patch_ncc_y_.aux_info();
  }
}

const xreg::ImgSimMetric2DPatchNCCCPU&
xreg::ImgSimMetric2DPatchGradNCCCPU::patch_ncc_x() const
{
  return patch_ncc_x_;
}

const xreg::ImgSimMetric2DPatchNCCCPU&
xreg::ImgSimMetric2DPatchGradNCCCPU::patch_ncc_y() const
{
  return patch_ncc_y_;
}

bool xreg::ImgSimMetric2DPatchGradNCCCPU::enforce_same_patches_in_both_x_and_y() const
{
  return enforce_same_patches_in_both_x_and_y_;
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::set_enforce_same_patches_in_both_x_and_y(const bool same_patches)
{
  enforce_same_patches_in_both_x_and_y_ = same_patches;
}

bool xreg::ImgSimMetric2DPatchGradNCCCPU::use_fixed_img_patch_variances_as_wgts() const
{
  return use_fixed_img_patch_variances_as_wgts_;
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::set_use_fixed_img_patch_variances_as_wgts(const bool use_vars_as_wgts)
{
  use_fixed_img_patch_variances_as_wgts_ = use_vars_as_wgts;
}

bool xreg::ImgSimMetric2DPatchGradNCCCPU::use_variances_in_grad_imgs_as_wgts() const
{
  return use_variances_in_grad_imgs_as_wgts_;
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::set_use_variances_in_grad_imgs_as_wgts(const bool use_grad_vars_as_wgts)
{
  use_variances_in_grad_imgs_as_wgts_ = use_grad_vars_as_wgts;
}

bool xreg::ImgSimMetric2DPatchGradNCCCPU::use_mov_img_patch_variances_as_wgts() const
{
  return use_mov_img_patch_variances_as_wgts_;
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::set_use_mov_img_patch_variances_as_wgts(const bool use_vars_as_wgts)
{
  use_mov_img_patch_variances_as_wgts_ = use_vars_as_wgts;
}

std::shared_ptr<xreg::H5ReadWriteInterface>
xreg::ImgSimMetric2DPatchGradNCCCPU::aux_info()
{
  return sim_aux_;
}
  
void xreg::ImgSimMetric2DPatchGradNCCCPU::process_mask()
{
  // NOTE: we are only entering this call if the mask has been changed, or it is the initial run

  ImgSimMetric2DGradImgCPU::process_mask();
  
  patch_ncc_x_.set_mask(this->mask_);
  patch_ncc_y_.set_mask(this->mask_);

  // if use_variances_in_grad_imgs_as_wgts_ == true, then we have already set
  // the appropriate flags of the x,y NCC objects to use variances
  // in allocate_resources()

  if (!use_variances_in_grad_imgs_as_wgts_)
  {
    // not using the gradient image intensities to derive weights

    if (use_fixed_img_patch_variances_as_wgts_)
    {
      // Compute variance in the fixed image and then set as weight image in the x and y dirs
      // Since the mask is used to compute these statistics, then we must recompute the statistics
      // if the mask has changed.
    
      auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

      WgtImgPtr patch_vars     = MakeITK2DVol<Scalar>(itk_size[0], itk_size[0]);
      cv::Mat   patch_vars_ocv = ShallowCopyItkToOpenCV(patch_vars.GetPointer());

      const bool use_mask = this->mask_;
      
      cv::Mat ocv_mask;
      if (use_mask)
      {
        ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
      }

      cv::Mat fixed_ocv_img = ShallowCopyItkToOpenCV(this->fixed_img_.GetPointer());

      auto patch_std_devs_fn = [&] (const RangeType& r)
      {
        for (size_type patch_idx = r.begin(); patch_idx < r.end(); ++patch_idx)
        {
          PatchInfo& patch_info = this->patch_infos_[patch_idx];
          
          const auto roi = patch_info.ocv_roi();

          cv::Mat mask_roi;
          if (use_mask)
          {
            mask_roi = ocv_mask(roi);
          }
        
          cv::Mat fixed_roi = fixed_ocv_img(roi);
       
          Scalar tmp_std_dev = 0;
          std::tie(std::ignore, tmp_std_dev, std::ignore) = 
                       detail::ComputePatchMeanStdDev(fixed_roi,
                                                      use_mask ? &mask_roi : nullptr,
                                                      use_mask);
                                                      //this->use_mask_for_patch_stats_);

          const auto patch_row_col = patch_info.center_row_col();
          
          patch_vars_ocv.at<Scalar>(patch_row_col[0], patch_row_col[1]) = tmp_std_dev * tmp_std_dev;
        }
      };

      ParallelFor(patch_std_devs_fn, RangeType(0, this->patch_infos_.size()));

      patch_ncc_x_.set_use_fixed_img_patch_variances_as_wgts(false);
      patch_ncc_y_.set_use_fixed_img_patch_variances_as_wgts(false);
      patch_ncc_x_.set_use_mov_img_patch_variances_as_wgts(false);
      patch_ncc_y_.set_use_mov_img_patch_variances_as_wgts(false);

      patch_ncc_x_.set_wgt_img(patch_vars);
      patch_ncc_y_.set_wgt_img(patch_vars);
      
      // nothing done in this branch should cause a recomputation of weights stored
      // in this object's list of patch infos, e.g. this->compute_weights() should return false
    }
    else
    {
      // if computing weights from the moving image variances, then the weights need to be recomputed
      // since we use mask information when computing these statistics
      //
      // if not computing weights from moving image variances, then the mask is used to effect
      // potential weighting
      //
      // in all cases we need to trigger the weights to be recomputed in the next call to compute()
      
      this->need_to_recompute_weights_ = true;
    }
  }
  // else, variances in the x,y grad images will be used to compute weights, by already setting the
  // mask in the sub objects, their weights will be recomputed. the weights in this object do not
  // need to be computed
}
    
void xreg::ImgSimMetric2DPatchGradNCCCPU::SimAux::write(H5::Group* h5)
{
  WriteStringH5("sim-aux-type", "patch-grad-ncc-aux", h5, false);
  
  H5::Group xg = h5->createGroup("x");
  sim_aux_x->write(&xg);
  
  H5::Group yg = h5->createGroup("y");
  sim_aux_y->write(&yg);
}

void xreg::ImgSimMetric2DPatchGradNCCCPU::SimAux::read(const H5::Group& h5)
{
  sim_aux_x = std::make_shared<PatchNCCSimAux>();
  sim_aux_x->read(h5.openGroup("x"));
  
  sim_aux_y = std::make_shared<PatchNCCSimAux>();
  sim_aux_y->read(h5.openGroup("y"));
}

