/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#include "xregImgSimMetric2DPatchGradNCCMetal.h"

#include "xregAssert.h"
#include "xregITKOpenCVUtils.h"

xreg::ImgSimMetric2DPatchGradNCCMetal::ImgSimMetric2DPatchGradNCCMetal(const MetalDevice& dev)
  : ImgSimMetric2DGradImgMetal(dev)
{ }

xreg::ImgSimMetric2DPatchGradNCCMetal::ImgSimMetric2DPatchGradNCCMetal(const MetalCmdQueue& queue)
  : ImgSimMetric2DGradImgMetal(queue)
{ }

void xreg::ImgSimMetric2DPatchGradNCCMetal::allocate_resources()
{
  // unsupported options when running on the GPU:
  xregASSERT(!this->use_mask_for_patch_stats_);

  grad_x_sim_ = std::make_unique<ImgSimMetric2DPatchNCCMetal>(this->queue_);
  grad_y_sim_ = std::make_unique<ImgSimMetric2DPatchNCCMetal>(this->queue_);

  // masks are set to grad_x_sim_ and grad_y_sim_ via the parent call to allocate
  // resources, which will make the initial call to process_updated_mask() and
  // in turn call process_mask()
  ImgSimMetric2DGradImgMetal::allocate_resources();

  const auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  // the mask is only used here to compute initial weights on the CPU, so we do
  // not require any other previous processing done to it
  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  this->setup_patches(itk_size[1], itk_size[0], this->mask_ ? &ocv_mask : nullptr,
                      this->num_mov_imgs_);

  grad_x_sim_->set_num_moving_images(this->num_mov_imgs_);
  grad_x_sim_->set_fixed_image(this->fixed_img_);  // still needs to be set for metadata
  grad_x_sim_->set_fixed_image_dev(this->fixed_grad_x_dev_buf_);
  grad_x_sim_->set_mov_imgs_metal_buf(this->mov_grad_x_dev_buf_.get());
  grad_x_sim_->set_from_other(*this);
  grad_x_sim_->allocate_resources();

  grad_y_sim_->set_num_moving_images(this->num_mov_imgs_);
  grad_y_sim_->set_fixed_image(this->fixed_img_);  // still needs to be set for metadata
  grad_y_sim_->set_fixed_image_dev(this->fixed_grad_y_dev_buf_);
  grad_y_sim_->set_mov_imgs_metal_buf(this->mov_grad_y_dev_buf_.get());
  grad_y_sim_->set_from_other(*this);
  grad_y_sim_->allocate_resources();
}

void xreg::ImgSimMetric2DPatchGradNCCMetal::compute()
{
  this->pre_compute();

  this->compute_sobel_grads();

  // perform the similarity calculations on each direction

  grad_x_sim_->set_num_moving_images(this->num_mov_imgs_);
  grad_y_sim_->set_num_moving_images(this->num_mov_imgs_);

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // This uses some state to determine if the weights actually need to be
  // recomputed, returns true if the weights were updated, false otherwise
  if (this->compute_weights(this->mask_ ? &ocv_mask : nullptr))
  {
    grad_x_sim_->set_weights_from_other(*this);
    grad_y_sim_->set_weights_from_other(*this);
  }

  // the same patches are used in each direction
  if (!this->do_not_update_patch_inds_to_use_)
  {
    this->patch_inds_to_use_ = this->patch_indices_to_use();
  }

  grad_x_sim_->set_patches_to_use(this->patch_inds_to_use_);
  grad_y_sim_->set_patches_to_use(this->patch_inds_to_use_);

  grad_x_sim_->compute();
  grad_y_sim_->compute();

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    this->sim_vals_[mov_idx] = 0.5f * (grad_x_sim_->sim_val(mov_idx) + grad_y_sim_->sim_val(mov_idx));
  }
}

void xreg::ImgSimMetric2DPatchGradNCCMetal::process_mask()
{
  // the mask is not used by this object for the gradient computations, only to
  // compute the patch weights
  grad_x_sim_->set_mask(this->mask_);
  grad_y_sim_->set_mask(this->mask_);
}
