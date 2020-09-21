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

#include "xregImgSimMetric2DGradNCCCPU.h"

#include "xregITKOpenCVUtils.h"

void xreg::ImgSimMetric2DGradNCCCPU::allocate_resources()
{
  ImgSimMetric2DGradImgCPU::allocate_resources();

  ncc_sim_x_.set_num_moving_images(this->num_mov_imgs_);
  ncc_sim_x_.set_mov_imgs_host_buf(&this->grad_x_mov_imgs_buf_[0]);

  ncc_sim_y_.set_num_moving_images(this->num_mov_imgs_);
  ncc_sim_y_.set_mov_imgs_host_buf(&this->grad_y_mov_imgs_buf_[0]);

  // gradients of the fixed image are the fixed images in the NCC metrics
  ncc_sim_x_.set_fixed_image(ShallowCopyOpenCVToItk<Scalar>(
                                                      this->fixed_grad_img_x_));
  ncc_sim_y_.set_fixed_image(ShallowCopyOpenCVToItk<Scalar>(
                                                      this->fixed_grad_img_y_));

  this->process_updated_mask();

  ncc_sim_x_.allocate_resources();
  ncc_sim_y_.allocate_resources();
}

void xreg::ImgSimMetric2DGradNCCCPU::compute()
{
  this->pre_compute();

  this->compute_sobel_grads();

  ncc_sim_x_.compute();
  ncc_sim_y_.compute();

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    this->sim_vals_[mov_idx] = 0.5 * (ncc_sim_x_.sim_val(mov_idx) +
                                             ncc_sim_y_.sim_val(mov_idx));
  }
}

const xreg::ImgSimMetric2DNCCCPU& xreg::ImgSimMetric2DGradNCCCPU::ncc_sim_x() const
{
  return ncc_sim_x_;
}

const xreg::ImgSimMetric2DNCCCPU& xreg::ImgSimMetric2DGradNCCCPU::ncc_sim_y() const
{
  return ncc_sim_y_;
}

void xreg::ImgSimMetric2DGradNCCCPU::process_mask()
{
  ncc_sim_x_.set_mask(this->mask_);
  ncc_sim_y_.set_mask(this->mask_);
}

