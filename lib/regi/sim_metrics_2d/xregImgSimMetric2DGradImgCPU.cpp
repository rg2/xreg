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

#include "xregImgSimMetric2DGradImgCPU.h"

#include <opencv2/imgproc/imgproc.hpp>

#include "xregOpenCVUtils.h"
#include "xregITKOpenCVUtils.h"

void xreg::ImgSimMetric2DGradImgCPU::allocate_resources()
{
  ImgSimMetric2DCPU::allocate_resources();

  cv::Mat fixed_ocv_img = ShallowCopyItkToOpenCV(this->fixed_img_.GetPointer());

  mov_grad_imgs_x_ = AllocContiguousBufferForOpenCVImages<Scalar>(fixed_ocv_img.rows,
                                                                  fixed_ocv_img.cols,
                                                                  this->num_mov_imgs_,
                                                                  &grad_x_mov_imgs_buf_);

  mov_grad_imgs_y_ = AllocContiguousBufferForOpenCVImages<Scalar>(fixed_ocv_img.rows,
                                                                  fixed_ocv_img.cols,
                                                                  this->num_mov_imgs_,
                                                                  &grad_y_mov_imgs_buf_);

  if (smooth_img_kernel_rad_)
  {
    // temporary image buffer to store a smoothed image prior to grad. computation
    tmp_smooth_img_ = cv::Mat::zeros(fixed_ocv_img.size(), fixed_ocv_img.type());
    
    // smooth the fixed image
    cv::GaussianBlur(fixed_ocv_img, tmp_smooth_img_,
                     cv::Size(smooth_img_kernel_rad_,smooth_img_kernel_rad_), 0, 0);
  }

  // compute gradients of the fixed image
  fixed_grad_img_x_ = cv::Mat::zeros(fixed_ocv_img.size(), fixed_ocv_img.type());
  fixed_grad_img_y_ = cv::Mat::zeros(fixed_ocv_img.size(), fixed_ocv_img.type());

  cv::Sobel(smooth_img_kernel_rad_ ? tmp_smooth_img_ : fixed_ocv_img,
            fixed_grad_img_x_, -1, 1, 0);
  cv::Sobel(smooth_img_kernel_rad_ ? tmp_smooth_img_ : fixed_ocv_img,
            fixed_grad_img_y_, -1, 0, 1);
}

xreg::size_type
xreg::ImgSimMetric2DGradImgCPU::smooth_img_before_sobel_kernel_radius() const
{
  return smooth_img_kernel_rad_;
}

void xreg::ImgSimMetric2DGradImgCPU::set_smooth_img_before_sobel_kernel_radius(const size_type r)
{
  smooth_img_kernel_rad_ = r;
}

void xreg::ImgSimMetric2DGradImgCPU::compute_sobel_grads()
{
  const size_type num_rows = fixed_grad_img_x_.rows;
  const size_type num_cols = fixed_grad_img_x_.cols;

  const size_type num_pix = num_rows * num_cols;

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    cv::Mat mov_ocv_img(num_rows, num_cols, fixed_grad_img_x_.type(),
                        this->mov_imgs_buf_ + (mov_idx * num_pix));
  
    if (smooth_img_kernel_rad_)
    {
      cv::GaussianBlur(mov_ocv_img, tmp_smooth_img_,
                       cv::Size(smooth_img_kernel_rad_,smooth_img_kernel_rad_), 0, 0);
    }

    cv::Sobel(smooth_img_kernel_rad_ ? tmp_smooth_img_ : mov_ocv_img,
              mov_grad_imgs_x_[mov_idx], -1, 1, 0);
    cv::Sobel(smooth_img_kernel_rad_ ? tmp_smooth_img_ : mov_ocv_img,
              mov_grad_imgs_y_[mov_idx], -1, 0, 1);
  }
}
