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

#ifndef XREGIMGSIMMETRIC2DGRADIMGCPU_H_
#define XREGIMGSIMMETRIC2DGRADIMGCPU_H_

#include <opencv2/core/core.hpp>

#include "xregImgSimMetric2DCPU.h"
#include "xregImgSimMetric2DGradImgParamInterface.h"

namespace xreg
{

/// \brief Abstract class for 2D/3D similarity metrics using
///        2D gradient images on the CPU.
///
/// This base class will compute horizontal and vertical Sobel
/// derivatives of the fixed and moving images; it is up to the
/// derived class to compute the appropriate similarity score.
class ImgSimMetric2DGradImgCPU
  : public ImgSimMetric2DCPU,
    public ImgSimMetric2DGradImgParamInterface
{
public:
  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DGradImgCPU() = default;
  
  /// \brief Allocation of other resources required and computation of fixed image gradients.
  ///
  /// This call allocates memory for storing the horizontal and vertical fixed image gradients
  /// and a buffer for storing the gradients of moving images.
  /// This call also computes the fixed image gradients.
  void allocate_resources() override;

  size_type smooth_img_before_sobel_kernel_radius() const override;

  void set_smooth_img_before_sobel_kernel_radius(const size_type r) override;

protected:
  using PixelBuffer = std::vector<Scalar>;
  using cvMatList   = std::vector<cv::Mat>;
  
  /// \brief Computation of the fixed image gradients. Should be called by
  ///        the derived class in compute()
  ///
  /// For each gradient direction, this makes serial calls to cv::Sobel
  /// (which may be threaded) for each moving image.
  void compute_sobel_grads();

  cv::Mat fixed_grad_img_x_;
  cv::Mat fixed_grad_img_y_;

  PixelBuffer grad_x_mov_imgs_buf_;
  PixelBuffer grad_y_mov_imgs_buf_;

  cvMatList mov_grad_imgs_x_;
  cvMatList mov_grad_imgs_y_; 

  size_type smooth_img_kernel_rad_ = 5;
  cv::Mat tmp_smooth_img_;
};

}  // xreg

#endif

