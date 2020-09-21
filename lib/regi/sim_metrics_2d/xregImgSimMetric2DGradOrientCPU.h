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

#ifndef XREGIMGSIMMETRIC2DGRADORIENTCPU_H_
#define XREGIMGSIMMETRIC2DGRADORIENTCPU_H_

#include "xregImgSimMetric2DGradImgCPU.h"

namespace xreg
{

/// \brief 2D/3D similarity metric that compares orientations of
///        2D Sobel gradient vectors in the fixed and moving images.
///
/// For every 2D pixel whose fixed image gradient magnitude and moving
/// image gradient magnitude are both above some thresholds, compute
/// the cosine of the angle between the fixed image gradient unit vector
/// and the moving image gradient unit vector, cos(theta).
/// Compute the mean of (1 - cos(theta))^2 for each of these pixels as
/// the similarity score.
class ImgSimMetric2DGradOrientCPU : public ImgSimMetric2DGradImgCPU
{
public:
  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DGradOrientCPU() = default;

  bool use_median_as_thresh() const;

  void set_use_median_as_thresh(const bool use_med);

  /// \brief Sets the minimum gradient magnitude threshold in the
  ///        fixed image.
  /// Default is 0.1.
  void set_fixed_grad_mag_thresh(const Scalar thresh);

  /// \brief Sets the minimum gradient magnitude threshold in the
  ///        moving images.
  /// Default is 0.1.
  void set_mov_grad_mag_thresh(const Scalar thresh);

  /// \brief Allocation of other resources required and computation of
  ///        fixed image gradient magnitudes and initial candidate mask.
  ///
  /// This will compute the magnitudes and gradient angles of the fixed image.
  /// \see GradImageSimMetric2DCPU for a description of gradient image
  /// reoursces allocated.
  virtual void allocate_resources();

  /// \brief Computes the moving gradient magnitudes and the angles between
  ///        fixed and moving gradient vectors. 
  ///
  /// \see GradImageSimMetric2DCPU for a description of the initial
  ///      gradient image computations.
  virtual void compute();

private:
  Scalar fixed_grad_mag_thresh_ = 0.1;
  Scalar mov_grad_mag_thresh_   = 0.1;

  cv::Mat fixed_grad_mag_;

  cv::Mat use_fixed_mag_;

  PixelBuffer mov_grad_mags_buf_;

  cvMatList mov_grad_mags_;

  PixelBuffer mov_grad_mags_for_med_buf_;

  cvMatList mov_grad_mags_for_med_;

  bool use_median_as_thresh_ = true;;
};

}  // xreg

#endif

