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

#ifndef XREGIMGSIMMETRIC2DSSDCPU_H_
#define XREGIMGSIMMETRIC2DSSDCPU_H_

#include "xregImgSimMetric2DCPU.h"

namespace xreg
{

/// \brief CPU version of the Sum of Squared Distances similarity metric
///
class ImgSimMetric2DSSDCPU : public ImgSimMetric2DCPU
{
public:
  /// \brief Constructor - trivial, no computation
  ImgSimMetric2DSSDCPU() = default;
  
  /// \brief No additional resources are needed.
  ///
  /// Will apply a mask, if set, to the fixed image.
  void allocate_resources() override;

  /// \brief Perform the similarity metric computations
  ///
  /// This will thread/execute concurrently using the computation of a moving
  /// image's similarity metric as the unit of execution.
  void compute() override;

  void process_mask() override;

private:
  using ImageVec           = Eigen::Matrix<Scalar, 1, Eigen::Dynamic>;
  using ImageMaskVec       = Eigen::Matrix<MaskScalar, 1, Eigen::Dynamic>;
  using MappedImageVec     = Eigen::Map<ImageVec>;
  using MappedImageMaskVec = Eigen::Map<ImageMaskVec>;
  
  ImageVec fixed_img_vec_;

  ImageMaskVec mask_vec_;
};

}  // xreg

#endif

