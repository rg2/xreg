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

#ifndef XREGIMGSIMMETRIC2DNCCCPU_H_
#define XREGIMGSIMMETRIC2DNCCCPU_H_

#include "xregImgSimMetric2DCPU.h"

namespace xreg
{

/// \brief Normalized Cross Correlation similarity metric
///
/// This similarity metric does not modify the fixed image passed to it.
/// THE CONTENTS OF THE MOVING IMAGES BUFFER WILL BE MODIFIED ON CALLS TO compute().
/// This buffer must store all data in a contiguous, row-major, format.
/// For example, images first, then rows, then columns.
class ImgSimMetric2DNCCCPU : public ImgSimMetric2DCPU
{
public:
  /// \brief Constructor - trivial, no computation
  ImgSimMetric2DNCCCPU() = default;
  
  /// \brief Allocates additional resources needed to compute the NCC similarity
  ///        metrices on each moving image, fixed image pair.
  ///
  /// This allocates an additional buffer to store the zero-mean fixed image.
  void allocate_resources() override;

  /// \brief Perform the similarity metric computations
  ///
  /// This will thread/execute concurrently using the computation of a moving
  /// image's similarity metric as the unit of execution.
  void compute() override;

protected:
  void process_mask() override;

private:
  using ImageVec           = Eigen::Matrix<Scalar,1,Eigen::Dynamic>;
  using ImageMaskVec       = Eigen::Matrix<MaskScalar,1,Eigen::Dynamic>;
  using MappedImageMaskVec = Eigen::Map<ImageMaskVec>;

  size_type img_num_rows_ = 0;
  size_type img_num_cols_ = 0;
  size_type img_num_pix_  = 0;

  ImageVec zero_mean_fixed_vec_;

  //ImageVecList zero_mean_mov_vecs_;

  Scalar fixed_img_mean_   = 0;
  Scalar fixed_img_stddev_ = 0;

  ImageMaskVec mask_vec_;

  size_type mask_len_ = 0;
};

}  // xreg

#endif

