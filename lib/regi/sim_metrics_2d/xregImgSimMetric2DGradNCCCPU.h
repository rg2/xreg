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

#ifndef XREGIMGSIMMETRIC2DGRADNCCCPU_H_
#define XREGIMGSIMMETRIC2DGRADNCCCPU_H_

#include "xregImgSimMetric2DGradImgCPU.h"
#include "xregImgSimMetric2DNCCCPU.h"

namespace xreg
{

/// \brief Normalized Cross Correlation on Gradient Images
///
/// This computes NCC between the horizontal and vertical derivative (Sobel)
/// fixed and moving images and returns the average as the similarity value.
class ImgSimMetric2DGradNCCCPU : public ImgSimMetric2DGradImgCPU
{
public:
  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DGradNCCCPU() = default;

  /// \brief Allocation of other resources required and computation of
  ///        fixed image gradients.
  ///
  /// \see GradImageSimMetric2DCPU for a description of gradient image
  /// reoursces allocated.
  /// This will also allocate additional resources for two instances of
  /// NCCSimilarityMetric.
  void allocate_resources() override;

  /// \brief Computation of the similarity metric, by computing NCC on the moving and fixed image gradients.
  ///
  void compute() override;

  const ImgSimMetric2DNCCCPU& ncc_sim_x() const;

  const ImgSimMetric2DNCCCPU& ncc_sim_y() const;

protected:
  void process_mask() override;

private:
  ImgSimMetric2DNCCCPU ncc_sim_x_;
  ImgSimMetric2DNCCCPU ncc_sim_y_;
};

}  // xreg

#endif

