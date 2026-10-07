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

#ifndef XREGIMGSIMMETRIC2DNCCMETAL_H_
#define XREGIMGSIMMETRIC2DNCCMETAL_H_

#include "xregImgSimMetric2DMetal.h"

namespace xreg
{

/// \brief Normalized cross correlation similarity metric using Metal.
///
/// The counterpart of ImgSimMetric2DNCCOCL, the similarity value is
/// 0.5 * (1 - NCC), computed over the pixels that are not masked out.
class ImgSimMetric2DNCCMetal : public ImgSimMetric2DMetal
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DNCCMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DNCCMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DNCCMetal(const MetalCmdQueue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  /// \brief Also normalizes the fixed image, using the statistics of the
  ///        pixels that are not masked out.
  void process_mask() override;

private:
  MetalComputePipeline fixed_pipeline_;
  MetalComputePipeline ncc_pipeline_;

  /// \brief The fixed image with zero mean and unit standard deviation.
  DevBuf norm_fixed_img_dev_;

  DevBuf sim_vals_dev_;
};

}  // xreg

#endif
