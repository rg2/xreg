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

#ifndef XREGIMGSIMMETRIC2DGRADNCCMETAL_H_
#define XREGIMGSIMMETRIC2DGRADNCCMETAL_H_

#include "xregImgSimMetric2DGradImgMetal.h"
#include "xregImgSimMetric2DNCCMetal.h"

namespace xreg
{

/// \brief Normalized cross correlation on gradient images using Metal.
///
/// The counterpart of ImgSimMetric2DGradNCCOCL. This computes NCC between the
/// horizontal and vertical derivative (Sobel) fixed and moving images and
/// returns the average as the similarity value.
class ImgSimMetric2DGradNCCMetal : public ImgSimMetric2DGradImgMetal
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DGradNCCMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DGradNCCMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DGradNCCMetal(const MetalCmdQueue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  void process_mask() override;

private:
  // These are created when allocating resources, since this object's command
  // queue may change prior to that (e.g. when using a ray caster's buffer)
  std::unique_ptr<ImgSimMetric2DNCCMetal> grad_x_sim_;
  std::unique_ptr<ImgSimMetric2DNCCMetal> grad_y_sim_;
};

}  // xreg

#endif
