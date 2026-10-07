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

#ifndef XREGIMGSIMMETRIC2DPATCHGRADNCCMETAL_H_
#define XREGIMGSIMMETRIC2DPATCHGRADNCCMETAL_H_

#include "xregImgSimMetric2DGradImgMetal.h"
#include "xregImgSimMetric2DPatchNCCMetal.h"

namespace xreg
{

/// \brief Patch-based normalized cross correlation on gradient images using Metal.
///
/// The counterpart of ImgSimMetric2DPatchGradNCCOCL. This computes Patch NCC
/// between the horizontal and vertical derivative (Sobel) fixed and moving
/// images, using the same patches for each direction, and returns the average
/// as the similarity value.
class ImgSimMetric2DPatchGradNCCMetal
  : public ImgSimMetric2DGradImgMetal,
    public ImgSimMetric2DPatchCommon
{
public:
  // Need to redefine these as both parent classes have this alias
  // (they should be the same, but the ambiguity must be resolved)
  using Scalar     = ImgSimMetric2DGradImgMetal::Scalar;
  using MaskScalar = ImgSimMetric2DGradImgMetal::MaskScalar;

  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DPatchGradNCCMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DPatchGradNCCMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DPatchGradNCCMetal(const MetalCmdQueue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  void process_mask() override;

private:
  // These are created when allocating resources, since this object's command
  // queue may change prior to that (e.g. when using a ray caster's buffer)
  std::unique_ptr<ImgSimMetric2DPatchNCCMetal> grad_x_sim_;
  std::unique_ptr<ImgSimMetric2DPatchNCCMetal> grad_y_sim_;
};

}  // xreg

#endif
