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

#ifndef XREGRAYCASTSURRENDERMETAL_H_
#define XREGRAYCASTSURRENDERMETAL_H_

#include "xregRayCastBaseMetal.h"

namespace xreg
{

/// \brief Ray casting surface rendering using Metal.
///
/// The counterpart of RayCasterSurRenderOCL and RayCasterSurRenderCPU. The
/// surface is located at the first sample along a ray greater than or equal to
/// the render threshold (optionally refined with backtracking) and shaded with
/// the Phong illumination model, with the light source and viewer at the focal
/// point. The intensity is added to the existing pixel value, pixels without a
/// surface are not modified.
class RayCasterSurRenderMetal : public RayCasterMetal, public RayCasterSurRenderParamInterface
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  RayCasterSurRenderMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit RayCasterSurRenderMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit RayCasterSurRenderMetal(const MetalCmdQueue& queue);

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// The memory buffers are allocated by the parent class, this class only
  /// needs to create the pipeline for the surface rendering kernel.
  void allocate_resources() override;

  void compute(const size_type vol_idx = 0) override;

protected:
  std::string ray_cast_kernels_src() const override;

private:
  MetalComputePipeline pipeline_;
};

}  // xreg

#endif
