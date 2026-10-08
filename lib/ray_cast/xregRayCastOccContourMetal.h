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

#ifndef XREGRAYCASTOCCCONTOURMETAL_H_
#define XREGRAYCASTOCCCONTOURMETAL_H_

#include "xregRayCastBaseMetal.h"

namespace xreg
{

/// \brief Ray casting occluding contours using Metal.
///
/// The counterpart of RayCasterOccludingContoursOCL and
/// RayCasterOccludingContoursCPU. A pixel is on an occluding contour when the
/// surface normal, at the surface location along its ray, is nearly
/// perpendicular to the ray (see occlusion_angle_thresh_rad()). 1 is added to
/// the existing value of a pixel on an occluding contour, other pixels are not
/// modified.
///
/// Unlike RayCasterOccludingContoursOCL, backtracking (a binary search to
/// refine the surface location) and continuing after a collision are
/// supported, as in RayCasterOccludingContoursCPU.
class RayCasterOccludingContoursMetal : public RayCasterMetal, public RayCasterOccludingContours
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  RayCasterOccludingContoursMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit RayCasterOccludingContoursMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit RayCasterOccludingContoursMetal(const MetalCmdQueue& queue);

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// The memory buffers are allocated by the parent class, this class only
  /// needs to create the pipeline for the occluding contours kernel.
  void allocate_resources() override;

  void compute(const size_type vol_idx = 0) override;

protected:
  std::string ray_cast_kernels_src() const override;

private:
  MetalComputePipeline pipeline_;
};

}  // xreg

#endif
