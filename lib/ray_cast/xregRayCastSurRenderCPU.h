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

#ifndef XREGRAYCASTSURRENDERCPU_H_
#define XREGRAYCASTSURRENDERCPU_H_

#include "xregRayCastBaseCPU.h"

namespace xreg
{

class RayCasterSurRenderCPU : public RayCasterCPU, public RayCasterSurRenderParamInterface
{
public:
  /// Trivial constructor - no work done
  ///
  /// Relies on the constructor of CameraRayCasterCPU
  RayCasterSurRenderCPU() = default;

  /// \brief Perform each ray cast.
  ///
  /// This is a mult-threaded computation performed on the host (CPU).
  /// Each ray represents a computation task, and collections of tasks
  /// are evaluated in separate threads.
  void compute(const size_type vol_idx = 0) override;

private:

  //using SurRenderParams = typename RayCasterSurfaceRenderParamInterface<PixelType3D>::RenderParams;


};

}  // xreg

#endif

