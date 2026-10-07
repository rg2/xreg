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

#include "xregRayCastOccContourMetal.h"

#include "xregAssert.h"
#include "xregRayCastOccContourMetalArgs.h"

namespace xreg
{

// embedded at build time, see lib/ray_cast/CMakeLists.txt
extern const char* const kRAY_CAST_OCC_CONTOUR_METAL_ARGS_SRC;
extern const char* const kRAY_CAST_OCC_CONTOUR_METAL_SRC;

}  // xreg

xreg::RayCasterOccludingContoursMetal::RayCasterOccludingContoursMetal(const MetalDevice& dev)
  : RayCasterMetal(dev)
{ }

xreg::RayCasterOccludingContoursMetal::RayCasterOccludingContoursMetal(const MetalCmdQueue& queue)
  : RayCasterMetal(queue)
{ }

void xreg::RayCasterOccludingContoursMetal::allocate_resources()
{
  RayCasterMetal::allocate_resources();

  pipeline_ = make_ray_cast_pipeline("xregOccContourKernel");
}

void xreg::RayCasterOccludingContoursMetal::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  RayCastOccContourMetalArgs contour_args;

  contour_args.sur_coll_thresh = this->render_thresh();

  contour_args.num_backtracking_steps = static_cast<ray_cast_metal_types::UInt>(
                                                        this->num_backtracking_steps());

  contour_args.occlusion_angle_thresh_rad = static_cast<float>(this->occlusion_angle_thresh_rad());

  contour_args.stop_after_collision = this->stop_after_collision() ? 1 : 0;

  run_ray_cast_kernel(pipeline_, vol_idx, &contour_args, sizeof(contour_args));

  compute_helper_post_kernels(vol_idx);
}

std::string xreg::RayCasterOccludingContoursMetal::ray_cast_kernels_src() const
{
  std::string src = kRAY_CAST_OCC_CONTOUR_METAL_ARGS_SRC;
  src += '\n';
  src += kRAY_CAST_OCC_CONTOUR_METAL_SRC;

  return src;
}
