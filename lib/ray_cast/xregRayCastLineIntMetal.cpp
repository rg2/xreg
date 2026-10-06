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

#include "xregRayCastLineIntMetal.h"

#include "xregAssert.h"
#include "xregExceptionUtils.h"

namespace xreg
{

// embedded at build time from xregRayCastLineIntMetal.metal
extern const char* const kRAY_CAST_LINE_INT_METAL_SRC;

}  // xreg

xreg::RayCasterLineIntMetal::RayCasterLineIntMetal(const MetalDevice& dev)
  : RayCasterMetal(dev)
{ }

xreg::RayCasterLineIntMetal::RayCasterLineIntMetal(const MetalCmdQueue& queue)
  : RayCasterMetal(queue)
{ }

void xreg::RayCasterLineIntMetal::allocate_resources()
{
  RayCasterMetal::allocate_resources();

  std::string kernel_name;

  switch (this->kernel_id())
  {
    case kRAY_CAST_LINE_INT_SUM_KERNEL:
      kernel_name = "xregLineIntSumKernel";
      break;
    case kRAY_CAST_LINE_INT_MAX_KERNEL:
      kernel_name = "xregLineIntMaxKernel";
      break;
    default:
      xregThrow("Unsupported Line Integral Kernel!");
  }

  pipeline_ = make_ray_cast_pipeline(kernel_name);
}

void xreg::RayCasterLineIntMetal::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  run_ray_cast_kernel(pipeline_, vol_idx);

  compute_helper_post_kernels(vol_idx);
}

std::string xreg::RayCasterLineIntMetal::ray_cast_kernels_src() const
{
  return kRAY_CAST_LINE_INT_METAL_SRC;
}

