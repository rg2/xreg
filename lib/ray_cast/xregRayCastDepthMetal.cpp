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

#include "xregRayCastDepthMetal.h"

#include "xregAssert.h"
#include "xregITKBasicImageUtils.h"
#include "xregMetalConvert.h"
#include "xregRayCastDepthMetalArgs.h"

namespace xreg
{

// embedded at build time, see lib/ray_cast/CMakeLists.txt
extern const char* const kRAY_CAST_DEPTH_METAL_ARGS_SRC;
extern const char* const kRAY_CAST_DEPTH_METAL_SRC;

}  // xreg

xreg::RayCasterDepthMetal::RayCasterDepthMetal()
{
  this->set_default_bg_pixel_val(kRAY_CAST_MAX_DEPTH);
}

xreg::RayCasterDepthMetal::RayCasterDepthMetal(const MetalDevice& dev)
  : RayCasterMetal(dev)
{
  this->set_default_bg_pixel_val(kRAY_CAST_MAX_DEPTH);
}

xreg::RayCasterDepthMetal::RayCasterDepthMetal(const MetalCmdQueue& queue)
  : RayCasterMetal(queue)
{
  this->set_default_bg_pixel_val(kRAY_CAST_MAX_DEPTH);
}

void xreg::RayCasterDepthMetal::allocate_resources()
{
  RayCasterMetal::allocate_resources();

  pipeline_ = make_ray_cast_pipeline("xregDepthKernel");
}

void xreg::RayCasterDepthMetal::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  RayCastDepthMetalArgs depth_args;

  depth_args.itk_idx_to_itk_phys_pt_xform = ConvertToMetal(
                      ITKImagePhysicalPointTransformsAsEigen(this->vols_[vol_idx].GetPointer()));

  depth_args.sur_coll_thresh = this->render_thresh();

  depth_args.num_backtracking_steps = static_cast<ray_cast_metal_types::UInt>(
                                                    this->num_backtracking_steps());

  run_ray_cast_kernel(pipeline_, vol_idx, &depth_args, sizeof(depth_args));

  compute_helper_post_kernels(vol_idx);
}

std::string xreg::RayCasterDepthMetal::ray_cast_kernels_src() const
{
  std::string src = kRAY_CAST_DEPTH_METAL_ARGS_SRC;
  src += '\n';
  src += kRAY_CAST_DEPTH_METAL_SRC;

  return src;
}

