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

#include "xregRayCastSurRenderMetal.h"

#include "xregAssert.h"
#include "xregRayCastSurRenderMetalArgs.h"

namespace xreg
{

// embedded at build time, see lib/ray_cast/CMakeLists.txt
extern const char* const kRAY_CAST_SUR_RENDER_METAL_ARGS_SRC;
extern const char* const kRAY_CAST_SUR_RENDER_METAL_SRC;

}  // xreg

xreg::RayCasterSurRenderMetal::RayCasterSurRenderMetal(const MetalDevice& dev)
  : RayCasterMetal(dev)
{ }

xreg::RayCasterSurRenderMetal::RayCasterSurRenderMetal(const MetalCmdQueue& queue)
  : RayCasterMetal(queue)
{ }

void xreg::RayCasterSurRenderMetal::allocate_resources()
{
  RayCasterMetal::allocate_resources();

  pipeline_ = make_ray_cast_pipeline("xregSurRenderKernel");
}

void xreg::RayCasterSurRenderMetal::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  const RayCasterSurRenderShadingParams& shading = this->surface_render_params();

  RayCastSurRenderMetalArgs sur_args;

  sur_args.sur_coll_thresh = this->render_thresh();

  sur_args.num_backtracking_steps = static_cast<ray_cast_metal_types::UInt>(
                                                    this->num_backtracking_steps());

  sur_args.ambient_reflection_ratio  = shading.ambient_reflection_ratio;
  sur_args.diffuse_reflection_ratio  = shading.diffuse_reflection_ratio;
  sur_args.specular_reflection_ratio = shading.specular_reflection_ratio;
  sur_args.alpha_shininess           = shading.alpha_shininess;

  run_ray_cast_kernel(pipeline_, vol_idx, &sur_args, sizeof(sur_args));

  compute_helper_post_kernels(vol_idx);
}

std::string xreg::RayCasterSurRenderMetal::ray_cast_kernels_src() const
{
  std::string src = kRAY_CAST_SUR_RENDER_METAL_ARGS_SRC;
  src += '\n';
  src += kRAY_CAST_SUR_RENDER_METAL_SRC;

  return src;
}
