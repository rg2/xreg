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

// Surface rendering ray casting kernel.

#ifndef XREG_RAY_CAST_SUR_RENDER_METAL_METAL_
#define XREG_RAY_CAST_SUR_RENDER_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastBaseMetal.metal"
#include "xregRayCastSurRenderMetalArgs.h"
#include "xregMetalInterp.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief Intensity of a surface location with the Phong illumination model,
///        with the light source and viewer located at the focal point.
///
/// The surface normal is the negated, normalized, volume gradient. When the
/// gradient is zero there is no normal, and the intensity is only the ambient
/// term.
inline float PhongIntensity(constant RayCastSurRenderMetalArgs& params, const float3 vol_grad,
                            const float3 dir_to_light)
{
  float intensity = params.ambient_reflection_ratio;

  const float grad_len = metal::length(vol_grad);

  if (grad_len > 0)
  {
    const float3 normal = -vol_grad / grad_len;

    const float diffuse_dot = metal::dot(dir_to_light, normal);

    if (diffuse_dot > 1.0e-6f)
    {
      intensity += params.diffuse_reflection_ratio * diffuse_dot;

      // the direction of perfect reflection of the light source
      const float3 reflect_dir = ((2 * diffuse_dot) * normal) - dir_to_light;

      // the viewer is also located at the light source
      const float specular_dot = metal::dot(dir_to_light, reflect_dir);

      if (specular_dot > 1.0e-6f)
      {
        intensity += params.specular_reflection_ratio *
                       metal::pow(specular_dot, params.alpha_shininess);
      }
    }
  }

  return intensity;
}

/// \brief Surface rendering kernel.
///
/// Each thread computes one pixel of one projection (see RayCastPixel). The
/// surface is located at the first sample along the ray from the focal point
/// through the detector pixel that is greater than or equal to the threshold,
/// optionally refined with backtracking. The Phong intensity of the surface is
/// added to the existing pixel value (e.g. the background value or a previous
/// result), pixels without a surface are not modified.
[[host_name("xregSurRenderKernel")]]
kernel void SurRenderKernel(constant RayCastMetalArgs& args                   [[buffer(kRAY_CAST_METAL_ARGS)]],
                            const device float3* det_pts                      [[buffer(kRAY_CAST_METAL_DET_PTS)]],
                            const device float3* focal_pts                    [[buffer(kRAY_CAST_METAL_FOCAL_PTS)]],
                            const device metal::float4x4* cam_to_itk_phys     [[buffer(kRAY_CAST_METAL_CAM_TO_ITK_PHYS)]],
                            const device uint* cam_model_for_proj             [[buffer(kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ)]],
                            device float* proj_pixels                         [[buffer(kRAY_CAST_METAL_PROJ_PIXELS)]],
                            constant RayCastSurRenderMetalArgs& sur_args      [[buffer(kRAY_CAST_METAL_EXTRA_ARGS)]],
                            LinearVolumeSampler::Texture vol_tex              [[texture(kRAY_CAST_METAL_VOL_TEX)]],
                            const uint3 thread_idx                            [[thread_position_in_grid]])
{
  const RayCastPixel pixel(args, thread_idx);

  if (pixel.valid)
  {
    // the ray is not limited by the detector, a surface may be located beyond it
    const RaySegment seg = ComputeRaySegment(args, det_pts, focal_pts, cam_to_itk_phys,
                                             cam_model_for_proj, pixel, RayExtent::kRAY);

    const LinearVolumeSampler vol(vol_tex);

    const float thresh = sur_args.sur_coll_thresh;

    float3 cur_idx = seg.start_idx;

    for (uint step_idx = 0; step_idx <= seg.num_steps; ++step_idx, cur_idx += seg.step_idx)
    {
      const float cur_val = vol(cur_idx);

      if (cur_val >= thresh)
      {
        const float3 sur_idx = BacktrackToSurface(vol, cur_idx, seg.step_idx, cur_val, thresh,
                                                  sur_args.num_backtracking_steps);

        // the light source is located at the focal point
        proj_pixels[pixel.pixel_idx] += PhongIntensity(sur_args, CentralDiffGradient(vol, sur_idx),
                                                       -metal::normalize(seg.step_idx));

        break;
      }
    }
  }
}

}  // xreg

#endif
