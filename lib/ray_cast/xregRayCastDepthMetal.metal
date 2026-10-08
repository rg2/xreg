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

// Depth ray casting kernel.

#ifndef XREG_RAY_CAST_DEPTH_METAL_METAL_
#define XREG_RAY_CAST_DEPTH_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastBaseMetal.metal"
#include "xregRayCastDepthMetalArgs.h"
#include "xregMetalInterp.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief Depth ray casting kernel.
///
/// Each thread computes one pixel of one projection (see RayCastPixel). Samples
/// are taken along the ray from the focal point through the detector pixel,
/// and the surface is located at the first sample greater than or equal to the
/// threshold. When backtracking steps are requested, a binary search between
/// that sample and the previous sample refines the location.
///
/// The pixel is set to the minimum of the depth (the distance from the focal
/// point to the surface) and the existing pixel value, so pixels without a
/// surface keep the background value (e.g. kRAY_CAST_MAX_DEPTH).
[[host_name("xregDepthKernel")]]
kernel void DepthKernel(constant RayCastMetalArgs& args                 [[buffer(kRAY_CAST_METAL_ARGS)]],
                        const device float3* det_pts                    [[buffer(kRAY_CAST_METAL_DET_PTS)]],
                        const device float3* focal_pts                  [[buffer(kRAY_CAST_METAL_FOCAL_PTS)]],
                        const device metal::float4x4* cam_to_itk_phys   [[buffer(kRAY_CAST_METAL_CAM_TO_ITK_PHYS)]],
                        const device uint* cam_model_for_proj           [[buffer(kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ)]],
                        device float* proj_pixels                       [[buffer(kRAY_CAST_METAL_PROJ_PIXELS)]],
                        constant RayCastDepthMetalArgs& depth_args      [[buffer(kRAY_CAST_METAL_EXTRA_ARGS)]],
                        LinearVolumeSampler::Texture vol_tex            [[texture(kRAY_CAST_METAL_VOL_TEX)]],
                        const uint3 thread_idx                          [[thread_position_in_grid]])
{
  const RayCastPixel pixel(args, thread_idx);

  if (pixel.valid)
  {
    // the ray is not limited by the detector, a surface may be located beyond it
    const RaySegment seg = ComputeRaySegment(args, det_pts, focal_pts, cam_to_itk_phys,
                                             cam_model_for_proj, pixel, RayExtent::kRAY);

    const LinearVolumeSampler vol(vol_tex);

    const float thresh = depth_args.sur_coll_thresh;

    float3 cur_idx = seg.start_idx;

    for (uint step_idx = 0; step_idx <= seg.num_steps; ++step_idx, cur_idx += seg.step_idx)
    {
      const float cur_val = vol(cur_idx);

      if (cur_val >= thresh)
      {
        const float3 sur_idx = BacktrackToSurface(vol, cur_idx, seg.step_idx, cur_val, thresh,
                                                  depth_args.num_backtracking_steps);

        // The distance from the focal point in physical units, which is equal
        // to the distance in the camera frame since the camera pose is rigid.
        const float depth = metal::length(XformVec(depth_args.itk_idx_to_itk_phys_pt_xform,
                                                   sur_idx - seg.pinhole_idx));

        proj_pixels[pixel.pixel_idx] = metal::fmin(proj_pixels[pixel.pixel_idx], depth);

        break;
      }
    }
  }
}

}  // xreg

#endif

