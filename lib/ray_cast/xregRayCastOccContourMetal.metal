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

// Occluding contours ray casting kernel.

#ifndef XREG_RAY_CAST_OCC_CONTOUR_METAL_METAL_
#define XREG_RAY_CAST_OCC_CONTOUR_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastBaseMetal.metal"
#include "xregRayCastOccContourMetalArgs.h"
#include "xregMetalInterp.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief true when a surface location is on an occluding contour, e.g. the
///        surface normal is nearly perpendicular to the ray.
///
/// The angle between the ray and the normal differs from 90 degrees by
/// |asin(normal . ray_dir)|. The sign of the normal (and the gradient) does not
/// matter. There is no normal, and therefore no contour, when the gradient is
/// zero.
inline bool IsOccludingContour(const float3 vol_grad, const float3 ray_dir, const float angle_thresh_rad)
{
  const float grad_len = metal::length(vol_grad);

  return (grad_len > 0) &&
         (metal::abs(metal::asin(metal::clamp(metal::dot(vol_grad / grad_len, ray_dir), -1.0f, 1.0f))) <
            angle_thresh_rad);
}

/// \brief Occluding contours kernel.
///
/// Each thread computes one pixel of one projection (see RayCastPixel). Samples
/// are taken along the ray from the focal point through the detector pixel, and
/// a sample greater than or equal to the threshold is a surface location, which
/// is optionally refined with backtracking. When the surface location is on an
/// occluding contour, 1 is added to the existing pixel value (e.g. the
/// background value or a previous result).
///
/// Only the first sample greater than or equal to the threshold is checked
/// when stopping after a collision, otherwise the following samples are also
/// checked until a contour is found, consistent with RayCasterOccludingContoursCPU.
/// The search continues from each sample, not from the refined surface location.
[[host_name("xregOccContourKernel")]]
kernel void OccContourKernel(constant RayCastMetalArgs& args                  [[buffer(kRAY_CAST_METAL_ARGS)]],
                             const device float3* det_pts                     [[buffer(kRAY_CAST_METAL_DET_PTS)]],
                             const device float3* focal_pts                   [[buffer(kRAY_CAST_METAL_FOCAL_PTS)]],
                             const device metal::float4x4* cam_to_itk_phys    [[buffer(kRAY_CAST_METAL_CAM_TO_ITK_PHYS)]],
                             const device uint* cam_model_for_proj            [[buffer(kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ)]],
                             device float* proj_pixels                        [[buffer(kRAY_CAST_METAL_PROJ_PIXELS)]],
                             constant RayCastOccContourMetalArgs& contour_args [[buffer(kRAY_CAST_METAL_EXTRA_ARGS)]],
                             LinearVolumeSampler::Texture vol_tex             [[texture(kRAY_CAST_METAL_VOL_TEX)]],
                             const uint3 thread_idx                           [[thread_position_in_grid]])
{
  const RayCastPixel pixel(args, thread_idx);

  if (pixel.valid)
  {
    // the ray is not limited by the detector, a surface may be located beyond it
    const RaySegment seg = ComputeRaySegment(args, det_pts, focal_pts, cam_to_itk_phys,
                                             cam_model_for_proj, pixel, RayExtent::kRAY);

    const LinearVolumeSampler vol(vol_tex);

    const float thresh = contour_args.sur_coll_thresh;

    const float3 ray_dir = metal::normalize(seg.pinhole_to_det_idx);

    bool is_contour = false;

    float3 cur_idx = seg.start_idx;

    for (uint step_idx = 0; !is_contour && (step_idx <= seg.num_steps);
         ++step_idx, cur_idx += seg.step_idx)
    {
      const float cur_val = vol(cur_idx);

      if (cur_val >= thresh)
      {
        const float3 sur_idx = BacktrackToSurface(vol, cur_idx, seg.step_idx, cur_val, thresh,
                                                  contour_args.num_backtracking_steps);

        is_contour = IsOccludingContour(CentralDiffGradient(vol, sur_idx), ray_dir,
                                        contour_args.occlusion_angle_thresh_rad);

        if (contour_args.stop_after_collision)
        {
          break;
        }
      }
    }

    if (is_contour)
    {
      proj_pixels[pixel.pixel_idx] += 1;
    }
  }
}

}  // xreg

#endif
