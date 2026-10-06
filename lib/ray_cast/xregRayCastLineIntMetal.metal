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

// Line integral ray casting kernels.

#ifndef XREG_RAY_CAST_LINE_INT_METAL_METAL_
#define XREG_RAY_CAST_LINE_INT_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastBaseMetal.metal"
#include "xregMetalInterp.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief Sums the samples along a ray.
struct LineIntSumOp
{
  static float init()
  {
    return 0;
  }

  float operator()(const float a, const float b) const
  {
    return a + b;
  }
};

/// \brief Takes the maximum sample along a ray (maximum intensity projection).
struct LineIntMaxOp
{
  static float init()
  {
    return -FLT_MAX;
  }

  float operator()(const float a, const float b) const
  {
    return metal::fmax(a, b);
  }
};

/// \brief Line integral ray casting kernel, with the operation applied to each
///        sample along the ray (e.g. sum or max) as a template parameter.
///
/// Each thread computes one pixel of one projection, the grid is
/// (number of detector columns) x (number of detector rows) x (number of projections).
/// The operation is also
/// used to combine the ray cast value with the existing pixel value, which is
/// the background value or a previous result.
template <class tOp>
kernel void LineIntKernel(constant RayCastMetalArgs& args                 [[buffer(kRAY_CAST_METAL_ARGS)]],
                          const device float3* det_pts                    [[buffer(kRAY_CAST_METAL_DET_PTS)]],
                          const device float3* focal_pts                  [[buffer(kRAY_CAST_METAL_FOCAL_PTS)]],
                          const device metal::float4x4* cam_to_itk_phys   [[buffer(kRAY_CAST_METAL_CAM_TO_ITK_PHYS)]],
                          const device uint* cam_model_for_proj           [[buffer(kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ)]],
                          device float* proj_pixels                       [[buffer(kRAY_CAST_METAL_PROJ_PIXELS)]],
                          LinearVolumeSampler::Texture vol_tex            [[texture(kRAY_CAST_METAL_VOL_TEX)]],
                          const uint3 thread_idx                          [[thread_position_in_grid]])
{
  const uint det_col  = thread_idx.x;
  const uint det_row  = thread_idx.y;
  const uint proj_idx = thread_idx.z;

  if ((det_col < args.num_det_cols) && (det_row < args.num_det_rows) && (proj_idx < args.num_projs))
  {
    const uint det_idx = (det_row * args.num_det_cols) + det_col;

    const RaySegment seg = ComputeRaySegment(args, det_pts, focal_pts, cam_to_itk_phys,
                                             cam_model_for_proj, det_idx, proj_idx);

    const LinearVolumeSampler vol(vol_tex);

    const tOp op;

    float val = tOp::init();

    float3 cur_idx = seg.start_idx;

    for (uint step_idx = 0; step_idx <= seg.num_steps; ++step_idx, cur_idx += seg.step_idx)
    {
      val = op(val, vol(cur_idx));
    }

    const uint pixel_idx = (proj_idx * args.num_det_pts) + det_idx;

    proj_pixels[pixel_idx] = op(val, proj_pixels[pixel_idx]);
  }
}

template [[host_name("xregLineIntSumKernel")]]
kernel void LineIntKernel<LineIntSumOp>(constant RayCastMetalArgs&, const device float3*,
                                        const device float3*, const device metal::float4x4*,
                                        const device uint*, device float*,
                                        LinearVolumeSampler::Texture, const uint3);

template [[host_name("xregLineIntMaxKernel")]]
kernel void LineIntKernel<LineIntMaxOp>(constant RayCastMetalArgs&, const device float3*,
                                        const device float3*, const device metal::float4x4*,
                                        const device uint*, device float*,
                                        LinearVolumeSampler::Texture, const uint3);

}  // xreg

#endif

