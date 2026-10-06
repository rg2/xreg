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

// Routines common to the Metal ray casting kernels.

#ifndef XREG_RAY_CAST_BASE_METAL_METAL_
#define XREG_RAY_CAST_BASE_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastMetalArgs.h"
#include "xregMetalMath.metal"
#include "xregMetalSpatial.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief The portion of a ray, from the focal point to a detector point, that
///        intersects the volume, with respect to continuous volume indices.
struct RaySegment
{
  /// \brief The focal point.
  float3 pinhole_idx;

  /// \brief The vector from the focal point to the detector point.
  float3 pinhole_to_det_idx;

  /// \brief Parameters along pinhole_to_det_idx of the volume entry and exit,
  ///        (0, 0) when the ray does not intersect the volume.
  float2 t;

  /// \brief The first sample location (the volume entry point).
  float3 start_idx;

  /// \brief The vector between consecutive samples, the length corresponds to
  ///        the physical step size.
  float3 step_idx;

  /// \brief Samples are located at start_idx + (i * step_idx) for i in [0, num_steps].
  uint num_steps;
};

/// \brief Compute the ray segment through the volume for a detector pixel in a
///        projection.
///
/// This is the same computation used by the OpenCL ray casters.
inline RaySegment ComputeRaySegment(constant RayCastMetalArgs& args,
                                    const device float3* det_pts,
                                    const device float3* focal_pts,
                                    const device metal::float4x4* cam_to_itk_phys_xforms,
                                    const device uint* cam_model_for_proj,
                                    const uint det_idx,
                                    const uint proj_idx)
{
  const uint cam_idx = cam_model_for_proj[proj_idx];

  const float3 focal_pt_wrt_cam   = focal_pts[cam_idx];
  const float3 cur_det_pt_wrt_cam = det_pts[(cam_idx * args.num_det_pts) + det_idx];

  const metal::float4x4 xform_cam_to_itk_idx = args.itk_phys_pt_to_itk_idx_xform *
                                                 cam_to_itk_phys_xforms[proj_idx];

  RaySegment seg;

  seg.pinhole_idx        = XformPt(xform_cam_to_itk_idx, focal_pt_wrt_cam);
  seg.pinhole_to_det_idx = XformPt(xform_cam_to_itk_idx, cur_det_pt_wrt_cam) - seg.pinhole_idx;

  seg.t = LineSegmentRectIntersect(args.img_aabb_min, args.img_aabb_max,
                                   seg.pinhole_idx, seg.pinhole_to_det_idx);

  seg.start_idx = seg.pinhole_idx + (seg.t.x * seg.pinhole_to_det_idx);

  const float pinhole_to_det_len_idx = metal::length(seg.pinhole_to_det_idx);
  const float intersect_len_idx      = (seg.t.y - seg.t.x) * pinhole_to_det_len_idx;

  // NOTE: the transformation from camera to indices includes scaling (by the
  //       voxel spacings), so the norm of a transformed step vector must be
  //       computed
  const float step_len_idx = metal::length(XformVec(xform_cam_to_itk_idx,
                                 metal::normalize(cur_det_pt_wrt_cam - focal_pt_wrt_cam) *
                                   args.step_size));

  seg.num_steps = static_cast<uint>(intersect_len_idx / step_len_idx);

  seg.step_idx = seg.pinhole_to_det_idx * (step_len_idx / pinhole_to_det_len_idx);

  return seg;
}

}  // xreg

#endif

