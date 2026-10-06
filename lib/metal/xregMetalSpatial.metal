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

// Spatial primitive routines for Metal shaders, the counterpart of xregOpenCLSpatial.
//
// NOTE: these routines rely on IEEE infinity semantics (e.g. 1 / 0 == inf), so
//       libraries using them must be compiled without fast math, which is the
//       default for xreg::MetalLibrary.

#ifndef XREG_METAL_SPATIAL_METAL_
#define XREG_METAL_SPATIAL_METAL_

#include <metal_stdlib>

namespace xreg
{

namespace detail
{

/// \brief Intersection of the line start_pt + t * line_vec, t in [0, t_max],
///        with an axis-aligned box.
inline float2 LineRectIntersect(const float3 min_rect_corner, const float3 max_rect_corner,
                                const float3 line_start_pt, const float3 line_vec,
                                const float t_max)
{
  const float3 line_vec_inv = 1.0f / line_vec;

  const float3 t0 = (min_rect_corner - line_start_pt) * line_vec_inv;
  const float3 t1 = (max_rect_corner - line_start_pt) * line_vec_inv;

  const float3 t_near = metal::fmin(t0, t1);
  const float3 t_far  = metal::fmax(t0, t1);

  float2 t(metal::fmax(0.0f, metal::fmax(metal::fmax(t_near.x, t_near.y), t_near.z)),
           metal::fmin(t_max, metal::fmin(metal::fmin(t_far.x, t_far.y), t_far.z)));

  t = (t.x <= t.y) ? t : float2(0);

  // A line parallel to a box face (zero vector component) can only intersect
  // when its start point is within the box bounds along that dimension.
  const bool3 is_parallel = metal::fabs(line_vec) <= 1.0e-8f;
  const bool3 within_bounds = (line_start_pt >= min_rect_corner) && (line_start_pt <= max_rect_corner);

  return metal::all(!is_parallel || within_bounds) ? t : float2(0);
}

}  // detail

/// \brief Intersection of a line segment with an axis-aligned box.
///
/// The line segment is line_start_pt + t * line_vec, for t in [0, 1].
/// Returns the parameters (t_enter, t_exit) of the intersection, or (0, 0)
/// when there is no intersection.
inline float2 LineSegmentRectIntersect(const float3 min_rect_corner, const float3 max_rect_corner,
                                       const float3 line_start_pt, const float3 line_vec)
{
  return detail::LineRectIntersect(min_rect_corner, max_rect_corner, line_start_pt, line_vec, 1.0f);
}

/// \brief Intersection of a ray with an axis-aligned box.
///
/// The ray is ray_start_pt + t * ray_vec, for t >= 0.
/// Returns the parameters (t_enter, t_exit) of the intersection, or (0, 0)
/// when there is no intersection.
inline float2 RayRectIntersect(const float3 min_rect_corner, const float3 max_rect_corner,
                               const float3 ray_start_pt, const float3 ray_vec)
{
  return detail::LineRectIntersect(min_rect_corner, max_rect_corner, ray_start_pt, ray_vec, INFINITY);
}

}  // xreg

#endif

