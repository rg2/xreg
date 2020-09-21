/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
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

#include "xregOpenCLSpatial.h"

#include <boost/compute/utility/source.hpp>

const char* xreg::kOPENCL_SPATIAL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

float2 xregLineSegmentRectIntersect(const float3 min_rect_corner, const float3 max_rect_corner,
                                    const float3 line_start_pt, const float3 line_vec)
{
  float2 t = (float2) (0.0f, 1.0f);

  const float3 line_vec_inv = 1.0f / line_vec;

  const float3 t0 = (min_rect_corner - line_start_pt) * line_vec_inv;
  const float3 t1 = (max_rect_corner - line_start_pt) * line_vec_inv;

  t.x = fmax(t.x, fmax(fmax(fmin(t0.x,t1.x), fmin(t0.y,t1.y)), fmin(t0.z,t1.z)));

  t.y = fmin(t.y, fmin(fmin(fmax(t0.x,t1.x), fmax(t0.y,t1.y)), fmax(t0.z,t1.z)));

  t = (t.x <= t.y) ? t : (float2)(0.0f,0.0f);

  // checks for the case of a line being parallel to volume when at least one line/vector component is zero
  const int3 can_intersect_parallel = (int3) ( (fabs(line_vec.x) > 1.0e-8) ? 1 : (((line_start_pt.x >= min_rect_corner.x) && (line_start_pt.x <= max_rect_corner.x)) ? 1 : 0),
                                               (fabs(line_vec.y) > 1.0e-8) ? 1 : (((line_start_pt.y >= min_rect_corner.y) && (line_start_pt.y <= max_rect_corner.y)) ? 1 : 0),
                                               (fabs(line_vec.z) > 1.0e-8) ? 1 : (((line_start_pt.z >= min_rect_corner.z) && (line_start_pt.z <= max_rect_corner.z)) ? 1 : 0)
                                             );

  t = (can_intersect_parallel.x && can_intersect_parallel.y && can_intersect_parallel.z) ? t : (float2)(0.0f,0.0f);

  return t;
}

float2 xregRayRectIntersect(const float3 min_rect_corner, const float3 max_rect_corner,
                            const float3 line_start_pt, const float3 line_vec)
{
  float2 t = (float2) (0.0f, INFINITY);

  const float3 line_vec_inv = 1.0f / line_vec;

  const float3 t0 = (min_rect_corner - line_start_pt) * line_vec_inv;
  const float3 t1 = (max_rect_corner - line_start_pt) * line_vec_inv;

  t.x = fmax(t.x, fmax(fmax(fmin(t0.x,t1.x), fmin(t0.y,t1.y)), fmin(t0.z,t1.z)));

  t.y = fmin(t.y, fmin(fmin(fmax(t0.x,t1.x), fmax(t0.y,t1.y)), fmax(t0.z,t1.z)));

  t = (t.x <= t.y) ? t : (float2)(0.0f,0.0f);

  // checks for the case of a line being parallel to volume when at least one line/vector component is zero
  const int3 can_intersect_parallel = (int3) ( (fabs(line_vec.x) > 1.0e-8) ? 1 : (((line_start_pt.x >= min_rect_corner.x) && (line_start_pt.x <= max_rect_corner.x)) ? 1 : 0),
                                               (fabs(line_vec.y) > 1.0e-8) ? 1 : (((line_start_pt.y >= min_rect_corner.y) && (line_start_pt.y <= max_rect_corner.y)) ? 1 : 0),
                                               (fabs(line_vec.z) > 1.0e-8) ? 1 : (((line_start_pt.z >= min_rect_corner.z) && (line_start_pt.z <= max_rect_corner.z)) ? 1 : 0)
                                             );

  t = (can_intersect_parallel.x && can_intersect_parallel.y && can_intersect_parallel.z) ? t : (float2)(0.0f,0.0f);

  return t;
}

);
