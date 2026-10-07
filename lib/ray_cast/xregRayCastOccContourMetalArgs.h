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

#ifndef XREGRAYCASTOCCCONTOURMETALARGS_H_
#define XREGRAYCASTOCCCONTOURMETALARGS_H_

// Arguments of the Metal occluding contours kernel, shared by the host and the
// kernel. This file is valid C++ and Metal Shading Language, it is also embedded
// in the shader source compiled at runtime.

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastMetalArgs.h"
#endif

namespace xreg
{

/// \brief Occluding contours kernel specific arguments, passed at
///        kRAY_CAST_METAL_EXTRA_ARGS.
struct RayCastOccContourMetalArgs
{
  /// \brief A surface is located at a sample along a ray greater than or equal
  ///        to this value.
  float sur_coll_thresh;

  /// \brief The number of binary search iterations used to refine the location
  ///        of a surface between the last two samples along a ray.
  ray_cast_metal_types::UInt num_backtracking_steps;

  /// \brief A surface location is on an occluding contour when the angle
  ///        between the ray and the surface normal differs from 90 degrees by
  ///        less than this angle (radians).
  float occlusion_angle_thresh_rad;

  /// \brief non-zero -> only the first surface location along a ray is
  ///        checked, zero -> the following samples (greater than or equal to
  ///        the threshold) are also checked until a contour is found.
  ray_cast_metal_types::UInt stop_after_collision;
};

static_assert(sizeof(RayCastOccContourMetalArgs) == 16, "unexpected RayCastOccContourMetalArgs layout");

}  // xreg

#endif
