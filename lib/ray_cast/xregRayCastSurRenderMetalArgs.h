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

#ifndef XREGRAYCASTSURRENDERMETALARGS_H_
#define XREGRAYCASTSURRENDERMETALARGS_H_

// Arguments of the Metal surface rendering kernel, shared by the host and the
// kernel. This file is valid C++ and Metal Shading Language, it is also embedded
// in the shader source compiled at runtime.

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregRayCastMetalArgs.h"
#endif

namespace xreg
{

/// \brief Surface rendering kernel specific arguments, passed at
///        kRAY_CAST_METAL_EXTRA_ARGS.
struct RayCastSurRenderMetalArgs
{
  /// \brief A surface is located at the first sample along a ray greater than or
  ///        equal to this value.
  float sur_coll_thresh;

  /// \brief The number of binary search iterations used to refine the location
  ///        of a surface between the last two samples along a ray.
  ray_cast_metal_types::UInt num_backtracking_steps;

  // Phong illumination model parameters, see RayCasterSurRenderShadingParams
  float ambient_reflection_ratio;
  float diffuse_reflection_ratio;
  float specular_reflection_ratio;
  float alpha_shininess;
};

static_assert(sizeof(RayCastSurRenderMetalArgs) == 24, "unexpected RayCastSurRenderMetalArgs layout");

}  // xreg

#endif
