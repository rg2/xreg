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

#ifndef XREGRAYCASTMETALARGS_H_
#define XREGRAYCASTMETALARGS_H_

// Definitions shared by the host and the Metal ray casting kernels. This file is
// valid C++ and Metal Shading Language, it is also embedded in the shader source
// compiled at runtime, so the host and kernels always agree on argument layouts
// and binding indices.

#ifdef __METAL_VERSION__
#include <metal_stdlib>
#else
#include <cstdint>
#include <simd/simd.h>
#endif

namespace xreg
{

/// \brief Types with identical layouts on the host and in Metal shaders.
namespace ray_cast_metal_types
{

#ifdef __METAL_VERSION__
using Float3   = float3;
using Float4x4 = metal::float4x4;
using UInt     = uint;
#else
using Float3   = simd_float3;
using Float4x4 = simd_float4x4;
using UInt     = std::uint32_t;
#endif

}  // ray_cast_metal_types

/// \brief Arguments common to all Metal ray casting kernels.
///
/// The counterpart of RayCasterOCL::RayCastArgs.
struct RayCastMetalArgs
{
  /// \brief Transformation from ITK physical points to continuous volume indices.
  ray_cast_metal_types::Float4x4 itk_phys_pt_to_itk_idx_xform;

  /// \brief Axis-aligned bounds of the volume in continuous indices.
  ray_cast_metal_types::Float3 img_aabb_min;
  ray_cast_metal_types::Float3 img_aabb_max;

  /// \brief Ray casting step size in physical units (e.g. mm).
  float step_size;

  /// \brief Number of detector pixels in each projection.
  ray_cast_metal_types::UInt num_det_pts;

  /// \brief Number of detector rows and columns, num_det_pts == num_det_rows * num_det_cols.
  ray_cast_metal_types::UInt num_det_rows;
  ray_cast_metal_types::UInt num_det_cols;

  /// \brief Number of projections being computed.
  ray_cast_metal_types::UInt num_projs;
};

static_assert(sizeof(RayCastMetalArgs) == 128, "unexpected RayCastMetalArgs layout");

/// \brief Buffer argument indices common to all Metal ray casting kernels.
enum RayCastMetalBufIdx
{
  kRAY_CAST_METAL_ARGS = 0,            ///< constant RayCastMetalArgs&
  kRAY_CAST_METAL_DET_PTS,             ///< const device float3*: detector points of every camera (wrt camera)
  kRAY_CAST_METAL_FOCAL_PTS,           ///< const device float3*: focal point of each camera (wrt camera)
  kRAY_CAST_METAL_CAM_TO_ITK_PHYS,     ///< const device float4x4*: pose of the camera for each projection
  kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ,  ///< const device uint*: camera model index of each projection
  kRAY_CAST_METAL_PROJ_PIXELS,         ///< device float*: projection pixels, read and written
  kRAY_CAST_METAL_EXTRA_ARGS           ///< constant <kernel specific type>&: optional
};

/// \brief Texture argument indices common to all Metal ray casting kernels.
enum RayCastMetalTexIdx
{
  kRAY_CAST_METAL_VOL_TEX = 0  ///< texture3d<float, access::sample>: the volume being ray cast
};

}  // xreg

#endif

