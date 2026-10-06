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

// Volume interpolation routines for Metal shaders.

#ifndef XREG_METAL_INTERP_METAL_
#define XREG_METAL_INTERP_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregMetalShaderShared.h"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief true -> hardware linear filtering, false -> software trilinear
///        interpolation.
///
/// Hardware filtering of 32-bit float textures is not supported on every
/// device (see MTLDevice.supports32BitFloatFiltering).
constant bool kHW_LINEAR_INTERP [[function_constant(kMETAL_FN_CONST_HW_LINEAR_INTERP)]];

/// \brief Linear interpolation of a single channel volume at continuous indices.
///
/// Integer indices are located at voxel centers and values outside of the
/// volume are zero. This is consistent with the OpenCL ray casters, which use
/// CLK_ADDRESS_CLAMP with linear filtering.
class LinearVolumeSampler
{
public:
  using Texture = metal::texture3d<float, metal::access::sample>;

  explicit LinearVolumeSampler(const Texture vol_tex)
    : vol_tex_(vol_tex),
      dims_(vol_tex.get_width(), vol_tex.get_height(), vol_tex.get_depth()),
      inv_dims_(1.0f / float3(dims_))
  { }

  /// \brief Interpolated value at a continuous index.
  float operator()(const float3 cont_idx) const
  {
    return kHW_LINEAR_INTERP ? hw_interp(cont_idx) : sw_interp(cont_idx);
  }

private:
  float hw_interp(const float3 cont_idx) const
  {
    // Unnormalized coordinates are not supported for 3D textures, so convert
    // the index to a normalized coordinate (voxel centers are at 0.5 offsets).
    constexpr metal::sampler s(metal::coord::normalized,
                               metal::address::clamp_to_zero,
                               metal::filter::linear);

    return vol_tex_.sample(s, (cont_idx + 0.5f) * inv_dims_).r;
  }

  float sw_interp(const float3 cont_idx) const
  {
    const float3 idx0_f = metal::floor(cont_idx);
    const int3   idx0   = int3(idx0_f);

    // interpolation weights of the next voxel along each dimension
    const float3 w = cont_idx - idx0_f;

    const float c000 = voxel(idx0);
    const float c100 = voxel(idx0 + int3(1, 0, 0));
    const float c010 = voxel(idx0 + int3(0, 1, 0));
    const float c110 = voxel(idx0 + int3(1, 1, 0));
    const float c001 = voxel(idx0 + int3(0, 0, 1));
    const float c101 = voxel(idx0 + int3(1, 0, 1));
    const float c011 = voxel(idx0 + int3(0, 1, 1));
    const float c111 = voxel(idx0 + int3(1, 1, 1));

    const float c00 = metal::mix(c000, c100, w.x);
    const float c10 = metal::mix(c010, c110, w.x);
    const float c01 = metal::mix(c001, c101, w.x);
    const float c11 = metal::mix(c011, c111, w.x);

    const float c0 = metal::mix(c00, c10, w.y);
    const float c1 = metal::mix(c01, c11, w.y);

    return metal::mix(c0, c1, w.z);
  }

  /// \brief Value of a voxel, zero when outside of the volume.
  float voxel(const int3 idx) const
  {
    return metal::all((idx >= 0) && (idx < dims_)) ? vol_tex_.read(uint3(idx)).r : 0.0f;
  }

  Texture vol_tex_;

  int3 dims_;

  float3 inv_dims_;
};

}  // xreg

#endif

