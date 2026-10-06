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

#ifndef XREGMETALCONVERT_H_
#define XREGMETALCONVERT_H_

// Conversions from xreg (Eigen) types to the simd types used by Metal, the
// counterpart of xregOpenCLConvert.
//
// The simd types have the same size and alignment as the corresponding Metal
// Shading Language types (e.g. simd_float3 and float3 are both 16 bytes), so
// they may be used in structures and buffers shared with shaders.

#include <simd/simd.h>

#include "xregCommon.h"

namespace xreg
{

inline simd_float3 ConvertToMetal(const Pt3& p)
{
  return simd_make_float3(p[0], p[1], p[2]);
}

/// \brief Convert a frame transform to a column-major 4x4 matrix.
///
/// This acts on homogeneous column vectors, the same as FrameTransform.
inline simd_float4x4 ConvertToMetal(const FrameTransform& xform)
{
  const auto& m = xform.matrix();

  simd_float4x4 dst;

  for (int c = 0; c < 4; ++c)
  {
    dst.columns[c] = simd_make_float4(m(0,c), m(1,c), m(2,c), m(3,c));
  }

  return dst;
}

}  // xreg

#endif

