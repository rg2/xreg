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

// Basic math routines for Metal shaders, the counterpart of xregOpenCLMath.
//
// Metal Shading Language provides vector and matrix types with arithmetic
// operators (e.g. float4x4 * float4), so only routines not provided by Metal
// are needed here. NOTE: the matrix types are in the metal namespace, while the
// vector types are not.
//
// Frame transforms are stored as column-major float4x4 matrices acting on
// homogeneous column vectors, the same convention as xreg::FrameTransform.

#ifndef XREG_METAL_MATH_METAL_
#define XREG_METAL_MATH_METAL_

#include <metal_stdlib>

namespace xreg
{

/// \brief Transform a 3D point with a 4x4 homogeneous frame transform.
inline float3 XformPt(const metal::float4x4 xform, const float3 pt)
{
  return (xform * float4(pt, 1)).xyz;
}

/// \brief Transform a 3D vector with a 4x4 homogeneous frame transform.
///
/// The translation is not applied. Any scaling in the transform is applied,
/// so the transformed vector does not necessarily have the same norm.
inline float3 XformVec(const metal::float4x4 xform, const float3 vec)
{
  return (xform * float4(vec, 0)).xyz;
}

/// \brief Inverse of a rigid frame transform (rotation and translation only).
inline metal::float4x4 RigidInv(const metal::float4x4 xform)
{
  // R^-1 = R^T
  const metal::float3x3 rot_inv = metal::transpose(metal::float3x3(xform[0].xyz, xform[1].xyz,
                                                                   xform[2].xyz));

  // t^-1 = -R^T * t
  const float3 trans_inv = -(rot_inv * xform[3].xyz);

  return metal::float4x4(float4(rot_inv[0], 0), float4(rot_inv[1], 0), float4(rot_inv[2], 0),
                         float4(trans_inv, 1));
}

}  // xreg

#endif

