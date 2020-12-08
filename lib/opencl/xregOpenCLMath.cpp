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

#include "xregOpenCLMath.h"

#include <boost/compute/utility/source.hpp>

const char* xreg::kOPENCL_MATH_UTILS_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

float3 xregFloat4HmgToFloat3(const float4 a)
{
  return (float3) (a.x, a.y, a.z);
}

float xregFloat1Norm(const float x)
{
  return fabs(x);
}

float xregFloat2Norm(const float2 x)
{
  return sqrt((x.x * x.x) + (x.y * x.y));
}

float xregFloat3Norm(const float3 x)
{
  return sqrt((x.x * x.x) + (x.y * x.y) + (x.z * x.z));
}

float xregFloat4HmgNorm(const float4 x)
{
  return sqrt((x.x * x.x) + (x.y * x.y) + (x.z * x.z));
}

float xregFloat4Norm(const float4 x)
{
  return sqrt((x.x * x.x) + (x.y * x.y) + (x.z * x.z)  + (x.w * x.w));
}

float xregFloat8Norm(const float8 x)
{
  return sqrt((x.s0 * x.s0) + (x.s1 * x.s1) + (x.s2 * x.s2)  + (x.s3 * x.s3) +
              (x.s4 * x.s4) + (x.s5 * x.s5) + (x.s6 * x.s6)  + (x.s7 * x.s7));
}

float xregFloat16Norm(const float16 x)
{
  return sqrt((x.s0 * x.s0) + (x.s1 * x.s1) + (x.s2 * x.s2)  + (x.s3 * x.s3) +
              (x.s4 * x.s4) + (x.s5 * x.s5) + (x.s6 * x.s6)  + (x.s7 * x.s7) +
              (x.s8 * x.s8) + (x.s9 * x.s9) + (x.sa * x.sa)  + (x.sb * x.sb) +
              (x.sc * x.sc) + (x.sd * x.sd) + (x.se * x.se)  + (x.sf * x.sf));
}

float2 xregFloat2Normalize(const float2 x)
{
  return x / xregFloat2Norm(x);
}

float3 xregFloat3Normalize(const float3 x)
{
  return x / xregFloat3Norm(x);
}

float4 xregFloat4Normalize(const float4 x)
{
  return x / xregFloat4Norm(x);
}

float8 xregFloat8Normalize(const float8 x)
{
  return x / xregFloat8Norm(x);
}

float16 xregFloat16Normalize(const float16 x)
{
  return x / xregFloat16Norm(x);
}

float xregFloat1NormSq(const float x)
{
  return x * x;
}

float xregFloat2NormSq(const float2 x)
{
  return (x.x * x.x) + (x.y * x.y);
}

float xregFloat3NormSq(const float3 x)
{
  return (x.x * x.x) + (x.y * x.y) + (x.z * x.z);
}

float xregFloat4HmgNormSq(const float4 x)
{
  return (x.x * x.x) + (x.y * x.y) + (x.z * x.z);
}

float xregFloat4NormSq(const float4 x)
{
  return (x.x * x.x) + (x.y * x.y) + (x.z * x.z)  + (x.w * x.w);
}

float xregFloat8NormSq(const float8 x)
{
  return (x.s0 * x.s0) + (x.s1 * x.s1) + (x.s2 * x.s2)  + (x.s3 * x.s3) +
         (x.s4 * x.s4) + (x.s5 * x.s5) + (x.s6 * x.s6)  + (x.s7 * x.s7);
}

float xregFloat16NormSq(const float16 x)
{
  return (x.s0 * x.s0) + (x.s1 * x.s1) + (x.s2 * x.s2)  + (x.s3 * x.s3) +
         (x.s4 * x.s4) + (x.s5 * x.s5) + (x.s6 * x.s6)  + (x.s7 * x.s7) +
         (x.s8 * x.s8) + (x.s9 * x.s9) + (x.sa * x.sa)  + (x.sb * x.sb) +
         (x.sc * x.sc) + (x.sd * x.sd) + (x.se * x.se)  + (x.sf * x.sf);
}

float3 xregFrm4x4XformFloat3Pt(const float16 frm, const float3 a)
{
  float3 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y) + (frm.s2 * a.z) + frm.s3;
  b.y = (frm.s4 * a.x) + (frm.s5 * a.y) + (frm.s6 * a.z) + frm.s7;
  b.z = (frm.s8 * a.x) + (frm.s9 * a.y) + (frm.sa * a.z) + frm.sb;

  return b;
}

float3 xregFrm4x4XformFloat3Vec(const float16 frm, const float3 a)
{
  float3 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y) + (frm.s2 * a.z);
  b.y = (frm.s4 * a.x) + (frm.s5 * a.y) + (frm.s6 * a.z);
  b.z = (frm.s8 * a.x) + (frm.s9 * a.y) + (frm.sa * a.z);

  return b;
}

float4 xregFrm4x4XformFloat4Pt(const float16 frm, const float4 a)
{
  float4 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y) + (frm.s2 * a.z) + frm.s3;
  b.y = (frm.s4 * a.x) + (frm.s5 * a.y) + (frm.s6 * a.z) + frm.s7;
  b.z = (frm.s8 * a.x) + (frm.s9 * a.y) + (frm.sa * a.z) + frm.sb;
  b.w = 1;

  return b;
}

float4 xregFrm4x4XformFloat4Vec(const float16 frm, const float4 a)
{
  float4 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y) + (frm.s2 * a.z);
  b.y = (frm.s4 * a.x) + (frm.s5 * a.y) + (frm.s6 * a.z);
  b.z = (frm.s8 * a.x) + (frm.s9 * a.y) + (frm.sa * a.z);
  b.w = 0;

  return b;
}

float16 xregFrm4x4Composition(const float16 f, const float16 g)
{
  float16 h;

  h.s0 = (f.s0 * g.s0) + (f.s1 * g.s4) + (f.s2 * g.s8);
  h.s1 = (f.s0 * g.s1) + (f.s1 * g.s5) + (f.s2 * g.s9);
  h.s2 = (f.s0 * g.s2) + (f.s1 * g.s6) + (f.s2 * g.sa);
  h.s3 = f.s3 + (f.s0 * g.s3) + (f.s1 * g.s7) + (f.s2 * g.sb);
  h.s4 = (f.s4 * g.s0) + (f.s5 * g.s4) + (f.s6 * g.s8);
  h.s5 = (f.s4 * g.s1) + (f.s5 * g.s5) + (f.s6 * g.s9);
  h.s6 = (f.s4 * g.s2) + (f.s5 * g.s6) + (f.s6 * g.sa);
  h.s7 = f.s7 + (f.s4 * g.s3) + (f.s5 * g.s7) + (f.s6 * g.sb);
  h.s8 = (f.s8 * g.s0) + (f.s9 * g.s4) + (f.sa * g.s8);
  h.s9 = (f.s8 * g.s1) + (f.s9 * g.s5) + (f.sa * g.s9);
  h.sa = (f.s8 * g.s2) + (f.s9 * g.s6) + (f.sa * g.sa);
  h.sb = f.sb + (f.s8 * g.s3) + (f.s9 * g.s7) + (f.sa * g.sb);
  h.sc = 0;
  h.sd = 0;
  h.se = 0;
  h.sf = 1;

  return h;
}

float16 xregFrm4x4SE3Inv(const float16 h)
{
  float16 h_inv;
 
  // transpose for rotation inverse: R^-1 = R^T
  
  // row 1
  h_inv.s0 = h.s0;
  h_inv.s1 = h.s4;
  h_inv.s2 = h.s8;
  
  // row 2
  h_inv.s4 = h.s1;
  h_inv.s5 = h.s5;
  h_inv.s6 = h.s9;

  // row 3
  h_inv.s8 = h.s2;
  h_inv.s9 = h.s6;
  h_inv.sa = h.sa;

  // translation inverse: t^-1 = -R^T * t
  h_inv.s3 = -((h_inv.s0 * h.s3) + (h_inv.s1 * h.s7) + (h_inv.s2 * h.sb));
  h_inv.s7 = -((h_inv.s4 * h.s3) + (h_inv.s5 * h.s7) + (h_inv.s6 * h.sb));
  h_inv.sb = -((h_inv.s8 * h.s3) + (h_inv.s9 * h.s7) + (h_inv.sa * h.sb));

  h_inv.sc = 0;
  h_inv.sd = 0;
  h_inv.se = 0;
  h_inv.sf = 1;

  return h_inv;
}

float2 xregFrm3x3XformFloat2Pt(const float16 frm, const float2 a)
{
  float2 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y) + frm.s2;
  b.y = (frm.s3 * a.x) + (frm.s4 * a.y) + frm.s5;

  return b;
}

float2 xregFrm3x3XformFloat2Vec(const float16 frm, const float2 a)
{
  float2 b;

  b.x = (frm.s0 * a.x) + (frm.s1 * a.y);
  b.y = (frm.s3 * a.x) + (frm.s4 * a.y);

  return b;
}

float16 xregFrm3x3Identity()
{
  float16 A;

  A.s0 = 1;
  A.s1 = 0;
  A.s2 = 0;

  A.s3 = 0;
  A.s4 = 1;
  A.s5 = 0;

  A.s6 = 0;
  A.s7 = 0;
  A.s8 = 1;

  return A;
}

float16 xregFrm4x4Identity()
{
  float16 A;

  A.s0 = 1;
  A.s1 = 0;
  A.s2 = 0;
  A.s3 = 0;

  A.s4 = 0;
  A.s5 = 1;
  A.s6 = 0;
  A.s7 = 0;

  A.s8 = 0;
  A.s9 = 0;
  A.sa = 1;
  A.sb = 0;

  A.sc = 0;
  A.sd = 0;
  A.se = 0;
  A.sf = 1;

  return A;
}

float xregFloat2Inner(const float2 x, const float2 y)
{
  return (x.x * y.x) + (x.y * y.y);
}

float xregFloat3Inner(const float3 x, const float3 y)
{
  return (x.x * y.x) + (x.y * y.y) + (x.z * y.z);
}

float xregFloat4HmgInner(const float4 x, const float4 y)
{
  return (x.x * y.x) + (x.y * y.y) + (x.z * y.z);
}

float xregFloat4Inner(const float4 x, const float4 y)
{
  return (x.x * y.x) + (x.y * y.y) + (x.z * y.z) + (x.w * y.w);
}

float xregFloat8Inner(const float8 x, const float8 y)
{
  return (x.s0 * y.s0) + (x.s1 * y.s1) + (x.s2 * y.s2) + (x.s3 * y.s3) +
         (x.s4 * y.s4) + (x.s5 * y.s5) + (x.s6 * y.s6) + (x.s7 * y.s7);
}

float xregFloat16Inner(const float16 x, const float16 y)
{
  return (x.s0 * y.s0) + (x.s1 * y.s1) + (x.s2 * y.s2) + (x.s3 * y.s3) +
         (x.s4 * y.s4) + (x.s5 * y.s5) + (x.s6 * y.s6) + (x.s7 * y.s7) +
         (x.s8 * y.s8) + (x.s9 * y.s9) + (x.sa * y.sa) + (x.sb * y.sb) +
         (x.sc * y.sc) + (x.sd * y.sd) + (x.se * y.se) + (x.sf * y.sf);
}

// Useful for debugging:

void xregPrintVec2(const float2 x)
{
  printf("[ %+9.3f ; %+9.3f ]\n", x.x, x.y);
}

void xregPrintVec3(const float3 x)
{
  printf("[ %+9.3f ; %+9.3f ; %+9.3f ]\n",
          x.x, x.y, x.z);
}

void xregPrintVec4(const float4 x)
{
  printf("[ %+9.3f ; %+9.3f ; %+9.3f ; %+9.3f ]\n",
          x.x, x.y, x.z, x.w);
}

void xregPrintMat4x4(const float16 x)
{
  printf("[ %+9.3f , %+9.3f , %+9.3f , %+9.3f ;\n"
         "  %+9.3f , %+9.3f , %+9.3f , %+9.3f ;\n"
         "  %+9.3f , %+9.3f , %+9.3f , %+9.3f ;\n"
         "  %+9.3f , %+9.3f , %+9.3f , %+9.3f ]\n",
         x.s0, x.s1, x.s2, x.s3,
         x.s4, x.s5, x.s6, x.s7,
         x.s8, x.s9, x.sa, x.sb,
         x.sc, x.sd, x.se, x.sf);
}

);
