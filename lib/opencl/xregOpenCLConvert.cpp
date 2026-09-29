/*
 * MIT License
 *
 * Copyright (c) 2020-2026 Robert Grupp
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

#include "xregOpenCLConvert.h"

namespace  // un-named
{

template <class T, int opts, int maxrows, int maxcols>
cl_float3 ConvertToOpenCLHelper(const Eigen::Matrix<T,3,1,opts,maxrows,maxcols>& v)
{
  cl_float3 u;
  u.s[0] = static_cast<float>(v(0));
  u.s[1] = static_cast<float>(v(1));
  u.s[2] = static_cast<float>(v(2));
  return u;
}

template <class T, int opts, int maxrows, int maxcols>
cl_float2 ConvertToOpenCLHelper(const Eigen::Matrix<T,2,1,opts,maxrows,maxcols>& v)
{
  cl_float2 u;
  u.s[0] = static_cast<float>(v(0));
  u.s[1] = static_cast<float>(v(1));
  return u;
}

/// \brief Converts an Eigen 4x1 point/vector into an OpenCL float4.
template <class T, int opts, int maxrows, int maxcols>
cl_float4 ConvertToOpenCLHelper(const Eigen::Matrix<T,4,1,opts,maxrows,maxcols>& v)
{
  cl_float4 u;
  u.s[0] = static_cast<float>(v(0));
  u.s[1] = static_cast<float>(v(1));
  u.s[2] = static_cast<float>(v(2));
  u.s[3] = static_cast<float>(v(3));
  return u;
}

/// \brief Converts an Eigen 8x1 point/vector into an OpenCL float8.
template <class T, int opts, int maxrows, int maxcols>
cl_float8 ConvertToOpenCLHelper(const Eigen::Matrix<T,8,1,opts,maxrows,maxcols>& v)
{
  cl_float8 u;
  u.s[0] = static_cast<float>(v(0));
  u.s[1] = static_cast<float>(v(1));
  u.s[2] = static_cast<float>(v(2));
  u.s[3] = static_cast<float>(v(3));
  u.s[4] = static_cast<float>(v(4));
  u.s[5] = static_cast<float>(v(5));
  u.s[6] = static_cast<float>(v(6));
  u.s[7] = static_cast<float>(v(7));
  return u;
}

/// \brief Converts an Eigen 16x1 point/vector into an OpenCL float8.
template <class T, int opts, int maxrows, int maxcols>
cl_float16 ConvertToOpenCLHelper(const Eigen::Matrix<T,16,1,opts,maxrows,maxcols>& v)
{
  cl_float16 u;
  u.s[0]  = static_cast<float>(v(0));
  u.s[1]  = static_cast<float>(v(1));
  u.s[2]  = static_cast<float>(v(2));
  u.s[3]  = static_cast<float>(v(3));
  u.s[4]  = static_cast<float>(v(4));
  u.s[5]  = static_cast<float>(v(5));
  u.s[6]  = static_cast<float>(v(6));
  u.s[7]  = static_cast<float>(v(7));
  u.s[8]  = static_cast<float>(v(8));
  u.s[9]  = static_cast<float>(v(9));
  u.s[10] = static_cast<float>(v(10));
  u.s[11] = static_cast<float>(v(11));
  u.s[12] = static_cast<float>(v(12));
  u.s[13] = static_cast<float>(v(13));
  u.s[14] = static_cast<float>(v(14));
  u.s[15] = static_cast<float>(v(15));
  return u;
}

template <class T, int _opts>
cl_float16 ConvertToOpenCLHelper(const Eigen::Transform<T,3,Eigen::Affine,_opts>& xform)
{
  using XformType  = Eigen::Transform<T,3,Eigen::Affine,_opts>;
  using MatrixType = typename XformType::MatrixType;

  const MatrixType& A = xform.matrix();

  cl_float16 cl_A;

  // store in row major
  cl_A.s[0]  = static_cast<float>(A(0,0));  // affine
  cl_A.s[1]  = static_cast<float>(A(0,1));  // affine
  cl_A.s[2]  = static_cast<float>(A(0,2));  // affine
  cl_A.s[3]  = static_cast<float>(A(0,3));  // translation
  cl_A.s[4]  = static_cast<float>(A(1,0));  // affine
  cl_A.s[5]  = static_cast<float>(A(1,1));  // affine
  cl_A.s[6]  = static_cast<float>(A(1,2));  // affine
  cl_A.s[7]  = static_cast<float>(A(1,3));  // translation
  cl_A.s[8]  = static_cast<float>(A(2,0));  // affine
  cl_A.s[9]  = static_cast<float>(A(2,1));  // affine
  cl_A.s[10] = static_cast<float>(A(2,2));  // affine
  cl_A.s[11] = static_cast<float>(A(2,3));  // translation
  cl_A.s[12] = 0;
  cl_A.s[13] = 0;
  cl_A.s[14] = 0;
  cl_A.s[15] = 1;

  return cl_A;
}

template <class T, int topts>
cl_float16 ConvertToOpenCLHelper(const Eigen::Transform<T,2,Eigen::Affine,topts>& xform)
{
  using XformType = Eigen::Transform<T,2,Eigen::Affine,topts>;
  using MatrixType = typename XformType::MatrixType;

  const MatrixType& A = xform.matrix();

  cl_float16 cl_A;  // we'll only use the first 9 elements

  // store in row major
  cl_A.s[0] = static_cast<float>(A(0,0));  // affine
  cl_A.s[1] = static_cast<float>(A(0,1));  // affine
  cl_A.s[2] = static_cast<float>(A(0,2));  // translation
  cl_A.s[3] = static_cast<float>(A(1,0));  // affine
  cl_A.s[4] = static_cast<float>(A(1,1));  // affine
  cl_A.s[5] = static_cast<float>(A(1,2));  // translation
  cl_A.s[6] = 0;
  cl_A.s[7] = 0;
  cl_A.s[8] = 1;

  // 9-15 are invalid/not set/not required

  return cl_A;
}

template <class tSourcePointList, class tOpenCLPointList>
void ConvertPointsToOpenCLHelper(const tSourcePointList& src_pts, tOpenCLPointList* ocl_pts)
{
  using size_type = typename tSourcePointList::size_type;

  const size_type num_pts = src_pts.size();

  ocl_pts->clear();
  ocl_pts->reserve(num_pts);

  for (size_type pt_idx = 0; pt_idx < num_pts; ++pt_idx)
  {
    ocl_pts->push_back(ConvertToOpenCLHelper(src_pts[pt_idx]));
  }
}

}  // un-named

cl_float3 xreg::ConvertToOpenCL(const Pt3& v)
{
  return ConvertToOpenCLHelper(v);
}

cl_float2 xreg::ConvertToOpenCL(const Pt2& v)
{
  return ConvertToOpenCLHelper(v);
}

cl_float4 xreg::ConvertToOpenCL(const Pt4& v)
{
  return ConvertToOpenCLHelper(v);
}

cl_float8 xreg::ConvertToOpenCL(const Pt8& v)
{
  return ConvertToOpenCLHelper(v);
}

cl_float16 xreg::ConvertToOpenCL(const Pt16& v)
{
  return ConvertToOpenCLHelper(v);
}

cl_float16 xreg::ConvertToOpenCL(const FrameTransform& xform)
{
  return ConvertToOpenCLHelper(xform);
}

cl_float16 xreg::ConvertToOpenCL(const Eigen::Transform<CoordScalar,2,Eigen::Affine>& xform)
{
  return ConvertToOpenCLHelper(xform);
}

std::vector<cl_float2> xreg::ConvertPointsToOpenCL(const Pt2List& src_pts)
{
  std::vector<cl_float2> ocl_pts;
  
  ConvertPointsToOpenCLHelper(src_pts, &ocl_pts);

  return ocl_pts;
}

std::vector<cl_float3> xreg::ConvertPointsToOpenCL(const Pt3List& src_pts)
{
  std::vector<cl_float3> ocl_pts;
  
  ConvertPointsToOpenCLHelper(src_pts, &ocl_pts);

  return ocl_pts;
}

std::vector<cl_float4> xreg::ConvertPointsToOpenCL(const Pt4List& src_pts)
{
  std::vector<cl_float4> ocl_pts;
  
  ConvertPointsToOpenCLHelper(src_pts, &ocl_pts);

  return ocl_pts;
}

std::vector<cl_float8> xreg::ConvertPointsToOpenCL(const Pt8List& src_pts)
{
  std::vector<cl_float8> ocl_pts;
  
  ConvertPointsToOpenCLHelper(src_pts, &ocl_pts);

  return ocl_pts;
}

std::vector<cl_float16> xreg::ConvertPointsToOpenCL(const Pt16List& src_pts)
{
  std::vector<cl_float16> ocl_pts;
  
  ConvertPointsToOpenCLHelper(src_pts, &ocl_pts);

  return ocl_pts;
}

boost::compute::float4_ xreg::OpenCLFloat3ToBoostComp4(const cl_float3& f3, const float fourth_comp)
{
  boost::compute::float4_ f4;
  f4[0] = f3.s[0];
  f4[1] = f3.s[1];
  f4[2] = f3.s[2];
  f4[3] = fourth_comp;

  return f4;
}

boost::compute::float16_ xreg::OpenCLFloat16ToBoostComp16(const cl_float16& src)
{
  boost::compute::float16_ dst;
  dst[0]  = src.s[0];
  dst[1]  = src.s[1];
  dst[2]  = src.s[2];
  dst[3]  = src.s[3];
  dst[4]  = src.s[4];
  dst[5]  = src.s[5];
  dst[6]  = src.s[6];
  dst[7]  = src.s[7];
  dst[8]  = src.s[8];
  dst[9]  = src.s[9];
  dst[10] = src.s[10];
  dst[11] = src.s[11];
  dst[12] = src.s[12];
  dst[13] = src.s[13];
  dst[14] = src.s[14];
  dst[15] = src.s[15];

  return dst;
}

