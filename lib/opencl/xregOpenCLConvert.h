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

#ifndef XREGOPENCLCONVERT_H_
#define XREGOPENCLCONVERT_H_

// including this first on Ubuntu 16.04 avoids a compiler error
// with VNL types from ITK
#include "xregCommon.h"

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/opencl.h>
#endif

#include <boost/compute/types/fundamental.hpp>

namespace xreg
{

/// \brief Converts an Eigen 3x1 point/vector into an OpenCL float3.
cl_float3 ConvertToOpenCL(const Pt3& v);

/// \brief Converts an Eigen 2x1 point/vector into an OpenCL float2.
cl_float2 ConvertToOpenCL(const Pt2& v);

/// \brief Converts an Eigen 4x1 point/vector into an OpenCL float4.
cl_float4 ConvertToOpenCL(const Pt4& v);

/// \brief Converts an Eigen 8x1 point/vector into an OpenCL float8.
cl_float8 ConvertToOpenCL(const Pt8& v);

/// \brief Converts an Eigen 16x1 point/vector into an OpenCL float8.
cl_float16 ConvertToOpenCL(const Pt16& v);

/// \brief Converts a list of Eigen point/vectors into OpenCL float tuples.
std::vector<cl_float2> ConvertPointsToOpenCL(const Pt2List& src_pts);

/// \brief Converts a list of Eigen point/vectors into OpenCL float tuples.
std::vector<cl_float3> ConvertPointsToOpenCL(const Pt3List& src_pts);

/// \brief Converts a list of Eigen point/vectors into OpenCL float tuples.
std::vector<cl_float4> ConvertPointsToOpenCL(const Pt4List& src_pts);

/// \brief Converts a list of Eigen point/vectors into OpenCL float tuples.
std::vector<cl_float8> ConvertPointsToOpenCL(const Pt8List& src_pts);

/// \brief Converts a list of Eigen point/vectors into OpenCL float tuples.
std::vector<cl_float16> ConvertPointsToOpenCL(const Pt16List& src_pts);

/// \brief Converts an Eigen 3D Affine Transform object, with 4x4 matrix representation,
///        into an OpenCL float16.
///
/// The OpenCL float16 stores the matrix in row-major format.
cl_float16 ConvertToOpenCL(const FrameTransform& xform);

/// \brief Converts an Eigen 2D Affine Transform object, with 3x3 matrix
///        representation, into an OpenCL float16.
///
/// The OpenCL float16 stores the matrix in row-major format and only utilizes
/// the first 9 elements.
cl_float16 ConvertToOpenCL(const Eigen::Transform<CoordScalar,2,Eigen::Affine>& xform);

/// \brief Convert a cl_float3 to a boost::compute::float4_
boost::compute::float4_ OpenCLFloat3ToBoostComp4(const cl_float3& f3, const float fourth_comp = 0);

/// \brief Convert a cl_float16 to a boost::compute::float16_
boost::compute::float16_ OpenCLFloat16ToBoostComp16(const cl_float16& src);

template <unsigned tM>
struct LookupOpenCLTuples { };

template <>
struct LookupOpenCLTuples<1>
{
  using float_tuple  = cl_float;
  using double_tuple = cl_double;
  using int_tuple    = cl_int;
  using uint_tuple   = cl_uint;
  using char_tuple   = cl_char;
  using uchar_tuple  = cl_uchar;
  using short_tuple  = cl_short;
  using ushort_tuple = cl_ushort;
  using long_tuple   = cl_long;
  using ulong_tuple  = cl_ulong;
};

#define XREG_LOOKUP_OPENCL_TUPLES_HELPER(N) \
template <>                                 \
struct LookupOpenCLTuples<N>                \
{                                           \
  using float_tuple  = cl_float##N;         \
  using double_tuple = cl_double##N;        \
  using int_tuple    = cl_int##N;           \
  using uint_tuple   = cl_uint##N;          \
  using char_tuple   = cl_char##N;          \
  using uchar_tuple  = cl_uchar##N;         \
  using short_tuple  = cl_short##N;         \
  using ushort_tuple = cl_ushort##N;        \
  using long_tuple   = cl_long##N;          \
  using ulong_tuple  = cl_ulong##N;         \
}

XREG_LOOKUP_OPENCL_TUPLES_HELPER(2);
XREG_LOOKUP_OPENCL_TUPLES_HELPER(3);
XREG_LOOKUP_OPENCL_TUPLES_HELPER(4);
XREG_LOOKUP_OPENCL_TUPLES_HELPER(8);
XREG_LOOKUP_OPENCL_TUPLES_HELPER(16);

#undef XREG_LOOKUP_OPENCL_TUPLES_HELPER

namespace detail
{

template <class T, unsigned tN>
struct IsOpenCLTupleTypeHelper
{
  static constexpr bool value = false;
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,1>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<1>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<1>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<1>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<1>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<1>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<1>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<1>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<1>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<1>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<1>::ulong_tuple, T>::value);
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,2>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<2>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<2>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<2>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<2>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<2>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<2>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<2>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<2>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<2>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<2>::ulong_tuple, T>::value);
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,3>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<3>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<3>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<3>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<3>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<3>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<3>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<3>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<3>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<3>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<3>::ulong_tuple, T>::value);
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,4>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<4>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<4>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<4>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<4>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<4>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<4>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<4>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<4>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<4>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<4>::ulong_tuple, T>::value);
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,8>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<8>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<8>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<8>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<8>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<8>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<8>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<8>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<8>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<8>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<8>::ulong_tuple, T>::value);
};

template <class T>
struct IsOpenCLTupleTypeHelper<T,16>
{
  static constexpr bool value = (std::is_same<LookupOpenCLTuples<16>::float_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<16>::double_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<16>::int_tuple, T>::value)    ||
                                (std::is_same<LookupOpenCLTuples<16>::uint_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<16>::char_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<16>::uchar_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<16>::short_tuple, T>::value)  ||
                                (std::is_same<LookupOpenCLTuples<16>::ushort_tuple, T>::value) ||
                                (std::is_same<LookupOpenCLTuples<16>::long_tuple, T>::value)   ||
                                (std::is_same<LookupOpenCLTuples<16>::ulong_tuple, T>::value);
};

}  // detail

template <class T>
struct IsOpenCLTupleType
{
  static constexpr bool value = detail::IsOpenCLTupleTypeHelper<T,1>::value ||
                                detail::IsOpenCLTupleTypeHelper<T,2>::value ||
                                detail::IsOpenCLTupleTypeHelper<T,3>::value ||
                                detail::IsOpenCLTupleTypeHelper<T,4>::value ||
                                detail::IsOpenCLTupleTypeHelper<T,8>::value ||
                                detail::IsOpenCLTupleTypeHelper<T,16>::value;
};

}  // xreg

#endif

