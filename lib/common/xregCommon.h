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

#ifndef XREGCOMMON_H_
#define XREGCOMMON_H_

#include <vector>
#include <string>
#include <unordered_map>

#include <boost/any.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <itkImage.h>

namespace xreg
{

using size_type = std::size_t;

using Float = float;

//using PixelScalar = Float;
using CoordScalar = Float;
//using LabelScalar = unsigned char;

using Pt2  = Eigen::Matrix<CoordScalar,2,1>;
using Pt3  = Eigen::Matrix<CoordScalar,3,1>;
using Pt4  = Eigen::Matrix<CoordScalar,4,1>;
using Pt5  = Eigen::Matrix<CoordScalar,5,1>;
using Pt6  = Eigen::Matrix<CoordScalar,6,1>;
using Pt8  = Eigen::Matrix<CoordScalar,8,1>;
using Pt16 = Eigen::Matrix<CoordScalar,16,1>;
using PtN  = Eigen::Matrix<CoordScalar,Eigen::Dynamic,1>;

using Mat2x2 = Eigen::Matrix<CoordScalar,2,2>;
using Mat3x3 = Eigen::Matrix<CoordScalar,3,3>;
using Mat4x4 = Eigen::Matrix<CoordScalar,4,4>;
using Mat3x4 = Eigen::Matrix<CoordScalar,3,4>;
using MatMxN = Eigen::Matrix<CoordScalar,Eigen::Dynamic,Eigen::Dynamic>;

using FrameTransform = Eigen::Transform<CoordScalar,3,Eigen::Affine>;

using CoordScalarList = std::vector<CoordScalar>;
//using PixelScalarList = std::vector<PixelScalar>;

using Pt2List  = std::vector<Pt2>;
using Pt3List  = std::vector<Pt3>;
using Pt4List  = std::vector<Pt4>;
using Pt5List  = std::vector<Pt5>;
using Pt6List  = std::vector<Pt6>;
using Pt8List  = std::vector<Pt8>;
using Pt16List = std::vector<Pt16>;
using PtNList  = std::vector<PtN>;

using Mat2x3List = std::vector<Mat2x2>;
using Mat3x3List = std::vector<Mat3x3>;
using Mat4x4List = std::vector<Mat4x4>;
using Mat3x4List = std::vector<Mat3x4>;
using MatMxNList = std::vector<MatMxN>;

using LandMap2 = std::unordered_map<std::string,Pt2>;
using LandMap3 = std::unordered_map<std::string,Pt3>;
using LandMap4 = std::unordered_map<std::string,Pt4>;

using LandMultiMap2 = std::unordered_multimap<std::string,Pt2>;
using LandMultiMap3 = std::unordered_multimap<std::string,Pt3>;
using LandMultiMap4 = std::unordered_multimap<std::string,Pt4>;

using FrameTransformList = std::vector<FrameTransform>;

using StrList = std::vector<std::string>;

//using Vol         = itk::Image<PixelScalar,3>;
//using VolPtr      = Vol::Pointer;
//using LabelVol    = itk::Image<LabelScalar,3>;
//using LabelVolPtr = LabelVol::Pointer;

using Pt2_f  = Eigen::Matrix<float,2,1>;
using Pt3_f  = Eigen::Matrix<float,3,1>;
using Pt4_f  = Eigen::Matrix<float,4,1>;
using Pt8_f  = Eigen::Matrix<float,8,1>;
using Pt16_f = Eigen::Matrix<float,16,1>;
using PtN_f  = Eigen::Matrix<float,Eigen::Dynamic,1>;

using Mat2x2_f = Eigen::Matrix<float,2,2>;
using Mat3x3_f = Eigen::Matrix<float,3,3>;
using Mat4x4_f = Eigen::Matrix<float,4,4>;
using Mat3x4_f = Eigen::Matrix<float,3,4>;
using MatMxN_f = Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>;

using Pt2_d  = Eigen::Matrix<double,2,1>;
using Pt3_d  = Eigen::Matrix<double,3,1>;
using Pt4_d  = Eigen::Matrix<double,4,1>;
using Pt8_d  = Eigen::Matrix<double,8,1>;
using Pt16_d = Eigen::Matrix<double,16,1>;
using PtN_d  = Eigen::Matrix<double,Eigen::Dynamic,1>;

using Mat2x2_d = Eigen::Matrix<double,2,2>;
using Mat3x3_d = Eigen::Matrix<double,3,3>;
using Mat4x4_d = Eigen::Matrix<double,4,4>;
using Mat3x4_d = Eigen::Matrix<double,3,4>;
using MatMxN_d = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>;

using RayCastPixelScalar = float;

/**
 * @brief A mask on a elements which may be indexed in a 1D vector.
 *
 * Generally speaking, a value of true indicates that the element will be used,
 * and a value of false indicates that an element is not used, or masked out.
 * @see TriMesh, ReadVertexMask
 **/
using MaskVec = std::vector<bool>;

constexpr CoordScalar kDEG2RAD = 3.141592653589793 / 180.0;
constexpr CoordScalar kRAD2DEG = 180.0 / 3.141592653589793;

using AnyParamMap = std::unordered_map<std::string,boost::any>;

}  // xreg

#endif

