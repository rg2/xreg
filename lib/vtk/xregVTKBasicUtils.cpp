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

#include "xregVTKBasicUtils.h"

#include <algorithm>

#include <vtkMatrix4x4.h>
#include <vtkFloatArray.h>

xreg::FrameTransform xreg::VTKMat4x4ToFrameTransform(vtkMatrix4x4* vtk_mat)
{
  FrameTransform xform;

  xform.matrix() = VTKMat4x4ToMat4x4(vtk_mat);

  return xform;
}

namespace
{

template <class T>
Eigen::Matrix<T,4,4> VTKMat4x4ToMat4x4Helper(vtkMatrix4x4* vtk_mat)
{
  Eigen::Matrix<T,4,4> dst;

  dst(0,0) = static_cast<T>(vtk_mat->GetElement(0, 0));
  dst(0,1) = static_cast<T>(vtk_mat->GetElement(0, 1));
  dst(0,2) = static_cast<T>(vtk_mat->GetElement(0, 2));
  dst(0,3) = static_cast<T>(vtk_mat->GetElement(0, 3));

  dst(1,0) = static_cast<T>(vtk_mat->GetElement(1, 0));
  dst(1,1) = static_cast<T>(vtk_mat->GetElement(1, 1));
  dst(1,2) = static_cast<T>(vtk_mat->GetElement(1, 2));
  dst(1,3) = static_cast<T>(vtk_mat->GetElement(1, 3));

  dst(2,0) = static_cast<T>(vtk_mat->GetElement(2, 0));
  dst(2,1) = static_cast<T>(vtk_mat->GetElement(2, 1));
  dst(2,2) = static_cast<T>(vtk_mat->GetElement(2, 2));
  dst(2,3) = static_cast<T>(vtk_mat->GetElement(2, 3));

  dst(3,0) = static_cast<T>(vtk_mat->GetElement(3, 0));
  dst(3,1) = static_cast<T>(vtk_mat->GetElement(3, 1));
  dst(3,2) = static_cast<T>(vtk_mat->GetElement(3, 2));
  dst(3,3) = static_cast<T>(vtk_mat->GetElement(3, 3));

  return dst;
}

}  // un-named

xreg::Mat4x4 xreg::VTKMat4x4ToMat4x4(vtkMatrix4x4* vtk_mat)
{
  return VTKMat4x4ToMat4x4Helper<CoordScalar>(vtk_mat);
}

void xreg::ToVTKMat4x4(const FrameTransform& xform, vtkMatrix4x4* vtk_mat)
{
  ToVTKMat4x4(Mat4x4(xform.matrix()), vtk_mat);
}

void xreg::ToVTKMat4x4(const Mat4x4& m, vtkMatrix4x4* vtk_mat)
{
  vtk_mat->SetElement(0, 0, static_cast<double>(m(0,0)));
  vtk_mat->SetElement(0, 1, static_cast<double>(m(0,1)));
  vtk_mat->SetElement(0, 2, static_cast<double>(m(0,2)));
  vtk_mat->SetElement(0, 3, static_cast<double>(m(0,3)));

  vtk_mat->SetElement(1, 0, static_cast<double>(m(1,0)));
  vtk_mat->SetElement(1, 1, static_cast<double>(m(1,1)));
  vtk_mat->SetElement(1, 2, static_cast<double>(m(1,2)));
  vtk_mat->SetElement(1, 3, static_cast<double>(m(1,3)));

  vtk_mat->SetElement(2, 0, static_cast<double>(m(2,0)));
  vtk_mat->SetElement(2, 1, static_cast<double>(m(2,1)));
  vtk_mat->SetElement(2, 2, static_cast<double>(m(2,2)));
  vtk_mat->SetElement(2, 3, static_cast<double>(m(2,3)));

  vtk_mat->SetElement(3, 0, static_cast<double>(m(3,0)));
  vtk_mat->SetElement(3, 1, static_cast<double>(m(3,1)));
  vtk_mat->SetElement(3, 2, static_cast<double>(m(3,2)));
  vtk_mat->SetElement(3, 3, static_cast<double>(m(3,3)));
}

namespace
{

/**
 * @brief Convert an STL container represented by iterators into a VTK array.
 *
 * @param begin_it The starting iterator of the source container
 * @param end_it The end iterator of the source container
 * @param vtk_arr The destination VTK array
 **/
template <class Itr>
void ConvertSTLContainerToVTKArrayHelper(Itr begin_it, Itr end_it, vtkFloatArray* vtk_arr)
{
  vtk_arr->SetNumberOfValues(std::distance(begin_it, end_it));

  int index = 0;
  for (Itr it = begin_it; it != end_it; ++it, ++index)
  {
    vtk_arr->SetValue(index, static_cast<float>(*it));
  }
}

}  // un-named

void xreg::ToVTKArray(const std::vector<float>& v, vtkFloatArray* vtk_arr)
{
  ConvertSTLContainerToVTKArrayHelper(v.begin(), v.end(), vtk_arr);
}

void xreg::ToVTKArray(const std::vector<double>& v, vtkFloatArray* vtk_arr)
{
  ConvertSTLContainerToVTKArrayHelper(v.begin(), v.end(), vtk_arr);
}

