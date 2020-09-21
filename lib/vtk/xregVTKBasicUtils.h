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

#ifndef XREGVTKBASICUTILS_H_
#define XREGVTKBASICUTILS_H_

#include "xregCommon.h"

#include <vtkType.h>

// forward declarations
class vtkMatrix4x4;
class vtkFloatArray;

namespace xreg
{

/**
 * @brief Converts a VTK 4x4 matrix into an Eigen transformation object.
 * @param vtk_mat The VTK matrix
 * @param eig_xform The destination Eigen transform object
 **/
FrameTransform VTKMat4x4ToFrameTransform(vtkMatrix4x4* vtk_mat);

Mat4x4 VTKMat4x4ToMat4x4(vtkMatrix4x4* vtk_mat);

/**
 * @brief Converts an Eigen transform object into a 4x4 VTK matrix.
 * @param eig_xform The source Eigen transform object
 * @param vtk_mat The destination VTK matrix
 **/
void ToVTKMat4x4(const FrameTransform& xform, vtkMatrix4x4* vtk_mat);

void ToVTKMat4x4(const Mat4x4& m, vtkMatrix4x4* vtk_mat);

void ToVTKArray(const std::vector<float>& v, vtkFloatArray* vtk_arr);
void ToVTKArray(const std::vector<double>& v, vtkFloatArray* vtk_arr);

template <class T>
struct LookupVTKDataTypeID
{
  static const vtkIdType value = VTK_VOID;
};

#define XREG_MAKE_LOOKUP_VTK_DATA_TYPE(T,V) \
template <> \
struct LookupVTKDataTypeID<T> \
{ \
  static const vtkIdType value = V; \
}

XREG_MAKE_LOOKUP_VTK_DATA_TYPE(char, VTK_CHAR);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(short, VTK_SHORT);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(int, VTK_INT);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(long, VTK_LONG);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(unsigned char, VTK_UNSIGNED_CHAR);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(unsigned short, VTK_UNSIGNED_SHORT);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(unsigned int, VTK_UNSIGNED_INT);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(unsigned long, VTK_UNSIGNED_LONG);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(float, VTK_FLOAT);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(double, VTK_DOUBLE);
XREG_MAKE_LOOKUP_VTK_DATA_TYPE(vtkIdType, VTK_ID_TYPE);

#undef XREG_MAKE_LOOKUP_VTK_DATA_TYPE


}  // xreg

#endif

