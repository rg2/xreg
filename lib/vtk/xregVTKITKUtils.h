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

#ifndef XREGVTKITKUTILS_H_
#define XREGVTKITKUTILS_H_

#include <itkImage.h>

#include <vtkSmartPointer.h>
#include <vtkImageData.h>


namespace xreg
{

/// \brief Converts an image in ITK data container to a VTK data container.
///
/// VTK uses the lower-left corner as index zero, whereas ITK uses the upper-left
/// corner as index zero, so the user will most likely wish to up/down flip the
/// image.
/// Cases may arise when this is not desirable (slicer), so the user may specify
/// that the flip is not performed.
/// Additionally, the VTK container does not store orientation information, so
/// an additional structure may be required, such as the MRML volume node in
/// Slicer. For this reason, physical measurements such as the origin and spacing
/// are not copied by default. To use an image as a texture, physical measurements
/// are not, generally, required.
vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<unsigned char,2>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<unsigned short,2>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<float,2>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<unsigned char,3>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<unsigned short,3>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

vtkSmartPointer<vtkImageData> ConvertITKImageToVTK(itk::Image<float,3>* itk_img,
                                                   const bool flip_ud = true,
                                                   const bool copy_phys_meta = false);

}  // xreg

#endif

