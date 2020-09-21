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

#include "xregVTKITKUtils.h"

#include <vtkNew.h>
#include <vtkImageImport.h>
#include <vtkImageFlip.h>
#include <itkImage.h>

#include "xregVTKBasicUtils.h"

namespace
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
template <class T, unsigned int N>
vtkSmartPointer<vtkImageData> ConvertITKImageToVTKHelper(itk::Image<T,N>* itk_img,
                                                         const bool flip_ud,
                                                         const bool copy_phys_meta)
{
  static_assert((N == 2) || (N == 3), "only supports 2D and 3D images currently");

  vtkNew<vtkImageImport> vtk_img_import;
  vtk_img_import->SetImportVoidPointer(itk_img->GetBufferPointer(), 1);  // 1 -> VTK will not manage the buffer (e.g. VTK will not delete it)
  vtk_img_import->SetDataScalarType(xreg::LookupVTKDataTypeID<T>::value);
  vtk_img_import->SetNumberOfScalarComponents(1);

  const auto img_size = itk_img->GetLargestPossibleRegion().GetSize();

  const unsigned long z_len = (N == 2) ? 0ul : static_cast<unsigned long>(img_size[2]);  // 2D -> use 0 length in z dim
  const unsigned long z_max = (z_len > 0) ? (z_len - 1) : z_len;

  // data extent is the region to display, whole extent is the entire buffer
  vtk_img_import->SetDataExtent( 0, img_size[0] - 1, 0, img_size[1] - 1, 0, z_max);
  vtk_img_import->SetWholeExtent(0, img_size[0] - 1, 0, img_size[1] - 1, 0, z_max);

  if (copy_phys_meta)
  {
    const auto img_spacing = itk_img->GetSpacing();

    vtk_img_import->SetDataSpacing(img_spacing[0], img_spacing[1], (N == 2) ? 0.0 : img_spacing[2]);  // 0 spacing in z dim for 2D

    const auto img_orig = itk_img->GetOrigin();
    vtk_img_import->SetDataOrigin(img_orig[0], img_orig[1], (N == 2) ? 0.0 : img_orig[2]);  // physical location of the voxel 0,0,0
  }

  vtk_img_import->Update();

  vtkSmartPointer<vtkImageData> vtk_img = vtk_img_import->GetOutput();

  if (flip_ud)
  {
    vtkNew<vtkImageFlip> flipper;
    flipper->SetInputData(vtk_img);
    flipper->SetFilteredAxis(1);    // 1 -> flip y (row) direction
    flipper->FlipAboutOriginOff();  // axis cuts through image center

    flipper->Update();

    vtk_img = flipper->GetOutput();
  }

  return vtk_img;
}

}  // un-named

#define XREG_MAKE_ConvertITKImageToVTK(T,N)                                          \
vtkSmartPointer<vtkImageData> xreg::ConvertITKImageToVTK(itk::Image<T,N>* itk_img,   \
                                                         const bool flip_ud,         \
                                                         const bool copy_phys_meta)  \
{                                                                                    \
  return ConvertITKImageToVTKHelper(itk_img, flip_ud, copy_phys_meta);               \
}

XREG_MAKE_ConvertITKImageToVTK(unsigned char,2)
XREG_MAKE_ConvertITKImageToVTK(unsigned short,2)
XREG_MAKE_ConvertITKImageToVTK(float,2)

XREG_MAKE_ConvertITKImageToVTK(unsigned char,3)
XREG_MAKE_ConvertITKImageToVTK(unsigned short,3)
XREG_MAKE_ConvertITKImageToVTK(float,3)

#undef XREG_MAKE_ConvertITKImageToVTK

