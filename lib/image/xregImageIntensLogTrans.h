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

#ifndef XREGIMAGEINTENSLOGTRANS_H_
#define XREGIMAGEINTENSLOGTRANS_H_

#include <itkImage.h>
#include <itkImageToImageFilter.h>

namespace xreg
{

/// \brief Transforms image intensities by -log. Useful for remapping C-Arm
///        fluoroscopic images.
///
/// Assumes the basic imaging equation: I_f(i,j) = I_0  exp(-L(i,j)), where
/// L(i,j) is the line integral that sums linear attenuation units and is
/// equal to \int_0^1 V(l(i,j,t)) dt.
/// For each pixel: L(i,j) = -log(I_f(i,j) / I_0) for I_f(i,j) != 0.
/// For I_f(i,j) == 0, we set L(i,j) = L(i*,j*), where i*,j* = argmin_u,v I_f(u,v), such that I_f(u,v) > 0
/// With a frame grabbed image, I_0 is not known, and may be set to an arbitrary
/// number (e.g. 1).
class ImageIntensLogTransFilter
  : public itk::ImageToImageFilter<itk::Image<float,2>,itk::Image<float,2>>
{
public:
  using Self = ImageIntensLogTransFilter;
  using Superclass = itk::ImageToImageFilter<itk::Image<float,2>,itk::Image<float,2>>;
  using Pointer = itk::SmartPointer<Self>;

  itkNewMacro(Self);

  itkTypeMacro(ImageIntensLogTransFilter, itk::ImageToImageFilter);

  using InputImagePixelType = Superclass::InputImagePixelType;

  /// \brief Set whether or not input image should be normalized to have values
  ///        in [0,1].
  ///
  /// Default is false.
  void SetNormalizeZeroOne(const bool normalize);

  /// \brief Set the default initial intensity emitted from the X-Ray source
  ///
  /// Default is one.
  void SetI0(const InputImagePixelType I0);

  /// \brief Smooths the image and then uses the maximum intensity as I0
  void SetUseMaxIntensityAsI0(const bool use_max);

protected:

  ImageIntensLogTransFilter() { }

  ~ImageIntensLogTransFilter() { }

  void GenerateData() override;

private:
  ImageIntensLogTransFilter(const Self&);
  Self& operator=(const Self&);

  bool normalize_zero_one_ = false;

  bool use_max_intensity_as_I0_ = true;

  InputImagePixelType I0_ = 1;
};

}  // xreg

#endif

