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

#ifndef XREGHUTOLINATT_H_
#define XREGHUTOLINATT_H_

#include <itkImage.h>
#include <itkImageToImageFilter.h>

namespace xreg
{

class HUToLinAttFilter
  : public itk::ImageToImageFilter<itk::Image<float,3>,itk::Image<float,3>>
{
public:
  using Self       = HUToLinAttFilter;
  using Vol        = itk::Image<float,3>;
  using Superclass = itk::ImageToImageFilter<Vol,Vol>;
  using Pointer    = itk::SmartPointer<Self>;

  itkNewMacro(Self);

  itkTypeMacro(HUToLinAttFilter, itk::ImageToImageFilter);

  /// \brief Sets the linear attenuation coefficient to use for water.
  ///
  /// The default value is 0.02.
  void SetLinearAttWater(const double mu_water);

  /// \brief Sets the linear attenuation coefficient to use for air.
  ///
  /// The default value is 0.
  void SetLinearAttAir(const double mu_air);

  void SetHULower(const double hul);

protected:

  HUToLinAttFilter() = default;

  ~HUToLinAttFilter() { }

  void GenerateData() override;

private:
  HUToLinAttFilter(const Self&);
  Self& operator=(const Self&);

  // TODO: add thresholding values

  double mu_water_ = 0.02683 * 1.0;
  double mu_air_   = 0.02485 * 0.0001;

  double hu_lower_ = -1000;
};

itk::Image<float,3>::Pointer HUToLinAtt(const itk::Image<float,3>* hu_vol,
                                        const float hu_lower = -1000);

}  // xreg

#endif

