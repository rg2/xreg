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

#include "xregHUToLinAtt.h"

#include <itkImageRegionIterator.h>
#include <itkImageRegionConstIterator.h>
  
void xreg::HUToLinAttFilter::SetLinearAttWater(const double mu_water)
{
  mu_water_ = mu_water;
}

void xreg::HUToLinAttFilter::SetLinearAttAir(const double mu_air)
{
  mu_air_ = mu_air;
}

void xreg::HUToLinAttFilter::SetHULower(const double hul)
{
  hu_lower_ = hul;
}

void xreg::HUToLinAttFilter::GenerateData()
{
  const auto* hu_img = this->GetInput();

  auto* att_img = this->GetOutput();
  att_img->SetRegions(hu_img->GetLargestPossibleRegion());
  att_img->Allocate();

  itk::ImageRegionConstIterator<Vol> hu_it(hu_img, hu_img->GetLargestPossibleRegion());
  itk::ImageRegionIterator<Vol> att_it(att_img, att_img->GetLargestPossibleRegion());

  const double hu_scale = (mu_water_ - mu_air_) * 1.0e-3;

  const double mu_lower = (hu_lower_ * hu_scale) + mu_water_;

  for (hu_it.GoToBegin(), att_it.GoToBegin(); !hu_it.IsAtEnd(); ++hu_it, ++att_it)
  {
    // TODO: can increase efficiency by making hu_scale and a temporary hu_water_ of type float when PixelType is float
    att_it.Set(static_cast<float>(std::max((hu_it.Get() * hu_scale) + mu_water_ - mu_lower, 0.0)));

    // these were experiments with a conversion that also implicitly thresholds the linear attenuation values
    //att_it.Set(static_cast<float>(std::max((hu_it.Get() * hu_scale) + (-0.2 * mu_water_), 0.0)));
    //att_it.Set(static_cast<float>(std::max((hu_it.Get() - 200) * (mu_water_ / 1000), 0.0)));
  }
}

itk::Image<float,3>::Pointer xreg::HUToLinAtt(const itk::Image<float,3>* hu_vol,
                                              const float hu_lower)
{
  auto hu2att = HUToLinAttFilter::New();
  hu2att->SetInput(hu_vol);
  hu2att->SetHULower(hu_lower);
  hu2att->Update();

  return hu2att->GetOutput();
}

