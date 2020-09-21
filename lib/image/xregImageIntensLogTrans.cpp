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

#include "xregImageIntensLogTrans.h"

#include <itkImageRegionIterator.h>
#include <itkImageRegionConstIterator.h>
#include <itkDiscreteGaussianImageFilter.h>

#include "xregTBBUtils.h"

void xreg::ImageIntensLogTransFilter::SetNormalizeZeroOne(const bool normalize)
{
  if (normalize != normalize_zero_one_)
  {
    normalize_zero_one_ = normalize;
    this->Modified();
  }
}

void xreg::ImageIntensLogTransFilter::SetI0(const InputImagePixelType I0)
{
  I0_ = I0;
  this->Modified();
}

void xreg::ImageIntensLogTransFilter::SetUseMaxIntensityAsI0(const bool use_max)
{
  use_max_intensity_as_I0_ = use_max;
  this->Modified();
}

void xreg::ImageIntensLogTransFilter::GenerateData()
{
  using PixelScalar = InputImagePixelType;

  constexpr PixelScalar eps = 1.0e-6;

  const auto* src_img = this->GetInput();

  const auto src_region = src_img->GetLargestPossibleRegion();
  const auto num_pix = src_region.GetNumberOfPixels();

  auto* log_img = this->GetOutput();
  log_img->SetRegions(src_region);
  log_img->Allocate();

  PixelScalar I0_to_use = I0_;

  const auto* src_buf = src_img->GetBufferPointer();
  auto* log_buf = log_img->GetBufferPointer();
  const auto* buf_to_log_xform = src_buf;

  if (normalize_zero_one_)
  {
    const auto max_val = *std::max_element(src_buf, src_buf + num_pix);

    // scale input to be in [0,1]
    const PixelScalar scale_factor = PixelScalar(1) / max_val;

    auto scale_fn = [scale_factor] (const PixelScalar& x)
    {
      return x * scale_factor;
    };

    ParallelTransform(src_buf, src_buf + num_pix, log_buf, scale_fn);

    buf_to_log_xform = log_buf;

    if (use_max_intensity_as_I0_)
    {
      I0_to_use = 1;
    }
  }
  else if (use_max_intensity_as_I0_)
  {
    auto smoother = itk::DiscreteGaussianImageFilter<Superclass::InputImageType,Superclass::InputImageType>::New();
    smoother->SetInput(src_img);
    smoother->SetUseImageSpacing(false);
    smoother->SetVariance(2);
    smoother->Update();

    const auto* smooth_buf = smoother->GetOutput()->GetBufferPointer();
    I0_to_use = *std::max_element(smooth_buf, smooth_buf + num_pix);
  }

  // Find minimum positive value in the input image
  PixelScalar min_pos = 0;
  {
    bool found_pos = false;
  
    for (std::decay<decltype(num_pix)>::type i = 0; i < num_pix; ++i)
    {
      if (buf_to_log_xform[i] > eps)
      {
        if (found_pos)
        {
          if (min_pos > buf_to_log_xform[i])
          {
            min_pos = buf_to_log_xform[i];
          }
        }
        else
        {
          min_pos = buf_to_log_xform[i];
          found_pos = true;
        }
      }
    }
  }

  // Perform log transform, mapping zero values to the negative log of the
  // current smallest positive value
  const PixelScalar out_max_val = -std::log(min_pos / I0_to_use);
  
  auto log_fn = [eps,out_max_val,I0_to_use] (const PixelScalar& x)
  {
    // NOTE: could gain small speedup by using log(I0) - log(x)
    return (x > eps) ? -std::log(x / I0_to_use) : out_max_val;
  };

  ParallelTransform(buf_to_log_xform, buf_to_log_xform + num_pix, log_buf, log_fn);
}
