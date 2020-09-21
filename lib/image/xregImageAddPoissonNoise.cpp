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

#include "xregImageAddPoissonNoise.h"

#include "xregITKBasicImageUtils.h"
#include "xregSampleUtils.h"
#include "xregTBBUtils.h"
#include "xregImageIntensLogTrans.h"

itk::Image<unsigned short,2>::Pointer
xreg::SamplePoissonProjFromAttProj(const itk::Image<float,2>* att_proj,
                                   const unsigned long num_photons)
{
  using PoissonDist = std::poisson_distribution<unsigned short>; 
  
  const auto sz = att_proj->GetLargestPossibleRegion().GetSize();

  const unsigned long num_pix = sz[0] * sz[1];

  const auto* src_buf = att_proj->GetBufferPointer();

  auto dst_proj = MakeITKVolWithSameCoords<unsigned short>(att_proj);

  auto* dst_buf = dst_proj->GetBufferPointer();

  auto sample_fn = [src_buf,dst_buf,num_photons] (const RangeType& r)
  {
    std::mt19937 rng_eng;
    SeedRNGEngWithRandDev(&rng_eng);
    
    for (unsigned long i = r.begin(); i < r.end(); ++i)
    {
      dst_buf[i] = PoissonDist(num_photons * std::exp(-src_buf[i]))(rng_eng);
      
      // more of a sanitity check to make sure the values are not saturating
      xregASSERT(dst_buf[i] < std::numeric_limits<unsigned short>::max()); 
    }
  };

  ParallelFor(sample_fn, RangeType(0,num_pix));

  return dst_proj;
}

itk::Image<float,2>::Pointer
xreg::AddPoissonNoiseToImage(const itk::Image<float,2>* src_img,
                             const unsigned long num_photons)
{
  auto counts = CastITKImageIfNeeded<float>(SamplePoissonProjFromAttProj(src_img, num_photons).GetPointer());
  
  auto log_xform = ImageIntensLogTransFilter::New();
    
  log_xform->SetNormalizeZeroOne(false);
  log_xform->SetUseMaxIntensityAsI0(false);
  log_xform->SetI0(num_photons);
  log_xform->SetInput(counts);

  log_xform->Update();

  return log_xform->GetOutput();
}

