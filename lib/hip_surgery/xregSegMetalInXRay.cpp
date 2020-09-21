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

#include "xregSegMetalInXRay.h"

#include <fmt/format.h>

#include <itkWatershedImageFilter.h>
#include <itkGradientMagnitudeRecursiveGaussianImageFilter.h>
#include <itkGrayscaleDilateImageFilter.h>
#include <itkBinaryBallStructuringElement.h>

#include "xregITKBasicImageUtils.h"
#include "xregITKRemapUtils.h"

void xreg::SegmentMetalInXRay::operator()()
{
  this->dout() << "setting up filters.." << std::endl;
  
  using GradMagFilt = itk::GradientMagnitudeRecursiveGaussianImageFilter<IntensImg,IntensImg>;

  auto grad_mag_filt = GradMagFilt::New();
  grad_mag_filt->SetInput(src_img);
  grad_mag_filt->SetSigma(1.0);
  grad_mag_filt->Update();

  grad_img = grad_mag_filt->GetOutput();

  const bool do_iter_adjust = (num_pix_in_mask_upper_thresh > 0) &&
                              (num_pix_in_mask_lower_thresh > 0);

  const auto itk_img_size = src_img->GetLargestPossibleRegion().GetSize();
  const unsigned long tot_num_pix = itk_img_size[0] * itk_img_size[1];

  const unsigned long max_num_pix_in_mask = static_cast<unsigned long>(
                                              std::ceil(tot_num_pix * num_pix_in_mask_upper_thresh));
  const unsigned long min_num_pix_in_mask = static_cast<unsigned long>(
                                              std::floor(tot_num_pix * num_pix_in_mask_lower_thresh));

  double cur_level = level;

  bool should_stop = false;

  unsigned long iter = 0;
  
  double frac_masked_pix = 0;
  
  while (!should_stop)
  {
    using WatershedFilt = itk::WatershedImageFilter<IntensImg>;

    auto watershed_filt = WatershedFilt::New();
    watershed_filt->SetInput(grad_img);
    watershed_filt->SetThreshold(thresh);
    watershed_filt->SetLevel(cur_level);

    this->dout() << "performing filtering (segmentation) ..." << std::endl;
    watershed_filt->Update();

    // cast to u16 for more efficient lookups
    using U16SegImg = itk::Image<unsigned short,2>;

    this->dout() << "casting to u16..." << std::endl;
    auto u16_seg = CastITKImageIfNeeded<unsigned short>(watershed_filt->GetOutput());

    using RemapTable = decltype(CreateImageIntConsecutiveLabelsMap(u16_seg.GetPointer()));

    RemapTable remap_table;

    if (!binarize)
    {
      this->dout() << "computing re-labeling table (consectutive)..." << std::endl;

      // false; let label 1 get mapped to 0, since the watershed filter seems to return background as a segment
      remap_table = CreateImageIntConsecutiveLabelsMap(u16_seg.GetPointer(), false);
    }
    else
    {
      this->dout() << "assigning binarized re-labeling table..." << std::endl;
      remap_table.resize(static_cast<RemapTable::size_type>(std::numeric_limits<unsigned short>::max()) + 1, 1);
      remap_table[0] = 0;
      remap_table[1] = 0;
    }

    auto remap_seg = ChangeImageIntValues(u16_seg.GetPointer(), remap_table); 

    if (binarize && dilation_radius)
    {
      using MorphKernel  = itk::BinaryBallStructuringElement<unsigned short,2>;
      using DilateFilter = itk::GrayscaleDilateImageFilter<U16SegImg,U16SegImg,MorphKernel>;

      this->dout() << "dilating binary labeling..." << std::endl;

      MorphKernel kern;
      kern.SetRadius(dilation_radius);
      kern.CreateStructuringElement();

      DilateFilter::Pointer dilate_filt = DilateFilter::New();
      dilate_filt->SetInput(remap_seg);
      dilate_filt->SetKernel(kern);

      dilate_filt->Update();
      
      remap_seg = dilate_filt->GetOutput();
    }

    this->dout() << "casting -> s16..." << std::endl;
    // I believe there is a bug in ITK's writing or reading of int nii files,
    // since if I do not cast from the unsigned int or unsigned short image output,
    // the output image read into slicer or mitk is garbage
    seg_img = CastITKImageIfNeeded<short>(remap_seg.GetPointer());
      
    const short* seg_buf = seg_img->GetBufferPointer();

    unsigned long num_pix_in_mask = 0;
    std::for_each(seg_buf, seg_buf + tot_num_pix,
                  [&num_pix_in_mask] (const short& m)
                  {
                    if (m)
                    {
                      ++num_pix_in_mask;
                    }
                  });
                                 
    frac_masked_pix = static_cast<double>(num_pix_in_mask) / tot_num_pix;

    this->dout() << fmt::format("{:2d}: {:.3f} ({:.4f})", iter, frac_masked_pix, cur_level) << std::endl;

    if (do_iter_adjust)
    {
      this->dout() << "adjust iteration: " << iter << std::endl; 

      if (num_pix_in_mask >= max_num_pix_in_mask)
      {
        this->dout() << "  too many pixels in mask" << std::endl;
        
        if ((iter > 0) && (level_inc < 0))
        {
          // was previously lowering the threshold, we now want to increase it,
          // but by half, so that we eventually converge
          level_inc = std::abs(level_inc) / 2;
        }
        else
        {
          // either the first iteration, or we have not switched directions,
          // so keep using the same positive increment
          level_inc = std::abs(level_inc);
        }

        cur_level += level_inc;
      }
      else if (num_pix_in_mask <= min_num_pix_in_mask)
      {
        this->dout() << "  not enough pixels in mask" << std::endl;
        
        if ((iter > 0) && (level_inc > 0))
        {
          // was previously increasing the threshold, we now want to decrease it,
          // but by half, so that we eventually converge
          level_inc = std::abs(level_inc) / -2;

          if (level_inc > -0.001)
          {
            // we have not found a threshold that works, and this method will most
            // likely not work on the current inputs, terminate now, on a case that
            // was decreasing the threshold, since it will not mask out the entire
            // image, it may not mask any screws, but that is better than masking
            // everything.
            should_stop = true;
          }
        }
        else
        {
          // either the first iteration, or we have not switched directions,
          // so keep using the same negative increment
          level_inc = -std::abs(level_inc);
        }

        cur_level += level_inc;

        // corner case of going to zero or negative... this has happened.
        if (cur_level < 1.0e-3)
        {
          should_stop = true;
        }
      }
      else
      {
        should_stop = true;
      }
    }
    else
    {
      should_stop = true;
    }

    ++iter;

    // check for maximum number of iterations
    if (max_iters && (iter >= max_iters))
    {
      this->dout() << "max num iters reached..." << std::endl;
      should_stop = true;
    }
  }

  // check to see if most of image is masked... if it is then mask nothing
  if (frac_masked_pix > 0.7)
  {
    this->dout() << "too many pixels in final mask... turning off mask." << std::endl;
    seg_img->FillBuffer(0);
  }
}

