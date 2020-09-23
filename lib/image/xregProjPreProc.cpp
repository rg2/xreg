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

#include "xregProjPreProc.h"

#include "xregITKBasicImageUtils.h"
#include "xregImageIntensLogTrans.h"
#include "xregSegMetalInXRay.h"

void xreg::ProjPreProc::operator()()
{
  const size_type num_projs = input_projs.size();

  output_projs.clear();
  output_mask_projs.clear();

  output_projs.reserve(num_projs);

  if (params.auto_mask)
  {
    output_mask_projs.reserve(num_projs);
  }

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    this->dout() << "proj: " << proj_idx << std::endl;
    
    const auto& input_proj_data = input_projs[proj_idx];

    auto output_proj = input_proj_data;

    auto& cam   = output_proj.cam; 
    auto& img   = output_proj.img;
    auto& lands = output_proj.landmarks;

    if (img)
    {
      this->dout() << "copying 2D intensity pixels..." << std::endl;
      img = ITKImageDeepCopy(img.GetPointer());
    }

    if (params.crop_width > 0)
    {
      this->dout() << "cropping intensity image and camera model..." << std::endl;
      std::tie(cam, img) = CropBoundaryPixels(cam, img.GetPointer(), params.crop_width);

      this->dout() << "updating landmarks for cropping..." << std::endl;
      for (auto& l : lands)
      {
        l.second[0] -= params.crop_width;
        l.second[1] -= params.crop_width;
      }
    }

    if (img && !params.no_log_remap)
    {
      this->dout() << "log remapping..." << std::endl;
      
      auto log_xform = ImageIntensLogTransFilter::New();
      log_xform->SetInput(img);
      log_xform->SetUseMaxIntensityAsI0(true);
      log_xform->Update();
      
      img = log_xform->GetOutput();
    }

    output_projs.push_back(output_proj);

    if (img && params.auto_mask)
    {
      this->dout() << "auto masking metal (naively!)..." << std::endl;

      SegmentMetalInXRay metal_seg;
      metal_seg.set_debug_output_stream(*this);
      
      metal_seg.binarize = true;
      metal_seg.dilation_radius = 3;
      metal_seg.src_img = img;
      
      metal_seg.thresh = params.auto_mask_thresh;
      metal_seg.level  = params.auto_mask_level;

      if (!params.allow_iterative_thresh)
      {
        metal_seg.num_pix_in_mask_upper_thresh = -1;
        metal_seg.num_pix_in_mask_lower_thresh = -1;
      }

      metal_seg();

      std::decay<decltype(output_mask_projs[0])>::type output_mask_proj;

      // the segmentation is in signed short, cast to the type we need here (should be unsigned char)
      output_mask_proj.img = CastITKImageIfNeeded<unsigned char>(metal_seg.seg_img.GetPointer());

      if (params.invert_mask)
      {
        auto& mask = output_mask_proj.img;

        const auto img_size = mask->GetLargestPossibleRegion().GetSize();
        
        const unsigned long tot_num_pix = static_cast<unsigned long>(img_size[0]) *
                                          static_cast<unsigned long>(img_size[1]);

        unsigned char* mask_buf = mask->GetBufferPointer();

        for (unsigned long i = 0; i < tot_num_pix; ++i)
        {
          mask_buf[i] = !mask_buf[i];
        }
      }

      output_mask_proj.cam = cam;
      output_mask_proj.landmarks = lands;
      
      output_mask_projs.push_back(output_mask_proj);
    }
  }

  this->dout() << "pre-processing complete..." << std::endl;
}

