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

#ifndef XREGSEGMETALINXRAY_H_
#define XREGSEGMETALINXRAY_H_

#include <itkImage.h>

#include "xregObjWithOStream.h"

namespace xreg
{

// This is very naive and does not work all that precisely
struct SegmentMetalInXRay : ObjWithOStream
{
  using IntensImg    = itk::Image<float,2>;
  using IntensImgPtr = IntensImg::Pointer;

  using SegImage    = itk::Image<short,2>;
  using SegImagePtr = SegImage::Pointer;

  double thresh = 0.04;
  double level  = 0.35;
 
  // testing on simulated data: 
  // 0.1 seems to allow too large
  // 0.05 seems to require many iterations
  double num_pix_in_mask_upper_thresh = 0.075;
  
  double num_pix_in_mask_lower_thresh = 0.01;

  double level_inc = 0.01;

  unsigned long max_iters = 20;

  bool binarize = true;

  unsigned long dilation_radius = 0;

  IntensImgPtr src_img;

  SegImagePtr seg_img;

  IntensImgPtr grad_img;

  void operator()();
};

}  // xreg

#endif

