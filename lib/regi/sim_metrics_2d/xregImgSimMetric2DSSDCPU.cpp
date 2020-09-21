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

#include "xregImgSimMetric2DSSDCPU.h"

#include "xregTBBUtils.h"

namespace  // un-named
{

template <class tDataMat, class tMaskMat>
void ApplyMaskToEigenMatInPlace(tDataMat* mat, const tMaskMat& mask, const typename tDataMat::Scalar masked_val)
{
  tDataMat& m = *mat;

  const unsigned long nr = m.rows();
  const unsigned long nc = m.cols();

  // cols first, since most eigen objects will be col-major by default
  for (unsigned long c = 0; c < nc; ++c)
  {
    for (unsigned long r = 0; r < nr; ++r)
    {
      if (!mask(r,c))
      {
        m(r,c) = masked_val;
      }
    }
  }
}

}  // un-named

void xreg::ImgSimMetric2DSSDCPU::allocate_resources()
{
  ImgSimMetric2DCPU::allocate_resources();
  
  this->process_updated_mask();
}

void xreg::ImgSimMetric2DSSDCPU::compute()
{
  this->pre_compute();

  const auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  const size_type img_num_cols = itk_size[0];
  const size_type img_num_rows = itk_size[1];
  const size_type img_num_pix  = img_num_cols * img_num_rows;
  
  auto ssd_fn = [&] (const RangeType& r)
  {
    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      MappedImageVec cur_mov_vec(this->mov_imgs_buf_ + (range_idx * img_num_pix), img_num_pix);

      if (this->mask_)
      {
        ApplyMaskToEigenMatInPlace(&cur_mov_vec, mask_vec_, Scalar(0));
      }

      this->sim_vals_[range_idx] = (fixed_img_vec_ - cur_mov_vec).array().square().sum() / img_num_pix;
    }
  };

  ParallelFor(ssd_fn, RangeType(0, this->num_mov_imgs_));
}


void xreg::ImgSimMetric2DSSDCPU::process_mask()
{
  ImgSimMetric2DCPU::process_mask();
  
  auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  const size_type img_num_cols = itk_size[0];
  const size_type img_num_rows = itk_size[1];
  const size_type img_num_pix  = img_num_cols * img_num_rows;

  fixed_img_vec_ = MappedImageVec(this->fixed_img_->GetBufferPointer(), img_num_pix);

  if (this->mask_)
  {
    mask_vec_ = MappedImageMaskVec(this->mask_->GetBufferPointer(), img_num_pix);

    ApplyMaskToEigenMatInPlace(&fixed_img_vec_, mask_vec_, Scalar(0));
  }
}
 
