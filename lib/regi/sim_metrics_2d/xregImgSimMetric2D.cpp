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

#include "xregImgSimMetric2D.h"
  
void xreg::ImgSimMetric2D::set_fixed_image(ImagePtr fixed_img)
{
  fixed_img_ = fixed_img;
}

xreg::ImgSimMetric2D::ImagePtr xreg::ImgSimMetric2D::fixed_image()
{
  return fixed_img_;
}

void xreg::ImgSimMetric2D::allocate_resources()
{
  sim_vals_.resize(num_mov_imgs_);
}
  
void xreg::ImgSimMetric2D::set_num_moving_images(const size_type n)
{
  num_mov_imgs_ = n;
}

xreg::size_type xreg::ImgSimMetric2D::num_moving_images() const
{
  return num_mov_imgs_;
}

xreg::ImgSimMetric2D::Scalar&
xreg::ImgSimMetric2D::sim_val(const size_type mov_img_idx)
{
  return sim_vals_[mov_img_idx];
}

const xreg::ImgSimMetric2D::Scalar&
xreg::ImgSimMetric2D::sim_val(const size_type mov_img_idx) const
{
  return sim_vals_[mov_img_idx];
}

xreg::ImgSimMetric2D::ScalarList& xreg::ImgSimMetric2D::sim_vals()
{
  return sim_vals_;
}

const xreg::ImgSimMetric2D::ScalarList& xreg::ImgSimMetric2D::sim_vals() const
{
  return sim_vals_;
}

void xreg::ImgSimMetric2D::set_mov_imgs_buf_from_ray_caster(
                              RayCaster* ray_caster, const size_type proj_offset)
{
  throw UnsupportedOperationException();
}
  
void xreg::ImgSimMetric2D::set_mask(ImageMaskPtr mask)
{
  // the mask is considered updated if it was previously updated
  // or the passed mask is a different pointer than the current mask
  mask_updated_ = mask_updated_ || (mask.GetPointer() != mask_.GetPointer());

  mask_ = mask;
}

xreg::ImgSimMetric2D::ImageMaskPtr xreg::ImgSimMetric2D::mask()
{
  return mask_;
}

void xreg::ImgSimMetric2D::set_save_aux_info(const bool save_aux)
{
  save_aux_info_ = save_aux;
}
  
std::shared_ptr<xreg::H5ReadWriteInterface> xreg::ImgSimMetric2D::aux_info()
{
  return nullptr;
}
  
xreg::size_type xreg::ImgSimMetric2D::num_pix_per_proj()
{
  const auto img_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();
  return img_size[0] * img_size[1];
}

void xreg::ImgSimMetric2D::process_updated_mask()
{
  if (this->mask_updated_)
  {
    process_mask();

    mask_updated_ = false;
  }
}

void xreg::ImgSimMetric2D::process_mask()
{ }

