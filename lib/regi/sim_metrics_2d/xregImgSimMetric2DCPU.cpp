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

#include "xregImgSimMetric2DCPU.h"

#include "xregRayCastInterface.h"
#include "xregRayCastSyncBuf.h"
#include "xregAssert.h"
#include "xregExceptionUtils.h"

void xreg::ImgSimMetric2DCPU::allocate_resources()
{
  ImgSimMetric2D::allocate_resources();

  if (sync_host_buf_)
  {
    sync_host_buf_->alloc();
    mov_imgs_buf_ = sync_host_buf_->host_buf().buf + (proj_off_ * this->num_pix_per_proj());
  }

  xregASSERT(mov_imgs_buf_);
}

void xreg::ImgSimMetric2DCPU::set_mov_imgs_buf_from_ray_caster(
                                      RayCaster* ray_caster,
                                      const size_type proj_offset)
{
  if (mov_imgs_buf_)
  {
    if (sync_host_buf_ != ray_caster->to_host_buf())
    {
      // The moving images buffer had been previously set, and the previous
      // sync object (if any) is not equal to the new one, indicating a completely
      // new input... this is probably a misuse
      // This case should go away once the similarity metric interface is re-designed
      xregThrow("Moving buffer already exists, and the new sync object does not match previous!");
    }
    else if (proj_off_ != proj_offset)
    {
      // the buffer has already been allocated, but the projection offset is being updated
      // this is a common use, when re-using similarity metrics between different registrations,
      // which each use a different number of moving images
      mov_imgs_buf_ = sync_host_buf_->host_buf().buf + (proj_offset * this->num_pix_per_proj());
    }
  }
  
  sync_host_buf_ = ray_caster->to_host_buf();
  proj_off_ = proj_offset;
}

void xreg::ImgSimMetric2DCPU::set_mov_imgs_host_buf(
                                      Scalar* mov_imgs_buf,
                                      const size_type proj_offset)
{
  xregASSERT(!sync_host_buf_);

  proj_off_ = proj_offset;

  if (proj_offset > 0)
  {
    mov_imgs_buf_ = mov_imgs_buf + (proj_offset * this->num_pix_per_proj());
  }
  else
  {
    mov_imgs_buf_ = mov_imgs_buf;
  }
}

void xreg::ImgSimMetric2DCPU::pre_compute()
{
  if (sync_host_buf_)
  {
    sync_host_buf_->sync();
  }
  
  this->process_updated_mask();
}

