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

#include "xregImgSimMetric2DOCL.h"

// ITK pollutes the global namespace with a macro, and causes
// a compile failure with vienna cl
#ifdef vcl_size_t
#undef vcl_size_t
#endif
#ifdef vcl_ptrdiff_t
#undef vcl_ptrdiff_t
#endif
#include <viennacl/ocl/backend.hpp>

#include "xregRayCastInterface.h"
#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregViennaCLManager.h"

namespace vcl = viennacl;

xreg::ImgSimMetric2DOCL::ImgSimMetric2DOCL()
  : ctx_(boost::compute::system::default_device()),
    queue_(ctx_, ctx_.get_device())
{ }

xreg::ImgSimMetric2DOCL::ImgSimMetric2DOCL(const boost::compute::device& dev)
  : ctx_(dev), queue_(ctx_, dev)
{ }

xreg::ImgSimMetric2DOCL::ImgSimMetric2DOCL(const boost::compute::context& ctx,
                                           const boost::compute::command_queue& queue)
  : ctx_(ctx), queue_(queue)
{ }

void xreg::ImgSimMetric2DOCL::allocate_resources()
{
  ImgSimMetric2D::allocate_resources();

  // only copy the fixed image from host if it has not already been set from
  // an existing device buffer
  if (!fixed_img_ocl_buf_)
  {
    auto* host_fixed_buf = this->fixed_img_->GetBufferPointer();

    fixed_img_ocl_buf_ = std::make_shared<DevBuf>(ctx_);
    fixed_img_ocl_buf_->assign(host_fixed_buf, host_fixed_buf + this->num_pix_per_proj(),
                               queue_);
  }

  if (sync_ocl_buf_)
  {
    sync_ocl_buf_->alloc();
  }

  if (setup_vienna_cl_ctx_)
  {
    setup_vcl_ctx_if_needed(vienna_cl_ctx_idx_, ctx_, queue_);
  }
  
  vcl::ocl::switch_context(vienna_cl_ctx_idx_); 
  
  this->process_updated_mask();
}

void xreg::ImgSimMetric2DOCL::set_mov_imgs_buf_from_ray_caster(
                                      RayCaster* ray_caster,
                                      const size_type proj_offset)
{
  if (mov_imgs_buf_)
  {
    if (sync_ocl_buf_ != ray_caster->to_ocl_buf())
    {
      xregThrow("internal moving image buffer has already been set and "
                "sync object cannot be changed!");
    }
    else if (!sync_ocl_buf_->ocl_buf_valid())
    {
      xregThrow("moving image buffer already set, but sync object is invalid!");
    }
  }
  else
  {
    sync_ocl_buf_ = ray_caster->to_ocl_buf();

    // An OpenCL ray caster will have already called set_ocl() on the sync object
    // using its internally allocated buffer. A CPU ray caster has no idea about
    // setting a device buffer and command queue, therefore we need to check
    // if the device buffer is valid before retrieving the buffer. If the
    /// buffer is not valid, then an internal version is created in this
    /// object.
    if (!sync_ocl_buf_->ocl_buf_valid())
    {
      internal_ocl_buf_.reset(new DevBuf(ctx_));
      sync_ocl_buf_->set_ocl(internal_ocl_buf_.get(), queue_);
      // the buffer will be allocated later in the call to alloc().
    }
    else
    {
      ctx_   = sync_ocl_buf_->ocl_buf().get_buffer().get_context();
      queue_ = sync_ocl_buf_->queue();
    }

    mov_imgs_buf_ = &sync_ocl_buf_->ocl_buf();
  }

  // This always updates, e.g. even if the sync buffer is already set and
  // valid, we may want to re-use this with different number of moving
  // images (similar to CPU case).
  proj_off_ = proj_offset;
}

void xreg::ImgSimMetric2DOCL::set_mov_imgs_ocl_buf(DevBuf* mov_imgs_buf, const size_type proj_offset)
{
  xregASSERT(!sync_ocl_buf_);

  mov_imgs_buf_ = mov_imgs_buf;
  proj_off_     = proj_offset;
}

void xreg::ImgSimMetric2DOCL::set_mov_imgs_host_buf(Scalar* mov_imgs_buf,
                                                    const size_type proj_offset)
{
  xregASSERT(!sync_ocl_buf_);

  // create a new device buffer and copy the data from the host
  internal_ocl_buf_.reset(new DevBuf(ctx_));
  internal_ocl_buf_->assign(mov_imgs_buf, mov_imgs_buf + num_pix_per_proj() * num_mov_imgs_, queue_);

  mov_imgs_buf_ = internal_ocl_buf_.get();

  proj_off_ = proj_offset;
}
  
long xreg::ImgSimMetric2DOCL::vienna_cl_ctx_idx() const
{
  return vienna_cl_ctx_idx_;
}

void xreg::ImgSimMetric2DOCL::set_vienna_cl_ctx_idx(const long ctx_idx)
{
  vienna_cl_ctx_idx_ = ctx_idx;
}

void xreg::ImgSimMetric2DOCL::set_setup_vienna_cl_ctx(const bool ctx)
{
  setup_vienna_cl_ctx_ = ctx;
}

void xreg::ImgSimMetric2DOCL::set_fixed_image_dev(std::shared_ptr<DevBuf>& fixed_dev)
{
  fixed_img_ocl_buf_ = fixed_dev;
}

void xreg::ImgSimMetric2DOCL::pre_compute()
{
  vcl::ocl::switch_context(vienna_cl_ctx_idx_); 
  
  if (sync_ocl_buf_)
  {
    sync_ocl_buf_->sync();
  }
  
  this->process_updated_mask();
}

void xreg::ImgSimMetric2DOCL::process_mask()
{
  if (this->mask_)
  {
    const size_type num_pix_per_img = this->num_pix_per_proj();
    
    const unsigned char* mask_buf_host = this->mask_->GetBufferPointer();

    // count the number of pixels not masked out.
    num_pix_per_proj_after_mask_ = 0;
    std::for_each(mask_buf_host, mask_buf_host + num_pix_per_img,
                  [&] (const unsigned char m)
                  {
                    if (m)
                    {
                      ++num_pix_per_proj_after_mask_;
                    }
                  });

    // convert the mask to float
    std::vector<Scalar> mask_float_host(num_pix_per_img);
    std::transform(mask_buf_host, mask_buf_host + num_pix_per_img,
                   mask_float_host.begin(),
                   [] (const unsigned char m)
                   {
                     return m ? Scalar(1) : Scalar(0);
                   });

    // now copy the float mask to device
    mask_ocl_buf_.reset(new DevBuf(ctx_));
    mask_ocl_buf_->assign(mask_float_host.begin(), mask_float_host.end(),
                          queue_);
  }
}
