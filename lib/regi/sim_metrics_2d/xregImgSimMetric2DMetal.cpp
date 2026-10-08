/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#include "xregImgSimMetric2DMetal.h"

#include <algorithm>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregMetalShaderSrc.h"
#include "xregRayCastInterface.h"

namespace xreg
{

// embedded at build time, see lib/regi/CMakeLists.txt
extern const char* const kIMG_SIM_METRIC_2D_METAL_ARGS_SRC;
extern const char* const kIMG_SIM_METRIC_2D_METAL_SRC;

}  // xreg

xreg::ImgSimMetric2DMetal::ImgSimMetric2DMetal()
  : ImgSimMetric2DMetal(MetalDevice::Default())
{ }

xreg::ImgSimMetric2DMetal::ImgSimMetric2DMetal(const MetalDevice& dev)
  : queue_(dev)
{ }

xreg::ImgSimMetric2DMetal::ImgSimMetric2DMetal(const MetalCmdQueue& queue)
  : queue_(queue)
{ }

void xreg::ImgSimMetric2DMetal::allocate_resources()
{
  ImgSimMetric2D::allocate_resources();

  // only copy the fixed image from host if it has not already been set from
  // an existing device buffer
  if (!fixed_img_metal_buf_)
  {
    const size_type num_pix = this->num_pix_per_proj();

    const Scalar* host_fixed_buf = this->fixed_img_->GetBufferPointer();

    fixed_img_metal_buf_ = std::make_shared<DevBuf>(device(), num_pix);

    CopyHostToMetal(host_fixed_buf, host_fixed_buf + num_pix, *fixed_img_metal_buf_, 0, queue_);
  }

  if (sync_metal_buf_)
  {
    sync_metal_buf_->alloc();
  }

  xregASSERT(mov_imgs_buf_);

  this->process_updated_mask();
}

void xreg::ImgSimMetric2DMetal::set_mov_imgs_buf_from_ray_caster(RayCaster* ray_caster,
                                                                 const size_type proj_offset)
{
  if (mov_imgs_buf_)
  {
    if (sync_metal_buf_ != ray_caster->to_metal_buf())
    {
      xregThrow("internal moving image buffer has already been set and "
                "sync object cannot be changed!");
    }
    else if (!sync_metal_buf_->metal_buf_valid())
    {
      xregThrow("moving image buffer already set, but sync object is invalid!");
    }
  }
  else
  {
    sync_metal_buf_ = ray_caster->to_metal_buf();

    // A Metal ray caster will have already called set_metal() on the sync
    // object using its internally allocated buffer. Otherwise, an internal
    // buffer is created in this object, which is allocated later in the call
    // to alloc().
    if (!sync_metal_buf_->metal_buf_valid())
    {
      internal_metal_buf_ = std::make_unique<DevBuf>(device());
      sync_metal_buf_->set_metal(internal_metal_buf_.get(), queue_);
    }
    else
    {
      queue_ = sync_metal_buf_->queue();
    }

    mov_imgs_buf_ = &sync_metal_buf_->metal_buf();
  }

  // This always updates, e.g. even if the sync buffer is already set and
  // valid, we may want to re-use this with different number of moving
  // images (similar to CPU case).
  proj_off_ = proj_offset;
}

void xreg::ImgSimMetric2DMetal::set_mov_imgs_metal_buf(DevBuf* mov_imgs_buf,
                                                       const size_type proj_offset)
{
  xregASSERT(!sync_metal_buf_);

  mov_imgs_buf_ = mov_imgs_buf;
  proj_off_     = proj_offset;
}

void xreg::ImgSimMetric2DMetal::set_mov_imgs_host_buf(Scalar* mov_imgs_buf,
                                                      const size_type proj_offset)
{
  xregASSERT(!sync_metal_buf_);

  const size_type num_pix = this->num_pix_per_proj();

  const Scalar* src = mov_imgs_buf + (proj_offset * num_pix);

  // only the images used are copied to the device, so no offset is needed
  // into the device buffer
  internal_metal_buf_ = std::make_unique<DevBuf>(device(), num_pix * this->num_mov_imgs_);

  CopyHostToMetal(src, src + (num_pix * this->num_mov_imgs_), *internal_metal_buf_, 0, queue_);

  mov_imgs_buf_ = internal_metal_buf_.get();
  proj_off_     = 0;
}

void xreg::ImgSimMetric2DMetal::set_fixed_image_dev(std::shared_ptr<DevBuf>& fixed_dev)
{
  fixed_img_metal_buf_ = fixed_dev;
}

const xreg::MetalCmdQueue& xreg::ImgSimMetric2DMetal::queue() const
{
  return queue_;
}

const xreg::MetalDevice& xreg::ImgSimMetric2DMetal::device() const
{
  return queue_.device();
}

xreg::MetalLibrary xreg::ImgSimMetric2DMetal::make_library(const std::string& kernels_src) const
{
  std::string src = MetalHelperShadersSrc();

  for (const char* s : { kIMG_SIM_METRIC_2D_METAL_ARGS_SRC, kIMG_SIM_METRIC_2D_METAL_SRC })
  {
    src += s;
    src += '\n';
  }

  src += kernels_src;

  return MetalLibrary(device(), src);
}

void xreg::ImgSimMetric2DMetal::pre_compute()
{
  // the device buffers are allocated for the number of moving images at the
  // time of allocate_resources()
  xregASSERT(this->num_mov_imgs_ <= this->sim_vals_.size());

  if (sync_metal_buf_)
  {
    sync_metal_buf_->sync();
  }

  this->process_updated_mask();
}

void xreg::ImgSimMetric2DMetal::process_mask()
{
  const size_type num_pix = this->num_pix_per_proj();

  std::vector<Scalar> mask_float_host(num_pix, Scalar(1));

  if (this->mask_)
  {
    const MaskScalar* mask_buf_host = this->mask_->GetBufferPointer();

    std::transform(mask_buf_host, mask_buf_host + num_pix, mask_float_host.begin(),
                   [] (const MaskScalar m)
                   {
                     return m ? Scalar(1) : Scalar(0);
                   });

    num_pix_per_proj_after_mask_ = std::count_if(mask_buf_host, mask_buf_host + num_pix,
                                                 [] (const MaskScalar m) { return m != 0; });
  }
  else
  {
    num_pix_per_proj_after_mask_ = num_pix;
  }

  mask_metal_buf_ = DevBuf(device(), num_pix);

  CopyHostToMetal(mask_float_host.data(), mask_float_host.data() + num_pix, mask_metal_buf_, 0, queue_);
}

xreg::ImgSimMetricMetalImgArgs xreg::ImgSimMetric2DMetal::img_args(const size_type num_imgs)
{
  const auto img_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  ImgSimMetricMetalImgArgs args;

  args.num_cols         = static_cast<sim_metric_metal_types::UInt>(img_size[0]);
  args.num_rows         = static_cast<sim_metric_metal_types::UInt>(img_size[1]);
  args.num_pix          = args.num_cols * args.num_rows;
  args.num_imgs         = static_cast<sim_metric_metal_types::UInt>(num_imgs);
  args.num_unmasked_pix = static_cast<float>(num_pix_per_proj_after_mask_);

  return args;
}

xreg::size_type xreg::ImgSimMetric2DMetal::mov_imgs_buf_offset()
{
  return proj_off_ * this->num_pix_per_proj();
}

void xreg::ImgSimMetric2DMetal::dispatch_reduction(MetalComputeEncoder& enc,
                                                   const MetalComputePipeline& pipeline,
                                                   const size_type num_rows)
{
  // the threadgroup reductions support a limited number of SIMD groups
  constexpr size_type kMAX_THREADGROUP_SIZE = 256;
  constexpr size_type kMAX_NUM_SIMD_GROUPS  = 32;  // ThreadgroupReducerScratch::kMAX_NUM_SIMD_GROUPS

  const size_type simd_width = pipeline.thread_execution_width();

  size_type tg_size = std::min({ kMAX_THREADGROUP_SIZE, pipeline.max_total_threads_per_threadgroup(),
                                 kMAX_NUM_SIMD_GROUPS * simd_width });

  // whole SIMD groups
  tg_size = std::max(simd_width, (tg_size / simd_width) * simd_width);

  enc.set_pipeline(pipeline);
  enc.dispatch_threads({ tg_size * num_rows, 1, 1 }, { tg_size, 1, 1 });
}

void xreg::ImgSimMetric2DMetal::copy_sim_vals_to_host(const DevBuf& sim_vals_dev)
{
  CopyMetalToHost(sim_vals_dev, 0, this->num_mov_imgs_, this->sim_vals_.data(), queue_);
}
