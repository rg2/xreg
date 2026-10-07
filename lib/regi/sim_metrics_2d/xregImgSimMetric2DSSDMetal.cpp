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

#include "xregImgSimMetric2DSSDMetal.h"

namespace xreg
{

// embedded at build time, see lib/regi/CMakeLists.txt
extern const char* const kIMG_SIM_METRIC_2D_SSD_METAL_SRC;

}  // xreg

xreg::ImgSimMetric2DSSDMetal::ImgSimMetric2DSSDMetal(const MetalDevice& dev)
  : ImgSimMetric2DMetal(dev)
{ }

xreg::ImgSimMetric2DSSDMetal::ImgSimMetric2DSSDMetal(const MetalCmdQueue& queue)
  : ImgSimMetric2DMetal(queue)
{ }

void xreg::ImgSimMetric2DSSDMetal::allocate_resources()
{
  ssd_pipeline_ = MetalComputePipeline(this->make_library(kIMG_SIM_METRIC_2D_SSD_METAL_SRC),
                                       "xregSSDKernel");

  sim_vals_dev_ = DevBuf(this->device(), this->num_mov_imgs_);

  ImgSimMetric2DMetal::allocate_resources();
}

void xreg::ImgSimMetric2DSSDMetal::compute()
{
  this->pre_compute();

  MetalComputeEncoder enc(this->queue_);

  enc.set_value(this->img_args(this->num_mov_imgs_), kSIM_METAL_IMG_ARGS);
  enc.set_buffer(*this->fixed_img_metal_buf_, kSIM_METAL_FIXED_IMG);
  enc.set_buffer(this->mask_metal_buf_, kSIM_METAL_MASK);
  enc.set_buffer(*this->mov_imgs_buf_, kSIM_METAL_MOV_IMGS, this->mov_imgs_buf_offset());
  enc.set_buffer(sim_vals_dev_, kSIM_METAL_OUT);

  this->dispatch_reduction(enc, ssd_pipeline_, this->num_mov_imgs_);

  enc.commit_and_wait();

  this->copy_sim_vals_to_host(sim_vals_dev_);
}
