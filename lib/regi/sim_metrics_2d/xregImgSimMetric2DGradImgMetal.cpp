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

#include "xregImgSimMetric2DGradImgMetal.h"

#include "xregAssert.h"
#include "xregNormDist.h"

namespace xreg
{

// embedded at build time, see lib/regi/CMakeLists.txt
extern const char* const kIMG_SIM_METRIC_2D_GRAD_IMG_METAL_SRC;

}  // xreg

namespace
{

using namespace xreg;

/// \brief A normalized, square, Gaussian kernel in row-major order.
///
/// This is the same kernel used by ImgSimMetric2DGradImgOCL, which uses the
/// OpenCV formula for the standard deviation given a kernel width.
std::vector<float> MakeGaussianSmoothKernel(const size_type width)
{
  const float sigma = ((((width - 1) * 0.5f) - 1.0f) * 0.3f) + 0.8f;

  // distribution with mean at the kernel center index
  NormalDist2DIndep norm_dist(0, 0, sigma, sigma);

  std::vector<float> kern;
  kern.reserve(width * width);

  float kern_sum = 0;

  const int half_width = static_cast<int>(width) / 2;

  for (int r = -half_width; r <= half_width; ++r)
  {
    for (int c = -half_width; c <= half_width; ++c)
    {
      const float cur_val = norm_dist(r,c);
      kern.push_back(cur_val);

      kern_sum += cur_val;
    }
  }

  // normalize so the kernel has sum of 1
  for (auto& k : kern)
  {
    k /= kern_sum;
  }

  return kern;
}

}  // un-named

xreg::ImgSimMetric2DGradImgMetal::ImgSimMetric2DGradImgMetal(const MetalDevice& dev)
  : ImgSimMetric2DMetal(dev)
{ }

xreg::ImgSimMetric2DGradImgMetal::ImgSimMetric2DGradImgMetal(const MetalCmdQueue& queue)
  : ImgSimMetric2DMetal(queue)
{ }

void xreg::ImgSimMetric2DGradImgMetal::allocate_resources()
{
  ImgSimMetric2DMetal::allocate_resources();

  const MetalLibrary lib = this->make_library(kIMG_SIM_METRIC_2D_GRAD_IMG_METAL_SRC);

  sobel_pipeline_ = MetalComputePipeline(lib, "xregSobelKernel");

  const MetalDevice& dev = this->device();

  const size_type num_pix_per_img = this->num_pix_per_proj();
  const size_type max_buf_size    = num_pix_per_img * this->num_mov_imgs_;

  if (smooth_img_kernel_rad_)
  {
    // width must be odd
    xregASSERT(smooth_img_kernel_rad_ & 1);

    smooth_pipeline_ = MetalComputePipeline(lib, "xregSmoothKernel");

    const std::vector<float> kern = MakeGaussianSmoothKernel(smooth_img_kernel_rad_);

    smooth_kernel_dev_buf_ = DevBuf(dev, kern.size());
    CopyHostToMetal(kern.data(), kern.data() + kern.size(), smooth_kernel_dev_buf_, 0, this->queue_);

    // also used to smooth the fixed image
    smooth_imgs_dev_buf_ = DevBuf(dev, std::max(max_buf_size, num_pix_per_img));
  }

  fixed_grad_x_dev_buf_ = std::make_shared<DevBuf>(dev, num_pix_per_img);
  fixed_grad_y_dev_buf_ = std::make_shared<DevBuf>(dev, num_pix_per_img);

  mov_grad_x_dev_buf_ = std::make_shared<DevBuf>(dev, max_buf_size);
  mov_grad_y_dev_buf_ = std::make_shared<DevBuf>(dev, max_buf_size);

  // compute the fixed image gradients
  MetalComputeEncoder enc(this->queue_);

  encode_sobel_grads(enc, *this->fixed_img_metal_buf_, 0, 1,
                     *fixed_grad_x_dev_buf_, *fixed_grad_y_dev_buf_);

  enc.commit_and_wait();
}

xreg::size_type xreg::ImgSimMetric2DGradImgMetal::smooth_img_before_sobel_kernel_radius() const
{
  return smooth_img_kernel_rad_;
}

void xreg::ImgSimMetric2DGradImgMetal::set_smooth_img_before_sobel_kernel_radius(const size_type r)
{
  smooth_img_kernel_rad_ = r;
}

void xreg::ImgSimMetric2DGradImgMetal::compute_sobel_grads()
{
  MetalComputeEncoder enc(this->queue_);

  encode_sobel_grads(enc, *this->mov_imgs_buf_, this->mov_imgs_buf_offset(), this->num_mov_imgs_,
                     *mov_grad_x_dev_buf_, *mov_grad_y_dev_buf_);

  enc.commit_and_wait();
}

void xreg::ImgSimMetric2DGradImgMetal::encode_sobel_grads(MetalComputeEncoder& enc,
                                                          const DevBuf& src_imgs,
                                                          const size_type src_off,
                                                          const size_type num_imgs,
                                                          DevBuf& grad_x_imgs, DevBuf& grad_y_imgs)
{
  enc.set_value(this->img_args(num_imgs), kSIM_METAL_GRAD_IMG_ARGS);

  if (smooth_img_kernel_rad_)
  {
    enc.set_buffer(src_imgs, kSIM_METAL_GRAD_SRC_IMGS, src_off);
    enc.set_buffer(smooth_kernel_dev_buf_, kSIM_METAL_GRAD_SMOOTH_KERNEL);
    enc.set_value(static_cast<sim_metric_metal_types::UInt>(smooth_img_kernel_rad_),
                  kSIM_METAL_GRAD_SMOOTH_WIDTH);
    enc.set_buffer(smooth_imgs_dev_buf_, kSIM_METAL_GRAD_OUT_X);

    dispatch_per_pixel(enc, smooth_pipeline_, num_imgs);

    // differentiate the smoothed images
    enc.set_buffer(smooth_imgs_dev_buf_, kSIM_METAL_GRAD_SRC_IMGS);
  }
  else
  {
    enc.set_buffer(src_imgs, kSIM_METAL_GRAD_SRC_IMGS, src_off);
  }

  enc.set_buffer(grad_x_imgs, kSIM_METAL_GRAD_OUT_X);
  enc.set_buffer(grad_y_imgs, kSIM_METAL_GRAD_OUT_Y);

  dispatch_per_pixel(enc, sobel_pipeline_, num_imgs);
}

void xreg::ImgSimMetric2DGradImgMetal::dispatch_per_pixel(MetalComputeEncoder& enc,
                                                          const MetalComputePipeline& pipeline,
                                                          const size_type num_imgs)
{
  const auto img_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  // 2D tiles of pixels, so that neighboring pixels read by each thread are
  // likely to be cached
  size_type tile_num_cols = 16;
  size_type tile_num_rows = 8;

  while ((tile_num_cols * tile_num_rows) > pipeline.max_total_threads_per_threadgroup())
  {
    tile_num_rows /= 2;
  }

  enc.set_pipeline(pipeline);
  enc.dispatch_threads({ img_size[0], img_size[1], num_imgs }, { tile_num_cols, tile_num_rows, 1 });
}
