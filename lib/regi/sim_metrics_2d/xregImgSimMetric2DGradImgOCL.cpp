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

#include "xregImgSimMetric2DGradImgOCL.h"

#include <boost/compute/utility/source.hpp>

#include "xregAssert.h"
#include "xregNormDist.h"

namespace
{

// This should actually be set based on the size of the input data (and the GPU HW)
const std::size_t kSOBEL_PROJ_DIMS_WORK_UNIT_DIV = 1;
const std::size_t kSMOOTH_PROJ_DIMS_WORK_UNIT_DIV = 1;

const char* kGRAD_NCC_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void GaussianKernel(__global float* src_imgs,
                             __global float* smooth_imgs,
                             __global float* smooth_kernel,
                             const uint kernel_width,
                             const uint num_imgs,
                             const uint img_nr,
                             const uint img_nc,
                             const uint proj_off)
{
  const uint row_inc = get_global_size(0);
  const uint col_inc = get_global_size(1);
  const uint img_idx = get_global_id(2);
  const uint img_len = img_nr * img_nc;

  if (img_idx < num_imgs)
  {
    const uint img_nr_minus_1 = img_nr - 1;
    const uint img_nc_minus_1 = img_nc - 1;

    const int kernel_half_width = (int) (kernel_width / 2);

    // this is typically from the ray caster buffer, which can be split up amongst different sim
    // metrics and thus why we need projection offset.
    __global float* cur_src_img = src_imgs + ((proj_off + img_idx) * img_len);
   
    // this buffer is local to this sim metric and we can just use the beginning section; no need
    // to worry about projection offset
    __global float* cur_smooth_img = smooth_imgs + (img_idx * img_len);

    for (uint row_idx = get_global_id(0); row_idx < img_nr; row_idx += row_inc)
    {
      __global float* smooth_row = cur_smooth_img + (row_idx * img_nc);

      for (uint col_idx = get_global_id(1); col_idx < img_nc; col_idx += col_inc)
      {
        float cur_sum = 0;

        uint kern_idx = 0;
        for (int kr = -kernel_half_width; kr <= kernel_half_width; ++kr)
        {
          const int conv_r_no_clamp = ((int)row_idx) + kr;
          
          // clamp row used in image
          const uint conv_r_clamp = (conv_r_no_clamp >= 0) ?
                                      ((conv_r_no_clamp < (int)img_nr) ? (uint)conv_r_no_clamp : img_nr_minus_1) : 0u;

          __global float* cur_conv_row = cur_src_img + (conv_r_clamp * img_nc);

          for (int kc = -kernel_half_width; kc <= kernel_half_width; ++kc, ++kern_idx)
          {
            const int conv_c_no_clamp = ((int)col_idx) + kc;

            // clamp column used in image
            const uint conv_c_clamp = (conv_c_no_clamp >= 0) ?
                                      ((conv_c_no_clamp < (int)img_nc) ? (uint)conv_c_no_clamp : img_nc_minus_1) : 0u;

            cur_sum += cur_conv_row[conv_c_clamp] * smooth_kernel[kern_idx];
          }
        }
        
        smooth_row[col_idx] = cur_sum;
      }
    }
  }
}

__kernel void SobelKernel(__global float* src_imgs,
                          __global float* grad_x_imgs,
                          __global float* grad_y_imgs,
                          const uint num_imgs,
                          const uint img_nr,
                          const uint img_nc,
                          const uint proj_off)
{
  const uint row_inc = get_global_size(0);
  const uint col_inc = get_global_size(1);
  const uint img_idx = get_global_id(2);
  const uint img_len = img_nr * img_nc;

  if (img_idx < num_imgs)
  {
    const uint img_nr_minus_1 = img_nr - 1;
    const uint img_nc_minus_1 = img_nc - 1;

    // this is typically from the ray caster buffer, which can be split up amongst different sim
    // metrics and thus why we need projection offset.
    __global float* cur_src_img = src_imgs + ((proj_off + img_idx) * img_len);
   
    // this buffer is local to this sim metric and we can just use the beginning section; no need
    // to worry about projection offset
    __global float* cur_grad_x_img = grad_x_imgs + (img_idx * img_len);
    __global float* cur_grad_y_img = grad_y_imgs + (img_idx * img_len);

    for (uint row_idx = get_global_id(0); row_idx < img_nr; row_idx += row_inc)
    {
      const uint cur_row_off  = row_idx * img_nc;
      const uint prev_row_off = (row_idx > 0) ? (cur_row_off - img_nc) : cur_row_off;
      const uint next_row_off = (row_idx < img_nr_minus_1) ? (cur_row_off + img_nc) : cur_row_off;

      __global float* cur_src_row  = cur_src_img + cur_row_off;
      __global float* prev_src_row = cur_src_img + prev_row_off;
      __global float* next_src_row = cur_src_img + next_row_off;

      __global float* grad_x_row = cur_grad_x_img + cur_row_off;
      __global float* grad_y_row = cur_grad_y_img + cur_row_off;

      for (uint col_idx = get_global_id(1); col_idx < img_nc; col_idx += col_inc)
      {
        const uint prev_col = (col_idx > 0) ? (col_idx - 1) : 0;
        const uint next_col = (col_idx < img_nc_minus_1) ? (col_idx + 1) : col_idx;

        // x-direction
        grad_x_row[col_idx] = -prev_src_row[prev_col] + prev_src_row[next_col] +
                              -(2.0f * cur_src_row[prev_col]) + (2.0f * cur_src_row[next_col]) +
                              -next_src_row[prev_col] + next_src_row[next_col];

        // y-direction
        grad_y_row[col_idx] = -prev_src_row[prev_col] - (2.0f * prev_src_row[col_idx]) - prev_src_row[next_col]
                              + next_src_row[prev_col] + (2.0f * next_src_row[col_idx]) + next_src_row[next_col];
      }
    }
  }
}

);

}  // un-named

xreg::ImgSimMetric2DGradImgOCL::ImgSimMetric2DGradImgOCL(const boost::compute::device& dev)
  : ImgSimMetric2DOCL(dev)
{ }

xreg::ImgSimMetric2DGradImgOCL::ImgSimMetric2DGradImgOCL(const boost::compute::context& ctx,
                                                         const boost::compute::command_queue& queue)
  : ImgSimMetric2DOCL(ctx, queue)
{ }

void xreg::ImgSimMetric2DGradImgOCL::allocate_resources()
{
  namespace bc = boost::compute;
  
  ImgSimMetric2DOCL::allocate_resources();

  const auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();
  const size_type img_num_rows = itk_size[1];
  const size_type img_num_cols = itk_size[0];

  // compile kernels
  
  bc::program prog = bc::program::create_with_source(kGRAD_NCC_OPENCL_SRC, this->ctx_);

  try
  {
    prog.build();
  }
  catch (bc::opencl_error &)
  {
    std::cerr << "OpenCL Kernel Compile Error (ImgSimMetric2DGradImgOCL):\n"
              << prog.build_log() << std::endl;
    throw;
  }

  const size_type num_pix_per_img = this->num_pix_per_proj();
  const size_type max_buf_size    = num_pix_per_img * this->num_mov_imgs_;

  if (smooth_img_kernel_rad_)
  {
    // width must be odd
    xregASSERT(smooth_img_kernel_rad_ & 1);

    // Doh... the radius is actually diameter/width
    const size_type tot_kernel_len = smooth_img_kernel_rad_ * smooth_img_kernel_rad_;
    
    // This is the OpenCV formula; mainly using to get results consistent
    // with the CPU implementation that uses cv::GaussianBlur
    const float sigma = ((((smooth_img_kernel_rad_ - 1) * 0.5f) - 1.0f) * 0.3f) + 0.8f;

    // Create distribution with mean at the kernel center index
    NormalDist2DIndep norm_dist(0, 0, sigma, sigma);
    
    float kern_sum = 0;
    std::vector<float> tmp_host_kern;
    tmp_host_kern.reserve(tot_kernel_len);
   
    const int kernel_half_width = static_cast<int>(smooth_img_kernel_rad_) / 2;

    for (int r = -kernel_half_width; r <= kernel_half_width; ++r)
    {
      for (int c = -kernel_half_width; c <= kernel_half_width; ++c)
      {
        const float cur_val = norm_dist(r,c);
        tmp_host_kern.push_back(cur_val);

        kern_sum += cur_val;
      }
    }
    xregASSERT(tmp_host_kern.size() == tot_kernel_len);

    // normalize so kernel has sum of 1
    std::transform(tmp_host_kern.begin(), tmp_host_kern.end(), tmp_host_kern.begin(),
                   [kern_sum] (const float& x)
                   {
                     return x / kern_sum;
                   });

    smooth_kernel_dev_buf_.reset(new DevBuf(this->ctx_));
    smooth_kernel_dev_buf_->assign(tmp_host_kern.begin(), tmp_host_kern.end(),
                                   this->queue_);
    
    mov_smooth_dev_buf_.reset(new DevBuf(this->ctx_));
    mov_smooth_dev_buf_->resize(max_buf_size, this->queue_);

    smooth_krnl_ = prog.create_kernel("GaussianKernel");
    
    // smooth the fixed image
    smooth_krnl_.set_arg(0, *this->fixed_img_ocl_buf_);
    smooth_krnl_.set_arg(1, *mov_smooth_dev_buf_);
    smooth_krnl_.set_arg(2, *smooth_kernel_dev_buf_);
    smooth_krnl_.set_arg(3, bc::uint_(smooth_img_kernel_rad_));
    smooth_krnl_.set_arg(4, bc::uint_(1));
    smooth_krnl_.set_arg(5, bc::uint_(img_num_rows));
    smooth_krnl_.set_arg(6, bc::uint_(img_num_cols));
    smooth_krnl_.set_arg(7, bc::uint_(0));

    smooth_ocl_kernel_global_size_[0] = img_num_rows / kSMOOTH_PROJ_DIMS_WORK_UNIT_DIV;
    smooth_ocl_kernel_global_size_[1] = img_num_cols / kSMOOTH_PROJ_DIMS_WORK_UNIT_DIV;
    smooth_ocl_kernel_global_size_[2] = 1 ;
    this->queue_.enqueue_nd_range_kernel(smooth_krnl_, 3, 0, smooth_ocl_kernel_global_size_.data(), 0).wait();
  }

  sobel_krnl_ = prog.create_kernel("SobelKernel");

  fixed_grad_x_dev_buf_ = std::make_shared<DevBuf>(this->ctx_);
  fixed_grad_y_dev_buf_ = std::make_shared<DevBuf>(this->ctx_);

  mov_grad_x_dev_buf_ = std::make_shared<DevBuf>(this->ctx_);
  mov_grad_y_dev_buf_ = std::make_shared<DevBuf>(this->ctx_);

  fixed_grad_x_dev_buf_->resize(num_pix_per_img, this->queue_);
  fixed_grad_y_dev_buf_->resize(num_pix_per_img, this->queue_);

  mov_grad_x_dev_buf_->resize(max_buf_size, this->queue_);
  mov_grad_y_dev_buf_->resize(max_buf_size, this->queue_);

  // compute fixed image sobel grads
  sobel_krnl_.set_arg(0, smooth_img_kernel_rad_ ? *mov_smooth_dev_buf_ :
                                                  *this->fixed_img_ocl_buf_);
  sobel_krnl_.set_arg(1, *fixed_grad_x_dev_buf_);
  sobel_krnl_.set_arg(2, *fixed_grad_y_dev_buf_);
  sobel_krnl_.set_arg(3, bc::uint_(1));
  sobel_krnl_.set_arg(4, bc::uint_(img_num_rows));
  sobel_krnl_.set_arg(5, bc::uint_(img_num_cols));
  sobel_krnl_.set_arg(6, bc::uint_(0));

  sobel_ocl_kernel_global_size_[0] = img_num_rows / kSOBEL_PROJ_DIMS_WORK_UNIT_DIV;
  sobel_ocl_kernel_global_size_[1] = img_num_cols / kSOBEL_PROJ_DIMS_WORK_UNIT_DIV;
  sobel_ocl_kernel_global_size_[2] = 1 ;
  this->queue_.enqueue_nd_range_kernel(sobel_krnl_, 3, 0, sobel_ocl_kernel_global_size_.data(), 0).wait();

  // setup some kernel arguments that will not change
  sobel_krnl_.set_arg(1, *mov_grad_x_dev_buf_);
  sobel_krnl_.set_arg(2, *mov_grad_y_dev_buf_);
  // image dimensions do not change

  if (smooth_img_kernel_rad_)
  {
    smooth_krnl_.set_arg(0, *this->mov_imgs_buf_);
  }
  else
  {
    sobel_krnl_.set_arg(0, *this->mov_imgs_buf_);
  }
}
  
xreg::size_type xreg::ImgSimMetric2DGradImgOCL::smooth_img_before_sobel_kernel_radius() const
{
  return smooth_img_kernel_rad_;
}

void xreg::ImgSimMetric2DGradImgOCL::set_smooth_img_before_sobel_kernel_radius(const size_type r)
{
  smooth_img_kernel_rad_ = r;
}

void xreg::ImgSimMetric2DGradImgOCL::compute_sobel_grads()
{
  namespace bc = boost::compute;
  
  // this should be called by the compute method of the sub-class
  //this->pre_compute();
  
  sobel_ocl_kernel_global_size_[2]  = this->num_mov_imgs_;
  smooth_ocl_kernel_global_size_[2] = this->num_mov_imgs_;
  
  // smooth moving images
  if (smooth_img_kernel_rad_)
  {
    smooth_krnl_.set_arg(4, bc::uint_(this->num_mov_imgs_));
    smooth_krnl_.set_arg(7, bc::uint_(this->proj_off_));
  
    this->queue_.enqueue_nd_range_kernel(smooth_krnl_, 3, 0, smooth_ocl_kernel_global_size_.data(), 0).wait();
    
    sobel_krnl_.set_arg(6, bc::uint_(0));
  }
  else
  {
    sobel_krnl_.set_arg(6, bc::uint_(this->proj_off_));
  }

  // compute the sobel of moving images
  sobel_krnl_.set_arg(3, bc::uint_(this->num_mov_imgs_));

  this->queue_.enqueue_nd_range_kernel(sobel_krnl_, 3, 0, sobel_ocl_kernel_global_size_.data(), 0).wait();
}

