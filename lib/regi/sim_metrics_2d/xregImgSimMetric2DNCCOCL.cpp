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

#include "xregImgSimMetric2DNCCOCL.h"

#include <boost/compute/utility/source.hpp>

// ITK pollutes the global namespace with a macro, and causes
// a compile failure with vienna cl
#ifdef vcl_size_t
#undef vcl_size_t
#endif
#ifdef vcl_ptrdiff_t
#undef vcl_ptrdiff_t
#endif

#include <viennacl/matrix.hpp>
#include <viennacl/vector.hpp>
#include <viennacl/linalg/prod.hpp>

#include "xregOpenCLMiscKernels.h"

namespace
{

const std::size_t kWORK_GROUP_DIV_FACTOR = 1;

const char* kNCC_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void SubMeanAndSquareKernel(__global float* sub_from_mean_imgs,
                                     __global float* sub_from_mean_sq_imgs,
                                     __global const float* means,
                                     const uint num_imgs,
                                     const uint img_len,
                                     const uint proj_off)
{
  const uint global_inc = get_global_size(0);
  const uint img_idx    = get_global_id(1);

  if (img_idx < num_imgs)
  {
    // this is typically from the ray caster buffer, which can be split up amongst different sim
    // metrics and thus why we need projection offset.
    __global float* cur_sub_from_mean_img = sub_from_mean_imgs + ((proj_off + img_idx) * img_len);
   
    // this buffer is local to this sim metric and we can just use the beginning section; no need
    // to worry about projection offset
    __global float* cur_sub_from_mean_sq_img = sub_from_mean_sq_imgs + (img_idx * img_len);

    const float cur_mean = means[img_idx];

    for (uint pixel_idx = get_global_id(0); pixel_idx < img_len;
         pixel_idx += global_inc)
    {
      const float pix_minus_mean = cur_sub_from_mean_img[pixel_idx] - cur_mean;
      cur_sub_from_mean_img[pixel_idx] = pix_minus_mean;
      cur_sub_from_mean_sq_img[pixel_idx] = pix_minus_mean * pix_minus_mean;
    }
  }
}

__kernel void NCCElemsKernel(__global float* sub_from_mean_mov_imgs,
                             __global const float* mov_std_devs,
                             __global const float* fixed_minus_mean_over_std_dev,
                             const uint num_imgs,
                             const uint img_len,
                             const uint proj_off)
{
  const uint global_inc = get_global_size(0);
  const uint img_idx    = get_global_id(1);

  if (img_idx < num_imgs)
  {
    __global float* cur_sub_from_mean_mov_img = sub_from_mean_mov_imgs + ((proj_off + img_idx) * img_len);

    const float cur_mov_std_dev = max(1.0e-6f, sqrt(mov_std_devs[img_idx]));

    for (uint pixel_idx = get_global_id(0); pixel_idx < img_len;
         pixel_idx += global_inc)
    {
      cur_sub_from_mean_mov_img[pixel_idx] *= fixed_minus_mean_over_std_dev[pixel_idx] / cur_mov_std_dev;
    }
  }
}

);

}  // un-named

xreg::ImgSimMetric2DNCCOCL::ImgSimMetric2DNCCOCL(const boost::compute::device& gpu)
  : ImgSimMetric2DOCL(gpu)
{ }

xreg::ImgSimMetric2DNCCOCL::ImgSimMetric2DNCCOCL(const boost::compute::context& ctx,
                                                 const boost::compute::command_queue& queue)
  : ImgSimMetric2DOCL(ctx, queue)
{ }

void xreg::ImgSimMetric2DNCCOCL::allocate_resources()
{
  namespace bc = boost::compute;
  
  // the parent call to allocate resources can trigger a call to process_mask,
  // so we need to make sure we have these buffers allocated ahead of time
  
  // compile, and create custom kernels 
  std::stringstream ss;
  ss << DivideBufElemsOutOfPlaceKernelSrc
     << kNCC_OPENCL_SRC;
  
  bc::program prog = bc::program::create_with_source(ss.str(), this->ctx_);
  
  try
  {
    prog.build();
  }
  catch (bc::opencl_error &)
  {
    std::cerr << "OpenCL Kernel Compile Error (ImgSimMetric2DNCCOCL):\n"
              << prog.build_log() << std::endl;
    throw;
  }

  div_elems_krnl_   = prog.create_kernel("DivideBufElemsOutOfPlace");
  sub_mean_sq_krnl_ = prog.create_kernel("SubMeanAndSquareKernel");
  ncc_elem_krnl_    = prog.create_kernel("NCCElemsKernel");
  
  const size_type num_pix_per_img = this->num_pix_per_proj();
  
  // Populate and compute the appropriate fixed image buffers
  
  // we'll use this now to store temp calculations on the fixed image
  // and later use it on the moving images
  tmp_mov_imgs_dev_.reset(new DevBuf(this->ctx_));
  tmp_mov_imgs_dev_->resize(num_pix_per_img * this->num_mov_imgs_, this->queue_);
  
  fixed_img_mean_dev_.reset(new DevPixelScalarBuf(this->ctx_));
  fixed_img_stddev_dev_.reset(new DevPixelScalarBuf(this->ctx_));

  one_over_n_dev_.reset(new DevBuf(this->ctx_));

  one_over_n_minus_1_dev_.reset(new DevBuf(this->ctx_));
  
  // MOVING IMAGE Buffer creation 

  mov_img_means_dev_.reset(new DevBuf(ctx_));
  mov_img_means_dev_->resize(this->num_mov_imgs_, queue_);

  mov_img_std_devs_dev_.reset(new DevBuf(ctx_));
  mov_img_std_devs_dev_->resize(this->num_mov_imgs_, queue_);

  nccs_dev_.reset(new DevBuf(ctx_));
  nccs_dev_->resize(this->num_mov_imgs_, queue_);

  ImgSimMetric2DOCL::allocate_resources();
 
  // setup static kernel parameters
  ncc_elem_krnl_.set_arg(0, *this->mov_imgs_buf_);
  ncc_elem_krnl_.set_arg(1, *mov_img_std_devs_dev_);
  ncc_elem_krnl_.set_arg(2, *this->fixed_img_ocl_buf_);
  ncc_elem_krnl_.set_arg(4, bc::uint_(num_pix_per_img));

  // allocate similarity score buffer
  this->sim_vals_.assign(this->num_mov_imgs_, 0);
}

void xreg::ImgSimMetric2DNCCOCL::compute()
{
  namespace bc  = boost::compute;
  namespace vcl = viennacl;
  
  this->pre_compute();

  const size_type num_pix_per_img = this->num_pix_per_proj();
  
  // wrap the existing GPU buffers with vienna CL objects; this first matrix represents
  // the maximal allocation; we'll take a sub-block of this.
  vcl::matrix<float> imgs_mat(this->mov_imgs_buf_->get_buffer().get(),
                              this->sim_vals_.size(),  // maximum allocated number of moving images
                              num_pix_per_img); 
  
  // this is a sub-matrix of the entire buffer for the projections represented by
  // not the maximal allocation, but the current offset and number of moving images 
  vcl::matrix_range<vcl::matrix<float>> imgs_mat2(imgs_mat,
                                                  vcl::range(this->proj_off_,
                                                             this->proj_off_ + this->num_mov_imgs_),
                                                  vcl::range(0, num_pix_per_img)); 
  
  vcl::vector<float> avg_vec(one_over_n_dev_->get_buffer().get(), num_pix_per_img);
  vcl::vector<float> n_minus_one_vec(one_over_n_minus_1_dev_->get_buffer().get(), num_pix_per_img);
  
  vcl::vector<float> means_vec(mov_img_means_dev_->get_buffer().get(), this->num_mov_imgs_);
  
  vcl::matrix<float> tmp_imgs_mat(this->tmp_mov_imgs_dev_->get_buffer().get(),
                                  this->num_mov_imgs_, num_pix_per_img);

  vcl::vector<float> std_devs_vec(mov_img_std_devs_dev_->get_buffer().get(), this->num_mov_imgs_);
  
  vcl::vector<float> nccs_vec(nccs_dev_->get_buffer().get(), this->num_mov_imgs_);

  // compute means
  vcl::linalg::prod_impl(imgs_mat2, avg_vec, means_vec);

  // compute std devs
  // first subtract mean from each element and square
  sub_mean_sq_krnl_.set_arg(3, bc::uint_(this->num_mov_imgs_));
  sub_mean_sq_krnl_.set_arg(5, bc::uint_(this->proj_off_));

  std::size_t global_size[2] = { num_pix_per_img / kWORK_GROUP_DIV_FACTOR, this->num_mov_imgs_ };
  this->queue_.enqueue_nd_range_kernel(sub_mean_sq_krnl_, 2, 0, global_size, 0).wait();
  
  // now compute std dev.
  vcl::linalg::prod_impl(tmp_imgs_mat, n_minus_one_vec, std_devs_vec);
  // the sqrt is computed in the next kernel

  // compute ncc elems
  ncc_elem_krnl_.set_arg(3, bc::uint_(this->num_mov_imgs_));
  ncc_elem_krnl_.set_arg(5, bc::uint_(this->proj_off_));
  this->queue_.enqueue_nd_range_kernel(ncc_elem_krnl_, 2, 0, global_size, 0).wait();

  // reduce into NCC scores
  vcl::linalg::prod_impl(imgs_mat2, avg_vec, nccs_vec);

  bc::copy(nccs_dev_->begin(), nccs_dev_->begin() + this->num_mov_imgs_,
           this->sim_vals_.begin(), this->queue_);
  
  // TODO: parallel transform?
  std::transform(this->sim_vals_.begin(), this->sim_vals_.begin() + this->num_mov_imgs_,
                 this->sim_vals_.begin(),
                 [] (const Scalar& x) { return 0.5 * (1 - x); });
}

void xreg::ImgSimMetric2DNCCOCL::process_mask()
{
  namespace bc  = boost::compute;
  namespace vcl = viennacl;
  
  ImgSimMetric2DOCL::process_mask();
  
  const size_type num_pix_per_img = this->num_pix_per_proj();

  if (this->mask_)
  {
    one_over_n_dev_->resize(num_pix_per_img, this->queue_);
    one_over_n_minus_1_dev_->resize(num_pix_per_img, this->queue_);
    
    const Scalar num_pix_float = static_cast<Scalar>(this->num_pix_per_proj_after_mask_);
    
    std::size_t global_size = this->mask_ocl_buf_->size();
    
    div_elems_krnl_.set_arg(0, *this->mask_ocl_buf_);
    div_elems_krnl_.set_arg(3, bc::ulong_(this->mask_ocl_buf_->size()));

    div_elems_krnl_.set_arg(1, *one_over_n_dev_);
    div_elems_krnl_.set_arg(2, num_pix_float);
    this->queue_.enqueue_nd_range_kernel(div_elems_krnl_, 1, nullptr, &global_size, nullptr).wait();
    
    div_elems_krnl_.set_arg(1, *one_over_n_minus_1_dev_);
    div_elems_krnl_.set_arg(2, num_pix_float - Scalar(1));
    this->queue_.enqueue_nd_range_kernel(div_elems_krnl_, 1, nullptr, &global_size, nullptr).wait();
  }
  else
  {
    one_over_n_dev_->assign(num_pix_per_img, 1.0f / num_pix_per_img, this->queue_); 
  
    one_over_n_minus_1_dev_->assign(num_pix_per_img, 1.0f / (num_pix_per_img - 1.0f),
                                    this->queue_);
  }
  
  // wrap the existing GPU buffers with vienna CL objects  
  vcl::matrix<float> fixed_img_mat(this->fixed_img_ocl_buf_->get_buffer().get(),
                                   1, num_pix_per_img); 
  
  vcl::matrix<float> tmp_fixed_img_mat(tmp_mov_imgs_dev_->get_buffer().get(),
                                       1, num_pix_per_img); 
  
  vcl::vector<float> fixed_img_mean_vec(fixed_img_mean_dev_->get_buffer().get(), 1);
  
  vcl::vector<float> fixed_img_stddev_vec(fixed_img_stddev_dev_->get_buffer().get(), 1);
  
  vcl::vector<float> avg_vec(one_over_n_dev_->get_buffer().get(), num_pix_per_img);
  vcl::vector<float> n_minus_one_vec(one_over_n_minus_1_dev_->get_buffer().get(), num_pix_per_img);

  // compute mean of fixed image
  vcl::linalg::prod_impl(fixed_img_mat, avg_vec, fixed_img_mean_vec);

  // setup and exec kernel to subtract mean from fixed image, and compute squares
  sub_mean_sq_krnl_.set_arg(0, *this->fixed_img_ocl_buf_);
  sub_mean_sq_krnl_.set_arg(1, *tmp_mov_imgs_dev_);
  sub_mean_sq_krnl_.set_arg(2, fixed_img_mean_dev_->get_buffer());
  sub_mean_sq_krnl_.set_arg(3, bc::uint_(1));  // num_mov_imgs_
  sub_mean_sq_krnl_.set_arg(4, bc::uint_(num_pix_per_img));
  sub_mean_sq_krnl_.set_arg(5, bc::uint_(0));  // proj_off_

  std::size_t global_size[2] = { num_pix_per_img / kWORK_GROUP_DIV_FACTOR, 1 };
  this->queue_.enqueue_nd_range_kernel(sub_mean_sq_krnl_, 2, 0, global_size, 0).wait();
    
  // compute std dev
  vcl::linalg::prod_impl(tmp_fixed_img_mat, n_minus_one_vec, fixed_img_stddev_vec);
  const float fixed_std_dev_host = std::max(1.0e-6f, std::sqrt(fixed_img_stddev_dev_->read(this->queue_)));

  // divide by std dev
  fixed_img_mat /= fixed_std_dev_host;
  
  // Set parameters for the kernel that are specific for the moving images

  sub_mean_sq_krnl_.set_arg(0, *this->mov_imgs_buf_);
  sub_mean_sq_krnl_.set_arg(1, *tmp_mov_imgs_dev_);
  sub_mean_sq_krnl_.set_arg(2, *mov_img_means_dev_);
}
