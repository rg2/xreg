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

#include "xregImgSimMetric2DSSDOCL.h"

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

const char* kSQUARE_DIST_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void SquareDistKernel(__global float* mov_imgs,
                               __global const float* fixed_img,
                               const uint num_imgs,
                               const uint img_len,
                               const uint proj_off)
{
  const uint global_inc = get_global_size(0);
  const uint img_idx    = get_global_id(1);

  if (img_idx < num_imgs)
  {
    __global float* cur_mov_img = mov_imgs + ((proj_off + img_idx) * img_len);

    for (uint pixel_idx = get_global_id(0); pixel_idx < img_len;
         pixel_idx += global_inc)
    {
      const float d = cur_mov_img[pixel_idx] - fixed_img[pixel_idx];
      cur_mov_img[pixel_idx] = d * d;
    }
  }
}

);

}  // un-named

xreg::ImgSimMetric2DSSDOCL::ImgSimMetric2DSSDOCL(const boost::compute::device& dev)
  : ImgSimMetric2DOCL(dev)
{ }

xreg::ImgSimMetric2DSSDOCL::ImgSimMetric2DSSDOCL(const boost::compute::context& ctx,
                                                 const boost::compute::command_queue& queue)
  : ImgSimMetric2DOCL(ctx, queue)
{ }

void xreg::ImgSimMetric2DSSDOCL::allocate_resources()
{
  namespace bc = boost::compute;
  
  // the parent call to allocate resources can trigger a call to process_mask,
  // so we need to make sure we have these buffers allocated ahead of time
  ssds_dev_.reset(new DevBuf(this->ctx_));
  ssds_dev_->resize(this->num_mov_imgs_, this->queue_);

  one_over_n_dev_.reset(new DevBuf(this->ctx_));
  
  std::stringstream ss;
  ss << DivideBufElemsOutOfPlaceKernelSrc
     << kSQUARE_DIST_OPENCL_SRC;

  bc::program prog = bc::program::create_with_source(ss.str(), this->ctx_);
  
  try
  {
    prog.build();
  }
  catch (bc::opencl_error &)
  {
    std::cerr << "OpenCL Kernel Compile Error (ImgSimMetric2DSSDOCL):\n"
              << prog.build_log() << std::endl;
    throw;
  }

  div_elems_krnl_ = prog.create_kernel("DivideBufElemsOutOfPlace");

  sq_dist_krnl_ = prog.create_kernel("SquareDistKernel");
  sq_dist_krnl_.set_arg(0, *this->mov_imgs_buf_);
  sq_dist_krnl_.set_arg(1, *this->fixed_img_ocl_buf_);  
  sq_dist_krnl_.set_arg(3, bc::uint_(one_over_n_dev_->size()));
  // arguments 2,4 are set in compute(), since they may differ from the values
  // used for the maximally allocated buffer.
  
  ImgSimMetric2DOCL::allocate_resources();
}

void xreg::ImgSimMetric2DSSDOCL::compute()
{
  namespace bc  = boost::compute;
  namespace vcl = viennacl;
  
  this->pre_compute();
  
  const size_type num_pix_per_img = this->num_pix_per_proj();

  sq_dist_krnl_.set_arg(2, bc::uint_(this->num_mov_imgs_));
  sq_dist_krnl_.set_arg(4, bc::uint_(this->proj_off_));

  // perform the squared distance calcuations
  std::size_t global_size[2] = { num_pix_per_img / 8, this->num_mov_imgs_ };
  this->queue_.enqueue_nd_range_kernel(sq_dist_krnl_, 2, 0, global_size, 0).wait();

  // wrap the existing device buffers with vienna CL objects; this first matrix represents
  // the maximal allocation; we'll take a sub-block of this.
  vcl::matrix<float> imgs_mat(this->mov_imgs_buf_->get_buffer().get(),
                              this->sim_vals_.size(),  // maximum number of moving images
                              num_pix_per_img);
 
  // this is a sub-matrix of the entire buffer for the projections represented by
  // not the maximal allocation, but the current offset and number of moving images 
  vcl::matrix_range<vcl::matrix<float>> imgs_mat2(imgs_mat,
                                                  vcl::range(this->proj_off_,
                                                             this->proj_off_ + this->num_mov_imgs_),
                                                  vcl::range(0, num_pix_per_img)); 
  
  vcl::vector<float> avg_vec(one_over_n_dev_->get_buffer().get(), num_pix_per_img);
  vcl::vector<float> ssds_vec(ssds_dev_->get_buffer().get(), this->num_mov_imgs_);

  // sum the squared distances
  vcl::linalg::prod_impl(imgs_mat2, avg_vec, ssds_vec);

  bc::copy(ssds_dev_->begin(), ssds_dev_->begin() + this->num_mov_imgs_,
           this->sim_vals_.begin(), this->queue_);
}

void xreg::ImgSimMetric2DSSDOCL::process_mask()
{
  namespace bc = boost::compute;
  
  ImgSimMetric2DOCL::process_mask();

  if (this->mask_)
  {
    one_over_n_dev_->resize(this->num_pix_per_proj(), this->queue_);

    div_elems_krnl_.set_arg(0, *this->mask_ocl_buf_);
    div_elems_krnl_.set_arg(1, *one_over_n_dev_);
    div_elems_krnl_.set_arg(2, static_cast<Scalar>(this->num_pix_per_proj_after_mask_));
    div_elems_krnl_.set_arg(3, bc::ulong_(this->mask_ocl_buf_->size()));

    std::size_t global_size = this->mask_ocl_buf_->size();
    this->queue_.enqueue_nd_range_kernel(div_elems_krnl_, 1, nullptr, &global_size, nullptr).wait();
  }
  else
  {
    one_over_n_dev_->assign(this->num_pix_per_proj(),
                            1.0f / this->num_pix_per_proj(), this->queue_);
  }
}

