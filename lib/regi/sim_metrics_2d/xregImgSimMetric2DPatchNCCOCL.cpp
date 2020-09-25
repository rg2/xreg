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

#include "xregImgSimMetric2DPatchNCCOCL.h"

// ITK pollutes the global namespace with a macro, and causes
// a compile failure with vienna cl
#ifdef vcl_size_t
#undef vcl_size_t
#endif
#ifdef vcl_ptrdiff_t
#undef vcl_ptrdiff_t
#endif

#include <boost/compute/utility/source.hpp>

#include <viennacl/matrix.hpp>
#include <viennacl/vector.hpp>
#include <viennacl/linalg/prod.hpp>

#include "xregAssert.h"
#include "xregITKOpenCVUtils.h"

namespace
{

const char* kPATCH_NCC_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

float2 xregComputePatchStats(__global const float* patch,
                             const ulong patch_stride,
                             const uint patch_num_cols,
                             const uint patch_num_rows)
{
  __global const float* patch_cur_row = patch;

  const uint num_pix_in_patch = patch_num_cols * patch_num_rows;

  float2 mean_and_std_dev = (float2) (0, 0);

  for (uint r = 0; r < patch_num_rows; ++r, patch_cur_row += patch_stride)
  {
    for (uint c = 0; c < patch_num_cols; ++c)
    {
      mean_and_std_dev.x += patch_cur_row[c];
    }
  }

  mean_and_std_dev.x /= num_pix_in_patch;

  patch_cur_row = patch;

  float tmp_diff = 0;

  for (uint r = 0; r < patch_num_rows; ++r, patch_cur_row += patch_stride)
  {
    for (uint c = 0; c < patch_num_cols; ++c)
    {
      tmp_diff = mean_and_std_dev.x - patch_cur_row[c];
      mean_and_std_dev.y += tmp_diff * tmp_diff;
    }
  }
  
  mean_and_std_dev.y = max(sqrt(mean_and_std_dev.y / (num_pix_in_patch - 1)), 1.0e-6f);

  return mean_and_std_dev;
}

__kernel void FixedImagePatchStats(__global const float* fixed_img,
                                   const ulong  img_num_cols,
                                   const ulong num_patches,
                                   __global float2* fixed_img_patch_means_and_std_devs,
                                   __global const uint4* patch_start_stop_infos)
{
  const uint patch_idx = get_global_id(0);

  if (patch_idx < num_patches)
  {
    const uint4 cur_patch_info = patch_start_stop_infos[patch_idx];

    const uint patch_start_row = cur_patch_info.x;
    const uint patch_start_col = cur_patch_info.y;
    const uint patch_stop_row  = cur_patch_info.z;
    const uint patch_stop_col  = cur_patch_info.w;

    fixed_img_patch_means_and_std_devs[patch_idx] = xregComputePatchStats(
                                                     fixed_img + (img_num_cols * patch_start_row) + patch_start_col,
                                                     img_num_cols,
                                                     patch_stop_row - patch_start_row + 1,
                                                     patch_stop_col - patch_start_col + 1);
  }
}

__kernel void ProcFixedImagePatches(__global const float* fixed_img,
                                    __global float* proc_fixed_img_patches, 
                                    const ulong  img_num_cols,
                                    const ulong  proc_patch_stride,
                                    const ulong num_patches,
                                    __global const float2* fixed_img_patch_means_and_std_devs,
                                    __global const uint4* patch_start_stop_infos)
{
  const uint patch_idx = get_global_id(0);

  if (patch_idx < num_patches)
  {
    const uint4 cur_patch_info = patch_start_stop_infos[patch_idx];

    const uint patch_start_row = cur_patch_info.x;
    const uint patch_start_col = cur_patch_info.y;
    const uint patch_stop_row  = cur_patch_info.z;
    const uint patch_stop_col  = cur_patch_info.w;
  
    const uint nr = patch_stop_row - patch_start_row + 1;
    const uint nc = patch_stop_col - patch_start_col + 1;

    __global const float* cur_fixed_row = fixed_img + (patch_start_row * img_num_cols) + patch_start_col;

    __global float* cur_proc_row = proc_fixed_img_patches + (patch_idx * proc_patch_stride);

    const float2 mean_and_std_dev = fixed_img_patch_means_and_std_devs[patch_idx];

    const float s = mean_and_std_dev.y * nr * nc;

    for (uint r = 0; r < nr; ++r, cur_proc_row += nc, cur_fixed_row += img_num_cols)
    {
      for (uint c = 0; c < nc; ++c)
      {
        cur_proc_row[c] = (cur_fixed_row[c] - mean_and_std_dev.x) / s;
      }
    }
  }
}

__kernel void ProcMovImagePatches(__global const  float* proc_fixed_img_patches,
                                  const ulong proc_fixed_patch_stride,
                                  const ulong num_patches,
                                  const uint num_imgs,
                                  const uint mov_proj_off,
                                  const ulong img_num_cols,
                                  const ulong img_num_rows,
                                  __global const uint4* patch_start_stop_infos,
                                  __global const float* mov_imgs,
                                  __global float* patch_nccs,
                                  __global const ulong* global_patch_idx_lut)
{
  const uint patch_idx_inc = get_global_size(0);
  const uint img_idx       = get_global_id(1);

  if (img_idx < num_imgs)
  {
    __global const float* cur_mov_img  = mov_imgs + ((mov_proj_off + img_idx) * img_num_rows * img_num_cols);
    __global       float* cur_img_nccs = patch_nccs + (img_idx * num_patches);

    for (uint local_patch_idx = get_global_id(0); local_patch_idx < num_patches; local_patch_idx += patch_idx_inc)
    {
      const ulong global_patch_idx = global_patch_idx_lut[local_patch_idx];

      __global const float* cur_proc_fixed_patch = proc_fixed_img_patches + (global_patch_idx * proc_fixed_patch_stride);
    
      const uint4 cur_patch_info = patch_start_stop_infos[global_patch_idx];
    
      const uint patch_start_row = cur_patch_info.x;
      const uint patch_start_col = cur_patch_info.y;
      const uint patch_stop_row  = cur_patch_info.z;
      const uint patch_stop_col  = cur_patch_info.w;

      const uint nr = patch_stop_row - patch_start_row + 1;
      const uint nc = patch_stop_col - patch_start_col + 1;
  
      __global const float* cur_mov_patch_row = cur_mov_img + (img_num_cols * patch_start_row) + patch_start_col;

      const float2 mov_patch_mean_std_dev = xregComputePatchStats(cur_mov_patch_row, img_num_cols, nr, nc);

      float ncc = 0;

      for (uint r = 0; r < nr; ++r, cur_mov_patch_row += img_num_cols, cur_proc_fixed_patch += nc)
      {
        for (uint c = 0; c < nc; ++c)
        {
          ncc += cur_proc_fixed_patch[c] * (cur_mov_patch_row[c] - mov_patch_mean_std_dev.x);
        }
      }

      ncc /= mov_patch_mean_std_dev.y;

      cur_img_nccs[local_patch_idx] = 1 - ncc;
    }
  }
}

);

}  // un-named

xreg::ImgSimMetric2DPatchNCCOCL::ImgSimMetric2DPatchNCCOCL(const boost::compute::device& dev)
  : ImgSimMetric2DOCL(dev)
{ }

xreg::ImgSimMetric2DPatchNCCOCL::ImgSimMetric2DPatchNCCOCL(const boost::compute::context& ctx,
                                                           const boost::compute::command_queue& queue)
  : ImgSimMetric2DOCL(ctx, queue)
{ }

void xreg::ImgSimMetric2DPatchNCCOCL::allocate_resources()
{
  namespace bc = boost::compute;
  
  xregASSERT(this->patch_radius_ > 0);

  // unsupported options when running on the GPU:
  xregASSERT(!this->use_mask_for_patch_stats_);
 
  auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  img_num_rows_ = itk_size[1];
  img_num_cols_ = itk_size[0];

  // create the initial, full, set of patches

  // the mask is only used here to compute initial weights on the CPU, so we do not require
  // any other previous processing done to it
  const bool use_mask = this->mask_;

	cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }
  this->setup_patches(img_num_rows_, img_num_cols_,
                      use_mask ? &ocv_mask : nullptr,
                      this->num_mov_imgs_);
  
  const size_type num_patches = this->patch_infos_.size();

  // compute the patch bounds on the gpu device
  {
    patch_start_stops_dev_.reset(new DevBufUInt4(this->ctx_));
  
    std::vector<bc::uint4_> patch_start_stops_host(num_patches);

    fixed_img_proc_patches_max_len_ = 0;

    for (size_type patch_idx = 0; patch_idx < num_patches; ++patch_idx)
    {
      const auto& cur_patch_info = this->patch_infos_[patch_idx];

      auto& cur_start_stop = patch_start_stops_host[patch_idx];
      
      cur_start_stop[0] = cur_patch_info.start_row;
      cur_start_stop[1] = cur_patch_info.start_col;
      cur_start_stop[2] = cur_patch_info.stop_row;
      cur_start_stop[3] = cur_patch_info.stop_col;
    
      fixed_img_proc_patches_max_len_ = std::max(fixed_img_proc_patches_max_len_,
                                             (cur_patch_info.stop_row - cur_patch_info.start_row + 1) *
                                             (cur_patch_info.stop_col - cur_patch_info.start_col + 1));
    }
    
    patch_start_stops_dev_->assign(patch_start_stops_host.begin(), patch_start_stops_host.end(), this->queue_);
  }

  // compile, and create custom kernels
  bc::program prog = bc::program::create_with_source(kPATCH_NCC_OPENCL_SRC, this->ctx_);
  
  try
  {
    prog.build();
  }
  catch (bc::opencl_error &)
  {
    std::cerr << "OpenCL Kernel Compile Error (ImgSimMetric2DPatchNCCOCL):\n"
              << prog.build_log() << std::endl;
    throw;
  }
  
  fixed_img_stats_krnl_        = prog.create_kernel("FixedImagePatchStats");
  fixed_img_proc_patches_krnl_ = prog.create_kernel("ProcFixedImagePatches");
  proc_mov_img_patches_krnl_   = prog.create_kernel("ProcMovImagePatches");

  fixed_img_stats_proc_done_ = false;
  
  // this will result in the process_mask method being called, which may trigger some weight recomputation,
  // and also fixed image statistics/pre-processing, which is why we have previously setup patches
  ImgSimMetric2DOCL::allocate_resources();
}

void xreg::ImgSimMetric2DPatchNCCOCL::compute()
{
  namespace bc  = boost::compute;
  namespace vcl = viennacl;
  
  this->pre_compute();

  // this is the current number patches to be used, e.g. if a random subset of 10 patches should be used
  // this number is 10
  const size_type num_patches = this->num_patches();

  const bool use_mask = this->mask_;

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // This uses some state to determine if the weights actually need to be recomputed
  this->compute_weights(this->mask_ ? &ocv_mask : nullptr);

  if (!this->do_not_update_patch_inds_to_use_)
  {
    // this is where random patches are sampled
    this->patch_inds_to_use_ = this->patch_indices_to_use();
  }
 
  // copy the patch indices and weights to use onto the GPU
  patch_inds_to_use_host_.clear();

  for (const auto& p : this->patch_inds_to_use_)
  {
    patch_inds_to_use_host_.push_back(p);
  }

  bc::copy(patch_inds_to_use_host_.begin(), patch_inds_to_use_host_.end(),
           patch_inds_to_use_dev_->begin(), this->queue_);
  
  wgts_to_use_host_.clear();
 
  Scalar tot_wgt = 0;
  if (this->weight_patch_sims_in_combine_)
  {
    for (const auto& p : this->patch_inds_to_use_)
    {
      const auto& w = this->patch_infos_[p].weight;

      tot_wgt += w;

      wgts_to_use_host_.push_back(w);
    }
  }
  else
  {
    wgts_to_use_host_.assign(num_patches, 1);
    tot_wgt = num_patches;
  }

  if (this->compute_mean_of_patch_sims_ || this->weight_patch_sims_in_combine_)
  {
    for (auto& w : wgts_to_use_host_)
    {
      w /= tot_wgt;
    }
  }

  bc::copy(wgts_to_use_host_.begin(), wgts_to_use_host_.end(),
           wgts_to_use_dev_->begin(), this->queue_);

  // TODO: it would be really nice to have to some state that avoids transferring the 
  //       patches used and/or weights when they are unchanged.

  proc_mov_img_patches_krnl_.set_arg(0, *proc_fixed_img_patches_dev_);
  proc_mov_img_patches_krnl_.set_arg(1, bc::ulong_(fixed_img_proc_patches_max_len_));
  proc_mov_img_patches_krnl_.set_arg(2, bc::ulong_(num_patches));
  proc_mov_img_patches_krnl_.set_arg(3, bc::uint_(this->num_mov_imgs_));
  proc_mov_img_patches_krnl_.set_arg(4, bc::uint_(this->proj_off_));
  proc_mov_img_patches_krnl_.set_arg(5, bc::ulong_(img_num_cols_));
  proc_mov_img_patches_krnl_.set_arg(6, bc::ulong_(img_num_rows_));
  proc_mov_img_patches_krnl_.set_arg(7, *patch_start_stops_dev_);
  proc_mov_img_patches_krnl_.set_arg(8, *this->mov_imgs_buf_);
  proc_mov_img_patches_krnl_.set_arg(9, *patch_nccs_dev_);
  proc_mov_img_patches_krnl_.set_arg(10, *patch_inds_to_use_dev_);

  std::array<std::size_t,2> global_size = { num_patches, this->num_mov_imgs_ };
  this->queue_.enqueue_nd_range_kernel(proc_mov_img_patches_krnl_, 2, nullptr, global_size.data(), nullptr).wait();

  // compute weighted sums, averages, whichever

  vcl::matrix<float> patch_nccs_mat(patch_nccs_dev_->get_buffer().get(),
                                    this->num_mov_imgs_, num_patches);

  vcl::vector<float> wgts_vec(wgts_to_use_dev_->get_buffer().get(), num_patches);

  vcl::vector<float> sims_vec(sim_vals_dev_->get_buffer().get(), this->num_mov_imgs_);

  vcl::linalg::prod_impl(patch_nccs_mat, wgts_vec, sims_vec);

  bc::copy(sim_vals_dev_->begin(), sim_vals_dev_->end(), this->sim_vals_.begin(), this->queue_);
}

void xreg::ImgSimMetric2DPatchNCCOCL::process_mask()
{
  namespace bc = boost::compute;
  
  // NOTE: this call is commented out, because we do not need the mask represented as a float image
  // and moved to the GPU for this sim metric
  //ImgSimMetric2DOCL::process_mask();
  
  const size_type num_patches = this->patch_infos_.size();
  
  if (!fixed_img_stats_proc_done_)
  {
    // mean and std. devs of fixed image patches
    fixed_img_patch_stats_dev_.reset(new DevBufFloat2(this->ctx_));
    fixed_img_patch_stats_dev_->resize(num_patches, this->queue_);

    fixed_img_stats_krnl_.set_arg(0, *this->fixed_img_ocl_buf_);
    fixed_img_stats_krnl_.set_arg(1, bc::ulong_(img_num_cols_));
    fixed_img_stats_krnl_.set_arg(2, bc::ulong_(num_patches));
    fixed_img_stats_krnl_.set_arg(3, *fixed_img_patch_stats_dev_);
    fixed_img_stats_krnl_.set_arg(4, *patch_start_stops_dev_);

    std::size_t global_size = num_patches;

    this->queue_.enqueue_nd_range_kernel(fixed_img_stats_krnl_, 1, nullptr, &global_size, nullptr).wait();

    // pre-process the fixed image patches using the mean and std. devs.
    proc_fixed_img_patches_dev_.reset(new DevBuf(this->ctx_));
    proc_fixed_img_patches_dev_->resize(num_patches * fixed_img_proc_patches_max_len_, this->queue_);


    fixed_img_proc_patches_krnl_.set_arg(0, *this->fixed_img_ocl_buf_);
    fixed_img_proc_patches_krnl_.set_arg(1, *proc_fixed_img_patches_dev_);
    fixed_img_proc_patches_krnl_.set_arg(2, bc::ulong_(img_num_cols_));
    fixed_img_proc_patches_krnl_.set_arg(3, bc::ulong_(fixed_img_proc_patches_max_len_));
    fixed_img_proc_patches_krnl_.set_arg(4, bc::ulong_(num_patches));
    fixed_img_proc_patches_krnl_.set_arg(5, *fixed_img_patch_stats_dev_);
    fixed_img_proc_patches_krnl_.set_arg(6, *patch_start_stops_dev_);

    this->queue_.enqueue_nd_range_kernel(fixed_img_proc_patches_krnl_, 1, nullptr, &global_size, nullptr).wait();

    // allocate maximum capacity buffers for storing which patches to use
    // and the patch weights

    patch_inds_to_use_host_.reserve(num_patches);
    patch_inds_to_use_dev_.reset(new DevBufULong(this->ctx_));
    patch_inds_to_use_dev_->resize(num_patches, this->queue_);

    wgts_to_use_host_.reserve(num_patches);
    wgts_to_use_dev_.reset(new DevBuf(this->ctx_));
    wgts_to_use_dev_->resize(num_patches, this->queue_);

    patch_nccs_dev_.reset(new DevBuf(this->ctx_));
    patch_nccs_dev_->resize(num_patches * this->num_mov_imgs_, this->queue_);
    
    sim_vals_dev_.reset(new DevBuf(this->ctx_));
    sim_vals_dev_->resize(this->num_mov_imgs_, this->queue_);

    fixed_img_stats_proc_done_ = true;
  }

  // TODO: check about using the fixed image patch std. devs as weights
 
  const bool use_mask = this->mask_;

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // the mask has changed, we need to make sure the weights are recomputed
  this->need_to_recompute_weights_ = true;
  this->compute_weights(use_mask ? &ocv_mask : nullptr);
}

