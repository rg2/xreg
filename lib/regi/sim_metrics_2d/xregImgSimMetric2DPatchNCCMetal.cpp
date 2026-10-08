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

#include "xregImgSimMetric2DPatchNCCMetal.h"

#include "xregAssert.h"
#include "xregITKOpenCVUtils.h"

namespace xreg
{

// embedded at build time, see lib/regi/CMakeLists.txt
extern const char* const kIMG_SIM_METRIC_2D_PATCH_NCC_METAL_SRC;

}  // xreg

xreg::ImgSimMetric2DPatchNCCMetal::ImgSimMetric2DPatchNCCMetal(const MetalDevice& dev)
  : ImgSimMetric2DMetal(dev)
{ }

xreg::ImgSimMetric2DPatchNCCMetal::ImgSimMetric2DPatchNCCMetal(const MetalCmdQueue& queue)
  : ImgSimMetric2DMetal(queue)
{ }

void xreg::ImgSimMetric2DPatchNCCMetal::allocate_resources()
{
  xregASSERT(this->patch_radius_ > 0);

  // unsupported options when running on the GPU:
  xregASSERT(!this->use_mask_for_patch_stats_);

  const auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  // create the initial, full, set of patches

  // the mask is only used here to compute initial weights on the CPU, so we do
  // not require any other previous processing done to it
  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  this->setup_patches(itk_size[1], itk_size[0], this->mask_ ? &ocv_mask : nullptr,
                      this->num_mov_imgs_);

  const size_type num_patches = this->patch_infos_.size();

  // copy the patch bounds to the device
  {
    std::vector<sim_metric_metal_types::UInt4> patch_bounds_host;
    patch_bounds_host.reserve(num_patches);

    proc_fixed_patch_stride_ = 0;

    using UInt = sim_metric_metal_types::UInt;

    for (const auto& p : this->patch_infos_)
    {
      patch_bounds_host.push_back(simd_make_uint4(static_cast<UInt>(p.start_row),
                                                  static_cast<UInt>(p.start_col),
                                                  static_cast<UInt>(p.stop_row),
                                                  static_cast<UInt>(p.stop_col)));

      proc_fixed_patch_stride_ = std::max(proc_fixed_patch_stride_,
                                          (p.stop_row - p.start_row + 1) * (p.stop_col - p.start_col + 1));
    }

    patch_bounds_dev_ = UInt4Buf(this->device(), num_patches);

    CopyHostToMetal(patch_bounds_host.data(), patch_bounds_host.data() + num_patches,
                    patch_bounds_dev_, 0, this->queue_);
  }

  const MetalLibrary lib = this->make_library(kIMG_SIM_METRIC_2D_PATCH_NCC_METAL_SRC);

  fixed_patches_pipeline_ = MetalComputePipeline(lib, "xregPatchNCCFixedKernel");
  patch_ncc_pipeline_     = MetalComputePipeline(lib, "xregPatchNCCKernel");
  combine_pipeline_       = MetalComputePipeline(lib, "xregWeightedRowSumKernel");

  fixed_img_patches_proc_done_ = false;

  // this will result in the process_mask method being called, which may trigger
  // some weight recomputation, and also fixed image pre-processing, which is
  // why we have previously setup patches
  ImgSimMetric2DMetal::allocate_resources();
}

void xreg::ImgSimMetric2DPatchNCCMetal::compute()
{
  this->pre_compute();

  // this is the current number patches to be used, e.g. if a random subset of
  // 10 patches should be used this number is 10
  const size_type num_patches = this->num_patches();

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

  xregASSERT(this->patch_inds_to_use_.size() == num_patches);

  // copy the patch indices and weights to use onto the GPU

  patch_inds_to_use_host_.assign(this->patch_inds_to_use_.begin(), this->patch_inds_to_use_.end());

  CopyHostToMetal(patch_inds_to_use_host_.data(), patch_inds_to_use_host_.data() + num_patches,
                  patch_inds_to_use_dev_, 0, this->queue_);

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

  CopyHostToMetal(wgts_to_use_host_.data(), wgts_to_use_host_.data() + num_patches,
                  wgts_to_use_dev_, 0, this->queue_);

  MetalComputeEncoder enc(this->queue_);

  // (1 - NCC) of each patch used in each moving image
  enc.set_pipeline(patch_ncc_pipeline_);
  enc.set_value(this->img_args(this->num_mov_imgs_), kSIM_METAL_PATCH_IMG_ARGS);
  enc.set_value(patch_args(num_patches), kSIM_METAL_PATCH_ARGS);
  enc.set_buffer(patch_bounds_dev_, kSIM_METAL_PATCH_BOUNDS);
  enc.set_buffer(patch_inds_to_use_dev_, kSIM_METAL_PATCH_IDX_LUT);
  enc.set_buffer(proc_fixed_patches_dev_, kSIM_METAL_PATCH_PROC_FIXED);
  enc.set_buffer(*this->mov_imgs_buf_, kSIM_METAL_PATCH_MOV_IMGS, this->mov_imgs_buf_offset());
  enc.set_buffer(patch_sims_dev_, kSIM_METAL_PATCH_SIMS);
  enc.dispatch_threads({ num_patches, this->num_mov_imgs_, 1 },
                       { patch_ncc_pipeline_.thread_execution_width(), 1, 1 });

  // combine the patch similarities of each moving image, e.g. a weighted sum
  ImgSimMetricMetalRowSumArgs combine_args;
  combine_args.num_rows = static_cast<sim_metric_metal_types::UInt>(this->num_mov_imgs_);
  combine_args.num_cols = static_cast<sim_metric_metal_types::UInt>(num_patches);

  enc.set_value(combine_args, kSIM_METAL_ROW_SUM_ARGS);
  enc.set_buffer(patch_sims_dev_, kSIM_METAL_ROW_SUM_MAT);
  enc.set_buffer(wgts_to_use_dev_, kSIM_METAL_ROW_SUM_WGTS);
  enc.set_buffer(sim_vals_dev_, kSIM_METAL_ROW_SUM_OUT);

  this->dispatch_reduction(enc, combine_pipeline_, this->num_mov_imgs_);

  enc.commit_and_wait();

  this->copy_sim_vals_to_host(sim_vals_dev_);
}

void xreg::ImgSimMetric2DPatchNCCMetal::process_mask()
{
  // NOTE: the parent is not called, because the mask is only used to compute
  //       the patch weights on the host

  const size_type num_patches = this->patch_infos_.size();

  if (!fixed_img_patches_proc_done_)
  {
    const MetalDevice& dev = this->device();

    proc_fixed_patches_dev_ = DevBuf(dev, num_patches * proc_fixed_patch_stride_);

    MetalComputeEncoder enc(this->queue_);

    enc.set_pipeline(fixed_patches_pipeline_);
    enc.set_value(this->img_args(1), kSIM_METAL_PATCH_IMG_ARGS);
    enc.set_value(patch_args(num_patches), kSIM_METAL_PATCH_ARGS);
    enc.set_buffer(patch_bounds_dev_, kSIM_METAL_PATCH_BOUNDS);
    enc.set_buffer(*this->fixed_img_metal_buf_, kSIM_METAL_PATCH_FIXED_IMG);
    enc.set_buffer(proc_fixed_patches_dev_, kSIM_METAL_PATCH_PROC_FIXED);
    enc.dispatch_threads({ num_patches, 1, 1 },
                         { fixed_patches_pipeline_.thread_execution_width(), 1, 1 });

    enc.commit_and_wait();

    // allocate maximum capacity buffers for storing which patches to use and
    // the patch weights

    patch_inds_to_use_host_.reserve(num_patches);
    patch_inds_to_use_dev_ = UIntBuf(dev, num_patches);

    wgts_to_use_host_.reserve(num_patches);
    wgts_to_use_dev_ = DevBuf(dev, num_patches);

    patch_sims_dev_ = DevBuf(dev, num_patches * this->num_mov_imgs_);

    sim_vals_dev_ = DevBuf(dev, this->num_mov_imgs_);

    fixed_img_patches_proc_done_ = true;
  }

  cv::Mat ocv_mask;
  if (this->mask_)
  {
    ocv_mask = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
  }

  // the mask has changed, we need to make sure the weights are recomputed
  this->need_to_recompute_weights_ = true;
  this->compute_weights(this->mask_ ? &ocv_mask : nullptr);
}

xreg::ImgSimMetricMetalPatchArgs
xreg::ImgSimMetric2DPatchNCCMetal::patch_args(const size_type num_patches) const
{
  ImgSimMetricMetalPatchArgs args;

  args.num_patches             = static_cast<sim_metric_metal_types::UInt>(num_patches);
  args.proc_fixed_patch_stride = static_cast<sim_metric_metal_types::UInt>(proc_fixed_patch_stride_);

  return args;
}
