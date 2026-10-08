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

#ifndef XREGIMGSIMMETRIC2DPATCHNCCMETAL_H_
#define XREGIMGSIMMETRIC2DPATCHNCCMETAL_H_

#include "xregImgSimMetric2DMetal.h"
#include "xregImgSimMetric2DPatchCommon.h"

namespace xreg
{

/// \brief Patch-based normalized cross correlation similarity metric using Metal.
///
/// The counterpart of ImgSimMetric2DPatchNCCOCL. The similarity value is a
/// combination (e.g. weighted mean) of (1 - NCC) computed over each patch.
/// The mask is only used to compute the patch weights, therefore
/// use_mask_for_patch_stats() is not supported.
class ImgSimMetric2DPatchNCCMetal
  : public ImgSimMetric2DMetal,
    public ImgSimMetric2DPatchCommon
{
public:
  // Need to redefine these as both parent classes have this alias
  // (they should be the same, but the ambiguity must be resolved)
  using Scalar     = ImgSimMetric2DMetal::Scalar;
  using MaskScalar = ImgSimMetric2DMetal::MaskScalar;

  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DPatchNCCMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DPatchNCCMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DPatchNCCMetal(const MetalCmdQueue& queue);

  void allocate_resources() override;

  void compute() override;

protected:
  /// \brief Normalizes the fixed image patches (on the first call) and
  ///        recomputes the patch weights.
  void process_mask() override;

private:
  using UInt4Buf = MetalVector<sim_metric_metal_types::UInt4>;
  using UIntBuf  = MetalVector<sim_metric_metal_types::UInt>;

  /// \brief The arguments of the patch kernels, processing a number of patches.
  ImgSimMetricMetalPatchArgs patch_args(const size_type num_patches) const;

  /// \brief The maximum number of pixels in a patch, which is the number of
  ///        elements between consecutive normalized fixed image patches.
  size_type proc_fixed_patch_stride_ = 0;

  /// \brief The bounds of every patch.
  UInt4Buf patch_bounds_dev_;

  DevBuf proc_fixed_patches_dev_;

  std::vector<sim_metric_metal_types::UInt> patch_inds_to_use_host_;
  UIntBuf patch_inds_to_use_dev_;

  std::vector<Scalar> wgts_to_use_host_;
  DevBuf wgts_to_use_dev_;

  /// \brief The similarity of each patch used for each moving image.
  DevBuf patch_sims_dev_;

  DevBuf sim_vals_dev_;

  MetalComputePipeline fixed_patches_pipeline_;
  MetalComputePipeline patch_ncc_pipeline_;
  MetalComputePipeline combine_pipeline_;

  bool fixed_img_patches_proc_done_ = false;
};

}  // xreg

#endif
