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

// Patch-based normalized cross correlation (Patch NCC) similarity metric kernels.

#ifndef XREG_IMG_SIM_METRIC_2D_PATCH_NCC_METAL_METAL_
#define XREG_IMG_SIM_METRIC_2D_PATCH_NCC_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregImgSimMetric2DMetal.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief Read-only access to a rectangular patch of an image.
class ImgPatch
{
public:
  /// \brief The bounds are (start row, start col, stop row, stop col), the
  ///        stop row and column are included in the patch.
  ImgPatch(const device float* img, const uint img_num_cols, const uint4 bounds)
    : first_(img + (bounds.x * img_num_cols) + bounds.y),
      img_num_cols_(img_num_cols),
      num_rows(bounds.z - bounds.x + 1),
      num_cols(bounds.w - bounds.y + 1)
  { }

  float operator()(const uint row, const uint col) const
  {
    return first_[(row * img_num_cols_) + col];
  }

  uint num_pix() const
  {
    return num_rows * num_cols;
  }

  /// \brief Mean and standard deviation of the pixels in the patch.
  ///
  /// The variance is normalized by (n - 1) and the standard deviation is at
  /// least 1.0e-6, the same as the OpenCL Patch NCC similarity metric.
  MeanStdDev mean_std_dev() const
  {
    MeanStdDev s = { 0, 0 };

    for (uint r = 0; r < num_rows; ++r)
    {
      for (uint c = 0; c < num_cols; ++c)
      {
        s.mean += (*this)(r,c);
      }
    }

    s.mean /= num_pix();

    for (uint r = 0; r < num_rows; ++r)
    {
      for (uint c = 0; c < num_cols; ++c)
      {
        const float d = s.mean - (*this)(r,c);
        s.std_dev += d * d;
      }
    }

    s.std_dev = metal::max(metal::sqrt(s.std_dev / (num_pix() - 1)), 1.0e-6f);

    return s;
  }

private:
  const device float* first_;

  const uint img_num_cols_;

public:
  const uint num_rows;
  const uint num_cols;
};

/// \brief Normalizes every patch of the fixed image, so that the NCC of a
///        moving image patch is a dot product with the normalized patch.
///
/// The normalized patch is (patch - mean) / (std. dev. * number of pixels),
/// stored in row-major order. Each thread processes one patch.
[[host_name("xregPatchNCCFixedKernel")]]
kernel void PatchNCCFixedKernel(constant ImgSimMetricMetalImgArgs& img_args     [[buffer(kSIM_METAL_PATCH_IMG_ARGS)]],
                                constant ImgSimMetricMetalPatchArgs& patch_args [[buffer(kSIM_METAL_PATCH_ARGS)]],
                                const device uint4* patch_bounds                [[buffer(kSIM_METAL_PATCH_BOUNDS)]],
                                const device float* fixed_img                   [[buffer(kSIM_METAL_PATCH_FIXED_IMG)]],
                                device float* proc_fixed_patches                [[buffer(kSIM_METAL_PATCH_PROC_FIXED)]],
                                const uint patch_idx                            [[thread_position_in_grid]])
{
  if (patch_idx < patch_args.num_patches)
  {
    const ImgPatch patch(fixed_img, img_args.num_cols, patch_bounds[patch_idx]);

    const MeanStdDev s = patch.mean_std_dev();

    const float scale = s.std_dev * patch.num_pix();

    device float* dst = proc_fixed_patches + (patch_idx * patch_args.proc_fixed_patch_stride);

    for (uint r = 0; r < patch.num_rows; ++r)
    {
      for (uint c = 0; c < patch.num_cols; ++c, ++dst)
      {
        *dst = (patch(r,c) - s.mean) / scale;
      }
    }
  }
}

/// \brief Computes (1 - NCC) for patches of each moving image.
///
/// The kernel is executed on a grid of (number of patches used) x (number of
/// moving images) threads, each thread processes one patch of one moving image.
[[host_name("xregPatchNCCKernel")]]
kernel void PatchNCCKernel(constant ImgSimMetricMetalImgArgs& img_args     [[buffer(kSIM_METAL_PATCH_IMG_ARGS)]],
                           constant ImgSimMetricMetalPatchArgs& patch_args [[buffer(kSIM_METAL_PATCH_ARGS)]],
                           const device uint4* patch_bounds                [[buffer(kSIM_METAL_PATCH_BOUNDS)]],
                           const device uint* patch_idx_lut                [[buffer(kSIM_METAL_PATCH_IDX_LUT)]],
                           const device float* proc_fixed_patches          [[buffer(kSIM_METAL_PATCH_PROC_FIXED)]],
                           const device float* mov_imgs                    [[buffer(kSIM_METAL_PATCH_MOV_IMGS)]],
                           device float* patch_sims                        [[buffer(kSIM_METAL_PATCH_SIMS)]],
                           const uint2 thread_idx                          [[thread_position_in_grid]])
{
  const uint local_patch_idx = thread_idx.x;
  const uint img_idx         = thread_idx.y;

  if ((local_patch_idx < patch_args.num_patches) && (img_idx < img_args.num_imgs))
  {
    const uint patch_idx = patch_idx_lut[local_patch_idx];

    const ImgPatch mov_patch(mov_imgs + (img_idx * img_args.num_pix), img_args.num_cols,
                             patch_bounds[patch_idx]);

    const MeanStdDev s = mov_patch.mean_std_dev();

    const device float* fixed_patch = proc_fixed_patches +
                                        (patch_idx * patch_args.proc_fixed_patch_stride);

    float ncc = 0;

    for (uint r = 0; r < mov_patch.num_rows; ++r)
    {
      for (uint c = 0; c < mov_patch.num_cols; ++c, ++fixed_patch)
      {
        ncc += *fixed_patch * (mov_patch(r,c) - s.mean);
      }
    }

    ncc /= s.std_dev;

    patch_sims[(img_idx * patch_args.num_patches) + local_patch_idx] = 1 - ncc;
  }
}

}  // xreg

#endif
