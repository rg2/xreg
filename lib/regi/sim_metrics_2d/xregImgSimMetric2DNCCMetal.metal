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

// Normalized cross correlation (NCC) similarity metric kernels.

#ifndef XREG_IMG_SIM_METRIC_2D_NCC_METAL_METAL_
#define XREG_IMG_SIM_METRIC_2D_NCC_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregImgSimMetric2DMetal.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief Normalizes the fixed image to have zero mean and unit standard
///        deviation, using the statistics of the pixels that are not masked out.
///
/// The normalized image is written to a separate buffer, the fixed image is
/// not modified. This is executed by a single threadgroup.
[[host_name("xregNCCFixedKernel")]]
kernel void NCCFixedKernel(constant ImgSimMetricMetalImgArgs& args [[buffer(kSIM_METAL_IMG_ARGS)]],
                           const device float* fixed_img           [[buffer(kSIM_METAL_FIXED_IMG)]],
                           const device float* mask                [[buffer(kSIM_METAL_MASK)]],
                           device float* norm_fixed_img            [[buffer(kSIM_METAL_OUT)]],
                           const uint tg_idx                       [[threadgroup_position_in_grid]],
                           const uint thread_idx                   [[thread_position_in_threadgroup]],
                           const uint num_threads                  [[threads_per_threadgroup]],
                           const uint simd_lane                    [[thread_index_in_simdgroup]],
                           const uint simd_group                   [[simdgroup_index_in_threadgroup]],
                           const uint num_simd_groups              [[simdgroups_per_threadgroup]])
{
  threadgroup ThreadgroupReducerScratch scratch;

  const RowReductionThreadgroup tg(scratch, tg_idx, thread_idx, num_threads,
                                   simd_lane, simd_group, num_simd_groups);

  if (tg.row_idx == 0)
  {
    const MeanStdDev s = ComputeMaskedMeanStdDev(tg, args, fixed_img, mask);

    for (uint i = tg.thread_idx; i < args.num_pix; i += num_threads)
    {
      norm_fixed_img[i] = (fixed_img[i] - s.mean) / s.std_dev;
    }
  }
}

/// \brief The terms of the moving image variance and its correlation with the
///        normalized fixed image, for one pixel.
struct NCCTerms
{
  const device float* mov_img;
  const device float* norm_fixed_img;
  const device float* mask;
  float mov_mean;

  /// \brief (squared difference from the mean, correlation with the fixed image)
  float2 operator()(const uint i) const
  {
    const float d        = mov_img[i] - mov_mean;
    const float masked_d = mask[i] * d;

    return float2(masked_d * d, masked_d * norm_fixed_img[i]);
  }
};

/// \brief Computes the NCC between each moving image and the fixed image, over
///        the pixels that are not masked out.
///
/// The similarity value is 0.5 * (1 - NCC), which is in [0,1] and is minimized
/// by a perfect correlation. Each threadgroup computes the similarity of one
/// moving image.
[[host_name("xregNCCKernel")]]
kernel void NCCKernel(constant ImgSimMetricMetalImgArgs& args [[buffer(kSIM_METAL_IMG_ARGS)]],
                      const device float* norm_fixed_img      [[buffer(kSIM_METAL_FIXED_IMG)]],
                      const device float* mask                [[buffer(kSIM_METAL_MASK)]],
                      const device float* mov_imgs            [[buffer(kSIM_METAL_MOV_IMGS)]],
                      device float* sims                      [[buffer(kSIM_METAL_OUT)]],
                      const uint img_idx                      [[threadgroup_position_in_grid]],
                      const uint thread_idx                   [[thread_position_in_threadgroup]],
                      const uint num_threads                  [[threads_per_threadgroup]],
                      const uint simd_lane                    [[thread_index_in_simdgroup]],
                      const uint simd_group                   [[simdgroup_index_in_threadgroup]],
                      const uint num_simd_groups              [[simdgroups_per_threadgroup]])
{
  threadgroup ThreadgroupReducerScratch scratch;

  const RowReductionThreadgroup tg(scratch, img_idx, thread_idx, num_threads,
                                   simd_lane, simd_group, num_simd_groups);

  if (tg.row_idx < args.num_imgs)
  {
    const device float* mov_img = mov_imgs + (tg.row_idx * args.num_pix);

    const float n = args.num_unmasked_pix;

    const float mov_mean = tg.sum(args.num_pix, MaskedPixel{ mov_img, mask }) / n;

    // the variance and correlation are accumulated together, so the moving
    // image is only read twice
    const float2 sums = tg.sum(args.num_pix, NCCTerms{ mov_img, norm_fixed_img, mask, mov_mean });

    const float mov_std_dev = metal::max(1.0e-6f, metal::sqrt(sums.x / (n - 1)));

    const float ncc = sums.y / (n * mov_std_dev);

    if (tg.thread_idx == 0)
    {
      sims[tg.row_idx] = 0.5f * (1 - ncc);
    }
  }
}

}  // xreg

#endif
