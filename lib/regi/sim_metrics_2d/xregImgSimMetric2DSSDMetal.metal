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

// Sum of squared differences (SSD) similarity metric kernels.

#ifndef XREG_IMG_SIM_METRIC_2D_SSD_METAL_METAL_
#define XREG_IMG_SIM_METRIC_2D_SSD_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregImgSimMetric2DMetal.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief The squared difference between a moving and fixed image pixel,
///        multiplied by its mask value.
struct MaskedSqDiffImgs
{
  const device float* fixed_img;
  const device float* mov_img;
  const device float* mask;

  float operator()(const uint i) const
  {
    const float d = mov_img[i] - fixed_img[i];
    return mask[i] * d * d;
  }
};

/// \brief Computes the mean of the squared differences between each moving
///        image and the fixed image, over the pixels that are not masked out.
///
/// Each threadgroup computes the similarity of one moving image.
[[host_name("xregSSDKernel")]]
kernel void SSDKernel(constant ImgSimMetricMetalImgArgs& args [[buffer(kSIM_METAL_IMG_ARGS)]],
                      const device float* fixed_img           [[buffer(kSIM_METAL_FIXED_IMG)]],
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

    const float ssd = tg.sum(args.num_pix, MaskedSqDiffImgs{ fixed_img, mov_img, mask }) /
                        args.num_unmasked_pix;

    if (tg.thread_idx == 0)
    {
      sims[tg.row_idx] = ssd;
    }
  }
}

}  // xreg

#endif
