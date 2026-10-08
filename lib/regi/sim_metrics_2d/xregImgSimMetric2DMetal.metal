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

// Routines common to the Metal similarity metric kernels.

#ifndef XREG_IMG_SIM_METRIC_2D_METAL_METAL_
#define XREG_IMG_SIM_METRIC_2D_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregImgSimMetric2DMetalArgs.h"
#include "xregMetalReduce.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief The threadgroup of a kernel that computes reductions (e.g. sums)
///        over the elements of a row (e.g. the pixels of an image).
///
/// These kernels are dispatched with one 1D threadgroup per row (see
/// ImgSimMetric2DMetal::dispatch_reduction()), so every thread of a threadgroup
/// processes the same row.
class RowReductionThreadgroup
{
public:
  RowReductionThreadgroup(threadgroup ThreadgroupReducerScratch& scratch,
                          const uint row_idx, const uint thread_idx, const uint num_threads,
                          const uint simd_lane, const uint simd_group, const uint num_simd_groups)
    : row_idx(row_idx), thread_idx(thread_idx),
      num_threads_(num_threads), reducer_(scratch, simd_lane, simd_group, num_simd_groups)
  { }

  /// \brief Sum of fn(i) for i in [0, num_elems), every thread receives the total.
  ///
  /// The threads of the threadgroup each sum a strided subset of the elements,
  /// which are then summed across the threadgroup. Every thread must call this.
  template <class tFn>
  auto sum(const uint num_elems, const tFn fn) const -> decltype(fn(0u))
  {
    decltype(fn(0u)) partial = 0;

    for (uint i = thread_idx; i < num_elems; i += num_threads_)
    {
      partial += fn(i);
    }

    return reducer_.sum(partial);
  }

  /// \brief The row (e.g. image) processed by this threadgroup.
  const uint row_idx;

  /// \brief The index of this thread within the threadgroup.
  const uint thread_idx;

private:
  const uint num_threads_;

  const ThreadgroupReducer reducer_;
};

/// \brief A pixel value multiplied by its mask value.
struct MaskedPixel
{
  const device float* img;
  const device float* mask;

  float operator()(const uint i) const
  {
    return mask[i] * img[i];
  }
};

/// \brief The squared difference of a pixel value from a value (e.g. the mean),
///        multiplied by its mask value.
struct MaskedSqDiff
{
  const device float* img;
  const device float* mask;
  float val;

  float operator()(const uint i) const
  {
    const float d = img[i] - val;
    return mask[i] * d * d;
  }
};

/// \brief Sample mean and standard deviation.
struct MeanStdDev
{
  float mean;
  float std_dev;
};

/// \brief Mean and standard deviation of the pixels in an image which are not
///        masked out.
///
/// The variance is normalized by (n - 1) and the standard deviation is at
/// least 1.0e-6, the same as the OpenCL NCC similarity metric. The image is
/// processed by every thread of the threadgroup.
inline MeanStdDev ComputeMaskedMeanStdDev(const RowReductionThreadgroup tg,
                                          constant ImgSimMetricMetalImgArgs& args,
                                          const device float* img,
                                          const device float* mask)
{
  const float n = args.num_unmasked_pix;

  MeanStdDev s;

  s.mean = tg.sum(args.num_pix, MaskedPixel{ img, mask }) / n;

  s.std_dev = metal::max(1.0e-6f, metal::sqrt(tg.sum(args.num_pix, MaskedSqDiff{ img, mask, s.mean }) /
                                                (n - 1)));

  return s;
}

/// \brief An element of a matrix row multiplied by the weight of its column.
struct WeightedRowElem
{
  const device float* row;
  const device float* wgts;

  float operator()(const uint i) const
  {
    return row[i] * wgts[i];
  }
};

/// \brief Weighted sum of each row of a matrix, e.g. a product of a matrix and
///        a vector.
///
/// Each threadgroup computes the sum of a row.
[[host_name("xregWeightedRowSumKernel")]]
kernel void WeightedRowSumKernel(constant ImgSimMetricMetalRowSumArgs& args [[buffer(kSIM_METAL_ROW_SUM_ARGS)]],
                                 const device float* mat                   [[buffer(kSIM_METAL_ROW_SUM_MAT)]],
                                 const device float* wgts                  [[buffer(kSIM_METAL_ROW_SUM_WGTS)]],
                                 device float* row_sums                    [[buffer(kSIM_METAL_ROW_SUM_OUT)]],
                                 const uint row_idx                        [[threadgroup_position_in_grid]],
                                 const uint thread_idx                     [[thread_position_in_threadgroup]],
                                 const uint num_threads                    [[threads_per_threadgroup]],
                                 const uint simd_lane                      [[thread_index_in_simdgroup]],
                                 const uint simd_group                     [[simdgroup_index_in_threadgroup]],
                                 const uint num_simd_groups                [[simdgroups_per_threadgroup]])
{
  threadgroup ThreadgroupReducerScratch scratch;

  const RowReductionThreadgroup tg(scratch, row_idx, thread_idx, num_threads,
                                   simd_lane, simd_group, num_simd_groups);

  // every thread of the threadgroup has the same row index, so either the
  // entire threadgroup participates in the reduction or none of it does
  if (tg.row_idx < args.num_rows)
  {
    const float s = tg.sum(args.num_cols, WeightedRowElem{ mat + (tg.row_idx * args.num_cols), wgts });

    if (tg.thread_idx == 0)
    {
      row_sums[tg.row_idx] = s;
    }
  }
}

}  // xreg

#endif
