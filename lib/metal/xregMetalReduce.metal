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

// Reductions (e.g. sums) across the threads of a threadgroup.

#ifndef XREG_METAL_REDUCE_METAL_
#define XREG_METAL_REDUCE_METAL_

#include <metal_stdlib>

namespace xreg
{

/// \brief Storage required by ThreadgroupReducer, which should be declared in
///        a kernel, e.g.:
///          threadgroup ThreadgroupReducerScratch scratch;
struct ThreadgroupReducerScratch
{
  /// \brief The maximum number of SIMD groups in a threadgroup.
  ///
  /// The SIMD group width varies by device (e.g. 8 to 32 on Intel GPUs and 32
  /// or 64 on AMD GPUs), so the host should limit threadgroup sizes to
  /// kMAX_NUM_SIMD_GROUPS * (SIMD group width) threads.
  static constexpr constant uint kMAX_NUM_SIMD_GROUPS = 32;

  /// \brief One partial result for each SIMD group, large enough for a float4.
  float4 partials[kMAX_NUM_SIMD_GROUPS];
};

/// \brief Computes sums over all threads of a threadgroup.
///
/// Every thread of the threadgroup must call sum() the same number of times,
/// in the same order, since it synchronizes the threadgroup.
class ThreadgroupReducer
{
public:
  ThreadgroupReducer(threadgroup ThreadgroupReducerScratch& scratch,
                     const uint simd_lane, const uint simd_group, const uint num_simd_groups)
    : scratch_(scratch), simd_lane_(simd_lane), simd_group_(simd_group),
      num_simd_groups_(num_simd_groups)
  { }

  /// \brief Sum a value over all threads of the threadgroup, every thread
  ///        receives the total.
  ///
  /// T may be a float or a float vector.
  template <class T>
  T sum(const T val) const
  {
    static_assert(sizeof(T) <= sizeof(float4), "unsupported type for a threadgroup sum");

    threadgroup T* partials = reinterpret_cast<threadgroup T*>(scratch_.partials);

    const T simd_total = metal::simd_sum(val);

    if (simd_lane_ == 0)
    {
      partials[simd_group_] = simd_total;
    }

    metal::threadgroup_barrier(metal::mem_flags::mem_threadgroup);

    T total = 0;

    for (uint i = 0; i < num_simd_groups_; ++i)
    {
      total += partials[i];
    }

    // the partials may be overwritten by a subsequent sum after every thread
    // has read them
    metal::threadgroup_barrier(metal::mem_flags::mem_threadgroup);

    return total;
  }

private:
  threadgroup ThreadgroupReducerScratch& scratch_;

  const uint simd_lane_;
  const uint simd_group_;
  const uint num_simd_groups_;
};

}  // xreg

#endif
