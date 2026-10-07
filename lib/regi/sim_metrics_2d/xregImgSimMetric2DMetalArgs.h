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

#ifndef XREGIMGSIMMETRIC2DMETALARGS_H_
#define XREGIMGSIMMETRIC2DMETALARGS_H_

// Definitions shared by the host and the Metal similarity metric kernels. This
// file is valid C++ and Metal Shading Language, it is also embedded in the
// shader source compiled at runtime, so the host and kernels always agree on
// argument layouts and binding indices.

#ifdef __METAL_VERSION__
#include <metal_stdlib>
#else
#include <cstdint>
#include <simd/simd.h>
#endif

namespace xreg
{

/// \brief Types with identical layouts on the host and in Metal shaders.
namespace sim_metric_metal_types
{

#ifdef __METAL_VERSION__
using UInt  = uint;
using UInt4 = uint4;
#else
using UInt  = std::uint32_t;
using UInt4 = simd_uint4;
#endif

}  // sim_metric_metal_types

/// \brief Dimensions of the images processed by a similarity metric kernel.
///
/// Images are stored contiguously, each in row-major order.
struct ImgSimMetricMetalImgArgs
{
  sim_metric_metal_types::UInt num_cols;
  sim_metric_metal_types::UInt num_rows;

  /// \brief Number of pixels in each image, num_rows * num_cols.
  sim_metric_metal_types::UInt num_pix;

  /// \brief Number of images processed, e.g. the number of moving images.
  sim_metric_metal_types::UInt num_imgs;

  /// \brief Number of pixels in each image that are not masked out, this is
  ///        num_pix when there is no mask.
  float num_unmasked_pix;
};

static_assert(sizeof(ImgSimMetricMetalImgArgs) == 20, "unexpected ImgSimMetricMetalImgArgs layout");

/// \brief Buffer argument indices of the kernels computing statistics or
///        similarity values over entire images (e.g. SSD and NCC).
enum ImgSimMetricMetalBufIdx
{
  kSIM_METAL_IMG_ARGS = 0,  ///< constant ImgSimMetricMetalImgArgs&
  kSIM_METAL_FIXED_IMG,     ///< const device float*: fixed image, or a processed fixed image
  kSIM_METAL_MASK,          ///< const device float*: 1 -> use the pixel, 0 -> ignore the pixel
  kSIM_METAL_MOV_IMGS,      ///< const device float*: moving images
  kSIM_METAL_OUT            ///< device float*: similarity values, or a processed image
};

/// \brief Buffer argument indices of the kernels computing gradient images.
enum ImgSimMetricMetalGradBufIdx
{
  kSIM_METAL_GRAD_IMG_ARGS = 0,   ///< constant ImgSimMetricMetalImgArgs&
  kSIM_METAL_GRAD_SRC_IMGS,       ///< const device float*: images to smooth or differentiate
  kSIM_METAL_GRAD_SMOOTH_KERNEL,  ///< const device float*: row-major, square, smoothing kernel
  kSIM_METAL_GRAD_SMOOTH_WIDTH,   ///< constant uint&: smoothing kernel width (odd)
  kSIM_METAL_GRAD_OUT_X,          ///< device float*: horizontal derivatives, or smoothed images
  kSIM_METAL_GRAD_OUT_Y           ///< device float*: vertical derivatives
};

/// \brief Arguments of the patch NCC kernels.
struct ImgSimMetricMetalPatchArgs
{
  /// \brief Number of patches processed, e.g. the number of patches used for
  ///        each moving image.
  sim_metric_metal_types::UInt num_patches;

  /// \brief Number of elements between consecutive processed fixed image patches.
  sim_metric_metal_types::UInt proc_fixed_patch_stride;
};

static_assert(sizeof(ImgSimMetricMetalPatchArgs) == 8, "unexpected ImgSimMetricMetalPatchArgs layout");

/// \brief Buffer argument indices of the patch NCC kernels.
enum ImgSimMetricMetalPatchBufIdx
{
  kSIM_METAL_PATCH_IMG_ARGS = 0,  ///< constant ImgSimMetricMetalImgArgs&
  kSIM_METAL_PATCH_ARGS,          ///< constant ImgSimMetricMetalPatchArgs&
  kSIM_METAL_PATCH_BOUNDS,        ///< const device uint4*: (start row, start col, stop row, stop col) of every patch
  kSIM_METAL_PATCH_IDX_LUT,       ///< const device uint*: index into the patch bounds of each patch used
  kSIM_METAL_PATCH_FIXED_IMG,     ///< const device float*: fixed image
  kSIM_METAL_PATCH_PROC_FIXED,    ///< device float*: normalized fixed image patches
  kSIM_METAL_PATCH_MOV_IMGS,      ///< const device float*: moving images
  kSIM_METAL_PATCH_SIMS           ///< device float*: similarity of each patch in each moving image
};

/// \brief Arguments of the kernel computing weighted sums of each row of a matrix.
struct ImgSimMetricMetalRowSumArgs
{
  sim_metric_metal_types::UInt num_rows;
  sim_metric_metal_types::UInt num_cols;
};

static_assert(sizeof(ImgSimMetricMetalRowSumArgs) == 8, "unexpected ImgSimMetricMetalRowSumArgs layout");

/// \brief Buffer argument indices of the kernel computing weighted sums of each
///        row of a matrix.
enum ImgSimMetricMetalRowSumBufIdx
{
  kSIM_METAL_ROW_SUM_ARGS = 0,  ///< constant ImgSimMetricMetalRowSumArgs&
  kSIM_METAL_ROW_SUM_MAT,       ///< const device float*: row-major matrix
  kSIM_METAL_ROW_SUM_WGTS,      ///< const device float*: weight of each column
  kSIM_METAL_ROW_SUM_OUT        ///< device float*: weighted sum of each row
};

}  // xreg

#endif
