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

// Kernels computing smoothed and gradient (Sobel) images for the similarity
// metrics on gradient images.

#ifndef XREG_IMG_SIM_METRIC_2D_GRAD_IMG_METAL_METAL_
#define XREG_IMG_SIM_METRIC_2D_GRAD_IMG_METAL_METAL_

#ifndef XREG_METAL_RUNTIME_SRC
#include "xregImgSimMetric2DMetal.metal"
#endif

#include <metal_stdlib>

namespace xreg
{

/// \brief The pixel computed by a thread of a kernel that is executed on a
///        grid of (number of columns) x (number of rows) x (number of images)
///        threads, which may be larger than the images.
struct ImgPixel
{
  ImgPixel(constant ImgSimMetricMetalImgArgs& args, const uint3 thread_idx)
    : valid((thread_idx.x < args.num_cols) && (thread_idx.y < args.num_rows) &&
            (thread_idx.z < args.num_imgs)),
      row(thread_idx.y),
      col(thread_idx.x),
      img_idx(thread_idx.z),
      buf_idx((thread_idx.z * args.num_pix) + (thread_idx.y * args.num_cols) + thread_idx.x)
  { }

  /// \brief false when the thread is outside of the images and should not
  ///        compute anything.
  const bool valid;

  const uint row;
  const uint col;
  const uint img_idx;

  /// \brief Index of the pixel in the buffer of all images.
  const uint buf_idx;
};

/// \brief Read-only access to the pixels of an image, locations outside of the
///        image use the nearest edge pixel (e.g. the edge pixels are repeated).
class ClampedImgView
{
public:
  ClampedImgView(constant ImgSimMetricMetalImgArgs& args, const device float* imgs, const uint img_idx)
    : pixels_(imgs + (img_idx * args.num_pix)),
      num_cols_(args.num_cols),
      last_row_(int(args.num_rows) - 1),
      last_col_(int(args.num_cols) - 1)
  { }

  float operator()(const int row, const int col) const
  {
    return pixels_[(metal::clamp(row, 0, last_row_) * num_cols_) + metal::clamp(col, 0, last_col_)];
  }

private:
  const device float* pixels_;

  const uint num_cols_;

  const int last_row_;
  const int last_col_;
};

/// \brief Convolves images with a square (e.g. Gaussian) smoothing kernel.
[[host_name("xregSmoothKernel")]]
kernel void SmoothKernel(constant ImgSimMetricMetalImgArgs& args [[buffer(kSIM_METAL_GRAD_IMG_ARGS)]],
                         const device float* src_imgs            [[buffer(kSIM_METAL_GRAD_SRC_IMGS)]],
                         const device float* smooth_kernel       [[buffer(kSIM_METAL_GRAD_SMOOTH_KERNEL)]],
                         constant uint& kernel_width             [[buffer(kSIM_METAL_GRAD_SMOOTH_WIDTH)]],
                         device float* smooth_imgs               [[buffer(kSIM_METAL_GRAD_OUT_X)]],
                         const uint3 thread_idx                  [[thread_position_in_grid]])
{
  const ImgPixel pix(args, thread_idx);

  if (pix.valid)
  {
    const ClampedImgView src(args, src_imgs, pix.img_idx);

    const int half_width = int(kernel_width / 2);

    const int row = int(pix.row);
    const int col = int(pix.col);

    float sum = 0;

    uint kernel_idx = 0;

    for (int kr = -half_width; kr <= half_width; ++kr)
    {
      for (int kc = -half_width; kc <= half_width; ++kc, ++kernel_idx)
      {
        sum += src(row + kr, col + kc) * smooth_kernel[kernel_idx];
      }
    }

    smooth_imgs[pix.buf_idx] = sum;
  }
}

/// \brief Computes the horizontal and vertical derivatives of images with 3x3
///        Sobel kernels.
[[host_name("xregSobelKernel")]]
kernel void SobelKernel(constant ImgSimMetricMetalImgArgs& args [[buffer(kSIM_METAL_GRAD_IMG_ARGS)]],
                        const device float* src_imgs            [[buffer(kSIM_METAL_GRAD_SRC_IMGS)]],
                        device float* grad_x_imgs               [[buffer(kSIM_METAL_GRAD_OUT_X)]],
                        device float* grad_y_imgs               [[buffer(kSIM_METAL_GRAD_OUT_Y)]],
                        const uint3 thread_idx                  [[thread_position_in_grid]])
{
  const ImgPixel pix(args, thread_idx);

  if (pix.valid)
  {
    const ClampedImgView src(args, src_imgs, pix.img_idx);

    const int r = int(pix.row);
    const int c = int(pix.col);

    // the same order of operations as the OpenCL kernel
    grad_x_imgs[pix.buf_idx] = -src(r - 1, c - 1) + src(r - 1, c + 1) +
                               -(2.0f * src(r, c - 1)) + (2.0f * src(r, c + 1)) +
                               -src(r + 1, c - 1) + src(r + 1, c + 1);

    grad_y_imgs[pix.buf_idx] = -src(r - 1, c - 1) - (2.0f * src(r - 1, c)) - src(r - 1, c + 1)
                               + src(r + 1, c - 1) + (2.0f * src(r + 1, c)) + src(r + 1, c + 1);
  }
}

}  // xreg

#endif
