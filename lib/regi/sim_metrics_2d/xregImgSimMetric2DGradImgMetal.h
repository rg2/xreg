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

#ifndef XREGIMGSIMMETRIC2DGRADIMGMETAL_H_
#define XREGIMGSIMMETRIC2DGRADIMGMETAL_H_

#include "xregImgSimMetric2DMetal.h"
#include "xregImgSimMetric2DGradImgParamInterface.h"

namespace xreg
{

/// \brief Base class for Metal similarity metrics using 2D gradient images.
///
/// The counterpart of ImgSimMetric2DGradImgOCL. This computes the horizontal
/// and vertical Sobel derivatives of the fixed and moving images, optionally
/// after smoothing with a Gaussian kernel. Pixels outside of an image use the
/// nearest edge pixel. It is up to the derived class to compute the appropriate
/// similarity score.
class ImgSimMetric2DGradImgMetal
  : public ImgSimMetric2DMetal,
    public ImgSimMetric2DGradImgParamInterface
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DGradImgMetal() = default;

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DGradImgMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DGradImgMetal(const MetalCmdQueue& queue);

  /// \brief Allocates the gradient image buffers and computes the fixed image
  ///        gradients.
  void allocate_resources() override;

  size_type smooth_img_before_sobel_kernel_radius() const override;

  /// \brief Sets the width (which must be odd) of the smoothing kernel, 0 -> no smoothing.
  ///
  /// The OpenCL and CPU implementations also refer to the width as a radius.
  void set_smooth_img_before_sobel_kernel_radius(const size_type r) override;

protected:
  /// \brief Compute the gradients of the moving images, should be called by
  ///        the derived class in compute() (after pre_compute()).
  void compute_sobel_grads();

  std::shared_ptr<DevBuf> fixed_grad_x_dev_buf_;
  std::shared_ptr<DevBuf> fixed_grad_y_dev_buf_;

  std::shared_ptr<DevBuf> mov_grad_x_dev_buf_;
  std::shared_ptr<DevBuf> mov_grad_y_dev_buf_;

private:
  /// \brief Encodes the smoothing (when enabled) and the gradient computations
  ///        of a number of images.
  void encode_sobel_grads(MetalComputeEncoder& enc, const DevBuf& src_imgs,
                          const size_type src_off, const size_type num_imgs,
                          DevBuf& grad_x_imgs, DevBuf& grad_y_imgs);

  /// \brief Dispatch a kernel with a thread for each pixel of each image.
  void dispatch_per_pixel(MetalComputeEncoder& enc, const MetalComputePipeline& pipeline,
                          const size_type num_imgs);

  MetalComputePipeline smooth_pipeline_;
  MetalComputePipeline sobel_pipeline_;

  DevBuf smooth_kernel_dev_buf_;

  /// \brief Smoothed images, used for the fixed image and the moving images.
  DevBuf smooth_imgs_dev_buf_;

  size_type smooth_img_kernel_rad_ = 5;
};

}  // xreg

#endif
