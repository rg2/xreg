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

#ifndef XREGIMGSIMMETRIC2DMETAL_H_
#define XREGIMGSIMMETRIC2DMETAL_H_

#include "xregImgSimMetric2D.h"
#include "xregImgSimMetric2DMetalArgs.h"
#include "xregRayCastSyncBuf.h"
#include "xregMetalCompute.h"

namespace xreg
{

/// \brief Base class for Metal based 2D/3D similarity metrics.
///
/// The counterpart of ImgSimMetric2DOCL. All computations are with single
/// precision floating point.
///
/// The fixed image (from the host or a device buffer) is never modified. The
/// moving images may be modified by a similarity metric computation.
///
/// A sub-class compiles its kernels with make_library(), which includes the
/// common similarity metric routines (xregImgSimMetric2DMetal.metal), creates
/// its pipelines and buffers prior to calling ImgSimMetric2DMetal::allocate_resources()
/// (which may call process_mask()), and calls pre_compute() at the start of
/// compute().
class ImgSimMetric2DMetal : public ImgSimMetric2D
{
public:
  using DevBuf = RayCastSyncBuf::MetalBuf;

  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  ImgSimMetric2DMetal();

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit ImgSimMetric2DMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit ImgSimMetric2DMetal(const MetalCmdQueue& queue);

  void allocate_resources() override;

  /// \brief Sets the moving images buffer to use from a ray caster.
  ///
  /// This should always be called after calling allocate_resources() on the
  /// ray caster object and before calling allocate_resources() on the
  /// similarity object.
  /// This will replace the current object's command queue with that of the
  /// ray caster.
  void set_mov_imgs_buf_from_ray_caster(RayCaster* ray_caster,
                                        const size_type proj_offset = 0) override;

  /// \brief Sets the moving images buffer to use from a device buffer.
  ///
  /// The buffer must be on the same device as this object.
  void set_mov_imgs_metal_buf(DevBuf* mov_imgs_buf, const size_type proj_offset = 0);

  /// \brief Sets the moving images buffer to use from a host buffer.
  ///
  /// The moving images (starting at proj_offset) are copied to the device
  /// when this is called, so the fixed image and number of moving images must
  /// already be set.
  /// NOTE: This provides no way of triggering a sync back to the device if the
  ///       contents of the host are modified!
  void set_mov_imgs_host_buf(Scalar* mov_imgs_buf, const size_type proj_offset = 0) override;

  /// \brief Use a fixed image already on the device, instead of copying the
  ///        fixed image from the host.
  ///
  /// The fixed image must still be set, since it describes the image
  /// dimensions. This buffer is not modified.
  void set_fixed_image_dev(std::shared_ptr<DevBuf>& fixed_dev);

  const MetalCmdQueue& queue() const;

  const MetalDevice& device() const;

protected:
  /// \brief Compile a library with the common similarity metric routines and
  ///        the kernels of a sub-class.
  MetalLibrary make_library(const std::string& kernels_src) const;

  /// \brief Synchronizes the moving images and processes an updated mask.
  void pre_compute();

  /// \brief Copies the mask to the device, as floats, which is all ones when
  ///        there is no mask.
  void process_mask() override;

  /// \brief The arguments of a kernel processing a number of images with the
  ///        dimensions of the fixed image.
  ImgSimMetricMetalImgArgs img_args(const size_type num_imgs);

  /// \brief The offset (in elements) of the first moving image in the moving
  ///        images buffer.
  size_type mov_imgs_buf_offset();

  /// \brief Sets the pipeline of a kernel that computes a reduction over each
  ///        row of a matrix (e.g. each image), and dispatches it using one
  ///        threadgroup per row.
  ///
  /// The kernel arguments should already be set.
  static void dispatch_reduction(MetalComputeEncoder& enc, const MetalComputePipeline& pipeline,
                                 const size_type num_rows);

  /// \brief Copy the similarity value of each moving image from the device.
  void copy_sim_vals_to_host(const DevBuf& sim_vals_dev);

  MetalCmdQueue queue_;

  std::shared_ptr<DevBuf> fixed_img_metal_buf_;

  /// \brief 1 for each pixel that is not masked out, 0 otherwise.
  DevBuf mask_metal_buf_;

  RayCastSyncMetalBuf* sync_metal_buf_ = nullptr;

  DevBuf* mov_imgs_buf_ = nullptr;

  size_type proj_off_ = 0;

  std::unique_ptr<DevBuf> internal_metal_buf_;

  size_type num_pix_per_proj_after_mask_ = 0;
};

}  // xreg

#endif
