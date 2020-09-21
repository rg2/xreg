/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
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

#ifndef XREGIMGSIMMETRIC2DOCL_H_
#define XREGIMGSIMMETRIC2DOCL_H_

#include "xregImgSimMetric2D.h"
#include "xregRayCastSyncBuf.h"

namespace xreg
{

/// \brief Base class for OpenCL based 2D/3D similarity metrics
class ImgSimMetric2DOCL : public ImgSimMetric2D
{
public:
  using DevBuf = RayCastSyncBuf::OCLBuf;

  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  ImgSimMetric2DOCL();

  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit ImgSimMetric2DOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  ImgSimMetric2DOCL(const boost::compute::context& ctx, const boost::compute::command_queue& queue);

  void allocate_resources() override;

  /// \brief Sets the moving images buffer to use from a ray caster.
  ///
  /// This should always be called after calling allocate_resources() on the
  /// ray caster object and before calling allocate_resources() on the
  /// similarity object.
  /// This will replace the current object's context and queue with those of
  /// the ray caster.
  void set_mov_imgs_buf_from_ray_caster(RayCaster* ray_caster,
                                        const size_type proj_offset = 0) override;

  /// \brief Sets the moving images buffer to use from a device buffer.
  void set_mov_imgs_ocl_buf(DevBuf* mov_imgs_buf, const size_type proj_offset = 0);

  /// \brief Sets the moving images buffer to use from a host buffer.
  ///
  /// NOTE: This provides no way of triggering a sync back to the device if the
  ///       contents of the host are modified!
  void set_mov_imgs_host_buf(Scalar* mov_imgs_buf, const size_type proj_offset = 0) override;

  long vienna_cl_ctx_idx() const;

  void set_vienna_cl_ctx_idx(const long ctx_idx);

  void set_setup_vienna_cl_ctx(const bool ctx);

  void set_fixed_image_dev(std::shared_ptr<DevBuf>& fixed_dev);

protected:
  void pre_compute();
  
  void process_mask() override;

  boost::compute::context ctx_;
  boost::compute::command_queue queue_;

  std::shared_ptr<DevBuf> fixed_img_ocl_buf_;

  std::unique_ptr<DevBuf> mask_ocl_buf_;

  RayCastSyncOCLBuf* sync_ocl_buf_ = nullptr;

  DevBuf* mov_imgs_buf_ = nullptr;

  size_type proj_off_ = 0;

  std::unique_ptr<DevBuf> internal_ocl_buf_;

  size_type num_pix_per_proj_after_mask_ = 0;

  long vienna_cl_ctx_idx_ = 0;
  
  bool setup_vienna_cl_ctx_ = true;
};

}  // xreg

#endif

