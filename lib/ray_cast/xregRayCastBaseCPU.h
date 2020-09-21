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

#ifndef XREGRAYCASTBASECPU_H_
#define XREGRAYCASTBASECPU_H_

#include "xregRayCastInterface.h"
#include "xregRayCastSyncBuf.h"

namespace xreg
{

class RayCasterCPU : public RayCaster
{
public:
  constexpr static CoordScalar kVOL_BB_STEP_INC_TOL = 1.0e-3;

  /// \brief Trivial constructor - no work done
  RayCasterCPU();

  /// \brief Calls parent, and additionally sets a range on the sync buffers
  /// (so the max allocated buffer is not transferred).
  void set_num_projs(const size_type num_projs) override;

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// This will allocate a buffer in host memory large enough to store all
  /// 2D ray cast results.
  void allocate_resources() override;

  /// \brief Retrieve a 2D line integral image.
  ///
  /// The returned image is a shallow reference into the buffer used for
  /// computation, therefore a subsequent call to compute() may change the
  /// pixel values.
  ProjPtr proj(const size_type proj_idx) override;

  /// \brief Retrieve a 2D line integral image in OpenCV format.
  ///
  /// The returned image is a shallow reference into the buffer used for
  /// computation, therefore a subsequent call to compute() may change the
  /// pixel values.
  cv::Mat proj_ocv(const size_type proj_idx) override;

  /// \brief Retrieve the raw pointer to the buffer used for storing computed
  ///        images.
  ///
  /// Should only be called after allocate_resources() has been called.
  PixelScalar2D* raw_host_pixel_buf() override
  {
    return pixel_buf_to_use();
  }

  /// \brief Use an external buffer for storing computed DRRs on the HOST
  ///
  /// The default behavior when this is not called, or a null pointer is provided,
  /// is to use an internally allocated buffer.
  /// The user is responsible for ensuring it has sufficient capacity and for
  /// managing the memory properly (e.g. deallocating when appropriate).
  void use_external_host_pixel_buf(void* buf) override
  {
    ext_pixel_buf_ = static_cast<PixelScalar2D*>(buf);
  }

  /// \brief The maximum number of projections that is possible to allocate
  ///        resources for (as limited by hardware or the system).
  ///
  /// For the CPU, we assume that unlimted memory is available and this returns
  /// the largest possible value of size_type.
  virtual size_type max_num_projs_possible() const override
  {
    return ~size_type(0);
  }

  /// \brief Use another ray caster's projection buffer - NOT CURRENTLY SUPPORTED
  ///        with the CPU ray caster - will throw an exception.
  void use_other_proj_buf(RayCaster* other_ray_caster) override
  {
    throw UnsupportedOperationException();
  }

  RayCastSyncOCLBuf* to_ocl_buf() override
  {
    return &sync_to_ocl_;
  }

  RayCastSyncHostBuf* to_host_buf() override
  {
    return &sync_to_host_;
  }

  /// \brief Sets the anti-aliasing factor
  ///
  /// A value of zero (the default) disables anti aliasing.
  /// Otherwise, this value indicates the number of rays that should
  /// be cast per output pixel. When this factor is set to 1, only
  /// a single ray is cast per output pixel, but the ray is still jittered.
  void set_anti_alias_factor(const size_type aa_fact)
  {
    aa_fact_ = aa_fact;
  }

  /// \brief Retrieves the anti-aliasing factor
  ///
  /// \see set_anti_alias_factor
  size_type anti_alias_factor() const
  {
    return aa_fact_;
  }

protected:
  using PixelBufferVec = std::vector<PixelScalar2D>;

  PixelScalar2D* pixel_buf_to_use()
  {
    return ext_pixel_buf_ ? ext_pixel_buf_ : &pixel_buf_[0];
  }

  /// \brief Initializes the projection buffers depending on the background images,
  ///        or pixel replacement settings.
  ///
  /// Should be called prior to computing the projections from the compute method
  /// in a derived class.
  void pre_compute();

  PixelScalar2D* ext_pixel_buf_;

  /// \brief Host buffer used to store the line integral images.
  ///
  /// This is addressed in row-major layout within each image, and each image
  /// is stored contiguously:
  ///  Image 1 row 1 col 1, Image 1 row 1 col 2, ..., Image 1 row 1 col M, ..., Image 1 row N col M, Image 2 row 1 col 1, ... Image P row N col M
  PixelBufferVec pixel_buf_;

  RayCastSyncHostBufFromHost sync_to_host_;
  RayCastSyncOCLBufFromHost  sync_to_ocl_;

  size_type aa_fact_ = 0;
};

}  // xreg

#endif

