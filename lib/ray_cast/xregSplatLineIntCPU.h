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

#ifndef XREGSPLATLINEINTCPU_H_
#define XREGSPLATLINEINTCPU_H_

#include <random>

#include <itkLinearInterpolateImageFunction.h>

#include "xregRayCastInterface.h"
#include "xregRayCastSyncBuf.h"

// TODO: re-architect RayCasterBaseCPU to share some implementations from this.
// e.g. the proj() method, allocate_resources(), etc..

namespace xreg
{

class SplatLineIntCPU : public RayCaster
{
public:
  using VolInd     = Vol::IndexType;
  using VolIndList = std::vector<VolInd>;

  /// \brief Trivial constructor - no work done
  SplatLineIntCPU();

  /// \brief Allocate resources required for computing each projection image.
  ///
  /// This will allocate a buffer in host memory large enough to store all
  /// 2D projection images.
  void allocate_resources() override;

  /// \brief Perform the splatting.
  void compute(const size_type vol_idx = 0) override;

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

  const CoordScalarList& num_inds_in_proj() const;

  /// \brief Retrieve the raw pointer to the buffer used for storing computed
  ///        images.
  ///
  /// Should only be called after allocate_resources() has been called.
  PixelScalar2D* raw_host_pixel_buf() override;

  /// \brief Use an external buffer for storing computed DRRs on the HOST
  ///
  /// The default behavior when this is not called, or a null pointer is provided,
  /// is to use an internally allocated buffer.
  /// The user is responsible for ensuring it has sufficient capacity and for
  /// managing the memory properly (e.g. deallocating when appropriate).
  void use_external_host_pixel_buf(void* buf) override;

  /// \brief The maximum number of projections that is possible to allocate
  ///        resources for (as limited by hardware or the system).
  ///
  /// For the CPU, we assume that unlimted memory is available and this returns
  /// the largest possible value of size_type.
  size_type max_num_projs_possible() const override;

  /// \brief Use another ray caster's projection buffer - NOT CURRENTLY SUPPORTED
  ///        with the CPU ray caster - will throw an exception.
  void use_other_proj_buf(RayCaster* other_ray_caster) override;

  RayCastSyncOCLBuf* to_ocl_buf() override;

  RayCastSyncHostBuf* to_host_buf() override;
  
  void set_use_all_vol_inds(const bool use_all);

  void set_inds_to_proj(const VolIndList& inds, const size_type vol_idx = 0,
                        const bool use_these_inds = true);
  
  void set_num_wobbles(const size_type num_wobbles);

  void set_do_post_splat_2D_smooth(const bool do_smooth);

  void set_post_splat_2D_smooth_y_radius(const size_type smooth_radius);
  
  void set_post_splat_2D_smooth_x_radius(const size_type smooth_radius);

  bool bilinear_in_2d() const;

  void set_bilinear_in_2d(const bool b);

protected:
  using PixelBufferVec = std::vector<PixelScalar2D>;

  PixelScalar2D* pixel_buf_to_use();

  /// \brief Initializes the projection buffers depending on the background images,
  ///        or pixel replacement settings.
  ///
  /// Should be called prior to computing the projections from the compute method
  /// in a derived class.
  void pre_compute();

  void vols_changed() override;

  void set_all_inds(const size_type vol_idx);

  PixelScalar2D* ext_pixel_buf_;

  /// \brief Host buffer used to store the line integral images.
  ///
  /// This is addressed in row-major layout within each image, and each image
  /// is stored contiguously:
  ///  Image 1 row 1 col 1, Image 1 row 1 col 2, ..., Image 1 row 1 col M, ..., Image 1 row N col M, Image 2 row 1 col 1, ... Image P row N col M
  PixelBufferVec pixel_buf_;

  RayCastSyncHostBufFromHost sync_to_host_;
  RayCastSyncOCLBufFromHost  sync_to_ocl_;

  // It may be worthwhile to eventually also cache the physical points
  // corresponding to each of these indices
  using ListOfVolIndLists = std::vector<VolIndList>;
  using BoolList = std::vector<bool>;

  bool proj_all_inds_ = true;

  ListOfVolIndLists vols_all_inds_;
  
  BoolList vols_all_inds_valid_;

  ListOfVolIndLists vols_custom_inds_to_proj_;

  using RNGEngine = std::mt19937;
  using NormalDistForRNG = std::normal_distribution<CoordScalar>;

  RNGEngine rng_eng_;

  using VolLinearInterpType = itk::LinearInterpolateImageFunction<Vol,CoordScalar>;
  using LinearInterpPtr     = VolLinearInterpType::Pointer;
  using LinearInterpList    = std::vector<LinearInterpPtr>;

  LinearInterpList vol_interp_fns_;

  // 0 --> no wobble
  size_type num_wobbles_ = 8;

  bool post_splat_2D_smooth_ = true;

  // 0 -> auto-compute
  size_type post_splat_2D_smooth_win_size_x_ = 0;
  size_type post_splat_2D_smooth_win_size_y_ = 0;

  cv::Mat_<PixelScalar2D> tmp_img_for_smooth_;

  CoordScalarList num_inds_in_proj_;

  bool bilinear_in_2d_ = false;
};

}  // xreg

#endif

