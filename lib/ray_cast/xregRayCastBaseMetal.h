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

#ifndef XREGRAYCASTBASEMETAL_H_
#define XREGRAYCASTBASEMETAL_H_

#include "xregRayCastInterface.h"
#include "xregRayCastSyncBuf.h"
#include "xregRayCastMetalArgs.h"
#include "xregMetalCompute.h"
#include "xregMetalTexture.h"

namespace xreg
{

/// \brief Common ray casting interface using Metal.
///
/// The counterpart of RayCasterOCL. All coordinate and volume interpolation
/// computations are with single precision floating point.
///
/// A sub-class provides the source of its kernels (ray_cast_kernels_src()),
/// which are compiled together with the common ray casting routines, creates
/// its pipeline(s) with make_ray_cast_pipeline() in allocate_resources(), and
/// implements compute() as:
///   compute_helper_pre_kernels(vol_idx);
///   run_ray_cast_kernel(pipeline, vol_idx, <optional kernel specific args>);
///   compute_helper_post_kernels(vol_idx);
class RayCasterMetal : public RayCaster
{
public:
  /// \brief Default constructor, uses the system default device and creates
  ///        a new command queue.
  RayCasterMetal();

  /// \brief Constructor specifying a device to use, creates a new command queue.
  explicit RayCasterMetal(const MetalDevice& dev);

  /// \brief Constructor specifying a command queue (and therefore device) to use.
  explicit RayCasterMetal(const MetalCmdQueue& queue);

  /// \brief Calls parent, and additionally sets a range on the sync buffers
  ///        (so the max allocated buffer is not transferred).
  void set_num_projs(const size_type num_projs) override;

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// This allocates device buffers large enough to store all 2D ray cast
  /// results and the pose of each projection, and a buffer in host memory for
  /// storing the projections (unless an external buffer is used). The ray
  /// casting library is also compiled on the first call.
  void allocate_resources() override;

  /// \brief Retrieve a 2D ray casting result.
  ///
  /// The returned image is a shallow reference into the host buffer, therefore
  /// a subsequent call to compute() may change the pixel values.
  ProjPtr proj(const size_type proj_idx) override;

  /// \brief Retrieve a 2D ray casting result in OpenCV format.
  ///
  /// \see proj
  cv::Mat proj_ocv(const size_type proj_idx) override;

  /// \brief Retrieve the raw host pointer to the buffer used for storing computed
  ///        images.
  ///
  /// Should only be called after allocate_resources() has been called.
  PixelScalar2D* raw_host_pixel_buf() override;

  /// \brief Use an external buffer for storing computed projections on the HOST
  ///
  /// \see RayCaster::use_external_host_pixel_buf
  void use_external_host_pixel_buf(void* buf) override;

  /// \brief The maximum number of projections that is possible to allocate
  ///        resources for.
  ///
  /// This is determined by the device's maximum buffer length, the size of a
  /// projection, and the fraction of the maximum buffer length set.
  size_type max_num_projs_possible() const override;

  /// \brief Sets the fraction of the device's maximum buffer length that may
  ///        be allocated for storing projections.
  ///
  /// Defaults to 1.0 at construction.
  void set_max_metal_alloc_size_fraction_to_use(const double s);

  /// \brief Retrieves the fraction of the device's maximum buffer length that
  ///        may be allocated for storing projections.
  double max_metal_alloc_size_fraction_to_use() const;

  /// \brief Use another ray caster's projection buffer.
  ///
  /// The other ray caster must be a Metal ray caster using the same device.
  void use_other_proj_buf(RayCaster* other_ray_caster) override;

  RayCastSyncMetalBuf* to_metal_buf() override;

  RayCastSyncHostBuf* to_host_buf() override;

  /// \brief Use software trilinear interpolation instead of hardware texture
  ///        filtering when sampling the volume.
  ///
  /// Software interpolation is always used when the device does not support
  /// filtering of 32-bit floating point textures. This must be called prior to
  /// allocate_resources() to take effect.
  void set_force_sw_interp(const bool force_sw);

  bool force_sw_interp() const;

  /// \brief true when hardware texture filtering is used to sample the volume.
  bool use_hw_interp() const;

  const MetalCmdQueue& queue() const;

  const MetalDevice& device() const;

protected:
  using PixelBufHost = std::vector<PixelScalar2D>;
  using PixelBufDev  = RayCastSyncBuf::MetalBuf;

  using PixelBufDevPtr  = std::shared_ptr<PixelBufDev>;
  using PixelBufDevList = std::vector<PixelBufDevPtr>;

  using Float3ListHost = std::vector<ray_cast_metal_types::Float3>;
  using Float3ListDev  = MetalVector<ray_cast_metal_types::Float3>;

  using Float4x4ListHost = std::vector<ray_cast_metal_types::Float4x4>;
  using Float4x4ListDev  = MetalVector<ray_cast_metal_types::Float4x4>;

  using UIntListHost = std::vector<ray_cast_metal_types::UInt>;
  using UIntListDev  = MetalVector<ray_cast_metal_types::UInt>;

  using VolumeTextureList = std::vector<MetalTexture3D>;

  /// \brief Source of the kernels implemented by a sub-class.
  ///
  /// This is compiled together with the common ray casting routines (in
  /// xregRayCastBaseMetal.metal) and the Metal helpers (e.g. xregMetalInterp.metal).
  virtual std::string ray_cast_kernels_src() const = 0;

  /// \brief Create a pipeline for a kernel in the ray casting library.
  ///
  /// Should be called after RayCasterMetal::allocate_resources(). This sets
  /// the values of the function constants used by the common routines, e.g.
  /// the volume interpolation method.
  MetalComputePipeline make_ray_cast_pipeline(const std::string& kernel_name) const;

  PixelScalar2D* host_pixel_buf_to_use();

  void camera_models_changed() override;

  /// \brief Called anytime volumes from the host are specified, copies them
  ///        into texture memory.
  void vols_changed() override;

  /// \brief Prepares the arguments and projection buffer for a ray casting kernel.
  ///
  /// Uploads the current projection poses and camera associations, then sets
  /// the background pixels of each projection (according to the projection
  /// storage method).
  void compute_helper_pre_kernels(const size_type vol_idx);

  /// \brief Runs a ray casting kernel and waits for it to complete.
  ///
  /// The common arguments (see RayCastMetalBufIdx and RayCastMetalTexIdx) are
  /// bound, along with optional kernel specific arguments at
  /// kRAY_CAST_METAL_EXTRA_ARGS (at most 4 KB). The kernel is executed on a grid of
  /// (number of detector columns) x (number of detector rows) x (number of projections)
  /// threads, e.g. a kernel's [[thread_position_in_grid]] is (column, row, projection).
  void run_ray_cast_kernel(const MetalComputePipeline& pipeline, const size_type vol_idx,
                           const void* extra_args = nullptr,
                           const size_type extra_args_num_bytes = 0);

  void compute_helper_post_kernels(const size_type vol_idx);

  MetalCmdQueue cmd_queue_;

  PixelScalar2D* ext_pixel_buf_ = nullptr;

  /// \brief Host buffer used to store the projections.
  ///
  /// Row-major layout within each image, and each image is stored contiguously.
  PixelBufHost pixel_buf_host_;

  Float3ListDev det_pts_dev_;

  Float3ListDev focal_pts_dev_;

  PixelBufDev  proj_pixels_dev_;
  PixelBufDev* proj_pixels_dev_to_use_;

  UIntListHost cam_model_for_proj_host_;
  UIntListDev  cam_model_for_proj_dev_;

  Float4x4ListHost cam_to_itk_phys_xforms_host_;
  Float4x4ListDev  cam_to_itk_phys_xforms_dev_;

  VolumeTextureList vol_texs_dev_;

  double max_metal_alloc_size_fraction_to_use_ = 1.0;

  RayCastMetalArgs ray_cast_kernel_args_;

  RayCastSyncHostBufFromMetal sync_to_host_;

  RayCastSyncMetalBufFromMetal sync_to_metal_;

  PixelBufDevList bg_projs_to_use_for_each_cam_dev_;

private:
  constexpr static bool kENFORCE_METAL_MAX_ALLOC = true;

  bool force_sw_interp_ = false;

  /// \brief The common ray casting routines and the sub-class kernels.
  MetalLibrary ray_cast_lib_;

  /// \brief Used to fill projections with the default background value.
  MetalComputePipeline fill_pipeline_;
};

}  // xreg

#endif

