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

#ifndef XREGRAYCASTBASEOCL_H_
#define XREGRAYCASTBASEOCL_H_

#include "xregRayCastInterface.h"
#include "xregRayCastSyncBuf.h"

namespace xreg
{

/// \brief Common ray casting interface using OpenCL.
///
/// All coordinate and volume interpolation computations are with
/// single precision floating point.
class RayCasterOCL : public RayCaster
{
public:
  /// \brief Default constructor, chooses a default device, creates a new
  ///        context and command queue.
  RayCasterOCL();

  /// \brief Constructor specifying a device to use, but creates a new context
  ///        and command queue.
  explicit RayCasterOCL(const boost::compute::device& dev);

  /// \brief Constructor specifying a specific context and command queue to use
  RayCasterOCL(const boost::compute::context& ctx,
               const boost::compute::command_queue& queue);


  /// \brief Calls parent, and additionally sets a range on the sync buffers
  /// (so the max allocated buffer is not transferred).
  void set_num_projs(const size_type num_projs) override;

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// This will allocate a buffer in device memory large enough to store all
  /// 2D ray cast results, a buffer in device memory for storing detector points,
  /// and a texture object on the device for interpolating the volume.
  /// TODO: change this behavior:
  /// This will also allocate a buffer in host memory large enough to store the
  /// projections.
  /// The detector points and volume will be transferred to device memory.
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

  /// \brief Retrieve the raw host pointer to the buffer used for storing computed
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
  /// This is determined by the device's maximum buffer allocation size,
  /// the size of a DRR, and the fraction of the maximum buffer allocation
  /// size set. This is available after setting the camera information and
  /// initializing the GPU device.
  size_type max_num_projs_possible() const override;

  /// \brief Sets the fraction of maximum allocation size to actually
  ///        be allocated for DRR computation.
  ///
  /// This may be greater than 1.0, as seen with TITAN Blacks on thin6,
  /// a buffer up to 5.9 GB may be allocated. May need to be set to a
  /// small (less than 1.0) value for cases when GPU memory usage is
  /// high.
  /// Defaults to 1.0 at construction.
  void set_max_opencl_alloc_size_fraction_to_use(const double s);

  /// \brief Retrieves the current fraction of maximum allocation size
  ///        to be used for DRR computations
  double max_opencl_alloc_size_fraction_to_use() const;

  // TODO: implement setters for the max alloc fraction to use
  //       that use all free memory or all memory for the device
  //       (retrieving the amount of free memory will probably
  //        require an OpenGL call as OpenCL does not provide that information -
  //        OpenCL says it is irrelevant due to virtual memory managers, but
  //        it has been my experience that the virtual memory managers do
  //        not work)

  /// \brief Use another ray caster's projection buffer.
  ///
  /// This may only use a buffer allocated with the same GPU context as this
  /// ray caster.
  void use_other_proj_buf(RayCaster* other_ray_caster) override;

  RayCastSyncOCLBuf* to_ocl_buf() override;

  RayCastSyncHostBuf* to_host_buf() override;
  
  /// \brief Arguments that will be passed to the GPU kernel
  ///
  /// This has public visibility in order to use the 
  /// BOOST_COMPUTE_ADAPT_STRUCT functionality.
  struct RayCastArgs
  {
    boost::compute::float16_ itk_phys_pt_to_itk_idx_xform;

    // these are used as float3's
    boost::compute::float4_ img_aabb_min;
    boost::compute::float4_ img_aabb_max;

    boost::compute::float_ step_size;
    boost::compute::float_ pad;  // un-used

    boost::compute::ulong_ num_det_pts;

    boost::compute::ulong_ num_projs;
  };

private:
  constexpr static bool kENFORCE_OPENCL_MAX_ALLOC = true;

protected:
  using PixelBufHost = std::vector<PixelScalar2D>;
  using PixelBufDev  = boost::compute::vector<PixelScalar2D>;

  using PixelBufDevPtr  = std::shared_ptr<PixelBufDev>;
  using PixelBufDevList = std::vector<PixelBufDevPtr>;

  using Float16ListHost = std::vector<boost::compute::float16_>;
  using Float16ListDev  = boost::compute::vector<boost::compute::float16_>;

  using ULongListHost = std::vector<boost::compute::ulong_>;
  using ULongListDev  = boost::compute::vector<boost::compute::ulong_>;

  using Float4ListHost = std::vector<boost::compute::float4_>;
  using Float4ListDev  = boost::compute::vector<boost::compute::float4_>;

  using VolumeTextureList = std::vector<boost::compute::image3d>;

  PixelScalar2D* host_pixel_buf_to_use();

  void camera_models_changed() override;

  void compute_helper_pre_kernels(const size_type vol_idx);

  void compute_helper_post_kernels(const size_type vol_idx);

  /// \brief Called anytime volumes from the host are specified,
  ///        performs the work to move into texture memory.
  void vols_changed() override;

  boost::compute::context ctx_;
  boost::compute::command_queue cmd_queue_;

  PixelScalar2D* ext_pixel_buf_;

  /// \brief Host buffer used to store the line integral images.
  ///
  /// This is addressed in row-major layout within each image, and each image
  /// is stored contiguously:
  ///  Image 1 row 1 col 1, Image 1 row 1 col 2, ..., Image 1 row 1 col M, ..., Image 1 row N col M, Image 2 row 1 col 1, ... Image P row N col M
  PixelBufHost pixel_buf_host_;

  Float4ListDev det_pts_dev_;

  Float4ListDev focal_pts_dev_;

  PixelBufDev  proj_pixels_dev_;
  PixelBufDev* proj_pixels_dev_to_use_;

  ULongListHost cam_model_for_proj_host_;

  ULongListDev cam_model_for_proj_dev_;

  Float16ListHost cam_to_itk_phys_xforms_host_;

  Float16ListDev cam_to_itk_phys_xforms_dev_;

  // NOTE: the default constructor for the texture objects should NOT
  //       create a default context, etc...
  VolumeTextureList vol_texs_dev_;

  double max_opencl_alloc_size_fraction_to_use_;

  RayCastArgs ray_cast_kernel_args_;

  RayCastSyncHostBufFromOCL sync_to_host_;

  RayCastSyncOCLBufFromOCL sync_to_ocl_;

  PixelBufDevList bg_projs_to_use_for_each_cam_dev_;

  // TODO: implement setters for the max alloc fraction to use
  //       that use all free memory or all memory for the device
  //       (retrieving the amount of free memory will probably
  //        require an OpenGL call as OpenCL does not provide that information -
  //        OpenCL says it is irrelevant due to virtual memory managers, but
  //        it has been my experience that the virtual memory managers do
  //        not work)
  
  static std::string RayCastBaseOCLStr();
};

}  // xreg

#endif

