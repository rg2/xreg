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

#if !__has_feature(objc_arc)
#error "xregRayCastBaseMetal.mm must be compiled with ARC (-fobjc-arc)"
#endif

#include "xregRayCastBaseMetal.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregMetalConvert.h"
#include "xregMetalShaderShared.h"
#include "xregMetalShaderSrc.h"

namespace xreg
{

// embedded at build time, see lib/ray_cast/CMakeLists.txt
extern const char* const kRAY_CAST_METAL_ARGS_SRC;
extern const char* const kRAY_CAST_BASE_METAL_SRC;

}  // xreg

namespace
{

using namespace xreg;

id<MTLBuffer> MTLBuf(const MetalBuffer& buf)
{
  return (__bridge id<MTLBuffer>) buf.native_handle();
}

template <class T>
id<MTLBuffer> MTLBuf(const MetalVector<T>& v)
{
  return MTLBuf(v.buffer());
}

// Commits a command buffer, waits for it to complete and throws when the
// GPU reports an error.
void CommitAndWait(id<MTLCommandBuffer> cmd_buf, const char* desc)
{
  [cmd_buf commit];
  [cmd_buf waitUntilCompleted];

  if (cmd_buf.status == MTLCommandBufferStatusError)
  {
    xregThrow("Metal %s failed: %s", desc,
              cmd_buf.error ? cmd_buf.error.localizedDescription.UTF8String : "unknown error");
  }
}

// Each threadgroup computes a 2D tile of detector pixels, since the rays of
// neighboring pixels sample nearby voxels, which makes better use of the
// texture cache than a threadgroup along a detector row (2-3x faster on an AMD
// Radeon Pro 5500M and 1.5x faster on an Intel UHD 630). A tile of 8 columns
// by 16 rows was the fastest, or nearly so, on both of those devices.
MTLSize RayCastThreadgroupSize(const MetalComputePipeline& p)
{
  xreg::size_type num_cols = 8;
  xreg::size_type num_rows = 16;

  while ((num_cols * num_rows) > p.max_total_threads_per_threadgroup())
  {
    num_rows /= 2;
  }

  return MTLSizeMake(num_cols, num_rows, 1);
}

}  // un-named

xreg::RayCasterMetal::RayCasterMetal()
  : RayCasterMetal(MetalCmdQueue(MetalDevice::Default()))
{ }

xreg::RayCasterMetal::RayCasterMetal(const MetalDevice& dev)
  : RayCasterMetal(MetalCmdQueue(dev))
{ }

xreg::RayCasterMetal::RayCasterMetal(const MetalCmdQueue& queue)
  : cmd_queue_(queue),
    det_pts_dev_(queue.device()),
    focal_pts_dev_(queue.device()),
    proj_pixels_dev_(queue.device()),
    proj_pixels_dev_to_use_(&proj_pixels_dev_),
    cam_model_for_proj_dev_(queue.device()),
    cam_to_itk_phys_xforms_dev_(queue.device())
{ }

void xreg::RayCasterMetal::set_num_projs(const size_type num_projs)
{
  RayCaster::set_num_projs(num_projs);

  if (!this->camera_models_.empty())
  {
    const size_type tot_num_dets_xfer = this->camera_models_[0].num_det_rows *
                                        this->camera_models_[0].num_det_cols *
                                        num_projs;

    sync_to_metal_.set_range(0, tot_num_dets_xfer);
    sync_to_host_.set_range(0, tot_num_dets_xfer);
  }
}

void xreg::RayCasterMetal::allocate_resources()
{
  RayCaster::allocate_resources();

  const size_type num_det_rows = this->camera_models_[0].num_det_rows;
  const size_type num_det_cols = this->camera_models_[0].num_det_cols;
  const size_type num_dets_per_proj = num_det_rows * num_det_cols;

  if (!ext_pixel_buf_)
  {
    pixel_buf_host_.resize(num_dets_per_proj * this->num_projs_);
  }

  if (max_num_projs_possible() < this->num_projs_)
  {
    const std::string msg = fmt::format(
            "Attempting to alloc more projections allowed in a single buffer by Metal ({:.2f} GB)",
              (this->num_projs_ * num_dets_per_proj * sizeof(PixelScalar2D)) / 1024.0 / 1024.0 / 1024.0);

    if (kENFORCE_METAL_MAX_ALLOC)
    {
      xregThrow(msg.c_str());
    }
    else
    {
      std::cerr << "WARNING: " << msg << std::endl;
    }
  }

  // NOTE: camera models should have already been initialized on the GPU when
  //       the set_camera_model(s) method(s) were called.

  // association of each projection to camera model - this is allowed to change
  // between calls to compute.
  cam_model_for_proj_host_.resize(this->num_projs_, 0);
  cam_model_for_proj_dev_.resize(this->num_projs_);

  const size_type tot_num_pix = this->num_projs_ * num_dets_per_proj;

  if (proj_pixels_dev_to_use_ == &proj_pixels_dev_)
  {
    proj_pixels_dev_.resize(tot_num_pix);
  }

  sync_to_host_.set_metal(proj_pixels_dev_to_use_, cmd_queue_);
  sync_to_metal_.set_metal(proj_pixels_dev_to_use_, cmd_queue_);

  xregASSERT(proj_pixels_dev_to_use_->size() == tot_num_pix);

  sync_to_host_.set_host(this->host_pixel_buf_to_use(), tot_num_pix);

  sync_to_host_.set_modified();
  sync_to_metal_.set_modified();

  cam_to_itk_phys_xforms_host_.resize(this->num_projs_);
  cam_to_itk_phys_xforms_dev_.resize(this->num_projs_);

  if (this->use_bg_projs_)
  {
    const size_type num_cams = this->camera_models_.size();

    bg_projs_to_use_for_each_cam_dev_.resize(num_cams);

    for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
    {
      bg_projs_to_use_for_each_cam_dev_[cam_idx] = std::make_shared<PixelBufDev>(device(), num_dets_per_proj);
    }

    // make sure the background projections are copied to the device
    this->bg_projs_updated_ = true;
  }

  // The kernels do not change, so the library only needs to be compiled once.
  if (!ray_cast_lib_.valid())
  {
    std::string src = MetalHelperShadersSrc();
    src += kRAY_CAST_METAL_ARGS_SRC;
    src += '\n';
    src += kRAY_CAST_BASE_METAL_SRC;
    src += '\n';
    src += ray_cast_kernels_src();

    ray_cast_lib_ = MetalLibrary(device(), src);

    fill_pipeline_ = MetalComputePipeline(ray_cast_lib_, "xregFillFloat");
  }
}

xreg::RayCasterMetal::ProjPtr
xreg::RayCasterMetal::proj(const size_type proj_idx)
{
  sync_to_host_.alloc();
  sync_to_host_.sync();

  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  const size_type det_num_rows = cam.num_det_rows;
  const size_type det_num_cols = cam.num_det_cols;
  const size_type num_dets = det_num_rows * det_num_cols;

  auto img_proj = Proj::New();

  auto img_proj_pixel_container = Proj::PixelContainer::New();
  img_proj_pixel_container->SetImportPointer(host_pixel_buf_to_use() + (num_dets * proj_idx),
                                             num_dets, false);

  img_proj->SetPixelContainer(img_proj_pixel_container);

  Proj::RegionType proj_region;
  proj_region.SetIndex(0, 0);
  proj_region.SetIndex(1, 0);
  proj_region.SetSize(0, det_num_cols);
  proj_region.SetSize(1, det_num_rows);

  img_proj->SetRegions(proj_region);

  const CoordScalar spacings[2] = { cam.det_col_spacing, cam.det_row_spacing };
  img_proj->SetSpacing(spacings);

  return img_proj;
}

cv::Mat xreg::RayCasterMetal::proj_ocv(const size_type proj_idx)
{
  sync_to_host_.alloc();
  sync_to_host_.sync();

  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  return cv::Mat(cam.num_det_rows, cam.num_det_cols,
                 cv::DataType<PixelScalar2D>::type,
                 host_pixel_buf_to_use() + (cam.num_det_rows * cam.num_det_cols * proj_idx));
}

xreg::RayCasterMetal::PixelScalar2D*
xreg::RayCasterMetal::raw_host_pixel_buf()
{
  return host_pixel_buf_to_use();
}

void xreg::RayCasterMetal::use_external_host_pixel_buf(void* buf)
{
  ext_pixel_buf_ = static_cast<PixelScalar2D*>(buf);
}

xreg::size_type xreg::RayCasterMetal::max_num_projs_possible() const
{
  const size_type num_bytes_per_proj = this->camera_models_[0].num_det_rows *
                                       this->camera_models_[0].num_det_cols *
                                       sizeof(PixelScalar2D);

  return static_cast<size_type>((max_metal_alloc_size_fraction_to_use_ * device().max_buffer_length())
                                    / num_bytes_per_proj);
}

void xreg::RayCasterMetal::set_max_metal_alloc_size_fraction_to_use(const double s)
{
  max_metal_alloc_size_fraction_to_use_ = s;
}

double xreg::RayCasterMetal::max_metal_alloc_size_fraction_to_use() const
{
  return max_metal_alloc_size_fraction_to_use_;
}

void xreg::RayCasterMetal::use_other_proj_buf(RayCaster* other_ray_caster)
{
  RayCasterMetal* other = dynamic_cast<RayCasterMetal*>(other_ray_caster);

  if (!other)
  {
    xregThrow("Incompatible ray caster to share buffer from!");
  }

  if (other->device().registry_id() != device().registry_id())
  {
    xregThrow("Cannot share a buffer with a Metal ray caster using a different device!");
  }

  proj_pixels_dev_to_use_ = &other->proj_pixels_dev_;
}

xreg::RayCastSyncMetalBuf* xreg::RayCasterMetal::to_metal_buf()
{
  return &sync_to_metal_;
}

xreg::RayCastSyncHostBuf* xreg::RayCasterMetal::to_host_buf()
{
  return &sync_to_host_;
}

void xreg::RayCasterMetal::set_force_sw_interp(const bool force_sw)
{
  force_sw_interp_ = force_sw;
}

bool xreg::RayCasterMetal::force_sw_interp() const
{
  return force_sw_interp_;
}

bool xreg::RayCasterMetal::use_hw_interp() const
{
  return !force_sw_interp_ && device().supports_32bit_float_filtering();
}

const xreg::MetalCmdQueue& xreg::RayCasterMetal::queue() const
{
  return cmd_queue_;
}

const xreg::MetalDevice& xreg::RayCasterMetal::device() const
{
  return cmd_queue_.device();
}

xreg::MetalComputePipeline
xreg::RayCasterMetal::make_ray_cast_pipeline(const std::string& kernel_name) const
{
  xregASSERT(ray_cast_lib_.valid());

  return MetalComputePipeline(ray_cast_lib_, kernel_name,
                              { { kMETAL_FN_CONST_HW_LINEAR_INTERP, use_hw_interp() } });
}

xreg::RayCasterMetal::PixelScalar2D*
xreg::RayCasterMetal::host_pixel_buf_to_use()
{
  return ext_pixel_buf_ ? ext_pixel_buf_ : pixel_buf_host_.data();
}

void xreg::RayCasterMetal::camera_models_changed()
{
  const size_type num_cams = this->camera_models_.size();

  const size_type num_det_rows = this->camera_models_[0].num_det_rows;
  const size_type num_det_cols = this->camera_models_[0].num_det_cols;
  const size_type num_dets_per_proj = num_det_rows * num_det_cols;

  // detector points of each camera, in row-major format
  Float3ListHost host_det_pts(num_cams * num_dets_per_proj);

  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    const auto det_pts_host = this->camera_models_[cam_idx].detector_grid();

    size_type off = cam_idx * num_dets_per_proj;

    for (size_type det_row = 0; det_row < num_det_rows; ++det_row, off += num_det_cols)
    {
      for (size_type det_col = 0; det_col < num_det_cols; ++det_col)
      {
        host_det_pts[off + det_col] = ConvertToMetal(det_pts_host(det_row,det_col));
      }
    }
  }

  det_pts_dev_.resize(host_det_pts.size());
  CopyHostToMetal(host_det_pts.data(), host_det_pts.data() + host_det_pts.size(),
                  det_pts_dev_, 0, cmd_queue_);

  // focal points (X-ray source points) of each camera
  Float3ListHost host_focal_pts(num_cams);

  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    host_focal_pts[cam_idx] = ConvertToMetal(this->camera_models_[cam_idx].pinhole_pt);
  }

  focal_pts_dev_.resize(num_cams);
  CopyHostToMetal(host_focal_pts.data(), host_focal_pts.data() + num_cams,
                  focal_pts_dev_, 0, cmd_queue_);
}

void xreg::RayCasterMetal::vols_changed()
{
  const size_type num_vols = this->vols_.size();

  vol_texs_dev_.clear();
  vol_texs_dev_.reserve(num_vols);

  for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
  {
    const auto vol_size = this->vols_[vol_idx]->GetLargestPossibleRegion().GetSize();

    vol_texs_dev_.emplace_back(device(), vol_size[0], vol_size[1], vol_size[2]);

    vol_texs_dev_.back().upload(this->vols_[vol_idx]->GetBufferPointer(), cmd_queue_);
  }
}

void xreg::RayCasterMetal::compute_helper_pre_kernels(const size_type vol_idx)
{
  if (this->interp_method_ != RayCaster::kRAY_CAST_INTERP_LINEAR)
  {
    throw UnsupportedOperationException();
  }

  // Having the bounding box computations here, ensures that they are valid,
  // even when the volumes are updated.

  // Get the index bounding box (axis-aligned in the index space) of the volume
  Pt3 img_aabb_min;
  Pt3 img_aabb_max;
  std::tie(img_aabb_min,img_aabb_max) = ITKImageIndexBoundsAsEigen(this->vols_[vol_idx].GetPointer());

  ray_cast_kernel_args_.img_aabb_min = ConvertToMetal(img_aabb_min);
  ray_cast_kernel_args_.img_aabb_max = ConvertToMetal(img_aabb_max);

  // Compute the frame transform from ITK physical space to index space
  const FrameTransform itk_idx_to_itk_phys_pt_xform =
                    ITKImagePhysicalPointTransformsAsEigen(this->vols_[vol_idx].GetPointer());

  ray_cast_kernel_args_.itk_phys_pt_to_itk_idx_xform = ConvertToMetal(itk_idx_to_itk_phys_pt_xform.inverse());

  ray_cast_kernel_args_.num_projs = static_cast<ray_cast_metal_types::UInt>(this->num_projs_);

  ray_cast_kernel_args_.num_det_rows = static_cast<ray_cast_metal_types::UInt>(
                                         this->camera_models_[0].num_det_rows);
  ray_cast_kernel_args_.num_det_cols = static_cast<ray_cast_metal_types::UInt>(
                                         this->camera_models_[0].num_det_cols);

  ray_cast_kernel_args_.num_det_pts = ray_cast_kernel_args_.num_det_rows *
                                      ray_cast_kernel_args_.num_det_cols;

  ray_cast_kernel_args_.step_size = this->ray_step_size_;

  // transfer the current projection poses and camera associations
  for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
  {
    cam_to_itk_phys_xforms_host_[proj_idx] = ConvertToMetal(this->xforms_cam_to_itk_phys_[proj_idx]);

    cam_model_for_proj_host_[proj_idx] = static_cast<ray_cast_metal_types::UInt>(
                                                    this->cam_model_for_proj_[proj_idx]);
  }

  CopyHostToMetal(cam_to_itk_phys_xforms_host_.data(),
                  cam_to_itk_phys_xforms_host_.data() + this->num_projs_,
                  cam_to_itk_phys_xforms_dev_, 0, cmd_queue_);

  CopyHostToMetal(cam_model_for_proj_host_.data(),
                  cam_model_for_proj_host_.data() + this->num_projs_,
                  cam_model_for_proj_dev_, 0, cmd_queue_);

  const size_type num_pix_per_proj = ray_cast_kernel_args_.num_det_pts;

  if (this->use_bg_projs_ && this->bg_projs_updated_)
  {
    // The background projection buffers have been modified on the host,
    // copy them into device memory

    const size_type num_cams = this->camera_models_.size();
    xregASSERT(this->bg_projs_for_each_cam_.size() == num_cams);

    for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
    {
      const PixelScalar2D* cur_host_buf = this->bg_projs_for_each_cam_[cam_idx]->GetBufferPointer();

      CopyHostToMetal(cur_host_buf, cur_host_buf + num_pix_per_proj,
                      *bg_projs_to_use_for_each_cam_dev_[cam_idx], 0, cmd_queue_);
    }

    this->bg_projs_updated_ = false;
  }

  const bool fill_with_default_bg = (this->proj_store_meth_ == RayCaster::kRAY_CAST_PIXEL_REPLACE) &&
                                    !this->use_bg_projs_;

  if (this->use_bg_projs_ || fill_with_default_bg)
  {
    @autoreleasepool
    {
      id<MTLCommandBuffer> cmd_buf = [(__bridge id<MTLCommandQueue>) cmd_queue_.native_handle()
                                        commandBuffer];

      id<MTLBuffer> proj_pixels_buf = MTLBuf(*proj_pixels_dev_to_use_);

      if (this->use_bg_projs_)
      {
        // copy the background of each projection's camera
        id<MTLBlitCommandEncoder> blit = [cmd_buf blitCommandEncoder];

        const size_type num_bytes_per_proj = num_pix_per_proj * sizeof(PixelScalar2D);

        for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
        {
          const size_type cam_idx = this->cam_model_for_proj_[proj_idx];

          [blit copyFromBuffer:MTLBuf(*bg_projs_to_use_for_each_cam_dev_[cam_idx])
                  sourceOffset:0
                      toBuffer:proj_pixels_buf
             destinationOffset:(proj_idx * num_bytes_per_proj)
                          size:num_bytes_per_proj];
        }

        [blit endEncoding];
      }
      else
      {
        // replace all pixels with the default background value
        const ray_cast_metal_types::UInt num_pix =
                        static_cast<ray_cast_metal_types::UInt>(proj_pixels_dev_to_use_->size());

        const float bg_val = this->default_bg_pixel_val_;

        id<MTLComputeCommandEncoder> enc = [cmd_buf computeCommandEncoder];

        [enc setComputePipelineState:(__bridge id<MTLComputePipelineState>) fill_pipeline_.native_handle()];
        [enc setBuffer:proj_pixels_buf offset:0 atIndex:kMETAL_FILL_FLOAT_BUF];
        [enc setBytes:&bg_val length:sizeof(bg_val) atIndex:kMETAL_FILL_FLOAT_VAL];
        [enc setBytes:&num_pix length:sizeof(num_pix) atIndex:kMETAL_FILL_FLOAT_LEN];

        [enc dispatchThreads:MTLSizeMake(num_pix, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(fill_pipeline_.max_total_threads_per_threadgroup(), 1, 1)];

        [enc endEncoding];
      }

      // Not waiting for completion here, since command buffers on a queue are
      // executed in order, this completes prior to the ray casting kernel.
      [cmd_buf commit];
    }
  }
}

void xreg::RayCasterMetal::run_ray_cast_kernel(const MetalComputePipeline& pipeline,
                                               const size_type vol_idx,
                                               const void* extra_args,
                                               const size_type extra_args_num_bytes)
{
  xregASSERT(pipeline.valid());
  xregASSERT(vol_idx < vol_texs_dev_.size());

  // the limit of setBytes
  constexpr size_type kMAX_EXTRA_ARGS_NUM_BYTES = 4096;

  if (extra_args_num_bytes > kMAX_EXTRA_ARGS_NUM_BYTES)
  {
    xregThrow("Metal ray casting kernel specific arguments are too large: %lu bytes (max: %lu)",
              static_cast<unsigned long>(extra_args_num_bytes),
              static_cast<unsigned long>(kMAX_EXTRA_ARGS_NUM_BYTES));
  }

  @autoreleasepool
  {
    id<MTLCommandBuffer> cmd_buf = [(__bridge id<MTLCommandQueue>) cmd_queue_.native_handle()
                                      commandBuffer];

    id<MTLComputeCommandEncoder> enc = [cmd_buf computeCommandEncoder];

    [enc setComputePipelineState:(__bridge id<MTLComputePipelineState>) pipeline.native_handle()];

    [enc setBytes:&ray_cast_kernel_args_ length:sizeof(ray_cast_kernel_args_) atIndex:kRAY_CAST_METAL_ARGS];

    [enc setBuffer:MTLBuf(det_pts_dev_)                offset:0 atIndex:kRAY_CAST_METAL_DET_PTS];
    [enc setBuffer:MTLBuf(focal_pts_dev_)              offset:0 atIndex:kRAY_CAST_METAL_FOCAL_PTS];
    [enc setBuffer:MTLBuf(cam_to_itk_phys_xforms_dev_) offset:0 atIndex:kRAY_CAST_METAL_CAM_TO_ITK_PHYS];
    [enc setBuffer:MTLBuf(cam_model_for_proj_dev_)     offset:0 atIndex:kRAY_CAST_METAL_CAM_MODEL_FOR_PROJ];
    [enc setBuffer:MTLBuf(*proj_pixels_dev_to_use_)    offset:0 atIndex:kRAY_CAST_METAL_PROJ_PIXELS];

    if (extra_args && extra_args_num_bytes)
    {
      [enc setBytes:extra_args length:extra_args_num_bytes atIndex:kRAY_CAST_METAL_EXTRA_ARGS];
    }

    [enc setTexture:(__bridge id<MTLTexture>) vol_texs_dev_[vol_idx].native_handle()
            atIndex:kRAY_CAST_METAL_VOL_TEX];

    [enc dispatchThreads:MTLSizeMake(ray_cast_kernel_args_.num_det_cols,
                                     ray_cast_kernel_args_.num_det_rows,
                                     this->num_projs_)
   threadsPerThreadgroup:RayCastThreadgroupSize(pipeline)];

    [enc endEncoding];

    CommitAndWait(cmd_buf, pipeline.kernel_name().c_str());
  }
}

void xreg::RayCasterMetal::compute_helper_post_kernels(const size_type)
{
  sync_to_host_.set_modified();
  sync_to_metal_.set_modified();
}

