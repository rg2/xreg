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

#include "xregRayCastBaseOCL.h"

#include <boost/compute/types/struct.hpp>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregOpenCLConvert.h"
#include "xregOpenCLMath.h"
#include "xregOpenCLSpatial.h"

xreg::RayCasterOCL::RayCasterOCL()
  : ctx_(boost::compute::system::default_device()),
    cmd_queue_(ctx_, ctx_.get_device()),
    det_pts_dev_(ctx_),
    focal_pts_dev_(ctx_),
    proj_pixels_dev_(ctx_),
    proj_pixels_dev_to_use_(&proj_pixels_dev_),
    cam_model_for_proj_dev_(ctx_),
    cam_to_itk_phys_xforms_dev_(ctx_)
{
  use_external_host_pixel_buf(nullptr);
  set_max_opencl_alloc_size_fraction_to_use(1.0);
}

xreg::RayCasterOCL::RayCasterOCL(const boost::compute::device& dev)
  : ctx_(dev),
    cmd_queue_(ctx_, dev),
    det_pts_dev_(ctx_),
    focal_pts_dev_(ctx_),
    proj_pixels_dev_(ctx_),
    proj_pixels_dev_to_use_(&proj_pixels_dev_),
    cam_model_for_proj_dev_(ctx_),
    cam_to_itk_phys_xforms_dev_(ctx_)
{
  use_external_host_pixel_buf(nullptr);
  set_max_opencl_alloc_size_fraction_to_use(1.0);
}

xreg::RayCasterOCL::RayCasterOCL(const boost::compute::context& ctx,
                                 const boost::compute::command_queue& queue)
  : ctx_(ctx),
    cmd_queue_(queue),
    det_pts_dev_(ctx_),
    focal_pts_dev_(ctx_),
    proj_pixels_dev_(ctx_),
    proj_pixels_dev_to_use_(&proj_pixels_dev_),
    cam_model_for_proj_dev_(ctx_),
    cam_to_itk_phys_xforms_dev_(ctx_)
{
  use_external_host_pixel_buf(nullptr);
  set_max_opencl_alloc_size_fraction_to_use(1.0);
}

void xreg::RayCasterOCL::set_num_projs(const size_type num_projs)
{
  RayCaster::set_num_projs(num_projs);

  if (!this->camera_models_.empty())
  {
    const size_type tot_num_dets_xfer = this->camera_models_[0].num_det_rows *
                                        this->camera_models_[0].num_det_cols *
                                        num_projs;

    sync_to_ocl_.set_range(0, tot_num_dets_xfer);
    sync_to_host_.set_range(0, tot_num_dets_xfer);
  }
}

void xreg::RayCasterOCL::allocate_resources()
{
  namespace bc = boost::compute;

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
            "Attempting to alloc more projections allowed in a single buffer by OpenCL ({:.2f} GB)",
              (this->num_projs_ * num_dets_per_proj * sizeof(PixelScalar2D)) / 1024.0 / 1024.0 / 1024.0);

    if (kENFORCE_OPENCL_MAX_ALLOC)
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
  cam_model_for_proj_dev_.resize(this->num_projs_, cmd_queue_);

  const size_type tot_num_pix = this->num_projs_ * num_dets_per_proj;

  if (proj_pixels_dev_to_use_ == &proj_pixels_dev_)
  {
    proj_pixels_dev_.resize(tot_num_pix, cmd_queue_);

    sync_to_host_.set_ocl(&proj_pixels_dev_, cmd_queue_);
    sync_to_ocl_.set_ocl(&proj_pixels_dev_, cmd_queue_);
  }
  else
  {
    sync_to_host_.set_ocl(proj_pixels_dev_to_use_, cmd_queue_);
    sync_to_ocl_.set_ocl(proj_pixels_dev_to_use_, cmd_queue_);
  }
  xregASSERT(proj_pixels_dev_to_use_->size() == tot_num_pix);

  sync_to_host_.set_host(this->host_pixel_buf_to_use(), tot_num_pix);

  sync_to_host_.set_modified();
  sync_to_ocl_.set_modified();

  // NOTE: the volume texture creation was moved to the method vols_changed()

  // determine if a separate buffer should be allocated to store the pose matrices
  // this will be used when using something like CMA-ES that will need 100's or 1000's
  // of projections at each iteration.

  cam_to_itk_phys_xforms_host_.resize(this->num_projs_);
  cam_to_itk_phys_xforms_dev_.resize(this->num_projs_, cmd_queue_);

  if (this->use_bg_projs_)
  {
    const size_type num_cams = this->camera_models_.size();

    this->bg_projs_to_use_for_each_cam_dev_.resize(num_cams);

    for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
    {
      bg_projs_to_use_for_each_cam_dev_[cam_idx] = std::make_shared<PixelBufDev>(num_dets_per_proj, ctx_);
    }
  }
}

xreg::RayCasterOCL::ProjPtr
xreg::RayCasterOCL::proj(const size_type proj_idx)
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

cv::Mat xreg::RayCasterOCL::proj_ocv(const size_type proj_idx)
{
  sync_to_host_.alloc();
  sync_to_host_.sync();

  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  return cv::Mat(cam.num_det_rows, cam.num_det_cols,
                 cv::DataType<PixelScalar2D>::type,
                 host_pixel_buf_to_use() + (cam.num_det_rows * cam.num_det_cols * proj_idx));
}

xreg::RayCasterOCL::PixelScalar2D*
xreg::RayCasterOCL::raw_host_pixel_buf()
{
  return host_pixel_buf_to_use();
}
  
void xreg::RayCasterOCL::use_external_host_pixel_buf(void* buf)
{
  ext_pixel_buf_ = static_cast<PixelScalar2D*>(buf);
}

xreg::size_type
xreg::RayCasterOCL::max_num_projs_possible() const
{
  const size_type num_bytes_per_drr = this->camera_models_[0].num_det_rows *
                                      this->camera_models_[0].num_det_cols *
                                      sizeof(PixelScalar2D);
 
  const size_type dev_max_mem_alloc = cmd_queue_.get_device().max_memory_alloc_size();
  
  return static_cast<size_type>((max_opencl_alloc_size_fraction_to_use_ * dev_max_mem_alloc)
                                    / num_bytes_per_drr);
}
  
void xreg::RayCasterOCL::set_max_opencl_alloc_size_fraction_to_use(const double s)
{
  max_opencl_alloc_size_fraction_to_use_ = s;
}

double xreg::RayCasterOCL::max_opencl_alloc_size_fraction_to_use() const
{
  return max_opencl_alloc_size_fraction_to_use_;
}

void xreg::RayCasterOCL::use_other_proj_buf(RayCaster* other_ray_caster)
{
  RayCasterOCL* other = dynamic_cast<RayCasterOCL*>(other_ray_caster);
  if (other)
  {
    proj_pixels_dev_to_use_ = &other->proj_pixels_dev_;
  }
  else
  {
    xregThrow("Incompatible ray caster to share buffer from!");
  }
}
  
xreg::RayCastSyncOCLBuf* xreg::RayCasterOCL::to_ocl_buf()
{
  return &sync_to_ocl_;
}

xreg::RayCastSyncHostBuf* xreg::RayCasterOCL::to_host_buf()
{
  return &sync_to_host_;
}
  
xreg::RayCasterOCL::PixelScalar2D*
xreg::RayCasterOCL::host_pixel_buf_to_use()
{
  return ext_pixel_buf_ ? ext_pixel_buf_ : &pixel_buf_host_[0];
}

void xreg::RayCasterOCL::camera_models_changed()
{
  const size_type num_cams = this->camera_models_.size();

  const size_type num_det_rows = this->camera_models_[0].num_det_rows;
  const size_type num_det_cols = this->camera_models_[0].num_det_cols;
  const size_type num_dets_per_proj = num_det_rows * num_det_cols;

  const size_type num_tot_dets = num_cams * num_dets_per_proj;

  // copy detector points to OpenCL compatible buffer in row-major format
  Float4ListHost host_ocl_det_pts(num_tot_dets);

  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    const auto det_pts_host = this->camera_models_[cam_idx].detector_grid();

    size_type off = cam_idx * num_dets_per_proj;

    for (size_type det_row = 0; det_row < num_det_rows; ++det_row, off += num_det_cols)
    {
      for (size_type det_col = 0; det_col < num_det_cols; ++det_col)
      {
        // convert from an Eigen::Matrix<> to cl_float3 to homogeneous bc::float4_
        host_ocl_det_pts[off + det_col] = OpenCLFloat3ToBoostComp4(
                                ConvertToOpenCL(det_pts_host(det_row,det_col)), 1);
      }
    }
  }

  // allocate device memory for detector points and initialize it with the contents of the host buffer
  det_pts_dev_.assign(host_ocl_det_pts.begin(), host_ocl_det_pts.end(), cmd_queue_);

  // convert focal points for each camera into open cl format.
  Float4ListHost host_ocl_focal_pts(num_cams);

  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    // convert from an Eigen::Matrix<> to cl_float3 to homogeneous bc::float4_
    host_ocl_focal_pts[cam_idx] = OpenCLFloat3ToBoostComp4(
                                    ConvertToOpenCL(this->camera_models_[cam_idx].pinhole_pt), 1);
  }

  // allocate device memory for focal points (x-ray source points) and copy
  focal_pts_dev_.assign(host_ocl_focal_pts.begin(), host_ocl_focal_pts.end(), cmd_queue_);
}

void xreg::RayCasterOCL::compute_helper_pre_kernels(const size_type vol_idx)
{
  namespace bc = boost::compute;

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

  ray_cast_kernel_args_.img_aabb_min = OpenCLFloat3ToBoostComp4(ConvertToOpenCL(img_aabb_min));
  ray_cast_kernel_args_.img_aabb_max = OpenCLFloat3ToBoostComp4(ConvertToOpenCL(img_aabb_max));

  // Compute the frame transform from ITK physical space to index space (sR + t)
  const FrameTransform itk_idx_to_itk_phys_pt_xform = ITKImagePhysicalPointTransformsAsEigen(this->vols_[vol_idx].GetPointer());

  const FrameTransform itk_phys_pt_to_itk_idx_xform = itk_idx_to_itk_phys_pt_xform.inverse();

  ray_cast_kernel_args_.itk_phys_pt_to_itk_idx_xform =
                                OpenCLFloat16ToBoostComp16(ConvertToOpenCL(itk_phys_pt_to_itk_idx_xform));

  ray_cast_kernel_args_.num_projs = this->num_projs_;

  // convert current projection poses
  for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
  {
    cam_to_itk_phys_xforms_host_[proj_idx] =
              OpenCLFloat16ToBoostComp16(ConvertToOpenCL(this->xforms_cam_to_itk_phys_[proj_idx]));
  }

  cam_to_itk_phys_xforms_dev_.assign(cam_to_itk_phys_xforms_host_.begin(),
                                     cam_to_itk_phys_xforms_host_.end(),
                                     cmd_queue_);

  // convert current camera associations
  for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
  {
    cam_model_for_proj_host_[proj_idx] = static_cast<bc::ulong_>(this->cam_model_for_proj_[proj_idx]);
  }

  // transfer current camera associations
  cam_model_for_proj_dev_.assign(cam_model_for_proj_host_.begin(),
                                 cam_model_for_proj_host_.end(), cmd_queue_);

  ray_cast_kernel_args_.num_det_pts = this->camera_models_[0].num_det_rows *
                                      this->camera_models_[0].num_det_cols;

  ray_cast_kernel_args_.step_size = this->ray_step_size_;

  if (this->use_bg_projs_)
  {
    const size_type num_cams = this->camera_models_.size();
    xregASSERT(this->bg_projs_for_each_cam_.size() == num_cams);

    const size_type num_pix_per_proj = this->bg_projs_to_use_for_each_cam_dev_[0]->size();

    if (this->bg_projs_updated_)
    {
      // The background projection buffers have been modified on the host,
      // copy them back into device memory

      for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
      {
        PixelScalar2D* cur_host_buf = this->bg_projs_for_each_cam_[cam_idx]->GetBufferPointer();
        bc::copy(cur_host_buf, cur_host_buf + num_pix_per_proj,
                 bg_projs_to_use_for_each_cam_dev_[cam_idx]->begin(),
                 cmd_queue_);
      }

      this->bg_projs_updated_ = false;
    }

    for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
    {
      const size_type cam_idx = this->cam_model_for_proj_[proj_idx];

      bc::copy(bg_projs_to_use_for_each_cam_dev_[cam_idx]->begin(),
               bg_projs_to_use_for_each_cam_dev_[cam_idx]->end(),
               proj_pixels_dev_to_use_->begin() + (num_pix_per_proj * proj_idx),
               cmd_queue_);
    }
  }

  switch (this->proj_store_meth_)
  {
  case RayCaster::kRAY_CAST_PIXEL_REPLACE:
    if (!this->use_bg_projs_)  // do not overwrite with zeros if we're using a background imge that has been set
    {
      bc::fill(proj_pixels_dev_to_use_->begin(), proj_pixels_dev_to_use_->end(),
               this->default_bg_pixel_val_, cmd_queue_);
    }
    break;
  case RayCaster::kRAY_CAST_PIXEL_ACCUM:
  default:
    // do nothing, keep previous value
    break;
  }
}

void xreg::RayCasterOCL::compute_helper_post_kernels(const size_type)
{
  sync_to_host_.set_modified();
  sync_to_ocl_.set_modified();
}

void xreg::RayCasterOCL::vols_changed()
{
  namespace bc = boost::compute;

  // initialize volume texture memory
  const size_type num_vols = this->vols_.size();

  vol_texs_dev_.resize(num_vols);

  for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
  {
    const auto vol_size = this->vols_[vol_idx]->GetLargestPossibleRegion().GetSize();

    vol_texs_dev_[vol_idx] = bc::image3d(ctx_, vol_size[0], vol_size[1], vol_size[2],
                               bc::image_format(bc::image_format::intensity,
                                                bc::image_format::float32),
                               bc::image3d::read_only | bc::image3d::use_host_ptr,
                               this->vols_[vol_idx]->GetBufferPointer());
  }
}

//////////////////////////////////////////////////////////////////////

BOOST_COMPUTE_ADAPT_STRUCT(xreg::RayCasterOCL::RayCastArgs, RayCastArgs,
                           (itk_phys_pt_to_itk_idx_xform,
                            img_aabb_min,
                            img_aabb_max,
                            step_size,
                            pad,
                            num_det_pts,
                            num_projs))

//////////////////////////////////////////////////////////////////////

std::string xreg::RayCasterOCL::RayCastBaseOCLStr()
{
  std::stringstream ss;
  ss << boost::compute::type_definition<RayCastArgs>()
     << kOPENCL_MATH_UTILS_SRC
     << kOPENCL_SPATIAL_SRC;
  
  return ss.str();
}

