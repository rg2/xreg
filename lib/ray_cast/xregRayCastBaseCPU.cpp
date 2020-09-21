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

#include "xregRayCastBaseCPU.h"

#include "xregExceptionUtils.h"
#include "xregAssert.h"

xreg::RayCasterCPU::RayCasterCPU()
{
  use_external_host_pixel_buf(nullptr);
}

void xreg::RayCasterCPU::set_num_projs(const size_type num_projs)
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

void xreg::RayCasterCPU::allocate_resources()
{
  RayCaster::allocate_resources();

  // Check that the images produced by each model have the same dimensions
  const size_type num_rows = this->camera_models_[0].num_det_rows;
  const size_type num_cols = this->camera_models_[0].num_det_cols;

  const size_type num_cams = this->camera_models_.size();

  for (size_type cam_idx = 1; cam_idx < num_cams; ++cam_idx)
  {
    if (num_rows != this->camera_models_[cam_idx].num_det_rows)
    {
      xregThrow("Inconsistent number rows in camera model images!");
    }
    else if (num_cols != this->camera_models_[cam_idx].num_det_cols)
    {
      xregThrow("Inconsistent number columns in camera model images!");
    }
  }

  const size_type num_tot_pix = num_rows * num_cols * this->num_projs_;

  if (!ext_pixel_buf_)
  {
    pixel_buf_.resize(num_tot_pix);
    sync_to_ocl_.set_host(pixel_buf_);
    sync_to_host_.set_host(pixel_buf_);
  }
  else
  {
    sync_to_ocl_.set_host(ext_pixel_buf_, num_tot_pix);
    sync_to_host_.set_host(ext_pixel_buf_, num_tot_pix);
  }

  sync_to_ocl_.set_modified();
  sync_to_host_.set_modified();
}

xreg::RayCasterCPU::ProjPtr
xreg::RayCasterCPU::proj(const size_type proj_idx)
{
  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  const size_type det_num_rows = cam.num_det_rows;
  const size_type det_num_cols = cam.num_det_cols;
  const size_type num_dets = det_num_rows * det_num_cols;

  auto img_proj = Proj::New();

  auto img_proj_pixel_container = Proj::PixelContainer::New();
  img_proj_pixel_container->SetImportPointer(pixel_buf_to_use() + (num_dets * proj_idx), num_dets, false);

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

cv::Mat xreg::RayCasterCPU::proj_ocv(const size_type proj_idx)
{
  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  return cv::Mat(cam.num_det_rows, cam.num_det_cols, cv::DataType<PixelScalar2D>::type,
                 pixel_buf_to_use() + (cam.num_det_rows * cam.num_det_cols * proj_idx));
}

void xreg::RayCasterCPU::pre_compute()
{
  const size_type num_pix_per_proj = this->camera_models_[0].num_det_rows * this->camera_models_[0].num_det_cols;
  const size_type num_tot_pix      = this->num_projs_ * num_pix_per_proj;

  if (this->use_bg_projs_)
  {
    xregASSERT(this->bg_projs_for_each_cam_.size() == this->num_camera_models());

    for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
    {
      memcpy(this->pixel_buf_to_use() + (proj_idx * num_pix_per_proj),
             this->bg_projs_for_each_cam_[this->cam_model_for_proj_[proj_idx]]->GetBufferPointer(),
             sizeof(PixelScalar2D) * num_pix_per_proj);
    }
  }

  switch (this->proj_store_meth_)
  {
  case kRAY_CAST_PIXEL_REPLACE:
    if (!this->use_bg_projs_)  // do not overwrite with zeros if we're using a background imge that has been set
    {
      std::fill(this->pixel_buf_to_use(), this->pixel_buf_to_use() + num_tot_pix, this->default_bg_pixel_val_);
    }
    break;
  case kRAY_CAST_PIXEL_ACCUM:
  default:
    // do nothing, keep previous value
    break;
  }
}

