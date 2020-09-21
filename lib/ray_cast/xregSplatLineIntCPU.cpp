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

#include "xregSplatLineIntCPU.h"

#include <boost/algorithm/clamp.hpp>

#include <opencv2/imgproc.hpp>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregITKBasicImageUtils.h"

xreg::SplatLineIntCPU::SplatLineIntCPU()
{
  use_external_host_pixel_buf(nullptr);
}

void xreg::SplatLineIntCPU::allocate_resources()
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

  std::random_device rand_dev;
  rng_eng_.seed(rand_dev());

  if (post_splat_2D_smooth_)
  {
    tmp_img_for_smooth_ = cv::Mat_<PixelScalar2D>(num_rows, num_cols);
  }

  num_inds_in_proj_.resize(this->num_projs_);
}

void xreg::SplatLineIntCPU::compute(const size_type vol_idx)
{
  namespace ba = boost::algorithm;

  xregASSERT(this->resources_allocated_);
  
  this->pre_compute();

  const bool perform_wobble = num_wobbles_ > 0;

  const VolIndList* inds_to_use = 0;
  
  if (proj_all_inds_)
  {
    set_all_inds(vol_idx);
    inds_to_use = &vols_all_inds_[vol_idx];
  }
  else
  {
    inds_to_use = &vols_custom_inds_to_proj_[vol_idx];
  }

  const auto& vol_inds = *inds_to_use;

  const auto* vol = this->vols_[vol_idx].GetPointer();

  auto max_vol_inds = vol->GetLargestPossibleRegion().GetSize();
  --max_vol_inds[0];
  --max_vol_inds[1];
  --max_vol_inds[2];

  // Compute the frame transform from ITK physical space to index space (sR + t)
  const FrameTransform itk_idx_to_itk_phys_pt_xform = ITKImagePhysicalPointTransformsAsEigen(vol);
  
  Pt3 tmp_vol_idx;
  
  const size_type proj_num_rows = this->camera_models_[0].num_det_rows;
  const size_type proj_num_cols = this->camera_models_[0].num_det_cols;
  const size_type proj_num_pix  = proj_num_rows * proj_num_cols;

  const CoordScalar proj_xps = this->camera_models_[0].det_col_spacing;
  const CoordScalar proj_yps = this->camera_models_[0].det_row_spacing;

  auto* vol_interp = vol_interp_fns_[vol_idx].GetPointer();

  NormalDistForRNG voxel_off_dist(0, 0.5);

  if (post_splat_2D_smooth_)
  {
    // This could probably be made a little more robust by also using the 3D
    // volume resolution.
  
    // may also want to look at changing sigma
    // also, does it ever make sense to not smooth (determine this automatically)

    if (!post_splat_2D_smooth_win_size_x_)
    {
      post_splat_2D_smooth_win_size_x_ = std::max(3l, std::lround(3.0 / proj_xps));
      
      // make it odd
      if (!(post_splat_2D_smooth_win_size_x_ & 1))
      {
        ++post_splat_2D_smooth_win_size_x_;
      }
    }

    if (!post_splat_2D_smooth_win_size_y_)
    {
      post_splat_2D_smooth_win_size_y_ = std::max(3l, std::lround(3.0 / proj_yps));
      
      // make it odd
      if (!(post_splat_2D_smooth_win_size_y_ & 1))
      {
        ++post_splat_2D_smooth_win_size_y_;
      }
    }
  }

  num_inds_in_proj_.assign(this->num_projs_, 0);

  for (size_type proj_idx = 0; proj_idx < this->num_projs_; ++proj_idx)
  {
    // already set to zero right before this loop
    CoordScalar& num_inds_in_cur_proj = num_inds_in_proj_[proj_idx];

    PixelScalar2D* cur_proj_buf = &this->pixel_buf_to_use()[proj_num_pix * proj_idx];

    const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

    const FrameTransform vol_idx_to_cam_world =
                          this->xforms_cam_to_itk_phys_[proj_idx].inverse() *
                                                      itk_idx_to_itk_phys_pt_xform;

    for (const auto& src_itk_idx : vol_inds)
    {
      if (perform_wobble)
      {
        for (size_type wobble_idx = 0; wobble_idx < num_wobbles_; ++wobble_idx)
        {
          const CoordScalar x_off = voxel_off_dist(rng_eng_);
          const CoordScalar y_off = voxel_off_dist(rng_eng_);
          const CoordScalar z_off = voxel_off_dist(rng_eng_);
  
          itk::ContinuousIndex<CoordScalar,3> cur_itk_idx = src_itk_idx;
          cur_itk_idx[0] += x_off;
          cur_itk_idx[1] += y_off;
          cur_itk_idx[2] += z_off;
          
          // clamp these to be in volume bounds
          cur_itk_idx[0] = ba::clamp(cur_itk_idx[0], CoordScalar(0),
                                     CoordScalar(max_vol_inds[0]));
          cur_itk_idx[1] = ba::clamp(cur_itk_idx[1], CoordScalar(0),
                                     CoordScalar(max_vol_inds[1]));
          cur_itk_idx[2] = ba::clamp(cur_itk_idx[2], CoordScalar(0),
                                     CoordScalar(max_vol_inds[2]));

          tmp_vol_idx[0] = cur_itk_idx[0];
          tmp_vol_idx[1] = cur_itk_idx[1];
          tmp_vol_idx[2] = cur_itk_idx[2];
         
          const Pt3 proj_img_idx_pt =
                          cam.phys_pt_to_ind_pt(vol_idx_to_cam_world * tmp_vol_idx);
           
          if (bilinear_in_2d_)
          {
            const int cf = static_cast<int>(std::floor(proj_img_idx_pt[0]));
            const int rf = static_cast<int>(std::floor(proj_img_idx_pt[1]));
            const int c1 = std::max(0,cf);
            const int r1 = std::max(0,rf);
            const int c2 = std::min(static_cast<int>(proj_num_cols-1), cf + 1);
            const int r2 = std::min(static_cast<int>(proj_num_rows-1), rf + 1);

            const int nr = r2 - r1 + 1;
            const int nc = c2 - c1 + 1;

            if (nr && nc)
            {
              ++num_inds_in_cur_proj;

              const PixelScalar2D v = vol_interp->EvaluateAtContinuousIndex(cur_itk_idx);
             
              const bool two_rows = nr == 2;
              const bool two_cols = nc == 2;
              const bool one_row  = nr == 1;
              const bool one_col  = nc == 1;
              
              if (two_rows && two_cols)
              {
                // standard case, all 4 neighbors available
                
                PixelScalar2D* row1_buf = &cur_proj_buf[r1 * proj_num_cols];
                PixelScalar2D* row2_buf = &cur_proj_buf[r2 * proj_num_cols];

                const PixelScalar2D wgt_r = static_cast<PixelScalar2D>(proj_img_idx_pt[1] - r1);
                const PixelScalar2D wgt_c = static_cast<PixelScalar2D>(proj_img_idx_pt[0] - c1);
                const PixelScalar2D one_minus_wgt_r = 1 - wgt_r;
                const PixelScalar2D one_minus_wgt_c = 1 - wgt_c;

                row1_buf[c1] += v * wgt_r * wgt_c;
                row1_buf[c2] += v * wgt_r * one_minus_wgt_c;
                row2_buf[c1] += v * one_minus_wgt_r * wgt_c;
                row2_buf[c2] += v * one_minus_wgt_r * one_minus_wgt_c;
              }
              else if (two_rows && one_col)
              {
                // single column, two row neighbors available
                const PixelScalar2D wgt = static_cast<PixelScalar2D>(proj_img_idx_pt[1] - r1);
                
                cur_proj_buf[(r1 * proj_num_cols) + c1] += v * wgt;
                cur_proj_buf[(r2 * proj_num_cols) + c1] += v * (1 - wgt);
              }
              else if (two_cols && one_row)
              {
                // single row, two column neighbors available
                const PixelScalar2D wgt = static_cast<PixelScalar2D>(proj_img_idx_pt[0] - c1);

                PixelScalar2D* row_buf = &cur_proj_buf[r1 * proj_num_cols];

                row_buf[c1] += v * wgt;
                row_buf[c2] += v * (1 - wgt);
              }
              // else single pixel, which would imply a single pixel image... this
              // is an extreme case - not handling.
            }
          }
          else
          {
            // do NN
            const long proj_col = std::lround(proj_img_idx_pt[0]);
            const long proj_row = std::lround(proj_img_idx_pt[1]);
            
            if ((proj_col >= 0) && (proj_col < static_cast<long>(proj_num_cols)) &&
                (proj_row >= 0) && (proj_row < static_cast<long>(proj_num_rows)))
            {
              ++num_inds_in_cur_proj;

              const PixelScalar2D v = vol_interp->EvaluateAtContinuousIndex(cur_itk_idx);
              cur_proj_buf[(proj_row * proj_num_cols) + proj_col] += v;
            }
          }
        }  // for num wobbles
      }
      else  // no wobble
      {
        tmp_vol_idx[0] = src_itk_idx[0];
        tmp_vol_idx[1] = src_itk_idx[1];
        tmp_vol_idx[2] = src_itk_idx[2];
       
        const Pt3 proj_img_idx_pt =
                            cam.phys_pt_to_ind_pt(vol_idx_to_cam_world * tmp_vol_idx);

        if (bilinear_in_2d_)
        {
          const int cf = static_cast<int>(std::floor(proj_img_idx_pt[0]));
          const int rf = static_cast<int>(std::floor(proj_img_idx_pt[1]));
          const int c1 = std::max(0,cf);
          const int r1 = std::max(0,rf);
          const int c2 = std::min(static_cast<int>(proj_num_cols-1), cf + 1);
          const int r2 = std::min(static_cast<int>(proj_num_rows-1), rf + 1);

          const int nr = r2 - r1 + 1;
          const int nc = c2 - c1 + 1;

          if (nr && nc)
          {
            ++num_inds_in_cur_proj;

            const PixelScalar2D& v = vol->GetPixel(src_itk_idx);
           
            const bool two_rows = nr == 2;
            const bool two_cols = nc == 2;
            const bool one_row  = nr == 1;
            const bool one_col  = nc == 1;
            
            if (two_rows && two_cols)
            {
              // standard case, all 4 neighbors available
              
              PixelScalar2D* row1_buf = &cur_proj_buf[r1 * proj_num_cols];
              PixelScalar2D* row2_buf = &cur_proj_buf[r2 * proj_num_cols];

              const PixelScalar2D wgt_r = static_cast<PixelScalar2D>(proj_img_idx_pt[1] - r1);
              const PixelScalar2D wgt_c = static_cast<PixelScalar2D>(proj_img_idx_pt[0] - c1);
              const PixelScalar2D one_minus_wgt_r = 1 - wgt_r;
              const PixelScalar2D one_minus_wgt_c = 1 - wgt_c;

              row1_buf[c1] += v * wgt_r * wgt_c;
              row1_buf[c2] += v * wgt_r * one_minus_wgt_c;
              row2_buf[c1] += v * one_minus_wgt_r * wgt_c;
              row2_buf[c2] += v * one_minus_wgt_r * one_minus_wgt_c;
            }
            else if (two_rows && one_col)
            {
              // single column, two row neighbors available
              const PixelScalar2D wgt = static_cast<PixelScalar2D>(proj_img_idx_pt[1] - r1);
              
              cur_proj_buf[(r1 * proj_num_cols) + c1] += v * wgt;
              cur_proj_buf[(r2 * proj_num_cols) + c1] += v * (1 - wgt);
            }
            else if (two_cols && one_row)
            {
              // single row, two column neighbors available
              const PixelScalar2D wgt = static_cast<PixelScalar2D>(proj_img_idx_pt[0] - c1);

              PixelScalar2D* row_buf = &cur_proj_buf[r1 * proj_num_cols];

              row_buf[c1] += v * wgt;
              row_buf[c2] += v * (1 - wgt);
            }
            // else single pixel, which would imply a single pixel image... this
            // is an extreme case - not handling.
          }
        }
        else
        {
          // just do NN
          const long proj_col = std::lround(proj_img_idx_pt[0]);
          const long proj_row = std::lround(proj_img_idx_pt[1]);
          
          if ((proj_col >= 0) && (proj_col < static_cast<long>(proj_num_cols)) &&
              (proj_row >= 0) && (proj_row < static_cast<long>(proj_num_rows)))
          {
            ++num_inds_in_cur_proj;
            
            cur_proj_buf[(proj_row * proj_num_cols) + proj_col] +=
                                                          vol->GetPixel(src_itk_idx);
          }
        }
      }
    }

    if (perform_wobble)
    {
      // average number of indices projecting in
      num_inds_in_cur_proj /= num_wobbles_;
    }

    if (post_splat_2D_smooth_)
    {
      // smooth the projection
      cv::Mat_<PixelScalar2D> p(proj_num_rows, proj_num_cols, cur_proj_buf);
      
      // NOTE: This copy could be avoided
      p.copyTo(tmp_img_for_smooth_);
      
      cv::GaussianBlur(tmp_img_for_smooth_, p,
                       cv::Size(post_splat_2D_smooth_win_size_x_,
                                post_splat_2D_smooth_win_size_y_), 0);
    }
  }

  this->sync_to_ocl_.set_modified();
  this->sync_to_host_.set_modified();
}

xreg::SplatLineIntCPU::ProjPtr
xreg::SplatLineIntCPU::proj(const size_type proj_idx)
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

cv::Mat xreg::SplatLineIntCPU::proj_ocv(const size_type proj_idx)
{
  const auto& cam = this->camera_models_[this->cam_model_for_proj_[proj_idx]];

  return cv::Mat(cam.num_det_rows, cam.num_det_cols, cv::DataType<PixelScalar2D>::type,
                 pixel_buf_to_use() + (cam.num_det_rows * cam.num_det_cols * proj_idx));
}

const xreg::CoordScalarList&
xreg::SplatLineIntCPU::num_inds_in_proj() const
{
  return num_inds_in_proj_;
}

xreg::SplatLineIntCPU::PixelScalar2D*
xreg::SplatLineIntCPU::raw_host_pixel_buf()
{
  return pixel_buf_to_use();
}

void xreg::SplatLineIntCPU::use_external_host_pixel_buf(void* buf)
{
  ext_pixel_buf_ = static_cast<PixelScalar2D*>(buf);
}

xreg::size_type xreg::SplatLineIntCPU::max_num_projs_possible() const
{
  return ~size_type(0);
}

void xreg::SplatLineIntCPU::use_other_proj_buf(RayCaster* other_ray_caster)
{
  throw UnsupportedOperationException();
}

xreg::RayCastSyncOCLBuf*
xreg::SplatLineIntCPU::to_ocl_buf()
{
  return &sync_to_ocl_;
}

xreg::RayCastSyncHostBuf*
xreg::SplatLineIntCPU::to_host_buf()
{
  return &sync_to_host_;
}

void xreg::SplatLineIntCPU::set_use_all_vol_inds(const bool use_all)
{
  proj_all_inds_ = use_all;
}

void xreg::SplatLineIntCPU::set_inds_to_proj(const VolIndList& inds, const size_type vol_idx,
                                             const bool use_these_inds)
{
  proj_all_inds_ = !use_these_inds;
  vols_custom_inds_to_proj_[vol_idx] = inds;
}

void xreg::SplatLineIntCPU::set_num_wobbles(const size_type num_wobbles)
{
  num_wobbles_ = num_wobbles;
}

void xreg::SplatLineIntCPU::set_do_post_splat_2D_smooth(const bool do_smooth)
{
  post_splat_2D_smooth_ = do_smooth;
}

void xreg::SplatLineIntCPU::set_post_splat_2D_smooth_y_radius(const size_type smooth_radius)
{
  post_splat_2D_smooth_win_size_y_ = smooth_radius;
}

void xreg::SplatLineIntCPU::set_post_splat_2D_smooth_x_radius(const size_type smooth_radius)
{
  post_splat_2D_smooth_win_size_x_ = smooth_radius;
}
  
bool xreg::SplatLineIntCPU::bilinear_in_2d() const
{
  return bilinear_in_2d_;
}

void xreg::SplatLineIntCPU::set_bilinear_in_2d(const bool b)
{
  bilinear_in_2d_ = b;
}

xreg::SplatLineIntCPU::PixelScalar2D* xreg::SplatLineIntCPU::pixel_buf_to_use()
{
  return ext_pixel_buf_ ? ext_pixel_buf_ : &pixel_buf_[0];
}

void xreg::SplatLineIntCPU::pre_compute()
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
  case RayCaster::kRAY_CAST_PIXEL_REPLACE:
    if (!this->use_bg_projs_)  // do not overwrite with zeros if we're using a background imge that has been set
    {
      memset(this->pixel_buf_to_use(), 0, sizeof(PixelScalar2D) * num_tot_pix);
    }
    break;
  case RayCaster::kRAY_CAST_PIXEL_ACCUM:
  default:
    // do nothing, keep previous value
    break;
  }
}
  
void xreg::SplatLineIntCPU::vols_changed()
{
  RayCaster::vols_changed();

  const size_type nv = this->vols_.size();

  vols_all_inds_.resize(nv);
  vols_all_inds_valid_.assign(nv, false);

  vols_custom_inds_to_proj_.resize(nv);

  vol_interp_fns_.resize(nv);
  for (size_type v = 0; v < nv; ++v)
  {
    vol_interp_fns_[v] = VolLinearInterpType::New();
    vol_interp_fns_[v]->SetInputImage(this->vols_[v]);
  }
} 

void xreg::SplatLineIntCPU::set_all_inds(const size_type vol_idx)
{
  if (!vols_all_inds_valid_[vol_idx])
  {
    VolIndList& vol_inds = vols_all_inds_[vol_idx];

    const auto vol_size = this->vols_[vol_idx]->GetLargestPossibleRegion().GetSize();

    vol_inds.clear();
    vol_inds.reserve(vol_size[0] * vol_size[1] * vol_size[2]);

    VolInd tmp_ind;

    // ITK is row major, this should be an efficient ordering
    for (size_type k = 0; k < vol_size[2]; ++k)
    {
      tmp_ind[2] = k;
      
      for (size_type j = 0; j < vol_size[1]; ++j)
      {
        tmp_ind[1] = j;
        
        for (size_type i = 0; i < vol_size[0]; ++i)
        {
          tmp_ind[0] = i;
          vol_inds.push_back(tmp_ind);
        }
      }
    }

    vols_all_inds_valid_[vol_idx] = true;
  }
}
