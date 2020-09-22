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

#include "xregEdgesFromRayCast.h"

#include <opencv2/imgproc.hpp>

#include "xregAssert.h"
#include "xregITKLabelUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregHUToLinAtt.h"

void xreg::EdgesFromRayCast::operator()()
{
  xregASSERT(do_canny || do_boundary || do_occ);

  auto common_init = [] (RayCaster* rc, std::vector<VolPtr>& vols, const CameraModel& cam)
  {
    if (rc)
    {
      rc->set_volumes(vols);
      rc->set_num_projs(1); 
      rc->set_ray_step_size(1);
      rc->set_camera_model(cam);
      rc->allocate_resources();
    }
  };
  
  std::vector<VolPtr> vols_to_use = { vol };

  if (label_vol)
  {
    xregASSERT(!obj_label_vals.empty());

    vols_to_use.clear();
    vols_to_use.reserve(obj_label_vals.size());

    for (const LabelScalar l : obj_label_vals)
    {
      this->dout() << "masking volume for object " << int(l) << "..." << std::endl;
      vols_to_use.push_back(ApplyMaskToITKImage(vol.GetPointer(), label_vol.GetPointer(),
                                                l, RayCaster::PixelScalar3D(-1000), true));
    }
  }
  
  const size_type num_vols = vols_to_use.size();

  xregASSERT(cam_wrt_vols.size() == num_vols);

  std::vector<VolPtr> att_vols;

  if (do_canny)
  {
    this->dout() << "canny init..." << std::endl;

    this->dout() << "HU -> Att..." << std::endl;

    att_vols.reserve(num_vols);

    for (const auto& v : vols_to_use)
    {
      auto hu2att = HUToLinAttFilter::New();
      hu2att->SetInput(v);
      hu2att->Update();

      att_vols.push_back(hu2att->GetOutput());
    }
  
    this->dout() << "line integral ray caster init..." << std::endl; 
    common_init(line_int_ray_caster.get(), att_vols, cam);
  }

  if (do_boundary)
  {
    this->dout() << "boundary ray caster init..." << std::endl;

    common_init(boundary_ray_caster.get(), vols_to_use, cam);
    
    auto* coll_rc = dynamic_cast<RayCasterCollisionParamInterface*>(boundary_ray_caster.get());
    xregASSERT(coll_rc);

    coll_rc->set_render_thresh(thresh);
  }

  if (do_occ)
  {
    this->dout() << "occluding contours ray caster init..." << std::endl;

    common_init(occ_ray_caster.get(), vols_to_use, cam);
    
    auto* cont_rc = dynamic_cast<RayCasterOccludingContours*>(occ_ray_caster.get());
    xregASSERT(cont_rc);

    cont_rc->set_render_thresh(thresh);
    cont_rc->set_occlusion_angle_thresh_deg(occ_ang_deg);
  }

  cv::Mat edge_img = cv::Mat::zeros(cam.num_det_rows,
                                    cam.num_det_cols,
                                    CV_8UC1);
  
  if (do_canny)
  {
    this->dout() << "computing DRR for canny edges..." << std::endl;
    
    line_int_ray_caster->use_proj_store_replace_method();
    
    for (size_type v = 0; v < num_vols; ++v)
    {
      line_int_ray_caster->distribute_xform_among_cam_models(cam_wrt_vols[v]);
      line_int_ray_caster->compute(v);
      line_int_ray_caster->use_proj_store_accum_method();
    }

    cv::Mat drr_8bpp = ShallowCopyItkToOpenCV(ITKImageRemap8bpp(
          line_int_ray_caster->proj(0).GetPointer()).GetPointer()).clone();
    
    if (canny_smooth_width > 1)
    {
      cv::GaussianBlur(drr_8bpp.clone(), drr_8bpp,
                       cv::Size(canny_smooth_width,canny_smooth_width), 0);
    } 
    
    cv::Canny(drr_8bpp, drr_8bpp, canny_low_thresh, canny_high_thresh); 
  
    cv::bitwise_or(edge_img, drr_8bpp, edge_img);
  }
  
  if (do_boundary)
  { 
    this->dout() << "computing depth map for boundary edges..." << std::endl;
    boundary_ray_caster->use_proj_store_replace_method();
    
    for (size_type v = 0; v < num_vols; ++v)
    {
      boundary_ray_caster->distribute_xform_among_cam_models(cam_wrt_vols[v]);
      boundary_ray_caster->compute(v);
      boundary_ray_caster->use_proj_store_accum_method();
    }
    
    cv::Mat boundary_edges;
    FindPixelsWithAdjacentIntensity(boundary_ray_caster->proj_ocv(0),
                                    &boundary_edges, kRAY_CAST_MAX_DEPTH);
    
    cv::bitwise_or(edge_img, boundary_edges, edge_img);
  }

  if (do_occ)
  {
    this->dout() << "computing occluding contours..." << std::endl;
    occ_ray_caster->use_proj_store_replace_method();
    
    for (size_type v = 0; v < num_vols; ++v)
    {
      occ_ray_caster->distribute_xform_among_cam_models(cam_wrt_vols[v]);
      occ_ray_caster->compute(v);
      occ_ray_caster->use_proj_store_accum_method();
    }
  
    cv::bitwise_or(edge_img, occ_ray_caster->proj_ocv(0), edge_img);
  }

  // change all non-zero values to 1
  for (int r = 0; r < edge_img.rows; ++r)
  {
    unsigned char* row_buf = &edge_img.at<unsigned char>(r,0);

    for (int c = 0; c < edge_img.cols; ++c)
    {
      if (row_buf[c] > 1)
      {
        row_buf[c] = 1;
      }
    }
  }

  if (edge_dilate_width > 1)
  {
    this->dout() << "dilating edges..." << std::endl;

    cv::dilate(edge_img.clone(), edge_img,
               cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                  cv::Size(edge_dilate_width,edge_dilate_width)));
  }
  
  final_edge_img = ITKImageDeepCopy(ShallowCopyOpenCVToItk<unsigned char>(edge_img).GetPointer());
}

