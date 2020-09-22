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

#include "xregProj3DLabelsTo2D.h"

#include "xregAssert.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKLabelUtils.h"
#include "xregProjData.h"

namespace // un-named
{

using namespace xreg;

using DepthScalar = RayCaster::PixelScalar2D;

itk::Image<DepthScalar,3>::Pointer
ConvertLabelVolToFloatMask(itk::Image<unsigned char,3>* lv)
{
  auto fv = CastITKImageIfNeeded<DepthScalar>(lv);

  const auto vol_size = fv->GetLargestPossibleRegion().GetSize();

  const size_type num_vox = static_cast<size_type>(vol_size[0]) *
                            static_cast<size_type>(vol_size[1]) *
                            static_cast<size_type>(vol_size[2]);

  auto* buf = fv->GetBufferPointer();

  std::transform(buf, buf + num_vox, buf,
                 [] (const DepthScalar& p)
                 {
                   return (std::abs(p) > 1.0e-6) ? 1 : 0;
                 });

  return fv;
}

void ConvertDepthToInter(cv::Mat* depth_map)
{
  const int nr = depth_map->rows;
  const int nc = depth_map->cols;

  for (int r = 0; r < nr; ++r)
  {
    auto* cur_row = &depth_map->at<DepthScalar>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      cur_row[c] = (cur_row[c] < kRAY_CAST_MAX_DEPTH) ? 1 : 0;
    }
  }
}

void NormalizeAtInters(cv::Mat* seg_channel, const cv::Mat& inter_img)
{
  const int nr = seg_channel->rows;
  const int nc = seg_channel->cols;

  for (int r = 0; r < nr; ++r)
  {
    auto* cur_seg_row = &seg_channel->at<DepthScalar>(r,0);
    
    const auto* cur_inter_row = &inter_img.at<DepthScalar>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      const auto& num_inters = cur_inter_row[c];

      if (num_inters > 1.0e-8)
      {
        cur_seg_row[c] /= num_inters;
      }
    }
  }
}

void IntersToBGMask(cv::Mat* inter_img)
{
  const int nr = inter_img->rows;
  const int nc = inter_img->cols;

  for (int r = 0; r < nr; ++r)
  {
    auto* cur_row = &inter_img->at<DepthScalar>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      cur_row[c] = (std::abs(cur_row[c]) < 1.0e-8) ? 1 : 0;
    }
  }
}

}  // un-named

void xreg::Proj3DLabelsTo2D::init()
{
  dout() << "initializing 3D label projector..." << std::endl;

  dout() << "extracting sub-label vols..." << std::endl;
  auto label_vols = MakeVolListFromVolAndLabels(label_vol.GetPointer(), label_vol.GetPointer(),
                                                labels_to_proj, 0);

  dout() << "converting label vols to float..." << std::endl;
  RayCaster::VolList label_vols_as_float;
  label_vols_as_float.reserve(label_vols.size());

  for (auto& lv : label_vols)
  {
    label_vols_as_float.push_back(ConvertLabelVolToFloatMask(lv.GetPointer()));
  }

  dout() << "setting up ray caster params..." << std::endl;
  {
    auto* rc = dynamic_cast<RayCasterCollisionParamInterface*>(depth_ray_caster.get());
    xregASSERT(rc);

    rc->set_render_thresh(0.5);
  }

  depth_ray_caster->set_num_projs(1);
  depth_ray_caster->set_ray_step_size(0.25);
  depth_ray_caster->use_proj_store_replace_method();
  depth_ray_caster->set_camera_model(cam);
  depth_ray_caster->set_volumes(label_vols_as_float);

  dout() << "allocating ray caster resources..." << std::endl;
  depth_ray_caster->allocate_resources();
 
  dout() << "setting up intermediate storage..." << std::endl;

  seg_channels.resize(labels_to_proj.size() + 1);
  seg_channels[0] = cv::Mat(cam.num_det_rows, cam.num_det_cols, cv::DataType<DepthScalar>::type);

  if (convert_to_single_chan_seg_fn)
  {
    single_chan_seg = MakeImageU8FromCam(cam);
  }
}

void xreg::Proj3DLabelsTo2D::run()
{
  dout() << "projecting labels..." << std::endl;
  
  const size_type num_non_bg_labels = labels_to_proj.size();

  for (size_type non_bg_label = 0; non_bg_label < num_non_bg_labels; ++non_bg_label)
  {
    dout() << "  depth computation for object: " << non_bg_label << std::endl;

    depth_ray_caster->xform_cam_to_itk_phys(0) = obj_poses[non_bg_label];
    
    depth_ray_caster->compute(non_bg_label);

    // +1 to skip bg channel  
    seg_channels[non_bg_label+1] = depth_ray_caster->proj_ocv(0).clone();
  }

  if (convert_to_single_chan_seg_fn)
  {
    dout() << "  running custom rule for single-channel seg..." << std::endl;
    convert_to_single_chan_seg_fn(seg_channels, single_chan_seg.GetPointer());
    dout() << "    done." << std::endl;
  }
    
  dout() << "  setting intersections counter to 0" << std::endl;
  
  auto& num_inters = seg_channels[0];
  num_inters.setTo(0);
 
  dout() << "  converting depths to binary intersects..." << std::endl;
  
  for (size_type non_bg_label = 0; non_bg_label < num_non_bg_labels; ++non_bg_label)
  {
    ConvertDepthToInter(&seg_channels[non_bg_label+1]);
    
    num_inters += seg_channels[non_bg_label+1];
  }

  dout() << "  normalizing intersected locations..." << std::endl;
  
  for (size_type non_bg_label = 0; non_bg_label < num_non_bg_labels; ++non_bg_label)
  {
    NormalizeAtInters(&seg_channels[non_bg_label+1], num_inters);
  }
  
  dout() << "  converting intersections to BG channel..." << std::endl;
  
  IntersToBGMask(&num_inters);
}

