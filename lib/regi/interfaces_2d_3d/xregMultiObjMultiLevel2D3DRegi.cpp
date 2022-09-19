/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

#include "xregMultiObjMultiLevel2D3DRegi.h"

#include "xregTBBUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKResampleUtils.h"
#include "xregExceptionUtils.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"
#include "xregTimer.h"

xreg::MultiLevelMultiObjRegi::RefFrameInfo
xreg::MultiLevelMultiObjRegi::StaticRefFrame::get(const MultiLevelMultiObjRegi*) const
{
  return info;
};
    
xreg::FrameTransform
xreg::MultiLevelMultiObjRegi::CamAlignRefFrameWithCurPose::compute_inter(const FrameTransform& cam_to_vol) const
{
  const Pt3 center_of_rot_wrt_cam_cur_est = cam_extrins * cam_to_vol.inverse() * center_of_rot_wrt_vol;

  FrameTransform cam_shift_center_of_rot = FrameTransform::Identity();
  cam_shift_center_of_rot.matrix().block(0,3,3,1) = -center_of_rot_wrt_cam_cur_est;

  return cam_shift_center_of_rot * cam_extrins;
}

xreg::MultiLevelMultiObjRegi::RefFrameInfo
xreg::MultiLevelMultiObjRegi::CamAlignRefFrameWithCurPose::get(
                                  const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const
{
  return { compute_inter(multi_level_multi_obj_regi->cur_cam_to_vols[vol_idx]), false, nullptr };
};

xreg::MultiLevelMultiObjRegi::RefFrameInfo
xreg::MultiLevelMultiObjRegi::DynVolRefFrame::get(const MultiLevelMultiObjRegi*) const
{
  const size_type dyn_vol_idx       = this->dyn_vol_idx;
  const FrameTransform inter_to_vol = this->inter_to_vol;
  const FrameTransform vol_to_inter = inter_to_vol.inverse();
   
  auto dyn_ref_frame_fn = [dyn_vol_idx,inter_to_vol,vol_to_inter]
                              (const size_type cur_vol_idx,
                               const Intensity2D3DRegi::ListOfFrameTransformLists& src_vol_xforms,
                               const Intensity2D3DRegi::ListOfFrameTransformLists& cur_dst_vol_xforms,
                               const FrameTransformList& inter_frames,
                               const std::vector<bool>& inter_frames_wrt_vol,
                               const FrameTransformList& regi_xform_guesses)
  {
    const size_type nv = src_vol_xforms.size();
    xregASSERT(nv > std::max(cur_vol_idx, dyn_vol_idx));
    xregASSERT(nv == cur_dst_vol_xforms.size());

    const auto& cur_vol_src_xforms = src_vol_xforms[cur_vol_idx];
    const auto& dyn_vol_dst_xforms = cur_dst_vol_xforms[dyn_vol_idx];

    const size_type num_xforms = cur_vol_src_xforms.size();
    xregASSERT(dyn_vol_dst_xforms.size() == num_xforms);

    FrameTransformList dst_xforms(num_xforms);

    auto compute_xforms = [&dst_xforms,&inter_to_vol,&vol_to_inter,
                           &cur_vol_src_xforms,&dyn_vol_dst_xforms] (const RangeType& r)
    {
      for (size_type xform_idx = r.begin(); xform_idx < r.end(); ++xform_idx)
      {
        dst_xforms[xform_idx] = inter_to_vol * cur_vol_src_xforms[xform_idx] * vol_to_inter * dyn_vol_dst_xforms[xform_idx];
      }
    };
    ParallelFor(compute_xforms, RangeType(0, num_xforms));

    return dst_xforms;
  };

  return { FrameTransform::Identity(), false, dyn_ref_frame_fn };
}

std::shared_ptr<xreg::MultiLevelMultiObjRegi::StaticRefFrame>
xreg::MultiLevelMultiObjRegi::MakeStaticRefFrame(const FrameTransform& ref_frame, const bool maps_vol_to_ref)
{
  auto srf = std::make_shared<MultiLevelMultiObjRegi::StaticRefFrame>();

  srf->info.ref_frame       = ref_frame;
  srf->info.maps_vol_to_ref = maps_vol_to_ref;

  return srf;
}

std::shared_ptr<xreg::MultiLevelMultiObjRegi::StaticRefFrame>
xreg::MultiLevelMultiObjRegi::MakeIdRefFrame()
{
  return MakeStaticRefFrame(FrameTransform::Identity(), true);
}

xreg::FrameTransform
xreg::MultiLevelMultiObjRegi::Level::SingleRegi::InitPoseId::get(
                                              const MultiLevelMultiObjRegi*) const
{
  return FrameTransform::Identity();
}

xreg::FrameTransform
xreg::MultiLevelMultiObjRegi::Level::SingleRegi::InitPosePrevPoseEst::get(
                    const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const
{
  return multi_level_multi_obj_regi->cur_cam_to_vols[vol_idx];
}

xreg::FrameTransform
xreg::MultiLevelMultiObjRegi::Level::SingleRegi::InitPosePrevPoseEstTransOnly::get(
                    const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const
{
  FrameTransform xform = multi_level_multi_obj_regi->cur_cam_to_vols[vol_idx];
  xform.matrix().block(0,0,3,3).setIdentity();

  return xform;
}

xreg::FrameTransform
xreg::MultiLevelMultiObjRegi::Level::SingleRegi::InitPosePrevPoseEstRotOnly::get(
                    const MultiLevelMultiObjRegi* multi_level_multi_obj_regi) const
{
  FrameTransform xform = multi_level_multi_obj_regi->cur_cam_to_vols[vol_idx];
  xform.matrix().block(0,3,3,1).setZero();

  return xform;
}
  
void xreg::MultiLevelMultiObjRegi::set_save_debug_info(const bool save_debug_info)
{
  if (save_debug_info)
  {
    debug_info = std::make_shared<DebugRegiResultsMultiLevel>();
  }
  else
  {
    debug_info = nullptr;
  }
}

void xreg::MultiLevelMultiObjRegi::run()
{
  Timer tmr;
  tmr.start();

  bool save_debug_info = debug_info.get();

  dout() << "Saving debug info: " << (save_debug_info ? "YES" : "NO") << std::endl;

  const size_type num_levels = levels.size();
  
  dout() << "Num Levels: " << num_levels << std::endl;

  if (save_debug_info)
  {
    debug_info->multi_res_levels.resize(num_levels);
    debug_info->regi_results.resize(num_levels);
  }

  const size_type num_fixed_imgs = fixed_proj_data.size();

  dout() << "Num Fixed Images: " << num_fixed_imgs << std::endl;

  const bool masks_provided = !masks_2d.empty();

  xregASSERT(!masks_provided || (masks_2d.size() == num_fixed_imgs));

  xregASSERT(init_cam_to_vols.size() == vols.size());

  cur_cam_to_vols = init_cam_to_vols;

  for (size_type lvl_idx = 0; lvl_idx < num_levels; ++lvl_idx)
  {
    dout() << "Starting level: " << lvl_idx << std::endl;

    Level& lvl = levels[lvl_idx];
    
    const size_type num_regis = lvl.regis.size();

    dout() << "number of regis at this level: " << num_regis << std::endl;

    // TODO: pre-compute the downsampling prior to looping through the levels
    
    const size_type num_fixed_imgs_this_level = lvl.fixed_imgs_to_use.size();
   
    dout() << "number of fixed images at this level: " << num_fixed_imgs_this_level << std::endl;
    
    if (save_debug_info)
    {
      debug_info->multi_res_levels[lvl_idx] = lvl.ds_factor;
      debug_info->regi_results[lvl_idx].resize(num_regis);
    }

    ProjDataF32List ds_proj_data(num_fixed_imgs_this_level);
    Mask2DList ds_masks_2d(num_fixed_imgs_this_level);

    for (size_type fixed_idx = 0; fixed_idx < num_fixed_imgs_this_level; ++fixed_idx)
    {
      const size_type global_fixed_idx = lvl.fixed_imgs_to_use[fixed_idx];
      dout() << "downsampling global fixed image: " << global_fixed_idx << std::endl;

      // Verify that the metadata in the projection matches that of the camera model
      {
        const auto& cam = fixed_proj_data[global_fixed_idx].cam;

        const auto img_spacings = fixed_proj_data[global_fixed_idx].img->GetSpacing();

        if (std::abs(img_spacings[0] - cam.det_col_spacing) > 1.0e-6)
        {
          xregThrow("ERROR: mismatch between image object and camera model column spacings! Image: %.4f, Cam: %.4f", img_spacings[0], cam.det_col_spacing);
        }
        
        if (std::abs(img_spacings[1] - cam.det_row_spacing) > 1.0e-6)
        {
          xregThrow("ERROR: mismatch between image object and camera model row spacings! Image: %.4f, Cam: %.4f", img_spacings[1], cam.det_row_spacing);
        }

        const auto img_size = fixed_proj_data[global_fixed_idx].img->GetLargestPossibleRegion().GetSize();

        if (img_size[0] != cam.num_det_cols)
        {
          xregThrow("ERROR: mismatch between image object and camera model number of rows! Image: %lu, Cam: %lu", img_size[0], cam.num_det_cols);
        }
        
        if (img_size[1] != cam.num_det_rows)
        {
          xregThrow("ERROR: mismatch between image object and camera model number of rows! Image: %lu, Cam: %lu", img_size[1], cam.num_det_rows);
        }
      }

      ds_proj_data[fixed_idx] = DownsampleProjData(fixed_proj_data[global_fixed_idx], lvl.ds_factor);

      if (masks_provided)
      {
        auto& cur_mask_to_ds = masks_2d[global_fixed_idx];

        if (cur_mask_to_ds)
        {
          dout() << "downsampling global fixed mask: " << global_fixed_idx << std::endl;

          ds_masks_2d[fixed_idx] = DownsampleBinaryImage(cur_mask_to_ds.GetPointer(),
                                                         lvl.ds_factor);
        }
      }
    }
    
    // determine the maximum number of moving images
    size_type max_num_mov_imgs = 0;
    
    // determine if bg projs are needed.
    bool need_bg_projs = false;

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      Level::SingleRegi& single_regi = lvl.regis[regi_idx];

      if (!single_regi.static_vols.empty())
      {
        need_bg_projs = true;
        dout() << "Regi " << regi_idx << " uses " << single_regi.static_vols.size()
               << " static vols..." << std::endl;
      }

      const size_type max_num_projs_for_this_regi = lvl.regis[regi_idx].regi->max_num_projs_per_view_per_iter() *
                                                      num_fixed_imgs_this_level;
      dout() << "max num projs needed by this regi: " << max_num_projs_for_this_regi << std::endl;

      max_num_mov_imgs = std::max(max_num_mov_imgs, max_num_projs_for_this_regi);
    }

    dout() << "max num projs needed to alloc ray caster: " << max_num_mov_imgs << std::endl;

    RayCaster& ray_caster = *lvl.ray_caster;

    ray_caster.set_num_projs(max_num_mov_imgs);
    ray_caster.set_use_bg_projs(need_bg_projs);
    
    if (ray_caster_needs_resources_alloc)
    {
      // TODO: create a subset of the volumes that are required at this level
      ray_caster.set_volumes(vols);
      
      ray_caster.set_camera_models(ExtractCamModels(ds_proj_data));
      
      dout() << "ray caster allocating resources..." << std::endl; 
      ray_caster.allocate_resources();
    }

    dout() << "setting up sim metrics for each view..." << std::endl;
    for (size_type fixed_idx = 0; fixed_idx < num_fixed_imgs_this_level; ++fixed_idx)
    {
      dout() << "view " << fixed_idx << std::endl;

      lvl.sim_metrics[fixed_idx]->set_num_moving_images(max_num_mov_imgs);

      if (sim_metrics_need_resources_alloc)
      {
        lvl.sim_metrics[fixed_idx]->set_fixed_image(ds_proj_data[fixed_idx].img);
      }
      
      dout() << "connecting ray caster buffer to sim metric..." << std::endl;
      lvl.sim_metrics[fixed_idx]->set_mov_imgs_buf_from_ray_caster(&ray_caster,
                                                                   max_num_mov_imgs * fixed_idx);

      if (sim_metrics_need_resources_alloc)
      {
        if (masks_provided && ds_masks_2d[fixed_idx])
        {
          dout() << "setting mask for sim metric..." << std::endl;
          lvl.sim_metrics[fixed_idx]->set_mask(ds_masks_2d[fixed_idx]);
        }

        dout() << "allocating resources..." << std::endl;
        lvl.sim_metrics[fixed_idx]->allocate_resources();
      }
    }

    // run each regi
    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      dout() << "regi: " << regi_idx << std::endl;

      Level::SingleRegi& single_regi = lvl.regis[regi_idx];
     
      const size_type num_mov_vols = single_regi.mov_vols.size();
      dout() << "num moving vols: " << num_mov_vols << std::endl;
      dout() << "  [ ";
      for (const auto& m : single_regi.mov_vols)
      {
        dout() << m << ' ';
      }
      dout() << "]" << std::endl;

      const size_type num_static_vols = single_regi.static_vols.size();
      dout() << "num static vols: " << num_static_vols << std::endl;

      FrameTransformList static_vol_poses_used;

      // setup background images (if needed)
      if (num_static_vols > 0)
      {
        dout() << "computing bg projs with static vols..." << std::endl;

        const auto& static_vol_poses = single_regi.static_vol_poses;
        xregASSERT(static_vol_poses.size() == num_static_vols);

        dout() << "computing the poses used for each static object..." << std::endl;
        static_vol_poses_used.resize(num_static_vols);
        for (size_type static_idx = 0; static_idx < num_static_vols; ++static_idx)
        {
          static_vol_poses_used[static_idx] = static_vol_poses[static_idx]->get(this);
        }

        dout() << "first static vol... global index: " << single_regi.static_vols[0] << std::endl;
        ray_caster.set_use_bg_projs(false);
        ray_caster.use_proj_store_replace_method();
        ray_caster.set_num_projs(num_fixed_imgs_this_level);
        ray_caster.distribute_xform_among_cam_models(static_vol_poses_used[0]);

        dout() << "computing..." << std::endl;
        ray_caster.compute(single_regi.static_vols[0]);

        for (size_type static_idx = 1; static_idx < num_static_vols; ++static_idx)
        {
          dout() << "static vol " << static_idx << " global index: "
                 << single_regi.static_vols[static_idx] << std::endl;

          ray_caster.use_proj_store_accum_method();
          ray_caster.distribute_xform_among_cam_models(static_vol_poses_used[static_idx]);
       
          dout() << "computing..." << std::endl; 
          ray_caster.compute(single_regi.static_vols[static_idx]);
        }

        // need to copy the bg images out of the ray caster's buffer, so they will not be
        // overwritten
        
        dout() << "deep copying bg projs from ray caster..." << std::endl;

        RayCaster::ProjList bg_imgs(num_fixed_imgs_this_level);
        for (size_type fixed_idx = 0; fixed_idx < num_fixed_imgs_this_level; ++fixed_idx)
        {
          bg_imgs[fixed_idx] = ITKImageDeepCopy(ray_caster.proj(fixed_idx).GetPointer());
        }

        dout() << "setting bg projs into ray caster..." << std::endl;
        ray_caster.set_use_bg_projs(true);
        ray_caster.set_bg_projs(bg_imgs);
      }
      else
      {
        dout() << "not using bg projs..." << std::endl;
        ray_caster.set_use_bg_projs(false);
        ray_caster.use_proj_store_replace_method();
      }

      auto& regi = *single_regi.regi;

      regi.set_has_a_static_vol(num_static_vols > 0);

      regi.set_debug_save_iter_debug_info(save_debug_info);

      dout() << "setting ray caster in regi obj" << std::endl;
      regi.set_ray_caster(lvl.ray_caster, single_regi.mov_vols, false);
      
      dout() << "setting sim metrics in regi obj" << std::endl;
      regi.set_sim_metrics(lvl.sim_metrics, false);
   
      dout() << "calling regi setup()..." << std::endl; 
      regi.setup();

      // set initial pose estimates and reference frames
      dout() << "setting initial pose estimates..." << std::endl;
      for (size_type mov_vol_idx = 0; mov_vol_idx < num_mov_vols; ++mov_vol_idx)
      {
        dout() << "current mov vol: " << mov_vol_idx << ", global vol: "
               << single_regi.mov_vols[mov_vol_idx] << std::endl;

        regi.set_regi_xform_guess(single_regi.init_mov_vol_poses[mov_vol_idx]->get(this),
                                  mov_vol_idx);

        if (!single_regi.ref_frames.empty())
        {
          const size_type ref_frame_idx = single_regi.ref_frames[mov_vol_idx];
          if (ref_frame_idx != Level::SingleRegi::kNO_REF_FRAME_USED)
          {
            dout() << "setting reference frame..." << std::endl;
              
            const auto ref_frame_info = ref_frames[ref_frame_idx]->get(this);
   
            if (!ref_frame_info.dyn_ref_frame_fn)
            { 
              // maps_vol_to_ref == true:
              // the registration interface assumes that an intermediate frame on the
              // volume coordinate frame maps from the intermediate frame to the volume,
              // while the multi-level higher level interface is designed for the
              // intermediate frame to map from the volume to the intermediate frame.
              //
              // maps_vol_to_ref == false:
              // This higher level interface assumes that the intermediate frame maps
              // camera to intermediate frame, whereas the registration interface
              // assumes that the intermediate frame maps the intermediate frame to 
              // the camera frame.

              regi.set_intermediate_frame(ref_frame_info.ref_frame.inverse(),
                                          ref_frame_info.maps_vol_to_ref,
                                          mov_vol_idx);
            }
            else
            {
              regi.set_intermediate_frame(ref_frame_info.dyn_ref_frame_fn, mov_vol_idx);
            }
          }
        }
      }

      dout() << "last minute callbacks before running regi..." << std::endl;
      // The initial use for this was to setup regularization object parameters
      // after the registration object had guesses and intermediate frames all set
      for (auto& f : single_regi.fns_to_call_right_before_regi_run)
      {
        f();
      }

      dout() << "running regi..." << std::endl; 
      regi.run();

      // retrieve registration params
      dout() << "retrieving registration transforms..." << std::endl;
      for (size_type mov_vol_idx = 0; mov_vol_idx < num_mov_vols; ++mov_vol_idx)
      {
        cur_cam_to_vols[single_regi.mov_vols[mov_vol_idx]] = regi.regi_xform(mov_vol_idx);
      }

      // save debug info
      if (save_debug_info)
      {
        auto& dst_debug_info = debug_info->regi_results[lvl_idx][regi_idx];

        dst_debug_info = regi.debug_info();

        dst_debug_info->projs_used = lvl.fixed_imgs_to_use;

        dst_debug_info->do_not_use_static_vol_heuristic = true;

        if (num_static_vols > 0)
        {
          dst_debug_info->static_vols      = single_regi.static_vols;
          dst_debug_info->static_vol_poses = static_vol_poses_used;
        }
      }

      if (dealloc_resources)
      {
        single_regi.regi = nullptr;
      }
    }  // end for each regi
   
    if (dealloc_resources)
    {
      lvl.ray_caster = nullptr;
      lvl.sim_metrics.clear();
    }
  }  // end for each level
  
  tmr.stop();
  
  const double multi_level_run_time_secs = tmr.elapsed_seconds();

  dout() << "multi-level run time (seconds): " << multi_level_run_time_secs << std::endl;

  if (save_debug_info)
  {
    debug_info->multi_level_run_time_secs = multi_level_run_time_secs;
  }
}

