/*
 * MIT License
 *
 * Copyright (c) 2020-2026 Robert Grupp
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

#include "xregRegi2D3DReplay.h"

#include <fmt/format.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregOpenCVUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregITKRemapUtils.h"
#include "xregWriteVideo.h"
#include "xregProjPreProc.h"
#include "xregRayCastInterface.h"
#include "xregSE3OptVars.h"
#include "xregRegi2D3DPenaltyFnLandReproj.h"
#include "xregRegi2D3DPenaltyFnCombo.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"

namespace  // un-named
{

using namespace xreg;

using IndexList = DebugRegiResultsMultiLevel::IndexList;

using VolList = RayCaster::VolList;

using ProjList                    = RayCaster::ProjList;
using ListOfProjLists             = std::vector<ProjList>;
using ListOfListOfProjLists       = std::vector<ListOfProjLists>;
using ListOfListOfListOfProjLists = std::vector<ListOfListOfProjLists>;

using Proj8bpp                        = itk::Image<unsigned char,2>;
using Proj8bppPtr                     = Proj8bpp::Pointer;
using Proj8bppList                    = std::vector<Proj8bppPtr>;
using ListOfProj8bppLists             = std::vector<Proj8bppList>;
using ListOfListOfProj8bppLists       = std::vector<ListOfProj8bppLists>;
using ListOfListOfListOfProj8bppLists = std::vector<ListOfListOfProj8bppLists>;

using MatList                    = std::vector<cv::Mat>;
using ListOfMatLists             = std::vector<MatList>;
using ListOfListOfMatLists       = std::vector<ListOfMatLists>;
using ListOfListOfListOfMatLists = std::vector<ListOfListOfMatLists>;

void ComputeProjs(RayCaster& rc, const FrameTransformList& poses, const IndexList& vol_inds, ProjList& projs)
{
  const size_type num_views = projs.size();

  const size_type num_vols = vol_inds.size();
  
  const bool orig_ray_caster_use_bg_projs = rc.use_bg_projs();

  rc.use_proj_store_replace_method();

  for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
  {
    const size_type cur_obj_idx = vol_inds[vol_idx];
    xregASSERT(cur_obj_idx < poses.size());

    rc.distribute_xform_among_cam_models(poses[cur_obj_idx]);

    rc.compute(cur_obj_idx);

    rc.use_proj_store_accum_method();
    rc.set_use_bg_projs(false);
  }

  rc.set_use_bg_projs(orig_ray_caster_use_bg_projs);

  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    projs[view_idx] = ITKImageDeepCopy(rc.proj(view_idx).GetPointer());
  }
}

}  // un-named

void xreg::Regi2D3DReplay::run(const DebugRegiResultsMultiLevel& regi_results)
{
  if (!line_int_ray_caster_fn)
  {
    xregThrow("Regi2D3DReplay: line integral ray caster factory has not been set!");
  }

  if (compute_boundary_edges && !depth_ray_caster_fn)
  {
    xregThrow("Regi2D3DReplay: depth ray caster factory has not been set "
              "(required for boundary edges)!");
  }

  drr_frames.clear();
  edge_frames.clear();
  frame_titles.clear();

  num_views_     = 0;
  num_view_cols_ = 0;

  std::ostream& vout = dout();

  const double ds_factor = proj_ds_factor;

  const bool write_debug_img_frames = !debug_img_prefix.empty();

  const bool write_raw_final_drr = !debug_raw_final_drr_prefix.empty();

  vout << "reading volumes (and converting HU --> lin. att.) for playback..." << std::endl;
  
  VolList vols;
  VolList hu_vols;
  std::tie(hu_vols,vols) = VolDataFromDebug(regi_results, true);
  
  if (!compute_boundary_edges)
  {
    // We do not require the HU volumes if we are not computing boundary edges,
    // free up the memory.
    // TODO: add a flag to VolDataFromDebug to not return this data
    hu_vols.clear();
  }

  vout << "reading proj data for playback..." << std::endl;
  auto fixed_proj_data = ProjDataFromDebug(regi_results);

  const size_type tot_num_views_in_proj_data = fixed_proj_data.size();
  vout << "fixed proj data data has " << tot_num_views_in_proj_data << " views" << std::endl;

  // check to see if a subset of views are used...
  size_type num_views_used = 0;

  // this is not ideal, but it gets us by for now
  // ensure that if a subset of views is specified, then the same views are used in all registrations
  {
    auto projs_used_are_same = [] (const IndexList& i1, const IndexList& i2)
    {
      bool are_same = true;
      
      const size_type nv = i1.size();

      if (nv == i2.size())
      {
        for (size_type j = 0; j < nv; ++j)
        {
          if (i1[j] != i2[j])
          {
            are_same = false;
            break;
          }
        }
      }
      else
      {
        are_same = false;
      }

      return are_same;
    };

    vout << "checking for subsets of views..." << std::endl;

    IndexList projs_used;
    
    for (const auto& lvl : regi_results.regi_results)
    {
      for (const auto& regi : lvl)
      {
        if (projs_used.empty())
        {
          projs_used = regi->projs_used;
        }
        else if (!projs_used_are_same(projs_used, regi->projs_used))
        {
          throw UnsupportedDataException("if subsets of views are used in registrations, they must be "
                                         "the same subsets (that is all that is supported for now... sorry).");
        }
      }
    }
    
    if (!projs_used.empty())
    {
      // there is a subset of views specified - prune the original list of projection data
      
      num_views_used = projs_used.size();
      
      vout << "subset of " << num_views_used << " found..." << std::endl;
      vout << "  [ ";
      for (size_type v = 0; v < num_views_used; ++v)
      {
        vout << projs_used[v];
        if (v < (num_views_used - 1))
        {
          vout << " , ";
        }
      }
      vout << " ]" << std::endl;


      decltype(fixed_proj_data) fixed_proj_data_to_use;
      fixed_proj_data_to_use.reserve(num_views_used);

      for (size_type v = 0; v < num_views_used; ++v)
      {
        fixed_proj_data_to_use.push_back(fixed_proj_data[projs_used[v]]);
      }
      
      std::swap(fixed_proj_data, fixed_proj_data_to_use);
    }
  }

  const size_type num_views = num_views_used ? num_views_used : tot_num_views_in_proj_data;
  
  vout << "num views used in this replay: " << num_views << std::endl;

  std::vector<int> flip_proj_rows(num_views, static_cast<int>(flip_rows));
  std::vector<int> flip_proj_cols(num_views, static_cast<int>(flip_cols));

  if (use_pat_rot_up)
  {
    vout << "checking proj data metadata for pat rot up flags..." << std::endl;
    
    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      const auto& cur_pd = fixed_proj_data[view_idx];
      
      if (cur_pd.rot_to_pat_up)
      {
        auto& flip_rows = flip_proj_rows[view_idx];
        auto& flip_cols = flip_proj_cols[view_idx];

        const auto rot_to_pat_up = *cur_pd.rot_to_pat_up;
        
        if (rot_to_pat_up == ProjDataRotToPatUp::kZERO)
        {
          flip_rows = false;
          flip_cols = false;
        }
        else if (rot_to_pat_up == ProjDataRotToPatUp::kONE_EIGHTY)
        {
          flip_rows = true;
          flip_cols = true;
        }
        else
        {
          vout << "  ignoring rot_to_pat_up value of: " << static_cast<int>(rot_to_pat_up) << std::endl;
        }
      }
    }
  }
  else
  {
    vout << "not checking for pat rot up metadata..." << std::endl;
  }

  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    vout << "View " << view_idx << ", flip rows: " << flip_proj_rows[view_idx]
         << ", flip cols: " << flip_proj_cols[view_idx] << std::endl;
  }

  if (do_preproc && regi_results.proj_pre_proc_info)
  {
    vout << "preprocessing projection data..." << std::endl;
   
    ProjPreProc pre_proc;
    pre_proc.set_debug_output_stream(*this);

    pre_proc.params = *regi_results.proj_pre_proc_info;

    pre_proc.input_projs = fixed_proj_data;
    
    pre_proc();

    fixed_proj_data = pre_proc.output_projs;
  }

  const int num_tile_cols = (max_tile_cols >= num_views) ? num_views : max_tile_cols;
  const int num_tile_rows  = static_cast<int>(std::ceil(static_cast<double>(num_views) / num_tile_cols));

  if (std::abs(1.0 - ds_factor) > 1.0e-6)
  {
    vout << "downsampling fixed images..." << std::endl;
    fixed_proj_data = DownsampleProjData(fixed_proj_data, ds_factor, true);
  }
  else
  {
    vout << "NO downsampling of fixed images!" << std::endl;
  }

  const size_type num_tot_objs = vols.size();

  const size_type tot_num_projs = regi_results.total_num_projs_per_view();
  vout << "total number of projections per view (in entire registration process): " << tot_num_projs << std::endl;

  const size_type num_levels = regi_results.multi_res_levels.size();

  // view_projs[v][l][r][k] is the kth projection of the rth registration of the lth level, at the vth view
  ListOfListOfListOfProjLists view_projs(num_views);
  ListOfListOfListOfProj8bppLists remapped_view_projs(num_views);
  ListOfListOfListOfMatLists view_cv_mats(num_views);
  ListOfListOfListOfMatLists edge_overlays_cv_mats(num_views);

  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    view_projs[view_idx].resize(num_levels);
    remapped_view_projs[view_idx].resize(num_levels);
    view_cv_mats[view_idx].resize(num_levels);
    edge_overlays_cv_mats[view_idx].resize(num_levels);
  }

  for (size_type lvl = 0; lvl < num_levels; ++lvl)
  {
    vout << "computing projections at level: " << lvl << std::endl;

    vout << "creating line integral ray caster..." << std::endl;
    auto ray_caster = line_int_ray_caster_fn();

    vout << "setting up camera parameters for ray caster..." << std::endl;
    ray_caster->set_num_projs(num_views);

    ray_caster->set_camera_models(ExtractCamModels(fixed_proj_data));

    vout << "setting ray caster volumes..." << std::endl;
    ray_caster->set_volumes(vols);

    // need this set to allocate resources that we'll probably need
    ray_caster->set_use_bg_projs(true);

    vout << "allocating ray caster resources..." << std::endl;
    ray_caster->allocate_resources();
    
    std::shared_ptr<RayCaster> depth_rend;
    MatList boundary_edge_maps;

    if (compute_boundary_edges)
    {
      vout << "creating depth ray caster..." << std::endl;
      depth_rend = depth_ray_caster_fn();

      vout << "setting depth ray caster params..." << std::endl;
      depth_rend->set_num_projs(num_views);
      depth_rend->set_camera_models(ray_caster->camera_models());
      depth_rend->set_volumes(hu_vols);
      depth_rend->set_use_bg_projs(true);

      auto* sr = dynamic_cast<RayCasterCollisionParamInterface*>(depth_rend.get());

      sr->set_render_thresh(boundary_edge_hu_thresh);
      sr->set_num_backtracking_steps(0);

      vout << "allocating depth ray caster resources..." << std::endl;
      depth_rend->allocate_resources();
    
      boundary_edge_maps.resize(num_views);
    }

    vout << "remapping fixed images... (for later edge overlay)" << std::endl;
    MatList remapped_fixed_views_ocv(num_views);
    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      remapped_fixed_views_ocv[view_idx] =
            ShallowCopyItkToOpenCV(ITKImageRemap8bpp(
                  fixed_proj_data[view_idx].img.GetPointer()).GetPointer()).clone();
    }

    const size_type num_regis = regi_results.regi_results[lvl].size();

    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      view_projs[view_idx][lvl].resize(num_regis);
      remapped_view_projs[view_idx][lvl].resize(num_regis);
      view_cv_mats[view_idx][lvl].resize(num_regis);
      edge_overlays_cv_mats[view_idx][lvl].resize(num_regis);
    }

    std::vector<size_type> global_vol_inds_registered;
    FrameTransformList global_vol_last_poses(num_tot_objs, FrameTransform::Identity());

    FrameTransformList poses_for_land_reproj;

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      vout << "---- computing projections for registration " << regi_idx << ", "
           << regi_results.regi_names[lvl][regi_idx] << std::endl;
      //vout << "-------- "

      const auto& cur_regi = *regi_results.regi_results[lvl][regi_idx];
      
      size_type land_debug_vol_idx = 0;

      Pt2List land_inds;
      Pt3List lands_wrt_vol;

      poses_for_land_reproj.clear();

      if (show_lands)
      {
        xregASSERT(num_views == 1);

        // Find the debug info containing landmarks used in regularization

        if (cur_regi.pen_fn_debug)
        {
          using ComboPenDebug = Regi2D3DPenaltyFnCombo::ComboDebugInfo;
          using LandPenDebug  = Regi2D3DPenaltyFnLandReproj::LandDebugInfo;

          const LandPenDebug* land_debug = nullptr;

          const ComboPenDebug* combo_debug = dynamic_cast<const ComboPenDebug*>(cur_regi.pen_fn_debug.get());

          if (combo_debug)
          {
            for (auto& p : combo_debug->pen_fns_debug_infos)
            {
              land_debug = dynamic_cast<const LandPenDebug*>(p.get());
              
              if (land_debug)
              {
                break;
              }
            }
          }
          else
          {
            land_debug = dynamic_cast<const LandPenDebug*>(cur_regi.pen_fn_debug.get());
          }

          if (land_debug)
          {
            // found landmark penalty debug object, reconstruct the landmark indices in the current camera model

            const CoordScalar regi_ds = regi_results.multi_res_levels[lvl];

            Pt2 adjust_crop = Pt2::Zero();

            // do not adjust this crop if we've pre-processed the projections with this crop
            if (regi_results.proj_pre_proc_info && !do_preproc)
            {
              adjust_crop(0) = regi_results.proj_pre_proc_info->crop_width;
              adjust_crop(1) = regi_results.proj_pre_proc_info->crop_width;
            }

            for (const auto& p : land_debug->lands_2d)
            {
              land_inds.push_back(((p / regi_ds) + adjust_crop) * CoordScalar(ds_factor));
            }

            lands_wrt_vol = land_debug->lands_3d;

            land_debug_vol_idx = land_debug->vol_idx;
          }
        }
      }

      const bool show_reproj_lands = show_lands && !land_inds.empty();

      const size_type num_mov_objs = cur_regi.vols_used.size();
      vout << "-------- number of moving objects: " << num_mov_objs << std::endl;

      const size_type num_its = cur_regi.iter_vars[0].size();
      vout << "-------- number of iterations: " << num_its << std::endl;

      for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
      {
        view_projs[view_idx][lvl][regi_idx].reserve(2 + num_its);
      }

      ProjList cur_bg_projs(num_views);

      ProjList tmp_projs(num_views);

      ProjList cur_bg_depth_rends;
      ProjList tmp_depth_rends;
      if (compute_boundary_edges)
      {
        cur_bg_depth_rends.resize(num_views);
        tmp_depth_rends.resize(num_views);
      }
      
      // depth_rends_for_boundary_edges[i][j] jth depth render of the ith view for this regi
      ListOfProjLists depth_rends_for_boundary_edges;
      if (compute_boundary_edges)
      {
        depth_rends_for_boundary_edges.resize(num_views);
      }

      bool need_bg_projs = false;
        
      if (!cur_regi.static_vols.empty())
      {
        const size_type num_static_vols = cur_regi.static_vols.size();
        
        global_vol_inds_registered = cur_regi.static_vols;

        for (size_type static_idx = 0; static_idx < num_static_vols; ++static_idx)
        {
          global_vol_last_poses[cur_regi.static_vols[static_idx]] =
                                            cur_regi.static_vol_poses[static_idx];
        }

        need_bg_projs = true;
      }
      else if (!cur_regi.do_not_use_static_vol_heuristic)
      {
        // NOTE: This is mainly here for legacy support
        // Determine if a background image is required.
        // If there is more than one moving object (e.g. all bones regi) OR no other
        // objects have been registered at this level OR
        // there is only one object, and it is the only object registered so far at this level
        // This is a bad heuristic!
        need_bg_projs = (num_mov_objs > 1) ||
                        global_vol_inds_registered.empty() ||
                        ((num_mov_objs == 1) &&
                         (global_vol_inds_registered.size() == 1) &&
                         (global_vol_inds_registered[0] == cur_regi.vols_used[0]));
      }

      if (!need_bg_projs)
      {
        vout << "-------- no background projection required..." << std::endl;
        ray_caster->set_use_bg_projs(false);
        ray_caster->use_proj_store_replace_method();
      
        if (compute_boundary_edges)
        {
          depth_rend->set_use_bg_projs(false);
          depth_rend->use_proj_store_replace_method();
        }
      }
      else
      {
        vout << "-------- computing background projection..." << std::endl;
        ray_caster->set_use_bg_projs(false);

        ComputeProjs(*ray_caster.get(), global_vol_last_poses, global_vol_inds_registered, cur_bg_projs);

        ray_caster->set_bg_projs(cur_bg_projs);
        ray_caster->set_use_bg_projs(true);
        
        if (compute_boundary_edges)
        {
          depth_rend->set_use_bg_projs(false);

          ComputeProjs(*depth_rend.get(), global_vol_last_poses, global_vol_inds_registered, cur_bg_depth_rends);
        
          depth_rend->set_bg_projs(cur_bg_depth_rends);
          depth_rend->set_use_bg_projs(true);
        }
      }

      // Initial projections
      vout << "-------- computing projections at initial poses..." << std::endl;
      for (size_type mov_obj_idx = 0; mov_obj_idx < num_mov_objs; ++mov_obj_idx)
      {
        const size_type global_vol_idx = cur_regi.vols_used[mov_obj_idx];
        global_vol_last_poses[global_vol_idx] = cur_regi.init_poses[mov_obj_idx];
      }
      ComputeProjs(*ray_caster.get(), global_vol_last_poses, cur_regi.vols_used, tmp_projs);
      for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
      {
        view_projs[view_idx][lvl][regi_idx].push_back(tmp_projs[view_idx]);
      }

      if (show_reproj_lands)
      {
        poses_for_land_reproj.push_back(global_vol_last_poses[land_debug_vol_idx]);
      }

      if (compute_boundary_edges)
      {
        ComputeProjs(*depth_rend.get(), global_vol_last_poses, cur_regi.vols_used, tmp_depth_rends);
        for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
        {
          depth_rends_for_boundary_edges[view_idx].push_back(tmp_depth_rends[view_idx]);
        }
      }

      const auto& se3 = *cur_regi.se3_params;

      vout << "-------- computing projections at iteration poses..." << std::endl;
      for (size_type it = 0; it < num_its; ++it)
      {
        // projections at iterations
        for (size_type mov_obj_idx = 0; mov_obj_idx < num_mov_objs; ++mov_obj_idx)
        {
          FrameTransform cur_pose = se3(cur_regi.iter_vars[mov_obj_idx][it]);

          const FrameTransform& inter_frame = cur_regi.inter_frames[mov_obj_idx];
          const FrameTransform& cur_guess   = cur_regi.init_poses[mov_obj_idx];

          if (cur_regi.inter_frames_wrt_vol[mov_obj_idx])
          {
            cur_pose = inter_frame * cur_pose * inter_frame.inverse() * cur_guess;
          }
          else
          {
            cur_pose = cur_guess * inter_frame * cur_pose * inter_frame.inverse();
          }

          const size_type global_vol_idx = cur_regi.vols_used[mov_obj_idx];
          global_vol_last_poses[global_vol_idx] = cur_pose;
        }
        ComputeProjs(*ray_caster.get(), global_vol_last_poses, cur_regi.vols_used, tmp_projs);
        for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
        {
          view_projs[view_idx][lvl][regi_idx].push_back(tmp_projs[view_idx]);
        }
        if (compute_boundary_edges)
        {
          ComputeProjs(*depth_rend.get(), global_vol_last_poses, cur_regi.vols_used, tmp_depth_rends);
          for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
          {
            depth_rends_for_boundary_edges[view_idx].push_back(tmp_depth_rends[view_idx]);
          }
        }
        
        if (show_reproj_lands)
        {
          poses_for_land_reproj.push_back(global_vol_last_poses[land_debug_vol_idx]);
        }
      }

      // Final projections
      vout << "-------- computing projections at final poses..." << std::endl;
      for (size_type mov_obj_idx = 0; mov_obj_idx < num_mov_objs; ++mov_obj_idx)
      {
        const size_type global_vol_idx = cur_regi.vols_used[mov_obj_idx];
        global_vol_last_poses[global_vol_idx] = cur_regi.final_poses[mov_obj_idx];
      }
      ComputeProjs(*ray_caster.get(), global_vol_last_poses, cur_regi.vols_used, tmp_projs);
      for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
      {
        view_projs[view_idx][lvl][regi_idx].push_back(tmp_projs[view_idx]);
      }
      
      if (compute_boundary_edges)
      {
        ComputeProjs(*depth_rend.get(), global_vol_last_poses, cur_regi.vols_used, tmp_depth_rends);
        for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
        {
          depth_rends_for_boundary_edges[view_idx].push_back(tmp_depth_rends[view_idx]);
        }
      }
      
      if (show_reproj_lands)
      {
        poses_for_land_reproj.push_back(global_vol_last_poses[land_debug_vol_idx]);
      }

      if (num_mov_objs == 1)
      {
        vout << "-------- adding volume to history: " << cur_regi.vols_used[0] << std::endl;
        global_vol_inds_registered.push_back(cur_regi.vols_used[0]);
      }
      else
      {
        vout << "-------- clearing volume history " << std::endl;
        global_vol_inds_registered.clear();
        // not sure if this is the right thing to do.
        global_vol_last_poses.assign(num_tot_objs, FrameTransform::Identity());
      }

      vout << "-------- remapping to 8bpp and computing edges" << std::endl;
      for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
      {
        const size_type num_cur_projs = view_projs[view_idx][lvl][regi_idx].size();

        remapped_view_projs[view_idx][lvl][regi_idx].resize(num_cur_projs);
        view_cv_mats[view_idx][lvl][regi_idx].resize(num_cur_projs);
        edge_overlays_cv_mats[view_idx][lvl][regi_idx].resize(num_cur_projs);

        for (size_type proj_idx = 0; proj_idx < num_cur_projs; ++proj_idx)
        {
          remapped_view_projs[view_idx][lvl][regi_idx][proj_idx] =
                      ITKImageRemap8bpp(view_projs[view_idx][lvl][regi_idx][proj_idx].GetPointer());

          view_cv_mats[view_idx][lvl][regi_idx][proj_idx] =
                  ShallowCopyItkToOpenCV(remapped_view_projs[view_idx][lvl][regi_idx][proj_idx].GetPointer());

          cv::Mat edge_img;
         
          if (compute_canny_edges)
          {
            edge_img = view_cv_mats[view_idx][lvl][regi_idx][proj_idx].clone();

            if (smooth_kernel_width > 1)
            {
              cv::GaussianBlur(view_cv_mats[view_idx][lvl][regi_idx][proj_idx], edge_img,
                               cv::Size(smooth_kernel_width,smooth_kernel_width), 0);
            }

            // &src == &dst is allowed
            cv::Canny(edge_img, edge_img, canny_lo_thresh, canny_hi_thresh);
            
            // Debugging:
            //cv::imwrite(fmt::format("edges_canny_{}_{}_{}_{:04d}.png",
            //                         lvl, regi_idx, view_idx, proj_idx),
            //            edge_img);
          }

          if (compute_boundary_edges)
          {
            cv::Mat boundary_edges;

            FindPixelsWithAdjacentIntensity(ShallowCopyItkToOpenCV(
                    depth_rends_for_boundary_edges[view_idx][proj_idx].GetPointer()),
                                               &boundary_edges, kRAY_CAST_MAX_DEPTH);

            // Debugging:
            //boundary_edges *= 255;
            //cv::imwrite(fmt::format("edges_boundary_{}_{}_{}_{:04d}.png",
            //                        lvl, regi_idx, view_idx, proj_idx),
            //            boundary_edges);

            if (compute_canny_edges)
            {
              // union with canny edges
              cv::bitwise_or(edge_img, boundary_edges, edge_img);
            }
            else
            {
              edge_img = boundary_edges;
            }
          }

          // enlarge edges:
          if (edge_dilate_width > 1)
          {
            cv::Mat tmp_src = edge_img.clone();
            cv::dilate(tmp_src, edge_img,
                       cv::getStructuringElement(cv::MORPH_ELLIPSE,
                         cv::Size(edge_dilate_width,edge_dilate_width)));
          }

          if (compute_canny_edges || compute_boundary_edges)
          {
            edge_overlays_cv_mats[view_idx][lvl][regi_idx][proj_idx] =
                                    OverlayEdges(remapped_fixed_views_ocv[view_idx], edge_img, 1);
          }
          else
          {
            edge_overlays_cv_mats[view_idx][lvl][regi_idx][proj_idx] = remapped_fixed_views_ocv[view_idx].clone();
          }
          
          if (show_reproj_lands)
          {
            cv::Mat cur_overlay = edge_overlays_cv_mats[view_idx][lvl][regi_idx][proj_idx];

            const std::string line_color = "cyan";

            const int thickness = (ds_factor >= 0.25) ? 2 : 1;
            
            const int radius = static_cast<int>((20 * ds_factor) + 0.5);

            cur_overlay = OverlayPts(cur_overlay, land_inds,
                                     { { "color", line_color } , { "marker", "o" },
                                       { "thickness", thickness }, { "radius", radius } });

            const FrameTransform& cur_vol_to_cam = poses_for_land_reproj[proj_idx].inverse();

            Pt2List proj_land_inds;
            proj_land_inds.reserve(lands_wrt_vol.size());

            const auto& cam = fixed_proj_data[view_idx].cam;

            for (const auto& p : lands_wrt_vol)
            {
              proj_land_inds.push_back(cam.phys_pt_to_ind_pt(cur_vol_to_cam * p).head(2));
            }
            
            cur_overlay = OverlayPts(cur_overlay, proj_land_inds,
                                     { { "color", line_color } , { "marker", "*" },
                                       { "thickness", thickness }, { "radius", radius } });

            const size_type num_lands = proj_land_inds.size();

            for (size_type land_idx = 0; land_idx < num_lands; ++land_idx)
            {
              const auto& p1 = land_inds[land_idx];
              const auto& p2 = proj_land_inds[land_idx];
            
              if ((p1 - p2).norm() > radius)
              {
                cv::line(cur_overlay, cv::Point(p1(0),p1(1)), cv::Point(p2(0),p2(1)),
                         OpenCVColorNameToScalar().find(line_color)->second,
                         thickness);
              }
            }

            edge_overlays_cv_mats[view_idx][lvl][regi_idx][proj_idx] = cur_overlay;
          }
        }
      }
    }
  }

  if (write_raw_final_drr)
  {
    vout << "writing final DRR view frames as raw images..." << std::endl;
    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      WriteITKImageToDisk(view_projs[view_idx].back().back().back().GetPointer(),
                          fmt::format("{}_{:02d}.nii.gz", debug_raw_final_drr_prefix, view_idx));
    }
  }

  vout << "all projections generated!" << std::endl;

  vout << "tiling images..." << std::endl;

  drr_frames.reserve(tot_num_projs);
  edge_frames.reserve(tot_num_projs);
  frame_titles.reserve(tot_num_projs);
        
  const int tile_border_thickness = std::max(1, static_cast<int>(std::lround(10 * ds_factor)));

  for (size_type lvl = 0; lvl < num_levels; ++lvl)
  {
    const CoordScalar lvl_ds_factor = regi_results.multi_res_levels[lvl];

    const size_type num_regis = regi_results.regi_results[lvl].size();

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      const size_type num_cur_projs = view_cv_mats[0][lvl][regi_idx].size();

      for (size_type proj_idx = 0; proj_idx < num_cur_projs; ++proj_idx)
      {
        MatList cur_proj_views(num_views);
        for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
        {
          cur_proj_views[view_idx] = view_cv_mats[view_idx][lvl][regi_idx][proj_idx];

          if (flip_proj_rows[view_idx])
          {
            FlipImageRows(&cur_proj_views[view_idx]);
          }

          if (flip_proj_cols[view_idx])
          {
            FlipImageColumns(&cur_proj_views[view_idx]);
          }

          if (write_debug_img_frames)
          {
            cv::imwrite(fmt::format("{}_{:02d}_mov_{:03d}.png",
                                    debug_img_prefix, view_idx, drr_frames.size()),
                        cur_proj_views[view_idx]);
          }
        }

        auto tiled_img = CreateSummaryTiledImages(cur_proj_views, StrList(),
                                                  num_tile_rows, num_tile_cols,
                                                  tile_border_thickness);

        drr_frames.push_back(tiled_img[0]);

        for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
        {
          cur_proj_views[view_idx] = edge_overlays_cv_mats[view_idx][lvl][regi_idx][proj_idx];

          if (flip_proj_rows[view_idx])
          {
            FlipImageRows(&cur_proj_views[view_idx]);
          }

          if (flip_proj_cols[view_idx])
          {
            FlipImageColumns(&cur_proj_views[view_idx]);
          }
          
          if (write_debug_img_frames)
          {
            cv::imwrite(fmt::format("{}_{:02d}_edges_{:03d}.png",
                                    debug_img_prefix, view_idx, edge_frames.size()),
                        cur_proj_views[view_idx]);
          }
        }

        tiled_img = CreateSummaryTiledImages(cur_proj_views, StrList(),
                                             num_tile_rows, num_tile_cols,
                                             tile_border_thickness);
        edge_frames.push_back(tiled_img[0]);

        frame_titles.push_back(fmt::format("Level {} ({:.3f}), {}, {:03d}",
                                           lvl, lvl_ds_factor, regi_results.regi_names[lvl][regi_idx], proj_idx));
      }
    }
  }

  xregASSERT(drr_frames.size() == tot_num_projs);

  num_views_     = num_views;
  num_view_cols_ = fixed_proj_data[0].cam.num_det_cols;
}

double xreg::Regi2D3DReplay::fps_for_video_len(const double video_len_secs) const
{
  return std::max(0.1, drr_frames.size() / video_len_secs);
}

void xreg::Regi2D3DReplay::write_videos(const std::string& drr_path, const std::string& edges_path,
                                        const double fps)
{
  const size_type num_frames = drr_frames.size();

  if (!num_frames)
  {
    xregThrow("Regi2D3DReplay: no frames are available, run() must be called before writing videos!");
  }

  xregASSERT(edge_frames.size() == num_frames);
  xregASSERT(frame_titles.size() == num_frames);

  std::ostream& vout = dout();

  const double ds_factor = proj_ds_factor;

  const bool write_debug_img_frames = !debug_img_prefix.empty();

  vout << "writing movie frames..." << std::endl;

  // a movie is not written when its path is empty
  std::unique_ptr<WriteImageFramesToVideo> mov_vid_writer;
  std::unique_ptr<WriteImageFramesToVideo> edge_vid_writer;

  if (!drr_path.empty())
  {
    mov_vid_writer = GetWriteImageFramesToVideo();
    mov_vid_writer->dst_vid_path = drr_path;
    mov_vid_writer->fps = fps;
    mov_vid_writer->open();
  }

  if (!edges_path.empty())
  {
    edge_vid_writer = GetWriteImageFramesToVideo();
    edge_vid_writer->dst_vid_path = edges_path;
    edge_vid_writer->fps = fps;
    edge_vid_writer->open();
  }
  
  const double font_scale = ((ds_factor > 0.499) ? 2.0 : 3.0) * ds_factor;
  const int font_line_thickness = (font_scale > 0.4999) ? 2 : 1;

  // Try and overlay the text in the "center" view - for an odd number of views the
  // center is obvious: index 0 for 1 view, index 1 for 3 views, index 2 for 5 views, etc.
  // For an even number of views, choose the left view of the two-most center views
  size_type middle_view_idx = num_views_ / 2;
  if (!(num_views_ % 2))
  {
    --middle_view_idx;
  }
  
  // left align within the center view to ensure that the entire text fits, using something
  // small, but greater than zero, to avoid the boundary coloring.
  const cv::Point txt_off((num_view_cols_ * middle_view_idx) + 5.0,
                          drr_frames[0].rows - (font_scale * 15.0));
    
  for (size_type frame_idx = 0; frame_idx < num_frames; ++frame_idx)
  {
    // The titles are burned into copies of the frames, so that the frames
    // stored in this object are not modified.
    cv::Mat drr_frame  = drr_frames[frame_idx].clone();
    cv::Mat edge_frame = edge_frames[frame_idx].clone();

    auto write_cur_debug_tile_imgs = [&] ()
    {
      if (write_debug_img_frames)
      {
        cv::imwrite(fmt::format("{}_tiled_mov_{:03d}.png",   debug_img_prefix, frame_idx), drr_frame);
        cv::imwrite(fmt::format("{}_tiled_edges_{:03d}.png", debug_img_prefix, frame_idx), edge_frame);
      }
    };

    if (!debug_img_include_text)
    {
      write_cur_debug_tile_imgs();
    }

    // burn in the text after writing the single image
    
    cv::putText(drr_frame, frame_titles[frame_idx], txt_off,
                cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(0, 255, 255), font_line_thickness);

    cv::putText(edge_frame, frame_titles[frame_idx], txt_off,
                cv::FONT_HERSHEY_SIMPLEX, font_scale,
                cv::Scalar(0, 255, 255), font_line_thickness);
    
    if (debug_img_include_text)
    {
      write_cur_debug_tile_imgs();
    }

    if (mov_vid_writer)
    {
      mov_vid_writer->write(drr_frame);
    }

    if (edge_vid_writer)
    {
      edge_vid_writer->write(edge_frame);
    }
  }
}

