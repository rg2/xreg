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

#include "xregIntensity2D3DRegiDebug.h"

#include "xregHDF5.h"
#include "xregHDF5ReadWriteInterface.h"
#include "xregSE3OptVars.h"
#include "xregH5SE3OptVarsIO.h"
#include "xregImgSimMetric2DPatchNCCCPU.h"
#include "xregImgSimMetric2DPatchGradNCCCPU.h"
#include "xregRegi2D3DPenaltyFnDebugH5.h"

void xreg::SingleRegiDebugResults::init(SE3OptVarsPtr se3, const IndexList& global_vol_inds,
                                        const size_type init_num_iters_capacity)
{
  se3_params = se3;

  vols_used.assign(global_vol_inds.begin(), global_vol_inds.end());

  const size_type num_objs = vols_used.size();

  init_poses.assign(num_objs, FrameTransform::Identity());
  final_poses.assign(num_objs, FrameTransform::Identity());

  inter_frames.assign(num_objs, FrameTransform::Identity());
  inter_frames_wrt_vol.assign(num_objs, false);

  iter_vars.resize(num_objs);
  for (size_type obj_idx = 0; obj_idx < num_objs; ++obj_idx)
  {
    iter_vars[obj_idx].reserve(init_num_iters_capacity);
  }

  sims.reserve(init_num_iters_capacity);
}

void xreg::WriteSingleRegiDebugResultsH5(const SingleRegiDebugResults& results, H5::Group* h5)
{
  H5::Group se3_g = h5->createGroup("se3-parameterization");
  WriteSE3OptVarsH5(*results.se3_params, &se3_g);

  const size_type num_params_per_obj = results.se3_params->num_params();

  if (!results.projs_used.empty())
  {
    WriteVectorH5("projs-used", results.projs_used, h5);
  }

  {
    H5::Group static_vol_g = h5->createGroup("static-vols");
    
    WriteSingleScalarH5("no-static-vol-heuristic",
                        results.do_not_use_static_vol_heuristic,
                        &static_vol_g);
    
    const size_type num_static_vols = results.static_vols.size();

    xregASSERT(num_static_vols == results.static_vol_poses.size());
  
    if (num_static_vols > 0)
    {
      WriteVectorH5("vols", results.static_vols, &static_vol_g);
      
      H5::Group static_poses_g = static_vol_g.createGroup("poses");

      for (size_type static_idx = 0; static_idx < num_static_vols; ++static_idx)
      {
        WriteAffineTransform4x4(fmt::format("{:03d}", static_idx),
                                results.static_vol_poses[static_idx],
                                &static_poses_g);
      }
    }
  }

  const size_type num_vols = results.init_poses.size();
  xregASSERT(num_vols == results.final_poses.size());
  xregASSERT(num_vols == results.iter_vars.size());
  xregASSERT(num_vols == results.vols_used.size());

  if (num_vols)
  {
    WriteSingleScalarH5("num-objs", num_vols, h5);
    
    const size_type num_its = results.iter_vars[0].size();
    xregASSERT(num_its == results.sims.size());
    
    WriteSingleScalarH5("num-iterations", num_its, h5);
  
    H5::Group global_idx_map_g = h5->createGroup("global-obj-index-map");
    H5::Group init_poses_g     = h5->createGroup("init-poses");
    H5::Group final_poses_g    = h5->createGroup("final-poses");
    H5::Group inter_frames_g   = h5->createGroup("inter-frames");

    H5::Group obj_iter_pose_vars_g = h5->createGroup("iter-pose-vars");

    Eigen::Matrix<CoordScalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor|Eigen::AutoAlign>
                                      pose_vars(num_its,num_params_per_obj);

    for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
    {
      const std::string cur_obj_idx_str = fmt::format("obj-{:03d}", vol_idx);

      WriteAffineTransform4x4(cur_obj_idx_str, results.init_poses[vol_idx],   &init_poses_g,   false);
      WriteAffineTransform4x4(cur_obj_idx_str, results.final_poses[vol_idx],  &final_poses_g,  false);
      WriteAffineTransform4x4(cur_obj_idx_str, results.inter_frames[vol_idx], &inter_frames_g, false);

      WriteSingleScalarH5(fmt::format("{}-wrt-vol", cur_obj_idx_str),
                          bool(results.inter_frames_wrt_vol[vol_idx]), &inter_frames_g);
    
      WriteSingleScalarH5(cur_obj_idx_str, results.vols_used[vol_idx], &global_idx_map_g);

      if (num_its)
      {
        const auto& cur_obj_iter_vars = results.iter_vars[vol_idx];

        for (size_type it = 0; it < num_its; ++it)
        {
          const auto& cur_pose_vars = cur_obj_iter_vars[it];

          for (size_type p = 0; p < num_params_per_obj; ++p)
          {
            pose_vars(it,p) = cur_pose_vars[p];
          }
        }

        WriteMatrixH5(cur_obj_idx_str, MatMxN(pose_vars), &obj_iter_pose_vars_g);
      }
    }

    if (results.regi_time_secs)
    {
      WriteSingleScalarH5("regi-elapsed-seconds", *results.regi_time_secs, h5);
    }

    if (num_its)
    {
      WriteVectorH5("sim-vals", results.sims, h5);
    }
  
    if (results.opt_aux)
    {
      results.opt_aux->write(h5);
    }

    {
      bool need_to_make_sim_aux_parent_g = true;
      
      H5::Group sims_aux_g;

      const size_type num_sim_aux = results.sims_aux.size();
      
      for (size_type si = 0; si < num_sim_aux; ++si)
      {
        if (results.sims_aux[si])
        {
          if (need_to_make_sim_aux_parent_g)
          {
            sims_aux_g = h5->createGroup("sims-aux");

            need_to_make_sim_aux_parent_g = false;
          }

          H5::Group cur_sim_aux_g = sims_aux_g.createGroup(fmt::format("{:03d}", si));

          results.sims_aux[si]->write(&cur_sim_aux_g);
        }
      }
    }
  }

  if (results.pen_fn_debug)
  {
    H5::Group pen_fn_debug_g = h5->createGroup("pen-fn");
    
    results.pen_fn_debug->write(&pen_fn_debug_g);
  }
}

xreg::SingleRegiDebugResults xreg::ReadSingleRegiDebugResultsH5(const H5::Group& h5)
{
  SingleRegiDebugResults regi_results;
      
  regi_results.se3_params = ReadSE3OptVarsH5(h5.openGroup("se3-parameterization"));
  const size_type num_params_per_obj = regi_results.se3_params->num_params();

  if (ObjectInGroupH5("projs-used", h5))
  {
    regi_results.projs_used = ReadVectorH5SizeType("projs-used", h5);
  }

  const size_type num_vols = ReadSingleScalarH5ULong("num-objs", h5);

  regi_results.init_poses.resize(num_vols);
  regi_results.final_poses.resize(num_vols);
  regi_results.inter_frames.resize(num_vols);
  regi_results.inter_frames_wrt_vol.resize(num_vols);
  regi_results.vols_used.resize(num_vols);
  regi_results.iter_vars.resize(num_vols);
  
  const size_type num_its = ReadSingleScalarH5ULong("num-iterations", h5);
    
  H5::Group global_idx_map_g = h5.openGroup("global-obj-index-map");
  H5::Group init_poses_g     = h5.openGroup("init-poses");
  H5::Group final_poses_g    = h5.openGroup("final-poses");
  H5::Group inter_frames_g   = h5.openGroup("inter-frames");

  H5::Group obj_iter_pose_vars_g = h5.openGroup("iter-pose-vars");

  for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
  {
    const std::string cur_obj_idx_str = fmt::format("obj-{:03d}", vol_idx);

    regi_results.init_poses[vol_idx]   = ReadAffineTransform4x4H5(cur_obj_idx_str, init_poses_g);
    regi_results.final_poses[vol_idx]  = ReadAffineTransform4x4H5(cur_obj_idx_str, final_poses_g);
    regi_results.inter_frames[vol_idx] = ReadAffineTransform4x4H5(cur_obj_idx_str, inter_frames_g);
    
    regi_results.inter_frames_wrt_vol[vol_idx] = ReadSingleScalarH5Bool(
                                                  fmt::format("{}-wrt-vol", cur_obj_idx_str),
                                                  inter_frames_g);

    regi_results.vols_used[vol_idx] = ReadSingleScalarH5ULong(cur_obj_idx_str, global_idx_map_g);

    auto& cur_obj_iter_vars = regi_results.iter_vars[vol_idx];
    
    if (num_its)
    {
      auto pose_vars = ReadMatrixH5CoordScalar(cur_obj_idx_str, obj_iter_pose_vars_g);
      xregASSERT(static_cast<size_type>(pose_vars.rows()) == num_its);
      xregASSERT(static_cast<size_type>(pose_vars.cols()) == num_params_per_obj); 

      cur_obj_iter_vars.resize(num_its);

      for (size_type it = 0; it < num_its; ++it)
      {
        auto& cur_pose_vars = cur_obj_iter_vars[it];
        cur_pose_vars.resize(num_params_per_obj);

        for (size_type p = 0; p < num_params_per_obj; ++p)
        {
          cur_pose_vars[p] = pose_vars(it,p);
        }
      }
    }
    else
    {
      xregASSERT(!ObjectInGroupH5(cur_obj_idx_str, obj_iter_pose_vars_g));
      cur_obj_iter_vars.clear();
    }
  }  // for vol_idx

  if (ObjectInGroupH5("static-vols", h5))
  {
    const H5::Group static_vol_g = h5.openGroup("static-vols");
    
    regi_results.do_not_use_static_vol_heuristic = ReadSingleScalarH5Bool(
                                        "no-static-vol-heuristic", static_vol_g);
    
    regi_results.static_vols.clear();  
    regi_results.static_vol_poses.clear();
   
    if (ObjectInGroupH5("vols", static_vol_g))
    {
      regi_results.static_vols = ReadVectorH5SizeType("vols", static_vol_g);
      
      const size_type num_static_vols = regi_results.static_vols.size();
      
      if (num_static_vols > 0)
      {
        const H5::Group static_poses_g = static_vol_g.openGroup("poses");

        regi_results.static_vol_poses.reserve(num_static_vols);

        for (size_type static_idx = 0; static_idx < num_static_vols; ++static_idx)
        {
          regi_results.static_vol_poses.push_back(
                        ReadAffineTransform4x4H5(
                              fmt::format("{:03d}", static_idx), static_poses_g));
        }
      }
    }
  }

  if (ObjectInGroupH5("regi-elapsed-seconds", h5))
  {
    regi_results.regi_time_secs = ReadSingleScalarH5Double("regi-elapsed-seconds", h5);
  }
 
  if (num_its)
  {
    regi_results.sims = ReadVectorH5CoordScalar("sim-vals", h5);
  }
  else
  {
    xregASSERT(!ObjectInGroupH5("sim-vals", h5));
  }

  // TODO: read optimizer auxiliary info
  //
  // read sim metric auxiliary infos
  
  if (ObjectInGroupH5("sims-aux", h5))
  {
    H5::Group sim_aux_g = h5.openGroup("sims-aux");
    
    size_type num_sim_auxs = 0;

    bool should_stop = false;
    while (!should_stop)
    {
      const std::string cur_sim_aux_g_name = fmt::format("{:03d}", num_sim_auxs);
      
      if (ObjectInGroupH5(cur_sim_aux_g_name, sim_aux_g))
      {
        ++num_sim_auxs;
        
        H5::Group cur_sim_aux_g = sim_aux_g.openGroup(cur_sim_aux_g_name);
        
        const std::string sim_aux_type = ReadStringH5("sim-aux-type", cur_sim_aux_g);
        
        std::shared_ptr<H5ReadWriteInterface> sim_aux;
        
        if (sim_aux_type == "patch-grad-ncc-aux")
        {
          sim_aux = std::make_shared<ImgSimMetric2DPatchGradNCCCPU::SimAux>();
        }
        else if (sim_aux_type == "patch-ncc-aux")
        {
          sim_aux = std::make_shared<ImgSimMetric2DPatchNCCCPU::SimAux>();
        }
        else
        {
          xregThrow("unsupported sim metric aux type: %s", sim_aux_type.c_str());
        }

        sim_aux->read(cur_sim_aux_g);

        regi_results.sims_aux.push_back(sim_aux);
      }
      else
      {
        should_stop = true;
      }
    }
  }
  
  if (ObjectInGroupH5("pen-fn", h5))
  {
    H5::Group pen_fn_debug_g = h5.openGroup("pen-fn");
    
    regi_results.pen_fn_debug = MakePenFnDebugObjH5(pen_fn_debug_g);
    regi_results.pen_fn_debug->read(pen_fn_debug_g);
  }

  return regi_results;
}

