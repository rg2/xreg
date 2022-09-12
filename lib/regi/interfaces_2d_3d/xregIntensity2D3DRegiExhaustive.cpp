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

#include "xregIntensity2D3DRegiExhaustive.h"

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregSE3OptVars.h"
#include "xregIntensity2D3DRegiDebug.h"
#include "xregHDF5.h"
#include "xregImgSimMetric2D.h"
#include "xregImgSimMetric2DCombine.h"
#include "xregRegi2D3DPenaltyFn.h"

//#define XREG_TBME_MOVIE_HACK

#ifdef XREG_TBME_MOVIE_HACK
#include "xregH5ProjDataIO.h"
#endif

void xreg::Intensity2D3DRegiExhaustive::set_cam_wrt_vols(const ListOfFrameTransformLists& xforms,
                                                         const size_type batch_size)
{
  use_mesh_grid_ = false;
  
  const size_type num_objs = xforms.size();

  tot_num_xforms_ = xforms[0].size();

  this->num_projs_per_view_ = batch_size ? batch_size : tot_num_xforms_;

  for (size_type i = 1; i < num_objs; ++i)
  {
    if (tot_num_xforms_ != xforms[i].size())
    {
      xregThrow("number of transforms per object is inconsistent!");
    }
  }

  cam_wrt_vols_ = xforms;    
}

void xreg::Intensity2D3DRegiExhaustive::set_cam_wrt_vols(const ConstSpacedMeshGrid& mesh_grid,
                                                         const size_type batch_size)
{
  mesh_grid_ = mesh_grid;
  use_mesh_grid_ = true;

  tot_num_xforms_ = mesh_grid_.size();
  
  this->num_projs_per_view_ = batch_size ? batch_size : tot_num_xforms_;
}

void xreg::Intensity2D3DRegiExhaustive::run()
{
  const size_type num_vols = this->num_vols();
  xregASSERT(use_mesh_grid_ || (num_vols == cam_wrt_vols_.size()));
  xregASSERT(num_vols);

  // more significant indices correspond to the first volume, less significant
  // indices correspond to the last volume.
  // e.g. for row-major, Voll1,Vol2,Vol3
  xregASSERT(!use_mesh_grid_ || (mesh_grid_.num_dims() == (this->num_vols() * this->opt_vars_->num_params())));

  const size_type num_views = this->sim_metrics_.size();
  xregASSERT(num_views);

  const auto& opt_map = *this->opt_vars_;

  const size_type num_opt_params_per_vol = opt_map.num_params();

  this->before_first_iteration();

  OptAux* opt_aux = nullptr;

  if (this->debug_save_iter_debug_info_)
  {
    this->debug_info_->opt_aux = std::make_shared<OptAux>();

    opt_aux = static_cast<OptAux*>(this->debug_info_->opt_aux.get());

    if (save_all_cam_wrt_vols_in_aux_)
    {
      opt_aux->cam_wrt_vols.resize(num_vols);
    }
  }

  const size_type orig_num_projs_per_view = this->num_projs_per_view_;
  
  // TODO: handle multiple camera models
  sim_vals_.assign(this->num_projs_per_view_, 0);
  
  min_sim_val_ = std::numeric_limits<Scalar>::max();
  
  FrameTransformList delta_xforms(num_vols, FrameTransform::Identity());

  cur_start_xform_idx_ = 0;
  
  size_type num_xforms_left = tot_num_xforms_;

  ListOfFrameTransformLists cur_cam_wrt_vols(num_vols);

  MeshGridIt mg_it;

  if (use_mesh_grid_)
  {
    mg_it = mesh_grid_.begin();
  }

  all_sim_vals_.clear();
  all_pen_vals_.clear();
  all_pen_log_prob_vals_.clear();

  double next_print_percent = print_status_inc_;

  while (num_xforms_left)
  {
    this->begin_of_iteration(delta_xforms);
    
    cur_num_xforms_ = std::min(num_xforms_left, this->num_projs_per_view_);
    
    sim_vals_.resize(cur_num_xforms_);
   
    if (use_mesh_grid_)
    {
      cur_mesh_grid_start_it_ = mg_it;
      
      // reset transforms list  
      for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
      {
        cur_cam_wrt_vols[vol_idx].clear();
        cur_cam_wrt_vols[vol_idx].reserve(cur_num_xforms_);
      }
      
      for (size_type i = 0; i < cur_num_xforms_; ++i, ++mg_it)
      {
        auto cur_pt = *mg_it;

        const Scalar* cur_vol_params_buf = &cur_pt[0];

        // split across volumes, create a transform for each volume
        for (size_type vol_idx = 0; vol_idx < num_vols;
             ++vol_idx, cur_vol_params_buf += num_opt_params_per_vol)
        {
          cur_cam_wrt_vols[vol_idx].push_back(opt_map(
                Eigen::Map<PtN>(const_cast<Scalar*>(cur_vol_params_buf), num_opt_params_per_vol)));
        }

        if (i == (cur_num_xforms_ - 1))
        {
          cur_mesh_grid_stop_it_ = mg_it;
        }
      }
    }
    else 
    { 
      // copy over transforms for this batch 
      for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
      { 
        cur_cam_wrt_vols[vol_idx].assign(
          cam_wrt_vols_[vol_idx].begin() + cur_start_xform_idx_,
          cam_wrt_vols_[vol_idx].begin() + cur_start_xform_idx_ + cur_num_xforms_);
      }
    }

    // set num projs for the objects that matter, optimizer, ray caster, sim metrics
    this->num_projs_per_view_ = cur_num_xforms_;
    this->ray_caster_->set_num_projs(num_views * cur_num_xforms_);
    for (auto& sm : this->sim_metrics_)
    {
      sm->set_num_moving_images(cur_num_xforms_);
    }
    this->sim_metric_combiner_->set_num_projs_per_sim_metric(cur_num_xforms_);

    this->obj_fn(cur_cam_wrt_vols, nullptr, &sim_vals_);

    if (opt_aux && save_all_cam_wrt_vols_in_aux_)
    {
      for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
      {
        opt_aux->cam_wrt_vols[vol_idx].insert(opt_aux->cam_wrt_vols[vol_idx].end(),
                                              cur_cam_wrt_vols[vol_idx].begin(),
                                              cur_cam_wrt_vols[vol_idx].begin() + cur_num_xforms_);
      }
    }

    if (save_all_sim_vals_)
    {
      all_sim_vals_.insert(all_sim_vals_.end(), sim_vals_.begin(), sim_vals_.begin() + cur_num_xforms_);
    }

    if (save_all_penalty_vals_)
    {
      const auto& pen_vals = penalty_fn_->reg_vals();

      all_pen_vals_.insert(all_pen_vals_.end(), pen_vals.begin(), pen_vals.begin() + cur_num_xforms_);

      if (penalty_fn_->compute_probs())
      {
        const auto& log_probs = penalty_fn_->log_probs();

        all_pen_log_prob_vals_.insert(all_pen_log_prob_vals_.end(), log_probs.begin(), log_probs.begin() + cur_num_xforms_);
      }
    }

#ifdef XREG_TBME_MOVIE_HACK
    // hacking for the tbme movie
    {
      static size_type pd_idx = 1;

      ProjDataF32List pd(cur_num_xforms_);

      const auto cam = this->ray_caster_->camera_model();

      for (size_type ii = 0; ii < cur_num_xforms_; ++ii)
      {
        pd[ii].img = this->ray_caster_->proj(ii);
        pd[ii].cam = cam;
        pd[ii].cam.extrins = pd[ii].cam.extrins * cur_cam_wrt_vols[0][ii];
      }

      WriteProjDataH5ToDisk(pd, fmt::format("hack_pd_{:02d}.h5", pd_idx));

      ++pd_idx;
    }
#endif
   
    // reset num projs for the other objects 
    this->num_projs_per_view_ = orig_num_projs_per_view;
    this->ray_caster_->set_num_projs(num_views * orig_num_projs_per_view);
    for (auto& sm : this->sim_metrics_)
    {
      sm->set_num_moving_images(orig_num_projs_per_view);
    }
    this->sim_metric_combiner_->set_num_projs_per_sim_metric(orig_num_projs_per_view);

    if (compute_min_)
    {
      // Find the minimum similarity score and the associated transforms
      for (size_type proj_idx = 0; proj_idx < cur_num_xforms_; ++proj_idx)
      {
        if (min_sim_val_ > sim_vals_[proj_idx])
        {
          min_sim_val_ = sim_vals_[proj_idx];

          for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
          {
            delta_xforms[vol_idx] = cur_cam_wrt_vols[vol_idx][proj_idx];
          }
        
          if (save_all_sim_vals_)
          {
            best_all_sim_vals_idx_ = all_sim_vals_.size() - cur_num_xforms_ + proj_idx;
          }
        }
      }
    }

    cur_start_xform_idx_ += cur_num_xforms_;
    num_xforms_left     -= cur_num_xforms_;

    this->end_of_iteration();
   
    const double cur_comp_percent = 1.0 - (static_cast<double>(num_xforms_left) / tot_num_xforms_);
    if ((cur_comp_percent + 1.0e-6) > next_print_percent)
    {
      std::cout << fmt::format("  ex. regi: {:4.2f}% ({} remaining)",
                               100 * cur_comp_percent, num_xforms_left) << std::endl;
      next_print_percent += print_status_inc_;
    }
  }

  if (opt_aux)
  {
    opt_aux->has_min = compute_min_;

    if (compute_min_)
    {
      opt_aux->min_sim_val = min_sim_val_;
    }

    opt_aux->has_all_sim_vals = save_all_sim_vals_;
    
    if (save_all_sim_vals_)
    {
      opt_aux->all_sim_vals = all_sim_vals_;

      if (compute_min_)
      {
        opt_aux->best_all_sim_vals_idx = best_all_sim_vals_idx_;
      }
    }
  }

  this->after_last_iteration();
  
  this->update_regi_xforms(delta_xforms);
}

xreg::size_type xreg::Intensity2D3DRegiExhaustive::max_num_projs_per_view_per_iter() const
{
  return this->num_projs_per_view_;
}

bool xreg::Intensity2D3DRegiExhaustive::save_all_sim_vals() const
{
  return save_all_sim_vals_;
}

void xreg::Intensity2D3DRegiExhaustive::set_save_all_sim_vals(const bool s)
{
  save_all_sim_vals_ = s;
}

bool xreg::Intensity2D3DRegiExhaustive::save_all_penalty_vals() const
{
  return save_all_penalty_vals_;
}

void xreg::Intensity2D3DRegiExhaustive::set_save_all_penalty_vals(const bool s)
{
  save_all_penalty_vals_ = s;
}

const xreg::Intensity2D3DRegiExhaustive::ScalarList&
xreg::Intensity2D3DRegiExhaustive::all_sim_vals() const
{
  return all_sim_vals_;
}

const xreg::Intensity2D3DRegiExhaustive::ScalarList&
xreg::Intensity2D3DRegiExhaustive::all_penalty_vals() const
{
  return all_pen_vals_;
}

const xreg::Intensity2D3DRegiExhaustive::ScalarList&
xreg::Intensity2D3DRegiExhaustive::all_penalty_log_probs() const
{
  return all_pen_log_prob_vals_;
}

void xreg::Intensity2D3DRegiExhaustive::set_save_all_cam_wrt_vols_in_aux(const bool s)
{
  save_all_cam_wrt_vols_in_aux_ = s;
}

bool xreg::Intensity2D3DRegiExhaustive::save_all_cam_wrt_vols_in_aux() const
{
  return save_all_cam_wrt_vols_in_aux_;
}

xreg::size_type xreg::Intensity2D3DRegiExhaustive::tot_num_xforms() const
{
  return tot_num_xforms_;
}

void xreg::Intensity2D3DRegiExhaustive::set_print_status_inc(const double inc)
{
  print_status_inc_ = inc;
}

void xreg::Intensity2D3DRegiExhaustive::write_debug()
{ }

void xreg::Intensity2D3DRegiExhaustive::init_opt()
{ }

void xreg::Intensity2D3DRegiExhaustive::OptAux::read(const H5::Group& h5)
{
  xregThrow("Exhaustive Intensity Regi 2D/3D OptAux::read() not implemented!");
}

void xreg::Intensity2D3DRegiExhaustive::OptAux::write(H5::Group* h5)
{
  H5::Group aux_g = h5->createGroup("opt-aux");

  WriteStringH5("name", "Exhaustive", &aux_g, false);
  
  const size_type num_vols = cam_wrt_vols.size();
  
  if (num_vols)
  {
    const size_type num_xforms = cam_wrt_vols[0].size();
    
    if (num_xforms)
    {
      H5::Group cam_wrt_vols_g = aux_g.createGroup("cam-wrt-vols");
      
      for (size_type vol_idx = 0; vol_idx < num_vols; ++vol_idx)
      {        
        xregASSERT(num_xforms == cam_wrt_vols[vol_idx].size());
       
        H5::Group vol_g = cam_wrt_vols_g.createGroup(fmt::format("vol-{:02d}", vol_idx));

        for (size_type i = 0; i  < num_xforms; ++i)
        {
          WriteAffineTransform4x4(fmt::format("{:06d}", i), cam_wrt_vols[vol_idx][i], &vol_g);
        }
      }
    }
  }
  
  if (has_min)
  {
    WriteSingleScalarH5("min-sim-val", min_sim_val, &aux_g);
    
    if (has_all_sim_vals)
    {
      WriteSingleScalarH5("min-sim-val-idx", best_all_sim_vals_idx, &aux_g);
    }
  }
  
  if (has_all_sim_vals)
  {
    WriteVectorH5("all-sim-vals", all_sim_vals, &aux_g);
  }
}

#ifdef XREG_TBME_MOVIE_HACK
#undef XREG_TBME_MOVIE_HACK
#endif

