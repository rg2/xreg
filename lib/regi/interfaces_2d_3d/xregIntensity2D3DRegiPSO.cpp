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

#include "xregIntensity2D3DRegiPSO.h"

#include <fmt/format.h>

#include "xregSE3OptVars.h"
#include "xregIntensity2D3DRegiDebug.h"

xreg::Intensity2D3DRegiPSO::Intensity2D3DRegiPSO()
{
  // number of particles default
  this->num_projs_per_view_ = 100;
  
  pso_.regi_ = this;
}

xreg::size_type xreg::Intensity2D3DRegiPSO::max_num_projs_per_view_per_iter() const
{
  return this->num_projs_per_view_;
}

void xreg::Intensity2D3DRegiPSO::set_num_particles(const size_type np)
{
  this->num_projs_per_view_ = np;
}

void xreg::Intensity2D3DRegiPSO::set_max_num_iters(const size_type max_iters)
{
  pso_.set_max_num_its(max_iters);
}

void xreg::Intensity2D3DRegiPSO::set_box_contraint(const ScalarList& bounds)
{
  pso_.set_box_about_zero(Eigen::Map<PtN>(
                          const_cast<Scalar*>(&bounds[0]), bounds.size()));
}

void xreg::Intensity2D3DRegiPSO::run()
{
  const auto& opt_vars = *this->opt_vars_;

  const size_type nv = this->num_vols();

  const size_type num_params_per_xform = opt_vars.num_params();

  const size_type tot_num_params = nv * num_params_per_xform;

  this->before_first_iteration();
  
  pso_.set_num_particles(this->num_projs_per_view_);
  
  PtN init_x(tot_num_params);
  init_x.setConstant(0);
  pso_.set_init_guess(init_x);
 
  tmp_params_per_iter_.assign(nv,
                         ListOfScalarLists(this->num_projs_per_view_,
                            ScalarList(num_params_per_xform, 0)));

  PtN regi_param;
  
  std::tie(regi_param, std::ignore) = pso_.compute();

  this->after_last_iteration();
  
  this->update_regi_xforms(this->opt_vec_to_frame_transforms(
                            ScalarList(&regi_param(0),
                                                  &regi_param(0) + tot_num_params)));
}

void xreg::Intensity2D3DRegiPSO::set_print_status_inc(const double inc)
{
  print_status_inc_ = inc;
}

// we'll do this at the start of run()
void xreg::Intensity2D3DRegiPSO::init_opt()
{ }

void xreg::Intensity2D3DRegiPSO::debug_write_comb_sim_score()
{
  if (this->write_combined_sim_scores_to_stream_)
  {
    this->dout() << fmt::format("{:+20.6f}\n", pso_.best_val());
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    this->debug_info_->sims.push_back(pso_.best_val());
  }
}

void xreg::Intensity2D3DRegiPSO::debug_write_opt_pose_vars()
{
  const size_type nv = this->num_vols();
  const size_type num_params_per_xform = this->opt_vars_->num_params();

  const auto& x = pso_.best_param();

  if (this->write_opt_vars_to_stream_)
  {
    size_type x_idx = 0;

    for (size_type v = 0; v < nv; ++v)
    {
      for (size_type i = 0; i < num_params_per_xform; ++i, ++x_idx)
      {
        this->dout() << fmt::format("{:+14.6f}, ", x(x_idx));
      }
      this->dout() << "| ";
    }
    this->dout() << '\n';
  }
  
  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    for (size_type v = 0; v < nv; ++v)
    {
      this->debug_info_->iter_vars[v].push_back(
          x.block(num_params_per_xform * v, 0, num_params_per_xform, 1));
    }
  }
}
    
xreg::Intensity2D3DRegiPSO::PSO::PSO()
{
  obj_fn_type_ = ParticleSwarmOpt::kALL_OBJ_FN;

  percent_of_range_for_init_velocities_ = 0.667;
  //percent_of_range_for_init_velocities_ = 0.0;
}

const xreg::Intensity2D3DRegiPSO::PSO::Vec&
xreg::Intensity2D3DRegiPSO::PSO::best_param() const
{
  return best_param_;
}

const xreg::Intensity2D3DRegiPSO::PSO::Scalar
xreg::Intensity2D3DRegiPSO::PSO::best_val() const
{
  return best_val_;
}

void xreg::Intensity2D3DRegiPSO::PSO::all_obj_fn(const VecList& params, ScalarList& obj_fn_vals)
{
  const size_type nv = regi_->num_vols();
  
  const size_type num_params_per_xform = regi_->opt_vars_->num_params();

  auto& tmp_params = regi_->tmp_params_per_iter_;

  // for each particle
  for (size_type part_idx = 0; part_idx < num_particles_; ++part_idx)
  {
    size_type x_idx = 0;
    for (size_type v = 0; v < nv; ++v)
    {
      for (size_type i = 0; i < num_params_per_xform; ++i, ++x_idx)
      {
        tmp_params[v][part_idx][i] = params[part_idx][x_idx];
      }
    }
  }

  regi_->obj_fn(tmp_params, &obj_fn_vals);
}

void xreg::Intensity2D3DRegiPSO::PSO::start_iter()
{
  regi_->begin_of_iteration(ScalarList(
                         &best_param_(0), &best_param_(0) + best_param_.size()));

  if (kSCALE_PHI_G)
  {     
    const size_type one_third_max_its  = static_cast<size_type>(max_num_its_ * 0.333);
    const size_type two_thirds_max_its = static_cast<size_type>(max_num_its_ * 0.667);

    if (iter_ > two_thirds_max_its)
    {
      phi_g_ = orig_phi_g_;
    }
    else if (iter_ > one_third_max_its)
    {
      phi_g_ = ((iter_ - one_third_max_its) / one_third_max_its) * orig_phi_g_;
    }
  }
}

void xreg::Intensity2D3DRegiPSO::PSO::end_iter()
{
  regi_->end_of_iteration();
  
  ++iter_;
  
  const double cur_comp_percent = static_cast<double>(iter_) / max_num_its_;
  if ((cur_comp_percent + 1.0e-6) > next_print_percent_)
  {
    std::cout << fmt::format("  PSO regi: {:4.2f}% ({} / {} iters)",
                              100 * cur_comp_percent, iter_, max_num_its_) << std::endl;
    next_print_percent_ += regi_->print_status_inc_;
  }
}

void xreg::Intensity2D3DRegiPSO::PSO::after_init_vals()
{
  iter_ = 0;
  
  orig_phi_g_ = phi_g_;
  
  if (kSCALE_PHI_G)
  { 
    phi_g_ = 0;
  }

  next_print_percent_ = regi_->print_status_inc_;
}

