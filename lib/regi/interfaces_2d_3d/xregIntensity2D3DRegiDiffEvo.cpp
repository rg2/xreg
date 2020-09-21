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

#include "xregIntensity2D3DRegiDiffEvo.h"

#include <fmt/format.h>

#include "xregSE3OptVars.h"
#include "xregIntensity2D3DRegiDebug.h"

xreg::Intensity2D3DRegiDiffEvo::Intensity2D3DRegiDiffEvo()
{
  // default population size
  this->num_projs_per_view_ = diff_evo_.pop_size;
 
  diff_evo_.dither = true;
  diff_evo_.evo_rate = 0.5;
  //diff_evo_.cross_over_prob = 0.9;
  diff_evo_.cross_over_prob = 0.2;
}

xreg::size_type
xreg::Intensity2D3DRegiDiffEvo::max_num_projs_per_view_per_iter() const
{
  return this->num_projs_per_view_;
}

void xreg::Intensity2D3DRegiDiffEvo::set_evo_rate(const CoordScalar F)
{
  diff_evo_.evo_rate = F;
}

void xreg::Intensity2D3DRegiDiffEvo::set_cross_over_prob(const CoordScalar cr)
{
  diff_evo_.cross_over_prob = cr;
}

void xreg::Intensity2D3DRegiDiffEvo::set_pop_size(const size_type ps)
{
  this->num_projs_per_view_ = ps;
}

void xreg::Intensity2D3DRegiDiffEvo::set_max_num_iters(const size_type max_iters)
{
  diff_evo_.max_num_its = max_iters;
}

void xreg::Intensity2D3DRegiDiffEvo::set_box_contraint(const ScalarList& bounds)
{
  const size_type dim = bounds.size();

  diff_evo_.box_constraints.resize(dim);
  for (size_type i = 0; i < dim; ++i)
  {
    diff_evo_.box_constraints(i) = bounds[i];
  }
}

void xreg::Intensity2D3DRegiDiffEvo::run()
{
  const auto& opt_vars = *this->opt_vars_;

  const size_type nv = this->num_vols();

  const size_type num_params_per_xform = opt_vars.num_params();

  const size_type tot_num_params = nv * num_params_per_xform;

  this->before_first_iteration();
  
  ListOfListsOfScalarLists tmp_params_per_iter(nv,
             ListOfScalarLists(this->num_projs_per_view_, ScalarList(num_params_per_xform, 0)));

  ScalarList tmp_obj_fn_vals(this->num_projs_per_view_);

  diff_evo_.pop_size = this->num_projs_per_view_;

  diff_evo_.init_guess.setConstant(tot_num_params, 0);

  diff_evo_.cost_fn = [&] (const PtNList& pts_to_eval)
  {
    for (size_type j = 0; j < this->diff_evo_.pop_size; ++j)
    {
      size_type x_idx = 0;
      for (size_type v = 0; v < nv; ++v)
      {
        for (size_type i = 0; i < num_params_per_xform; ++i, ++x_idx)
        {
          tmp_params_per_iter[v][j][i] = pts_to_eval[j][x_idx];
        }
      }
    }

    this->obj_fn(tmp_params_per_iter, &tmp_obj_fn_vals);

    PtN sim_vals(this->diff_evo_.pop_size);
    
    for (size_type j = 0; j < this->diff_evo_.pop_size; ++j)
    {
      sim_vals[j] = tmp_obj_fn_vals[j];
    }

    return sim_vals;
  };

  diff_evo_.begin_of_iter_callback = [&] (DifferentialEvolution*, const size_type)
  {
    const auto& cur_best_param = diff_evo_.best_param();
    
    this->begin_of_iteration(ScalarList(&cur_best_param(0),
                                        &cur_best_param(0) + cur_best_param.size()));
  };
  
  double next_print_percent = print_status_inc_;

  diff_evo_.end_of_iter_callback = [&] (DifferentialEvolution*, const size_type iter)
  {
    this->end_of_iteration();
    
    const double cur_comp_percent = static_cast<double>(iter) / diff_evo_.max_num_its;
    if ((cur_comp_percent + 1.0e-6) > next_print_percent)
    {
      std::cout << fmt::format("  DE regi: {:4.2f}% ({} / {} iters)",
                                100 * cur_comp_percent, iter, diff_evo_.max_num_its) << std::endl;
      next_print_percent += this->print_status_inc_;
    }
  };

  diff_evo_.run();

  this->after_last_iteration();

  const auto& regi_param = diff_evo_.best_param();
  
  this->update_regi_xforms(this->opt_vec_to_frame_transforms(
                            ScalarList(&regi_param(0), &regi_param(0) + tot_num_params)));
}

void xreg::Intensity2D3DRegiDiffEvo::set_print_status_inc(const double inc)
{
  print_status_inc_ = inc;
}

void xreg::Intensity2D3DRegiDiffEvo::init_opt()
{ }

void xreg::Intensity2D3DRegiDiffEvo::debug_write_comb_sim_score()
{
  if (this->write_combined_sim_scores_to_stream_)
  {
    this->dout() << fmt::format("{:+20.6f}\n", diff_evo_.best_fn_val);
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    this->debug_info_->sims.push_back(diff_evo_.best_fn_val);
  }
}

void xreg::Intensity2D3DRegiDiffEvo::debug_write_opt_pose_vars()
{
  const size_type nv = this->num_vols();
  const size_type num_params_per_xform = this->opt_vars_->num_params();

  const auto& x = diff_evo_.cur_pop[diff_evo_.best_pop_idx];

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
