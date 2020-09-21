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

#include "xregIntensity2D3DRegiNLOptInterface.h"

#include <fmt/format.h>

#include "xregIntensity2D3DRegiDebug.h"
#include "xregSE3OptVars.h"
#include "xregAssert.h"
#include "xregExceptionUtils.h"
  
xreg::Intensity2D3DRegiNLOptInterface::Intensity2D3DRegiNLOptInterface(
                                const nlopt::algorithm nlopt_alg_id,
                                const bool requires_bounds,
                                const bool stoch_pop)
  : nlopt_alg_id_(nlopt_alg_id), requires_bounds_(requires_bounds), stoch_pop_(stoch_pop)
{
  this->num_projs_per_view_ = 1;
  stoch_pop_size_ = 0;

  set_opt_obj_fn_tol(1.0e-3);
  set_opt_x_tol(-1);
}

void xreg::Intensity2D3DRegiNLOptInterface::set_bounds(const ScalarList& bounds)
{
  bounds_ = bounds;
}

void xreg::Intensity2D3DRegiNLOptInterface::remove_bounds()
{
  bounds_.clear();
}

void xreg::Intensity2D3DRegiNLOptInterface::set_steps(const ScalarList& steps)
{
  steps_ = steps;
}

void xreg::Intensity2D3DRegiNLOptInterface::remove_steps()
{
  steps_.clear();
}

void xreg::Intensity2D3DRegiNLOptInterface::set_stochastic_pop_size(const size_type stoch_pop_size)
{
  stoch_pop_size_ = stoch_pop_size;
}

void xreg::Intensity2D3DRegiNLOptInterface::reset_stochastic_pop_size()
{
  stoch_pop_size_ = 0;
}

xreg::size_type
xreg::Intensity2D3DRegiNLOptInterface::max_num_projs_per_view_per_iter() const
{
  return this->num_projs_per_view_;
}

void xreg::Intensity2D3DRegiNLOptInterface::run()
{
  const auto& opt_vars = *this->opt_vars_;

  const size_type nv = this->num_vols();

  const size_type num_params_per_xform = opt_vars.num_params();

  const size_type tot_num_params = nv * num_params_per_xform;

  // BOBYQA requires more than one parameter ... TODO should this be here?
  xregASSERT(tot_num_params > 1);

  opt_vec_space_vals_.assign(nv, ListOfScalarLists(1, ScalarList(num_params_per_xform, 0)));

  s_.resize(1);

  nlopt::opt opt_obj(nlopt_alg_id_, tot_num_params);

  opt_obj.set_min_objective(NLOptObjFn, this);

  // default convergence tolerances
  opt_obj.set_ftol_abs(-1);  // no absolute tolerance (e.g. abs(f(x)) < eps)
  opt_obj.set_ftol_rel(obj_fn_tol_);
  opt_obj.set_xtol_rel(x_tol_);

  // empty steps, allows the algorithm to choose the initial steps
  xregASSERT(steps_.empty() || (steps_.size() == tot_num_params));
  if (!steps_.empty())
  {
    opt_obj.set_initial_step(std::vector<double>(steps_.begin(), steps_.end()));
  }

  if (!bounds_.empty())
  {
    // set bounds
    // remove any signs from passed offsets
    std::transform(bounds_.begin(), bounds_.end(), bounds_.begin(),
                   [] (const Scalar& x) { return std::abs(x); });

    opt_obj.set_upper_bounds(std::vector<double>(bounds_.begin(), bounds_.end()));

    // negate bounds signs
    std::transform(bounds_.begin(), bounds_.end(), bounds_.begin(),
                   [] (const Scalar& x) { return -x; });

    opt_obj.set_lower_bounds(std::vector<double>(bounds_.begin(), bounds_.end()));
  }
  else if (requires_bounds_)
  {
    xregThrow("This NLOpt Alg requires bounds!");
  }

  if (stoch_pop_size_)
  {
    if (stoch_pop_)
    {
      opt_obj.set_population(stoch_pop_size_);
    }
    else
    {
      xregThrow("Stochastic Population not supported!");
    }
  }

  std::vector<double> regi_x(tot_num_params, 0);

  double regi_sim_val = std::numeric_limits<double>::max();

  if (this->max_num_iters_ < std::numeric_limits<size_type>::max())
  {
    set_nl_opt_max_num_fn_evals_for_max_iters(&opt_obj);
  }

  this->before_first_iteration();

  try
  {
    nlopt::result opt_result = opt_obj.optimize(regi_x, regi_sim_val);

    if (opt_result <= 0)
    {
      xregThrow("nopt did not return success! returned: %d", opt_result);
    }
  }
  catch (const nlopt::roundoff_limited&)
  { }

  this->after_last_iteration();

  FrameTransformList delta_xforms(this->num_vols());
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    delta_xforms[vol_idx] = this->opt_vars_->operator()(Eigen::Map<PtN_d>(&regi_x[vol_idx * num_params_per_xform],
                                                          num_params_per_xform).cast<Scalar>());
  }

  this->update_regi_xforms(delta_xforms);

  if (this->src_and_obj_pose_opt_vars_)
  {
    this->regi_cam_models_.assign(1,
        this->src_and_obj_pose_opt_vars_->cam(Eigen::Map<PtN_d>(&regi_x[0],
                                                          num_params_per_xform).cast<Scalar>())); 
  }
}

void xreg::Intensity2D3DRegiNLOptInterface::set_opt_obj_fn_tol(const Scalar& tol)
{
  obj_fn_tol_ = tol;
}

void xreg::Intensity2D3DRegiNLOptInterface::set_opt_x_tol(const Scalar& tol)
{
  x_tol_ = tol;
}

void xreg::Intensity2D3DRegiNLOptInterface::init_opt()
{
  // we'll init during the run call
}

void xreg::Intensity2D3DRegiNLOptInterface::debug_write_comb_sim_score()
{
  if (this->write_combined_sim_scores_to_stream_)
  {
    this->dout() << fmt::format("{:+20.6f}\n", s_[0]);
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    this->debug_info_->sims.push_back(s_[0]);
  }
}

void xreg::Intensity2D3DRegiNLOptInterface::debug_write_opt_pose_vars()
{
  const size_type nv = this->num_vols();
  const size_type num_params_per_xform = this->opt_vars_->num_params();

  if (this->write_opt_vars_to_stream_)
  {
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      for (size_type param_idx = 0; param_idx < num_params_per_xform; ++param_idx)
      {
        this->dout() << fmt::format("{:+14.6f}, ", opt_vec_space_vals_[vol_idx][0][param_idx]);
      }
      this->dout() << "| ";
    }
    this->dout() << '\n';
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      this->debug_info_->iter_vars[vol_idx].push_back(
            Eigen::Map<PtN>(const_cast<Scalar*>(&opt_vec_space_vals_[vol_idx][0][0]),
                            num_params_per_xform));
    }
  }
}

void xreg::Intensity2D3DRegiNLOptInterface::set_nl_opt_max_num_fn_evals_for_max_iters(nlopt::opt* opt_obj)
{
  // right now I do not have a good way to enforce maximum number of iterations
  // since NLopt only counts objective function evals, each optimization sub-class
  // should override this method of they are capable of handling this.
  xregThrow("max num iters unsupported by default for NL opt interface!");
}

double xreg::Intensity2D3DRegiNLOptInterface::NLOptObjFn(
                         const std::vector<double>& x,
                         std::vector<double>& XREG_NO_ASSERTS_UNUSED(grad),
                         void* regi_obj_void)
{
  xregASSERT(grad.empty());

  auto& regi = *static_cast<Intensity2D3DRegiNLOptInterface*>(regi_obj_void);

  regi.begin_of_iteration(ScalarList(x.begin(), x.end()));

  const size_type nv = regi.num_vols();

  const size_type num_params_per_xform = regi.opt_vars_->num_params();

  ScalarList tmp_vol_xform(num_params_per_xform);

  size_type start_x_idx = 0;
  for (size_type v = 0; v < nv; ++v, start_x_idx += num_params_per_xform)
  {
    tmp_vol_xform.assign(x.begin() + start_x_idx, x.begin() + start_x_idx + num_params_per_xform);
    regi.opt_vec_space_vals_[v].assign(1, tmp_vol_xform);
  }
  // regi.opt_vec_space_vals[i][0][k] refers to the kth parameter of the ith volume's transform.

  regi.s_[0] = 0;
  regi.obj_fn(regi.opt_vec_space_vals_, &regi.s_);

  // With NLOpt this is the best we can do for end of iteration
  regi.end_of_iteration();

  return regi.s_[0];
}

