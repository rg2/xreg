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

#ifndef XREGINTENSITY2D3DREGINLOPTINTERFACE_H_
#define XREGINTENSITY2D3DREGINLOPTINTERFACE_H_

#include <nlopt.hpp>

#include "xregIntensity2D3DRegi.h"

namespace xreg
{

/// \brief 2D/3D Intensity-Based Registration object using NLOpt optimization methods.
///
/// Typically a derived class is used to specify values of tRequiresBounds and tNLOPTAlgID.
//template <class tRayCaster, class tSimMetric, nlopt::algorithm tNLOPTAlgID,
//          bool tRequiresBounds, bool tStochPop>
class Intensity2D3DRegiNLOptInterface : public Intensity2D3DRegi
{
public:
  /// \brief Constructor; performs no work.
  Intensity2D3DRegiNLOptInterface(const nlopt::algorithm nlopt_alg_id,
                                  const bool requires_bounds,
                                  const bool stoch_pop);

  /// \brief Set bounds.
  ///
  /// These are box constraints with the valid range in component k equal to
  /// [-bounds[k], bounds[k]]. If kREQUIRES_BOUNDS is false, this will trigger
  /// constrained optimization.
  void set_bounds(const ScalarList& bounds);

  /// \brief Remove user specified bounds.
  ///
  /// If kREQUIRES_BOUNDS is false, this will trigger uncontrained optimization
  void remove_bounds();

  /// \brief Sets the initial step sizes to use.
  void set_steps(const ScalarList& steps);

  /// \brief Remove user specified steps.
  ///
  /// The NLOPT implementation will choose initial step sizes to use.
  void remove_steps();

  /// \brief Sets a stochastic population size to use
  ///
  /// This is only valid for the global algorithms using stochastic populations.
  void set_stochastic_pop_size(const size_type stoch_pop_size);

  /// \brief reset the stochastic population size.
  ///
  /// The NLopt implementation will choose the size.
  void reset_stochastic_pop_size();

  size_type max_num_projs_per_view_per_iter() const override;

  /// \brief Performs the registration; blocks until completion.
  void run() override;

  void set_opt_obj_fn_tol(const Scalar& tol) override;

  void set_opt_x_tol(const Scalar& tol) override;

protected:
  void init_opt() override;

  void debug_write_comb_sim_score() override;

  void debug_write_opt_pose_vars() override;

  virtual void set_nl_opt_max_num_fn_evals_for_max_iters(nlopt::opt* opt_obj);

private:

  /// \brief C-style interface function implementing the objective function
  ///       passed to the NLOpt implementation.
  static double NLOptObjFn(const std::vector<double>& x,
                           std::vector<double>& grad,
                           void* regi_obj_void);
  
  const nlopt::algorithm nlopt_alg_id_;
  
  /// \brief Indicator if bounds are required.
  ///
  /// When required and none are specified the run() method will throw an
  /// exception. When not required and none are specified the algorithm will
  /// operate in unconstrained fashion.
  const bool requires_bounds_;
  
  const bool stoch_pop_;

  ScalarList steps_;

  ScalarList bounds_;

  size_type stoch_pop_size_;

  ListOfListsOfScalarLists opt_vec_space_vals_;

  ScalarList s_;

  Scalar obj_fn_tol_;
  Scalar x_tol_;
};

}  // xreg

#endif
