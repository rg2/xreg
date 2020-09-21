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

#include "xregCMAESInterface.h"

#include <cmaes_interface.h>

#include "xregAssert.h"

xreg::CmaesOptimizer::CmaesOptimizer()
{
  run_unc_ = true;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const Pt& init_guess)
{
  xregASSERT(init_guess.size() > 0);

  run_unc_ = true;

  init_guess_ = init_guess;
  init_sigma_ = Pt::Ones(init_guess.size()) * kDEFAULT_SIGMA;
  scales_     = Pt::Ones(init_guess.size());

  pop_size_ = kDEFAULT_LAMBDA;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const Pt& init_guess, const Pt& init_sigma)
{
  xregASSERT(init_sigma.size() > 0);
  xregASSERT(init_sigma.size() == init_guess.size());

  run_unc_ = true;

  init_guess_ = init_guess;
  init_sigma_ = init_sigma;
  scales_     = Pt::Ones(init_guess.size());

  pop_size_ = kDEFAULT_LAMBDA;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const Pt& init_guess, const size_type pop_size)
{
  xregASSERT(init_guess.size() > 0);
  xregASSERT(pop_size > 0);

  run_unc_ = true;

  init_guess_ = init_guess;
  init_sigma_ = Pt::Ones(init_guess.size()) * kDEFAULT_SIGMA;
  scales_     = Pt::Ones(init_guess.size());

  pop_size_ = pop_size;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const Pt& init_guess, const Pt& init_sigma,
                                     const size_type pop_size)
{
  xregASSERT(init_guess.size() > 0);
  xregASSERT(init_sigma.size() == init_guess.size());
  xregASSERT(pop_size > 0);

  run_unc_ = true;

  init_guess_ = init_guess;
  init_sigma_ = init_sigma;
  scales_     = Pt::Ones(init_guess.size());

  pop_size_ = pop_size;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const size_type dim)
{
  xregASSERT(dim > 0);

  run_unc_ = true;

  init_guess_ = Pt::Zero(dim);
  init_sigma_ = Pt::Ones(dim) * kDEFAULT_SIGMA;
  scales_     = Pt::Ones(dim);

  pop_size_ = kDEFAULT_LAMBDA;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

xreg::CmaesOptimizer::CmaesOptimizer(const size_type dim, const size_type pop_size)
{
  xregASSERT(dim > 0);
  xregASSERT(pop_size > 0);

  run_unc_ = true;

  init_guess_ = Pt::Zero(dim);
  init_sigma_ = Pt::Ones(dim) * kDEFAULT_SIGMA;
  scales_     = Pt::Ones(dim);

  pop_size_ = pop_size;

  obj_fn_exec_type_ = kDEFAULT_OBJ_FN_EVAL;
}

void xreg::CmaesOptimizer::set_init_guess(const Pt& guess)
{
  init_guess_ = guess;
}

void xreg::CmaesOptimizer::set_sigma(const Pt& sigma)
{
  init_sigma_ = sigma;
}

void xreg::CmaesOptimizer::set_pop_size(const size_type l)
{
  xregASSERT(l > 0);
  pop_size_ = l;
}
  
xreg::size_type xreg::CmaesOptimizer::pop_size() const
{
  return pop_size_;
}

void xreg::CmaesOptimizer::ParallelObjFnObj::operator()(const xreg::RangeType& r) const
{
  for (RangeType::size_type i = r.begin(); i < r.end(); ++i)
  {
    obj_fn_vals[i] = cmaes_opt->obj_fn(pop.col(i));
  }
}

void xreg::CmaesOptimizer::run()
{
  xregASSERT(pop_size_ > 0);
  xregASSERT(init_sigma_.size() == init_guess_.size());
  xregASSERT(scales_.size() == init_guess_.size());

  // call-back optionally implemented by sub-classes
  setup_optimization();

  const size_type dim = init_guess_.size();

  num_its_ = 0;

  sol_ = init_guess_;

  // will hold population, but scaled out of optimization space
  Mat pop_mat(dim, pop_size_);

  Pt scaled_init_guess = scale_for_opt(init_guess_);

  Pt x_in_bounds;
  if (!run_unc_)
  {
    xregASSERT(static_cast<size_type>(box_lower_bounds_.size()) == dim);
    xregASSERT(static_cast<size_type>(box_upper_bounds_.size()) == dim);

    x_in_bounds.resize(dim);
  }

  // The return value will store the objective function values at each of the
  // lambda search points
  // The "non" string parameter indicates that no parameter file should be read
  // or written to.
  cmaes_t evo;

  memset(&evo, 0, sizeof(evo));
  
  PtN_d tmp_scaled_init_guess = scaled_init_guess.cast<double>();
  PtN_d tmp_init_sigma        = init_sigma_.cast<double>();

  double* obj_fn_vals = cmaes_init(&evo, static_cast<int>(dim), &tmp_scaled_init_guess(0),
                                   &tmp_init_sigma(0), 0, static_cast<int>(pop_size_), "non");

  while (!cmaes_TestForTermination(&evo))
  {
    // call-back optionally implemented by sub-classes
    start_of_iteration();

    // generate lambda new search points, sample population
    // This is an array of double arrays, e.g. pop[i] has dim doubles.
    double* const* pop = cmaes_SamplePopulation(&evo);  // do not change content of pop

    if (run_unc_)
    {
      // rescale the data from optimization space
      for (size_type pop_ind = 0; pop_ind < pop_size_; ++pop_ind)
      {
        pop_mat.col(pop_ind) = unscale_from_opt(PtN_d::Map(pop[pop_ind], dim).cast<Scalar>());
      }
    }
    else
    {
      // rescale from optimization space, then enforce constraints
      for (size_type pop_ind = 0; pop_ind < pop_size_; ++pop_ind)
      {
        x_in_bounds = unscale_from_opt(PtN_d::Map(pop[pop_ind], dim).cast<Scalar>());

        while (((x_in_bounds - box_lower_bounds_).array().minCoeff() < 0) ||
               ((box_upper_bounds_ - x_in_bounds).array().minCoeff() < 0))
        {
          cmaes_ReSampleSingle(&evo, static_cast<int>(pop_ind));
          x_in_bounds = unscale_from_opt(PtN_d::Map(pop[pop_ind], dim).cast<Scalar>());
        }

        pop_mat.col(pop_ind) = x_in_bounds;
      }
    }

    // for each of the population parameterizations, compute the objective function
    // These options allow an serial evaluation, a parallel for, or something
    // that takes all the parameterizations and does some very smart threading
    switch (obj_fn_exec_type_)
    {
    case kENTIRE_POP_OBJ_FN_EVAL:
    {
      all_obj_fns(pop_mat, obj_fn_vals);
      break;
    }
    case kPARALLEL_OBJ_FN_EVAL:
    {
      ParallelObjFnObj obj_fn_obj = { this, dim, pop_mat, obj_fn_vals };
      ParallelFor(obj_fn_obj, xreg::RangeType(0, pop_size_));
      break;
    }
    case kSERIAL_OBJ_FN_EVAL:
    default:
      for (size_type cur_pt = 0; cur_pt < pop_size_; ++cur_pt)
      {
        obj_fn_vals[cur_pt] = obj_fn(pop_mat.col(cur_pt));
      }
      break;
    }

    // update the search distribution used for cmaes_SamplePopulation()
    cmaes_UpdateDistribution(&evo, obj_fn_vals);

    // call-back optionally implemented by sub-classes
    end_of_iteration(unscale_from_opt(PtN_d::Map(cmaes_GetPtr(&evo, "xmean"), dim).cast<Scalar>()));

    ++num_its_;
  }

  // "xbestever" might be used as well
  sol_ = unscale_from_opt(PtN_d::Map(cmaes_GetPtr(&evo, "xmean"), dim).cast<Scalar>());

  // clean up any memory allocated interally to CMAES
  cmaes_exit(&evo);

  // call-back optionally implemented by sub-classes
  end_optimization();
}
  
const xreg::CmaesOptimizer::Pt& xreg::CmaesOptimizer::sol() const
{
  return sol_;
}
  
const xreg::CmaesOptimizer::Pt& xreg::CmaesOptimizer::solution() const
{
  return sol();
}

xreg::size_type xreg::CmaesOptimizer::num_its() const
{
  return num_its_;
}

void xreg::CmaesOptimizer::set_scales(const Pt& s)
{
  scales_ = s;
}

void xreg::CmaesOptimizer::set_scale(const double s, const size_type i)
{
  scales_(i) = s;
}

void xreg::CmaesOptimizer::set_as_unc()
{
  run_unc_ = true;
}

void xreg::CmaesOptimizer::set_box_constraints(const Pt& lower, const Pt& upper)
{
  run_unc_ = false;
  box_lower_bounds_ = lower;
  box_upper_bounds_ = upper;
}

double xreg::CmaesOptimizer::obj_fn(const Pt&)
{
  throw UnsupportedObjFnException("obj_fn() not implemented!");
}

void xreg::CmaesOptimizer::all_obj_fns(const Mat&, double*)
{
  throw UnsupportedObjFnException("all_obj_fns() not implemented!");
}

xreg::CmaesOptimizer::Pt xreg::CmaesOptimizer::scale_for_opt(const Pt& v)
{
  Pt scaled = v.array() / scales_.array();

  return scaled;
}

xreg::CmaesOptimizer::Pt xreg::CmaesOptimizer::unscale_from_opt(const Pt& v)
{
  Pt unscaled = v.array() * scales_.array();

  return unscaled;
}
