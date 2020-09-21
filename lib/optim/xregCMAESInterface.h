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

#ifndef XREGCMAESINTERFACE_H_
#define XREGCMAESINTERFACE_H_

#include "xregCommon.h"
#include "xregExceptionUtils.h"
#include "xregTBBUtils.h"

namespace xreg
{

/**
 * @brief Generic CMA-ES class; sub-classes provide the objective function
 * implementations.
 *
 * This class uses the C version of CMA-ES provided by Nikolaus Hansen at
 * https://www.lri.fr/~hansen/cmaes_inmatlab.html#C.
 **/
class CmaesOptimizer
{
public:
  using Scalar = CoordScalar;
  using Pt     = PtN;
  using Mat    = MatMxN;

  xregDeriveStringMessageException(UnsupportedObjFnException);

  CmaesOptimizer();

  CmaesOptimizer(const Pt& init_guess);

  CmaesOptimizer(const Pt& init_guess, const Pt& init_sigma);

  CmaesOptimizer(const Pt& init_guess, const size_type pop_size);

  CmaesOptimizer(const Pt& init_guess, const Pt& init_sigma,
                 const size_type pop_size);

  CmaesOptimizer(const size_type dim);

  CmaesOptimizer(const size_type dim, const size_type pop_size);

  virtual ~CmaesOptimizer() { }
  
  // No copying
  CmaesOptimizer(const CmaesOptimizer&) = delete;
  CmaesOptimizer& operator=(const CmaesOptimizer&) = delete;

  void set_init_guess(const Pt& guess);

  void set_sigma(const Pt& sigma);

  void set_pop_size(const size_type l);

  size_type pop_size() const;

  virtual void run();

  const Pt& sol() const;
  
  const Pt& solution() const;

  size_type num_its() const;

  void set_scales(const Pt& s);

  void set_scale(const double s, const size_type i);

  /// \brief Remove any constraints and run unconstrained.
  void set_as_unc();

  /// \brief Set basic box constraints, upper and lower bounds in each parameter dimension
  ///
  /// These are in the un-scaled space (that the user sees when setting guesses, etc.)
  void set_box_constraints(const Pt& lower, const Pt& upper);

private:
  struct ParallelObjFnObj
  {
    CmaesOptimizer* cmaes_opt;

    size_type dim;
    const Mat& pop;
    double* obj_fn_vals;

    void operator()(const RangeType& r) const;
  };

  static constexpr double kDEFAULT_SIGMA = 0.3;

  enum { kDEFAULT_LAMBDA = 50 };

protected:

  enum ObjFnExecType
  {
    kSERIAL_OBJ_FN_EVAL = 0,
    kPARALLEL_OBJ_FN_EVAL,
    kENTIRE_POP_OBJ_FN_EVAL,
    kDEFAULT_OBJ_FN_EVAL = kSERIAL_OBJ_FN_EVAL
  };

  Pt scale_for_opt(const Pt& v);

  Pt unscale_from_opt(const Pt& v);

  virtual double obj_fn(const Pt& x);

  virtual void all_obj_fns(const Mat& pop, double* obj_fn_vals);

  virtual void setup_optimization() { }

  virtual void end_optimization() { }

  virtual void start_of_iteration() { }

  // vector is the current estimate of the solution
  virtual void end_of_iteration(const Pt&) { }

  Pt init_sigma_;
  Pt init_guess_;
  Pt sol_;

  Pt scales_;

  bool run_unc_;
  Pt box_lower_bounds_;
  Pt box_upper_bounds_;

  size_type num_its_;
  size_type pop_size_;

  ObjFnExecType obj_fn_exec_type_;
};

}  // xreg

#endif
