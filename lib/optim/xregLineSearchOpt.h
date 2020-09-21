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

#ifndef XREGLINESEARCHOPTIMIZATION_H_
#define XREGLINESEARCHOPTIMIZATION_H_

#include <boost/variant/variant.hpp>

#include <Eigen/Eigen>

#include "xregCommon.h"
#include "xregObjWithOStream.h"

namespace xreg
{

/// \brief Generic line search optimization aggregate.
///
/// Template parameters specify function object classes for computing objective
/// functions (including gradients and Hessians), computing a search direction,
/// and determining step length.
struct LineSearchOptimization : public ObjWithOStream
{
  using Scalar = CoordScalar;
  using Pt     = PtN;
  using Mat    = MatMxN;

  // Inputs : ( x , compute grad, compute Hessian )
  // Outputs: ( f(x) , grad f(x) , Hessian f(x) )
  using ObjFn = std::function<std::tuple<Scalar,Pt,Mat>(const Pt&,const bool,const bool)>;
  
  // Inputs: ( grad f(x) )
  // Outputs: search direction in parameter space
  using SearchDirFnFirstOrder = std::function<Pt(const Pt&)>;
  
  // Inputs: ( grad f(x) , Hessian f(x) )
  // Outputs: search direction in parameter space
  using SearchDirFnSecOrder = std::function<Pt(const Pt&,const Mat&)>;

  using SearchDirFn = boost::variant<SearchDirFnFirstOrder,SearchDirFnSecOrder>;

  //   Input: ( f , search dir p , x_0, grad f(x_0), compute Hessian )
  // Outputs: ( x_1, f(x_1) , grad f(x_1) , Hessian f(x_1) )
  using BacktrackFn = std::function<std::tuple<Pt,Scalar,Pt,Mat>(const ObjFn&,  // f
                                                                 const Pt&,     // search dir, p
                                                                 const Pt&,     // x_0
                                                                 const Scalar,  // f(x_0)
                                                                 const Pt&,     // grad f(x_0)
                                                                 const bool     // compute Hessian
                                                                )>;

  enum TermStatus
  {
    kGRAD_CONVERGED,
    kMAX_ITS_PERFORMED,
    kNO_POS_CHANGE,
    kOTHER
  };

  ObjFn obj_fn;
  
  SearchDirFn search_dir_fn;
  
  BacktrackFn backtrack_fn;

  Scalar param_tol = 1.0e-8;
  Scalar grad_tol  = 1.0e-8;

  // 0 --> no limit
  size_type max_its = 0;
  
  LineSearchOptimization() = default;

  // no copying
  LineSearchOptimization(const LineSearchOptimization&) = delete;
  LineSearchOptimization& operator=(const LineSearchOptimization&) = delete;

  std::tuple<TermStatus,Pt,Scalar,size_type> solve(const Pt& init_x) const;

  static std::string TermStatusString(const TermStatus status);
};

/// \brief Move in the direction of the negative gradient.
///
/// This would be used for a gradient descent search
LineSearchOptimization::Pt
NegativeGradSearchDir(const LineSearchOptimization::Pt& grad,
                      const LineSearchOptimization::Mat& hessian);

/// \brief Move in the direction of the Newton step.
///
/// Solves for step p: H p = -g
LineSearchOptimization::Pt
NewtonSearchDir(const LineSearchOptimization::Pt& grad,
                const LineSearchOptimization::Mat& hessian);

/// \brief Move in a modified Netwon direction, replaces sufficiently
///        negative definite Hessian with a positive definite version, B.
///
/// Solves for step p: B p = -g
struct ModNewtonSearchDir
{
  using Scalar = LineSearchOptimization::Scalar;
  using Pt     = LineSearchOptimization::Pt;
  using Mat    = LineSearchOptimization::Mat;
  
  /// This assumes symmetric input, therefore this implementation is limited
  /// (mainly) to C2 functions (where the Hessian is symmetric)
  using SpectralDecomp = Eigen::SelfAdjointEigenSolver<Mat>;

  /// \brief Bound on the condition number of the modified Hessian.
  ///
  /// The general rule of thumb is if beta = 10^k, then k digits of precision
  /// are lost. 64-bit double precision number give 15-17 digits of precision,
  /// so a beta on the order of 10^6, should still yield 10 digits of precision.
  /// https://en.wikipedia.org/wiki/Condition_number
  /// https://en.wikipedia.org/wiki/Double-precision_floating-point_format
  Scalar beta = 1.0e6;

  SpectralDecomp spectral_dcomp;

  Pt mod_eigen_vals;

  Mat B;

  Pt operator()(const Pt& grad, const Mat& hessian);

  LineSearchOptimization::SearchDirFn callback_fn();
};

/// \brief Move in a constant, fixed-length, step - no backtracking conditions.
std::tuple<LineSearchOptimization::Pt,
           LineSearchOptimization::Scalar,
           LineSearchOptimization::Pt,
           LineSearchOptimization::Mat>
FixedStepNoBacktracking(const LineSearchOptimization::ObjFn& obj_fn,
                        const LineSearchOptimization::Pt& p,
                        const LineSearchOptimization::Pt& x,
                        const bool compute_hessian,
                        const LineSearchOptimization::Scalar alpha);

xreg::LineSearchOptimization::BacktrackFn
MakeFixedStepNoBacktrackingCallback(const LineSearchOptimization::Scalar alpha);

/// \brief Determine search length via backtracking until the Armijo condition is satisfied.
std::tuple<LineSearchOptimization::Pt,
           LineSearchOptimization::Scalar,
           LineSearchOptimization::Pt,
           LineSearchOptimization::Mat>
BacktrackingArmijo(const LineSearchOptimization::ObjFn& obj_fn,
                   const LineSearchOptimization::Pt& p,
                   const LineSearchOptimization::Pt& x,
                   const LineSearchOptimization::Scalar F,
                   const LineSearchOptimization::Pt& g,
                   const bool compute_hessian,
                   const LineSearchOptimization::Scalar init_alpha, ///< Initial fraction of step.
                   const LineSearchOptimization::Scalar eta,        ///< Slope of line definiing Armijo condition
                   const LineSearchOptimization::Scalar tau         ///< Backtracking scale factor (e.g. 0.5 cuts steps in half)
                  );

xreg::LineSearchOptimization::BacktrackFn
MakeBacktrackingArmijoCallback(const LineSearchOptimization::Scalar alpha = 1,
                               const LineSearchOptimization::Scalar eta   = 0.001,
                               const LineSearchOptimization::Scalar tau   = 0.5);

}  // xreg

#endif
