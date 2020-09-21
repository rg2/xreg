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

#include "xregLineSearchOpt.h"

#include <boost/variant.hpp>

#include <fmt/printf.h>

#include "xregAssert.h"

namespace
{

using namespace xreg;

using Pt  = LineSearchOptimization::Pt;
using Mat = LineSearchOptimization::Mat;

using SearchDirFnFirstOrder = LineSearchOptimization::SearchDirFnFirstOrder;
using SearchDirFnSecOrder   = LineSearchOptimization::SearchDirFnSecOrder;

struct SearchDirNeedsHessian : public boost::static_visitor<bool>
{
  bool operator()(const SearchDirFnFirstOrder& search_fn) const
  {
    xregASSERT(bool(search_fn));
    return false;
  }

  bool operator()(const SearchDirFnSecOrder& search_fn) const
  {
    xregASSERT(bool(search_fn));
    return true;
  }
};

struct ComputeSearchDir : public boost::static_visitor<Pt>
{
  const Pt*  g;
  const Mat* H;

  Pt operator()(const SearchDirFnFirstOrder& search_fn) const
  {
    return search_fn(*g);
  }
  
  Pt operator()(const SearchDirFnSecOrder& search_fn) const
  {
    return search_fn(*g, *H);
  }
};

}  // un-named

std::tuple<xreg::LineSearchOptimization::TermStatus,
           xreg::LineSearchOptimization::Pt,
           xreg::LineSearchOptimization::Scalar,
           xreg::size_type>
xreg::LineSearchOptimization::solve(const Pt& init_x) const
{
  xregASSERT(bool(obj_fn));
  xregASSERT(bool(backtrack_fn));

  TermStatus status = kOTHER;

  Scalar cur_F;
  Pt cur_x = init_x;
  Pt cur_grad;
  Mat cur_H;

  // search direction
  Pt p;

  Pt prev_x;

  const bool requires_hessian = boost::apply_visitor(SearchDirNeedsHessian(), search_dir_fn);
  
  std::tie(cur_F,cur_grad,cur_H) = obj_fn(cur_x, true, requires_hessian);

  Scalar cur_grad_norm = cur_grad.norm();

  const Scalar grad_term_tol = std::max(Scalar(1), cur_grad_norm) * grad_tol;

  bool should_stop = false;

  size_type cur_it = 0;

  dout() << " Iter.      F       ||grad F||" << std::endl;

  while (!should_stop)
  {
    dout() << fmt::sprintf("%04lu   %+10.4f  %+10.4f", cur_it, cur_F, cur_grad_norm) << std::endl;

    if (cur_grad_norm <= grad_term_tol)
    {
      status = kGRAD_CONVERGED;
      should_stop = true;
    }
    else if ((cur_it > 0) && ((prev_x - cur_x).norm() < param_tol))
    {
      status = kNO_POS_CHANGE;
      should_stop = true;
    }
    else if (max_its && (cur_it >= max_its))
    {
      status = kMAX_ITS_PERFORMED;
      should_stop = true;
    }
    else
    {
      {
        ComputeSearchDir compute_search_dir_visitor;
        compute_search_dir_visitor.g = &cur_grad;
        compute_search_dir_visitor.H = &cur_H;

        p = boost::apply_visitor(compute_search_dir_visitor, search_dir_fn);
      }

      prev_x = cur_x;

      std::tie(cur_x,cur_F,cur_grad,cur_H) = backtrack_fn(obj_fn, p, prev_x, cur_F, cur_grad, requires_hessian);

      cur_grad_norm = cur_grad.norm();

      ++cur_it;
    }
  }

  dout() << "Termination Status: " << TermStatusString(status) << std::endl;

  return std::make_tuple(status, cur_x, cur_F, cur_it);
}

std::string xreg::LineSearchOptimization::TermStatusString(const TermStatus status)
{
  std::string s;

  switch (status)
  {
  case kGRAD_CONVERGED:
    s = "Gradient Converged";
    break;
  case kMAX_ITS_PERFORMED:
    s = "Maximum Iterations Performed";
    break;
  case kNO_POS_CHANGE:
    s = "Insufficient Parameter Change";
    break;
  case kOTHER:
  default:
    s = "Other/Unknown";
    break;
  }

  return s;
}

xreg::LineSearchOptimization::Pt
xreg::NegativeGradSearchDir(const LineSearchOptimization::Pt& grad,
                            const LineSearchOptimization::Mat& /*hessian*/)
{
  return -1 * grad;
}

xreg::LineSearchOptimization::Pt
xreg::NewtonSearchDir(const LineSearchOptimization::Pt& grad,
                      const LineSearchOptimization::Mat& hessian)
{
  using Mat = LineSearchOptimization::Mat;

  // solves hessian \ -grad
  return Eigen::JacobiSVD<Mat>(hessian, Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-1 * grad);
}

xreg::ModNewtonSearchDir::Pt xreg::ModNewtonSearchDir::operator()(const Pt& grad, const Mat& hessian)
{
  spectral_dcomp.compute(hessian, Eigen::ComputeEigenvectors);

  const SpectralDecomp::RealVectorType& eig_vals = spectral_dcomp.eigenvalues();

  const Scalar H_L2_norm = eig_vals.array().abs().maxCoeff();

  const Scalar eps = (H_L2_norm > 1.0e-6) ? (H_L2_norm / beta) : Scalar(1);

  bool use_hessian = true;

  const unsigned long d = static_cast<unsigned long>(grad.size());

  mod_eigen_vals.resize(d);

  for (unsigned long i = 0; i < d; ++i)
  {
    if (eig_vals(i) >= eps)
    {
      mod_eigen_vals(i) = eig_vals(i);
    }
    else if (eig_vals(i) <= -eps)
    {
      mod_eigen_vals(i) = -eig_vals(i);
      use_hessian = false;
    }
    else
    {
      mod_eigen_vals(i) = eps;
      use_hessian = false;
    }
  }

  if (!use_hessian)
  {
    const Mat& eigen_vecs = spectral_dcomp.eigenvectors();

    B = eigen_vecs * mod_eigen_vals.asDiagonal() * eigen_vecs.transpose();
  }

  // solve B \ -grad
  return NewtonSearchDir(grad, use_hessian ? hessian : B);
}
  
xreg::LineSearchOptimization::SearchDirFn xreg::ModNewtonSearchDir::callback_fn()
{
  return [this] (const Pt& grad, const Mat& hessian) { return this->operator()(grad, hessian); };
}

std::tuple<xreg::LineSearchOptimization::Pt,
           xreg::LineSearchOptimization::Scalar,
           xreg::LineSearchOptimization::Pt,
           xreg::LineSearchOptimization::Mat>
xreg::FixedStepNoBacktracking(const LineSearchOptimization::ObjFn& obj_fn,
                              const LineSearchOptimization::Pt& p,
                              const LineSearchOptimization::Pt& x,
                              const bool compute_hessian,
                              const LineSearchOptimization::Scalar alpha)
{
  using Pt     = LineSearchOptimization::Pt;
  using Mat    = LineSearchOptimization::Mat;
  using Scalar = LineSearchOptimization::Scalar;
  
  Pt next_x = x + (alpha * p);
  
  Scalar next_F;
  Pt next_g;
  Mat next_H;

  std::tie(next_F, next_g, next_H) = obj_fn(next_x, true, compute_hessian);

  return std::make_tuple(next_x, next_F, next_g, next_H);
}

xreg::LineSearchOptimization::BacktrackFn
xreg::MakeFixedStepNoBacktrackingCallback(const LineSearchOptimization::Scalar alpha)
{
  return [alpha] (const LineSearchOptimization::ObjFn& obj_fn,
                  const LineSearchOptimization::Pt& p,
                  const LineSearchOptimization::Pt& x,
                  const LineSearchOptimization::Scalar /*F*/,
                  const LineSearchOptimization::Pt& /*g*/,
                  const bool compute_hessian)
  {
    return FixedStepNoBacktracking(obj_fn, p, x, compute_hessian, alpha);
  };
}

std::tuple<xreg::LineSearchOptimization::Pt,
           xreg::LineSearchOptimization::Scalar,
           xreg::LineSearchOptimization::Pt,
           xreg::LineSearchOptimization::Mat>
xreg::BacktrackingArmijo(const LineSearchOptimization::ObjFn& obj_fn,
                         const LineSearchOptimization::Pt& p,
                         const LineSearchOptimization::Pt& x,
                         const LineSearchOptimization::Scalar F,
                         const LineSearchOptimization::Pt& g,
                         const bool compute_hessian,
                         const LineSearchOptimization::Scalar init_alpha,
                         const LineSearchOptimization::Scalar eta,
                         const LineSearchOptimization::Scalar tau)
{
  using Pt     = LineSearchOptimization::Pt;
  using Mat    = LineSearchOptimization::Mat;
  using Scalar = LineSearchOptimization::Scalar;

  const Scalar eta_grad_dot_p = eta * g.dot(p);
  
  Scalar alpha = init_alpha;
  
  Pt next_x = x + (alpha * p);
  
  Scalar next_F;

  // passing false for gradient and hessian defers their computation until
  // after we stop backtracking - this really helps for expensive computations

  std::tie(next_F,std::ignore,std::ignore) = obj_fn(next_x, false, false);

  while (next_F > (F + (alpha * eta_grad_dot_p)))
  {
    alpha *= tau;
    next_x = x + (alpha * p);
    std::tie(next_F,std::ignore,std::ignore) = obj_fn(next_x, false, false);
  }
  
  Pt next_g;

  Mat next_H;

  std::tie(next_F,next_g,next_H) = obj_fn(next_x, true, compute_hessian);
  
  return std::make_tuple(next_x, next_F, next_g, next_H);
}

xreg::LineSearchOptimization::BacktrackFn
xreg::MakeBacktrackingArmijoCallback(const LineSearchOptimization::Scalar alpha,
                                     const LineSearchOptimization::Scalar eta,
                                     const LineSearchOptimization::Scalar tau)
{
  return [alpha,eta,tau] (const LineSearchOptimization::ObjFn& obj_fn,
                          const LineSearchOptimization::Pt& p,
                          const LineSearchOptimization::Pt& x,
                          const LineSearchOptimization::Scalar F,
                          const LineSearchOptimization::Pt& g,
                          const bool compute_hessian)
  {
    return BacktrackingArmijo(obj_fn, p, x, F, g, compute_hessian, alpha, eta, tau);
  };
}

