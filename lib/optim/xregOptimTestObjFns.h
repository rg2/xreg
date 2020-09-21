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

#ifndef XREGOPTTESTOBJFNS_H_
#define XREGOPTTESTOBJFNS_H_

#include <tuple>

#include "xregCommon.h"
#include "xregExceptionUtils.h"

namespace xreg
{

// https://en.wikipedia.org/wiki/Test_functions_for_optimization

template <unsigned long N>
std::tuple<CoordScalar,PtN,MatMxN> SphereFnND(const PtN& x, const bool compute_grad, const bool compute_hessian)
{
  PtN    grad;
  MatMxN hessian;

  if (compute_grad)
  {
    grad = x.array() * CoordScalar(2);
  }

  if (compute_hessian)
  {
    hessian.resize(N,N);

    hessian.setZero();

    for (unsigned long i = 0; i < N; ++i)
    {
      hessian(i,i) = 2;
    }
  }

  return std::make_tuple(x.array().square().sum(), grad, hessian);
}
  
/// f(x,y) = (a - x)^2 + b(y - (x^2))^2
/// Global minimum at (a,a^2).
struct RosenbrockFn2D
{
  CoordScalar a = 1;
  CoordScalar b = 100;
  
  std::tuple<CoordScalar,PtN,MatMxN>
  operator()(const PtN& x, const bool compute_grad, const bool compute_hessian) const;
};

template <unsigned long N>
struct RosenbrockFnND
{
  static_assert(N >= 2, "Rosenbrock must have 2 or more dimensions");

  Eigen::Array<CoordScalar,N,1> tmp_vals_sq;  // x .^ 2
  Eigen::Array<CoordScalar,N,1> tmp_vals_cu;  // x .^ 3

  std::tuple<CoordScalar,PtN,MatMxN>
  operator()(const PtN& x, const bool compute_grad, const bool compute_hessian)
  {
    CoordScalar f = 0;
    
    PtN    grad;
    MatMxN hessian;

    tmp_vals_sq = x.array().square();

    for (size_type i = 0; i < (N-1); ++i)
    {
      const CoordScalar x_i_plus_1_minus_x_i_sq = x(i+1) - tmp_vals_sq(i);
      const CoordScalar x_i_minus_1 = x(i) - 1;

      f += (100 * x_i_plus_1_minus_x_i_sq * x_i_plus_1_minus_x_i_sq) + (x_i_minus_1 * x_i_minus_1);
    }

    if (compute_grad)
    {
      // THIS MAY BE IMPLEMENTED MUCH MORE CLEVERLY BY RETAINING COMPUTED VALUES BETWEEN ITERATIONS OVER i

      tmp_vals_cu = x.array().cube();

      auto& g = grad;
      g.resize(N,1);

      g.setZero();

      g(0) = (-400 * x(0) * x(1)) + (400 * tmp_vals_cu(0)) + (2 * x(0)) - 2;

      for (size_type i = 1; i < (N - 1); ++i)
      {
        g(i) = -2 + (202 * x(i)) - (200 * tmp_vals_sq(i-1)) - (400 * x(i+1) * x(i)) + (400 * tmp_vals_cu(i));
      }

      g(N-1) = 200 * (x(N-1) - tmp_vals_sq(N-2));
    }

    if (compute_hessian)
    {
      auto& H = hessian;
      H.resize(N,N);

      H.setZero();

      H(0,0) = (-400 * x(1)) + (1200 * tmp_vals_sq(0)) + 2;
      H(1,0) = -400 * x(0);
      H(0,1) = H(1,0);

      for (size_type i = 1; i < (N - 1); ++i)
      {
        H(i-1,i) = -400 * x(i-1);
        H(i,i-1) = H(i-1,i);
        H(i,i)   = 202 + (1200 * tmp_vals_sq(i) - (400 * x(i+1)));
      }

      H(N-2,N-1) = -400 * x(N-2);
      H(N-1,N-2) = H(N-2,N-1);
      H(N-1,N-1) = 200;
    }

    return std::make_tuple(f,grad,hessian);
  }
};

std::tuple<CoordScalar,PtN,MatMxN>
EasomFn(const PtN& x, const bool compute_grad, const bool compute_hessian);

std::tuple<CoordScalar,PtN,MatMxN>
BoothsFn(const PtN& x, const bool compute_grad, const bool compute_hessian);

std::tuple<CoordScalar,PtN,MatMxN>
BealesFn(const PtN& x, const bool compute_grad, const bool compute_hessian);

template <unsigned long N>
std::tuple<CoordScalar,PtN,MatMxN>
StyblinskiTangFn(const PtN& x, const bool compute_grad, const bool compute_hessian)
{
  if (compute_grad || compute_hessian)
  {
    xregThrow("Gradient and Hessian currently not implemented for Styblinski-Tang");
  }
  
  CoordScalar sum = 0;
  
  for (unsigned long i = 0; i < N; ++i)
  {
    const CoordScalar& x_i = x(i);

    const CoordScalar x_i_sq = x_i * x_i;

    sum += (x_i_sq * x_i_sq) - (16 * x_i_sq) + (5 * x_i);
  }
  
  return std::make_tuple(sum / 2, PtN(), MatMxN());
}

}  // xreg

#endif
