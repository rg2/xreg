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

#include "xregOptimTestObjFns.h"

  
std::tuple<xreg::CoordScalar,xreg::PtN,xreg::MatMxN>
xreg::RosenbrockFn2D::operator()(const PtN& x, const bool compute_grad, const bool compute_hessian) const
{
  const CoordScalar a_minus_x0     = a - x(0);
  const CoordScalar x0_sq          = x(0) * x(0);
  const CoordScalar x1_minus_x0_sq = x(1) - x0_sq;

  PtN    grad;
  MatMxN hessian;

  if (compute_grad)
  {
    grad.resize(2,1);

    grad(0) = (-2 * a_minus_x0) - ((4 * b * x(0)) * x1_minus_x0_sq);
    grad(1) = 2 * b * x1_minus_x0_sq;
  }

  if (compute_hessian)
  {
    hessian.resize(2,2);

    hessian(0,0) = 2 - (4 * b * x1_minus_x0_sq) + (8 * b * x0_sq);
    hessian(1,0) = -4 * b * x(0);
    hessian(0,1) = hessian(1,0);
    hessian(1,1) = 2 * b;
  }

  return std::make_tuple((a_minus_x0 * a_minus_x0) + (b * x1_minus_x0_sq * x1_minus_x0_sq),
                         grad, hessian);;
}

std::tuple<xreg::CoordScalar,xreg::PtN,xreg::MatMxN>
xreg::EasomFn(const PtN& x, const bool compute_grad, const bool compute_hessian)
{
  PtN    grad;
  MatMxN hessian;
  
  constexpr CoordScalar kPI = 3.141592653589793;

  const CoordScalar cos_x = std::cos(x(0));
  const CoordScalar cos_y = std::cos(x(1));

  const CoordScalar x_minus_pi = x(0) - kPI;
  const CoordScalar y_minus_pi = x(1) - kPI;
  const CoordScalar exp_part = std::exp(-(x_minus_pi * x_minus_pi) - (y_minus_pi * y_minus_pi));

  // only used for gradient and hessian
  const CoordScalar sin_x = std::sin(x(0));
  const CoordScalar sin_y = std::sin(x(1));

  // just used MATLAB for symbolic differentiation

  if (compute_grad)
  {
    Pt2 g;

    g(0) = exp_part * cos_y * (sin_x - (2 * kPI * cos_x) + (2 * x(0) * cos_x));
    g(1) = exp_part * cos_x * (sin_y - (2 * kPI * cos_y) + (2 * x(1) * cos_y));
    
    grad = g;
  }

  if (compute_hessian)
  {
    Mat2x2 H;

    H(0,0) = cos_y * ((3 * cos_x) - (4 * x(0) * x(0) * cos_x) + (4 * kPI * sin_x) - (4 * x(0) * sin_x) - (4 * kPI * kPI * cos_x) + (8 * kPI * x(0) * cos_x));

    H(1,0) = -1 * (sin_x - (2 * kPI * cos_x) + (2 * x(0) * cos_x)) * (sin_y - (2 * kPI * cos_y) + (2 * x(1) * cos_y));
    H(0,1) = H(1,0);

    H(1,1) = cos_x * ((3 * cos_y) - (4 * x(1) * x(1) * cos_y) + (4 * kPI * sin_y) - (4 * x(1) * sin_y) - (4 * kPI * kPI * cos_y) + (8 * kPI * x(1) * cos_y));

    H *= exp_part;
    
    hessian = H;
  }

  return std::make_tuple(-cos_x * cos_y * exp_part, grad, hessian);
}

std::tuple<xreg::CoordScalar,xreg::PtN,xreg::MatMxN>
xreg::BoothsFn(const PtN& x, const bool compute_grad, const bool compute_hessian)
{
  PtN    grad;
  MatMxN hessian;
  
  const CoordScalar tmp1 = x(0) + (2 * x(1)) - 7;
  const CoordScalar tmp2 = (2 * x(0)) + x(1) -5;

  if (compute_grad)
  {
    Pt2 g;

    g(0) = (10 * x(0)) + (8 * x(1)) - 34;
    g(1) = (8 * x(0)) + (10 * x(1)) - 38;
    
    grad = g;
  }

  if (compute_hessian)
  {
    Mat2x2 H;
  
    H(0,0) = 10;
    H(1,0) = 8;
    H(0,1) = 8;
    H(1,1) = 10;
    
    hessian = H;
  }

  return std::make_tuple((tmp1 * tmp1) + (tmp2 * tmp2), grad, hessian);
}

std::tuple<xreg::CoordScalar,xreg::PtN,xreg::MatMxN>
xreg::BealesFn(const PtN& x, const bool compute_grad, const bool compute_hessian)
{
  PtN    grad;
  MatMxN hessian;
  
  const CoordScalar y_sq = x(1) * x(1);
  const CoordScalar y_cu = y_sq * x(1);

  const CoordScalar x_times_y    = x(0) * x(1);
  const CoordScalar x_times_y_sq = x(0) * y_sq;
  const CoordScalar x_times_y_cu = x(0) * y_cu;

  const CoordScalar tmp1 = 1.5 - x(0) + x_times_y;
  const CoordScalar tmp2 = 2.25 - x(0) + x_times_y_sq;
  const CoordScalar tmp3 = 2.625 - x(0) + x_times_y_cu;

  if (compute_grad)
  {
    Pt2 g;

    g(0) = 2 * (((y_sq - 1) * (x_times_y_sq - x(0) + 2.24)) + ((y_cu - 1) * (x_times_y_cu - x(0) + 2.625)) + ((x(1) - 1) * (x_times_y - x(0) + 1.5)));
    g(1) = (2 * x(0) * (x_times_y - x(0) + 1.5)) + (4 * x_times_y * (x_times_y_sq - x(0) + 2.25)) + (6 * x_times_y_sq * (x_times_y_cu - x(0) + 2.625));
    
    grad = g;
  }

  if (compute_hessian)
  {
    Mat2x2 H;

    const CoordScalar y_minus_1 = x(0) - 1;
    const CoordScalar y_sq_minus_1 = y_sq - 1;
    const CoordScalar y_cu_minus_1 = y_cu - 1;

    H(0,0) = 2 * ((y_minus_1 * y_minus_1) + (y_sq_minus_1 * y_sq_minus_1) + (y_cu_minus_1 * y_cu_minus_1));
    H(1,0) = (9 * x(1)) - (4 * x(0)) - (4 * x_times_y) - (12 * x_times_y_sq) + (8 * x_times_y_cu) + (12 * x_times_y_cu * y_sq) + (15.75 * y_sq) + 3;
    H(0,1) = H(1,0);
    H(1,1) = 0.5 * x(0) * ((63 * x(1)) - (4 * x(0)) - (24 * x_times_y) + (24 * x_times_y_sq) + (60 * x_times_y_cu * x(1)) + 18);
    
    hessian = H;
  }

  return std::make_tuple((tmp1 * tmp1) + (tmp2 * tmp2) + (tmp3 * tmp3), grad, hessian);
}

