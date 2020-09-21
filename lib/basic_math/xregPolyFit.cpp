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

#include "xregPolyFit.h"

#include <Eigen/Dense>

#include "xregAssert.h"

xreg::PtN xreg::SolveVandermondeMat(const PtN& x, const PtN& y, const size_type order)
{
  const size_type num_pts = x.size();
  xregASSERT(y.size() == num_pts);

  MatMxN V(num_pts, order + 1);

  CoordScalar tmp_x_pow = 0;

  for (size_type i = 0; i < num_pts; ++i)
  {
    const CoordScalar& cur_x = x(i);

    tmp_x_pow = 1;

    for (size_type j = 0; j < (order + 1); ++j)
    {
      V(i,j) = tmp_x_pow;
      
      tmp_x_pow *= cur_x;
    }
  }

  Eigen::JacobiSVD<MatMxN> svd(V, Eigen::ComputeThinU | Eigen::ComputeThinV);
  
  return svd.solve(y);
}

