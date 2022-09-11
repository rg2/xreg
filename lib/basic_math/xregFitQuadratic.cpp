/*
 * MIT License
 *
 * Copyright (c) 2022 Robert Grupp
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

#include "xregFitQuadratic.h"

#include "xregAssert.h"

xreg::MatMxN xreg::FitQuadradicForm(const MatMxN& params, const PtN& fn_vals)
{
  const size_type num_obs = params.cols();
  const size_type dim = params.rows();

  xregASSERT(num_obs == static_cast<size_type>(fn_vals.size()));

  MatMxN A(num_obs, dim * dim);

  // populate A

  for (size_type obs_idx = 0; obs_idx < num_obs; ++obs_idx)
  {
    auto x = params.col(obs_idx);

    size_type flat_idx = 0;
    for (size_type r = 0; r < dim; ++r)
    {
      for (size_type c = 0; c < dim; ++c, ++flat_idx)
      {
        A(obs_idx, flat_idx) = (x(r) * x(c)) / CoordScalar(2);
      }
    }
  }

  Eigen::JacobiSVD<MatMxN> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const PtN H_flat = svd.solve(fn_vals);

  MatMxN H(dim, dim);

  size_type flat_idx = 0;
  for (size_type r = 0; r < dim; ++r)
  {
    for (size_type c = 0; c < dim; ++c, ++flat_idx)
    {
      H(r,c) = H_flat(flat_idx);
    }
  }

  return H;
}

xreg::MatMxN xreg::FitQuadradicFormSymmetric(const MatMxN& params, const PtN& fn_vals)
{
  const size_type num_obs = params.cols();
  const size_type dim = params.rows();

  xregASSERT(num_obs == static_cast<size_type>(fn_vals.size()));

  // Need to construct a LUT from general dim x dim matrix indices to symmetric
  // flat indices along with the inverse
  Eigen::Matrix<size_type,Eigen::Dynamic,Eigen::Dynamic> sym_lut(dim, dim);
  std::vector<std::array<size_type,2>> inv_sym_lut;

  size_type flat_idx = 0;
  for (size_type r = 0; r < dim; ++r)
  {
    sym_lut(r,r) = flat_idx;
    
    inv_sym_lut.push_back({r,r});

    ++flat_idx;

    for (size_type c = (r + 1); c < dim; ++c, ++flat_idx)
    {
      sym_lut(r,c) = flat_idx;
      sym_lut(c,r) = flat_idx;

      inv_sym_lut.push_back({r,c});
    }
  }

  const size_type num_sym_el = flat_idx;

  MatMxN A(num_obs, num_sym_el);
  A.setZero();

  // populate A

  for (size_type obs_idx = 0; obs_idx < num_obs; ++obs_idx)
  {
    auto x = params.col(obs_idx);

    for (size_type r = 0; r < dim; ++r)
    {
      for (size_type c = 0; c < dim; ++c)
      {
        A(obs_idx,sym_lut(r,c)) += (x(r) * x(c)) / CoordScalar(2);
      }
    }
  }

  Eigen::JacobiSVD<MatMxN> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const PtN H_flat = svd.solve(fn_vals);

  MatMxN H(dim, dim);

  for (flat_idx = 0; flat_idx < num_sym_el; ++flat_idx)
  {
    const auto r = inv_sym_lut[flat_idx][0];
    const auto c = inv_sym_lut[flat_idx][1];

    H(r,c) = H_flat(flat_idx);

    if (r != c)
    {
      H(c,r) = H_flat(flat_idx);
    }
  }

  return H;
}
