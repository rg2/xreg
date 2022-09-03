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

#include "xregMultivarNormDistFit.h"

#include <Eigen/Dense>

#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregFitQuadratic.h"

std::tuple<xreg::MatMxN,xreg::MatMxN,xreg::CoordScalar>
xreg::FindCovMatFromZeroMeanLogPts(
  const MatMxN& x, const PtN& log_p_x, const CoordScalar log_p_0)
{
  const size_type dim = x.rows();

  xregASSERT(x.cols() == log_p_x.size());

  // p(x) = det(2pi * cov)^-1/2 exp((-1/2) * x^T * inv(cov) * x)
  // log p(x) = (-1/2) * log(det(2pi * cov)) + ((-1/2) * x^T * inv(cov) * x)
  // log p(0) = (-1/2) * log(det(2pi * cov))
  // log p(x) - log p(0) = (-1/2) * x^T * inv(cov) * x
  // log p(0) - log p(x) = (1/2) * x^T * inv(cov) * x
  // We can fit to the form: (1/2) * x^T * H * x which matches the RHS,
  // and therefore need to transform the log_y values to match

  // curvature of the log density defines the inverse covariance
  const MatMxN inv_cov = FitQuadradicFormSymmetric(x, (log_p_0 - log_p_x.array()).matrix());

  // spectral decomposition of inverse covariance
  Eigen::SelfAdjointEigenSolver<MatMxN> decomp(inv_cov);
  
  const PtN eig_vals = decomp.eigenvalues();

  CoordScalar det_inv_cov = eig_vals(0);
  for (size_type i = 1; i < dim; ++i)
  {
    det_inv_cov *= eig_vals(i);
  }

  if (std::abs(det_inv_cov) < CoordScalar(1.0e-8))
  {
    xregThrow("inverse covariance matrix not invertible!");
  }

  return std::make_tuple(
    inv_cov.inverse(), inv_cov, CoordScalar(1) / det_inv_cov);
}
