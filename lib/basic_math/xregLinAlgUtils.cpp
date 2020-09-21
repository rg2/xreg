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

#include "xregLinAlgUtils.h"

xreg::MatMxN xreg::ComputePseudoInverse(const MatMxN& a,
                                        Eigen::JacobiSVD<MatMxN>* svd_work)
{
  constexpr CoordScalar eps = std::numeric_limits<CoordScalar>::epsilon();
  
  // Adapted from: http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257

  Eigen::JacobiSVD<MatMxN> local_svd;

  Eigen::JacobiSVD<MatMxN>* svd = svd_work;
  if (!svd)
  {
    svd = &local_svd;
  }

  svd->compute(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const CoordScalar tol = eps * std::max(a.cols(), a.rows()) * svd->singularValues().array().abs().maxCoeff();

  return svd->matrixV() *
         MatMxN((svd->singularValues().array().abs() > tol).select(
                     svd->singularValues().array().inverse(), 0) ).asDiagonal() *
         svd->matrixU().adjoint();
}

