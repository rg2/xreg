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

#include "xregMultivarNormDist.h"

#include "xregAssert.h"

xreg::MultivarNormalDist::MultivarNormalDist(const PtN& mean, const MatMxN& cov)
{
  const size_type dim = static_cast<size_type>(mean.size());

  xregASSERT(dim > 0);
  xregASSERT(dim == static_cast<size_type>(cov.rows()));
  xregASSERT(dim == static_cast<size_type>(cov.cols()));

  Eigen::FullPivLU<MatMxN> lu(cov);

  xregASSERT(lu.isInvertible());

  cov_inv_ = lu.inverse();

  constexpr Scalar two_pi = static_cast<Scalar>(6.283185307179586);

  log_norm_const_ = std::log(std::pow(two_pi, static_cast<Scalar>(dim)) * lu.determinant()) / Scalar(2);

  mean_ = mean;
}

xreg::MultivarNormalDist::Scalar xreg::MultivarNormalDist::operator()(const PtN& x) const
{
  return density(x);
}
  
xreg::MultivarNormalDist::Scalar xreg::MultivarNormalDist::density(const PtN& x) const
{
  return std::exp(log_density(x));
}

xreg::MultivarNormalDist::Scalar xreg::MultivarNormalDist::log_density(const PtN& x) const
{
  return (static_cast<Scalar>(-0.5) * (x - mean_).transpose() * cov_inv_ * (x - mean_)) - log_norm_const_;
}

xreg::MultivarNormalDist::Scalar xreg::MultivarNormalDist::norm_const() const
{
  return std::exp(log_norm_const_);
}

xreg::MultivarNormalDist::Scalar xreg::MultivarNormalDist::log_norm_const() const
{
  return log_norm_const_;
}

bool xreg::MultivarNormalDist::normalized() const
{
  return true;
}

xreg::size_type xreg::MultivarNormalDist::dim() const
{
  return static_cast<size_type>(mean_.size());
}
