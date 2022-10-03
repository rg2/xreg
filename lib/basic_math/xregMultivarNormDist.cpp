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

#include <Eigen/Eigenvalues>

#include "xregAssert.h"

xreg::MultivarNormalDist::MultivarNormalDist(const PtN& mean, const MatMxN& cov, const bool enable_sampling)
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

  if (enable_sampling)
  {
    Eigen::SelfAdjointEigenSolver<MatMxN> eig(cov);
    A_ = eig.eigenvectors() * eig.eigenvalues().array().sqrt().matrix().asDiagonal();
  }
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

xreg::PtN xreg::MultivarNormalDist::draw_sample(std::mt19937& g) const
{
  const size_type d = dim();
  
  xregASSERT(static_cast<size_type>(A_.rows()) == d);
  xregASSERT(static_cast<size_type>(A_.cols()) == d);

  std::normal_distribution<Scalar> std_normal(0, 1);
  
  PtN z(d);

  for (size_type i = 0; i < d; ++i)
  {
    z(i) = std_normal(g);
  }

  return mean_ + (A_ * z);
}

xreg::MatMxN xreg::MultivarNormalDist::draw_samples(const size_type num_samples, std::mt19937& g) const
{
  const size_type d = dim();
  
  xregASSERT(static_cast<size_type>(A_.rows()) == d);
  xregASSERT(static_cast<size_type>(A_.cols()) == d);

  std::normal_distribution<Scalar> std_normal(0, 1);
  
  PtN z(d);

  MatMxN samples(d, num_samples);

  for (size_type j = 0; j < num_samples; ++j)
  {
    for (size_type i = 0; i < d; ++i)
    {
      z(i) = std_normal(g);
    }

    samples.col(j) = mean_ + (A_ * z);
  }

  return samples;
}

const xreg::MatMxN& xreg::MultivarNormalDist::cov_inv() const
{
  return cov_inv_;
}

xreg::MultivarNormalDistZeroCov::MultivarNormalDistZeroCov(const PtN& mean, const PtN& std_devs)
{
  const size_type dim = static_cast<size_type>(mean.size());

  xregASSERT(dim > 0);
  xregASSERT(dim == static_cast<size_type>(std_devs.size()));

  // Covariance matrix is invertible if all variances are non-zero
  xregASSERT((std_devs.array().abs() > Scalar(1.0e-12)).all());

  mean_ = mean;

  std_devs_ = std_devs;

  const PtN vars = std_devs.array().square().matrix();

  vars_inv_ = vars.array().inverse();

  constexpr Scalar log_two_pi = static_cast<Scalar>(1.8378770664093453);
  
  log_norm_const_ = ((static_cast<Scalar>(dim) * log_two_pi) * vars.array().log().sum()) / Scalar(2);
}

xreg::MultivarNormalDistZeroCov::Scalar xreg::MultivarNormalDistZeroCov::operator()(const PtN& x) const
{
  return density(x);
}
  
xreg::MultivarNormalDistZeroCov::Scalar xreg::MultivarNormalDistZeroCov::density(const PtN& x) const
{
  return std::exp(log_density(x));
}

xreg::MultivarNormalDistZeroCov::Scalar xreg::MultivarNormalDistZeroCov::log_density(const PtN& x) const
{
  return (static_cast<Scalar>(-0.5) * ((x - mean_).array().square() * vars_inv_).sum()) - log_norm_const_;
}

xreg::MultivarNormalDistZeroCov::Scalar xreg::MultivarNormalDistZeroCov::norm_const() const
{
  return std::exp(log_norm_const_);
}

xreg::MultivarNormalDistZeroCov::Scalar xreg::MultivarNormalDistZeroCov::log_norm_const() const
{
  return log_norm_const_;
}

bool xreg::MultivarNormalDistZeroCov::normalized() const
{
  return true;
}

xreg::size_type xreg::MultivarNormalDistZeroCov::dim() const
{
  return static_cast<size_type>(mean_.size());
}

xreg::PtN xreg::MultivarNormalDistZeroCov::draw_sample(std::mt19937& g) const
{
  const size_type d = dim();

  std::normal_distribution<Scalar> std_normal(0, 1);
  
  PtN x(d);

  for (size_type i = 0; i < d; ++i)
  {
    x(i) = mean_(i) + (std_devs_(i) * std_normal(g));
  }

  return x;
}

xreg::MatMxN xreg::MultivarNormalDistZeroCov::draw_samples(const size_type num_samples, std::mt19937& g) const
{
  const size_type d = dim();

  std::normal_distribution<Scalar> std_normal(0, 1);
  
  MatMxN samples(d, num_samples);

  for (size_type j = 0; j < num_samples; ++j)
  {
    for (size_type i = 0; i < d; ++i)
    {
      samples(i,j) = mean_(i) + (std_devs_(i) * std_normal(g));
    }
  }

  return samples;
}