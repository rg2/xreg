/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

#include "xregNormDist.h"

xreg::NormalDist1D::NormalDist1D()
  : mu_(0), sigma_(1), sigma_sq_(1), minus_one_over_two_sigma_sq_(-0.5),
    norm_const_(std::sqrt(Scalar(6.283185307179586))),
    log_norm_const_(std::log(norm_const_))
{ } 

xreg::NormalDist1D::NormalDist1D(const Scalar m, const Scalar s)
  : mu_(m), sigma_(s),
    sigma_sq_(s * s),
    minus_one_over_two_sigma_sq_(Scalar(-0.5) / sigma_sq_),
    norm_const_(s * std::sqrt(Scalar(6.283185307179586))),
    log_norm_const_(std::log(norm_const_))
{ }

xreg::NormalDist1D::Scalar
xreg::NormalDist1D::operator()(const Scalar x) const
{
  return std::exp(log_density(x));
}

xreg::NormalDist1D::Scalar
xreg::NormalDist1D::density(const PtN& x) const
{
  return operator()(x[0]);
}

xreg::NormalDist1D::Scalar
xreg::NormalDist1D::log_density(const PtN& x) const
{
  return log_density(x[0]);
}

xreg::NormalDist1D::Scalar
xreg::NormalDist1D::log_density(const Scalar x) const
{
  const Scalar x_minus_mu = x - mu_;
  return (minus_one_over_two_sigma_sq_ * x_minus_mu * x_minus_mu) - log_norm_const_;
}
  
xreg::NormalDist1D::Scalar xreg::NormalDist1D::norm_const() const
{
  return norm_const_;
}

xreg::NormalDist1D::Scalar xreg::NormalDist1D::log_norm_const() const
{
  return log_norm_const_;
}
  
bool xreg::NormalDist1D::normalized() const
{
  return true;
}

xreg::size_type xreg::NormalDist1D::dim() const
{
  return 1;
}

xreg::PtN xreg::NormalDist1D::draw_sample(std::mt19937& g) const
{
  std::normal_distribution<Scalar> dist(mu_, sigma_);

  return Eigen::Matrix<Scalar,1,1>(dist(g));
}

xreg::MatMxN xreg::NormalDist1D::draw_samples(const size_type num_samples, std::mt19937& g) const
{
  MatMxN samples(1, num_samples);

  std::normal_distribution<Scalar> dist(mu_, sigma_);

  for (size_type i = 0; i < num_samples; ++i)
  {
    samples(0,i) = dist(g);
  }

  return samples;
}

xreg::NormalDist2DIndep::NormalDist2DIndep(const Scalar m_x, const Scalar m_y,
                                           const Scalar s_x, const Scalar s_y)
  : mu_x_(m_x), mu_y_(m_y), sigma_x_(s_x), sigma_y_(s_y),
    one_over_sigma_x_sq_(Scalar(1) / (s_x * s_x)),
    one_over_sigma_y_sq_(Scalar(1) / (s_y * s_y)),
    norm_const_(s_x * s_y * Scalar(6.283185307179586)),
    log_norm_const_(std::log(norm_const_))
{ }

xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::operator()(const Scalar x, const Scalar y) const
{

  return std::exp(log_density(x,y));
}

xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::log_density(const Scalar x, const Scalar y) const
{
  const Scalar x_minus_mu_x = x - mu_x_;
  const Scalar y_minus_mu_y = y - mu_y_;
  
  return (Scalar(-0.5) *
           ((x_minus_mu_x * x_minus_mu_x * one_over_sigma_x_sq_)
              + (y_minus_mu_y * y_minus_mu_y * one_over_sigma_y_sq_))) - log_norm_const_;
}
  
xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::density(const PtN& x) const
{
  return operator()(x[0], x[1]);
}

xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::log_density(const PtN& x) const
{
  return log_density(x[0], x[1]);
}

xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::norm_const() const
{
  return norm_const_;
}

xreg::NormalDist2DIndep::Scalar
xreg::NormalDist2DIndep::log_norm_const() const
{
  return log_norm_const_;
}

bool xreg::NormalDist2DIndep::normalized() const
{
  return true;
}

xreg::size_type xreg::NormalDist2DIndep::dim() const
{
  return 2;
}

xreg::PtN xreg::NormalDist2DIndep::draw_sample(std::mt19937& g) const
{
  std::normal_distribution<Scalar> dist_x(mu_x_, sigma_x_);
  std::normal_distribution<Scalar> dist_y(mu_y_, sigma_y_);

  return Eigen::Matrix<Scalar,2,1>(dist_x(g), dist_y(g));
}

xreg::MatMxN xreg::NormalDist2DIndep::draw_samples(const size_type num_samples, std::mt19937& g) const
{
  MatMxN samples(2, num_samples);

  std::normal_distribution<Scalar> dist_x(mu_x_, sigma_x_);
  std::normal_distribution<Scalar> dist_y(mu_y_, sigma_y_);

  for (size_type i = 0; i < num_samples; ++i)
  {
    samples(0,i) = dist_x(g);
    samples(1,i) = dist_y(g);
  }

  return samples;
}
