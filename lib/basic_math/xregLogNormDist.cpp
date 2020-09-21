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

#include "xregLogNormDist.h"

#include <cmath>
  
xreg::LogNormDist::LogNormDist(const Scalar m_arg, const Scalar s_arg)
  : m_(m_arg), s_(s_arg), two_s_sq_(2 * s_ * s_),
    norm_const_(s_ * Scalar(2.506628274631001)),  // sigma * sqrt(2pi)
    log_norm_const_(std::log(norm_const_))
{ }

xreg::LogNormDist::Scalar xreg::LogNormDist::log_density(const Scalar x) const
{
  const Scalar log_x          = std::log(x);
  const Scalar log_x_minus_mu = log_x - m_;

  return ((log_x_minus_mu * log_x_minus_mu) / -two_s_sq_) - log_x - log_norm_const_;
}

xreg::LogNormDist::Scalar xreg::LogNormDist::operator()(const Scalar x) const
{
  return std::exp(log_density(x));
}
  
xreg::LogNormDist::Scalar xreg::LogNormDist::density(const PtN& x) const
{
  return operator()(x[0]);
}

xreg::LogNormDist::Scalar xreg::LogNormDist::log_density(const PtN& x) const
{
  return log_density(x[0]);
}

xreg::LogNormDist::Scalar xreg::LogNormDist::norm_const() const
{
  return norm_const_;
}

xreg::LogNormDist::Scalar xreg::LogNormDist::log_norm_const() const
{
  return log_norm_const_;
}

bool xreg::LogNormDist::normalized() const
{
  return true;
}

xreg::size_type xreg::LogNormDist::dim() const
{
  return 1;
}

