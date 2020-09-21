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

#include "xregFoldNormDist.h"

#include <limits>
#include <cmath>

xreg::FoldNormDist::FoldNormDist(const Scalar m_arg, const Scalar s_arg)
  : m_(m_arg), s_(s_arg), two_s_sq_(2 * s_ * s_),
    norm_const_(std::sqrt(two_s_sq_ * Scalar(3.141592653589793))),
    log_norm_const_(std::log(norm_const_))
{ }

xreg::FoldNormDist::Scalar xreg::FoldNormDist::operator()(const Scalar x) const
{
  if (x >= 0)
  {
    return exp_helper(x) / norm_const_;
  }
  else
  {
    return 0;
  }
}

xreg::FoldNormDist::Scalar xreg::FoldNormDist::log_density(const Scalar x) const
{
  if (x >= 0)
  {
    const Scalar un_norm_prob = exp_helper(x);
    return (un_norm_prob > Scalar(1.0e-14)) ? (std::log(un_norm_prob) - log_norm_const_) :
                                              std::numeric_limits<Scalar>::lowest();
  }
  else
  {
    return -std::numeric_limits<Scalar>::infinity();
  }
}
  
xreg::FoldNormDist::Scalar xreg::FoldNormDist::exp_helper(const Scalar x) const
{
  const Scalar x_minus_m = x - m_;
  const Scalar x_plus_m  = x + m_;
  
  return std::exp((x_minus_m * x_minus_m) / -two_s_sq_) +
            std::exp((x_plus_m * x_plus_m) / -two_s_sq_);
}
  
xreg::FoldNormDist::Scalar xreg::FoldNormDist::density(const PtN& x) const
{
  return operator()(x[0]);
}

xreg::FoldNormDist::Scalar xreg::FoldNormDist::log_density(const PtN& x) const
{
  return log_density(x[0]);
}

xreg::FoldNormDist::Scalar xreg::FoldNormDist::norm_const() const
{
  return norm_const_;
}

xreg::FoldNormDist::Scalar xreg::FoldNormDist::log_norm_const() const
{
  return log_norm_const_;
}

bool xreg::FoldNormDist::normalized() const
{
  return true;
}

xreg::size_type xreg::FoldNormDist::dim() const
{
  return 1;
}

