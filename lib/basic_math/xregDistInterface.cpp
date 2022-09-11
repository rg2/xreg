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

#include "xregDistInterface.h"

#include "xregTBBUtils.h"

xreg::Dist::Scalar xreg::Dist::norm_const() const
{
  throw UnsupportedOperation();
}

xreg::Dist::Scalar xreg::Dist::log_norm_const() const
{
  throw UnsupportedOperation();
}

xreg::PtN xreg::Dist::densities(const PtNList& pts) const
{
  const size_type num_pts = pts.size();

  PtN out(num_pts);

  auto compute_densities_helper = [&] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      out[i] = this->density(pts[i]);
    }
  };
  
  ParallelFor(compute_densities_helper, RangeType(0, num_pts));

  return out;
}

xreg::PtN xreg::Dist::log_densities(const PtNList& pts) const
{
  const size_type num_pts = pts.size();

  PtN out(num_pts);

  auto compute_log_densities_helper = [&] (const RangeType& r)
  {
    for (size_type i = r.begin(); i != r.end(); ++i)
    {
      out[i] = this->log_density(pts[i]);
    }
  };
  
  ParallelFor(compute_log_densities_helper, RangeType(0, num_pts));

  return out;
}

xreg::PtN xreg::Dist::draw_sample(std::mt19937&) const
{
  throw UnsupportedOperation();
}

xreg::MatMxN xreg::Dist::draw_samples(const size_type num_samples, std::mt19937& g) const
{
  MatMxN samples(dim(), num_samples);

  for (size_type i = 0; i < num_samples; ++i)
  {
    samples.col(i) = draw_sample(g);
  }

  return samples;
}
