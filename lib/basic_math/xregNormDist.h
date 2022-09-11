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

#ifndef XREGNORMDIST_H_
#define XREGNORMDIST_H_

#include "xregDistInterface.h"

namespace xreg
{

/// \brief One dimensional Normal distribution
///
/// Pre-computes as much as possible in the constructor.
class NormalDist1D final : public Dist
{
public:
  NormalDist1D();

  NormalDist1D(const Scalar m, const Scalar s);

  Scalar operator()(const Scalar x) const;

  Scalar density(const PtN& x) const override;

  Scalar log_density(const PtN& x) const override;
  
  Scalar log_density(const Scalar x) const;

  Scalar norm_const() const override;

  Scalar log_norm_const() const override;

  bool normalized() const override;

  size_type dim() const override;

  PtN draw_sample(std::mt19937& g) const override;

  MatMxN draw_samples(const size_type num_samples, std::mt19937& g) const override;

private:
  const Scalar mu_;

  const Scalar sigma_;

  const Scalar sigma_sq_;

  const Scalar minus_one_over_two_sigma_sq_;
  
  const Scalar norm_const_;

  const Scalar log_norm_const_;
};

/// \brief Two dimensional Normal distribution with independent dimensions
///
/// This DOES NOT use two NormalDist1D instances, but rather computes as much
/// as possible in the constructor and only evaluates a single std::exp in
/// the evaluation operator.
class NormalDist2DIndep final : public Dist
{
public:
  // Takes the mean x and y values and the standard deviations in each direction.
  NormalDist2DIndep(const Scalar m_x, const Scalar m_y,
                    const Scalar s_x, const Scalar s_y);

  Scalar operator()(const Scalar x, const Scalar y) const;
  
  Scalar log_density(const Scalar x, const Scalar y) const;
  
  Scalar density(const PtN& x) const override;

  Scalar log_density(const PtN& x) const override;

  Scalar norm_const() const override;

  Scalar log_norm_const() const override;
  
  bool normalized() const override;

  size_type dim() const override;

  PtN draw_sample(std::mt19937& g) const override;

  MatMxN draw_samples(const size_type num_samples, std::mt19937& g) const override;

private:

  const Scalar mu_x_;
  const Scalar mu_y_;

  const Scalar sigma_x_;
  const Scalar sigma_y_;

  const Scalar one_over_sigma_x_sq_;
  const Scalar one_over_sigma_y_sq_;

  const Scalar norm_const_;  // normalizing constant (2pi s_x * s_y)
  
  const Scalar log_norm_const_;
};

}  // xreg

#endif

