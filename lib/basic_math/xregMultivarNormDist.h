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

#ifndef XREGMULTIVARNORMDIST_H_
#define XREGMULTIVARNORMDIST_H_

#include "xregDistInterface.h"

namespace xreg
{

class MultivarNormalDist final : public Dist
{
public:
  MultivarNormalDist(const PtN& mean, const MatMxN& cov, const bool enable_sampling = true);

  Scalar operator()(const PtN& x) const;
    
  Scalar density(const PtN& x) const override;

  Scalar log_density(const PtN& x) const override;

  Scalar norm_const() const override;

  Scalar log_norm_const() const override;
  
  bool normalized() const override;

  size_type dim() const override;

  PtN draw_sample(std::mt19937& g) const override;

  MatMxN draw_samples(const size_type num_samples, std::mt19937& g) const override;

  const MatMxN& cov_inv() const;

private:
  PtN mean_;
  
  MatMxN cov_inv_;

  Scalar log_norm_const_;

  // used for drawing samples
  MatMxN A_;
};

// Multivariate Normal distribution where the dimensions are independent (no covariance).
class MultivarNormalDistZeroCov final : public Dist
{
public:
  MultivarNormalDistZeroCov(const PtN& mean, const PtN& std_devs);

  Scalar operator()(const PtN& x) const;
    
  Scalar density(const PtN& x) const override;

  Scalar log_density(const PtN& x) const override;

  Scalar norm_const() const override;

  Scalar log_norm_const() const override;
  
  bool normalized() const override;

  size_type dim() const override;

  PtN draw_sample(std::mt19937& g) const override;

  MatMxN draw_samples(const size_type num_samples, std::mt19937& g) const override;

private:
  PtN mean_;
  
  Eigen::Array<Scalar,Eigen::Dynamic,1> vars_inv_;

  Scalar log_norm_const_;

  // This is used when sampling
  PtN std_devs_;
};

}  // xreg

#endif
