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

#include "xregImgSimMetric2DCombine.h"

#include "xregAssert.h"

void xreg::ImgSimMetric2DCombine::allocate_resources()
{
  sim_objs_.resize(num_sim_metrics_);
  sim_vals_.resize(num_projs_per_sim_metric_);
}

void xreg::ImgSimMetric2DCombine::set_num_sim_metrics(const size_type num_sim_metrics)
{
  num_sim_metrics_ = num_sim_metrics;
}

void xreg::ImgSimMetric2DCombine::set_num_projs_per_sim_metric(const size_type num_projs_per_sim_metric)
{
  num_projs_per_sim_metric_ = num_projs_per_sim_metric;
}

void xreg::ImgSimMetric2DCombine::set_sim_metric(const size_type sim_idx, ImgSimMetric2D* sim)
{
  sim_objs_[sim_idx] = sim;
  xregASSERT(sim->num_moving_images() == num_projs_per_sim_metric_);
}

const xreg::ImgSimMetric2DCombine::ScalarList&
xreg::ImgSimMetric2DCombine::sim_vals() const
{
  return sim_vals_;
}

const xreg::ImgSimMetric2DCombine::Scalar
xreg::ImgSimMetric2DCombine::sim_val(const size_type proj_idx) const
{
  xregASSERT(proj_idx < num_projs_per_sim_metric_);
  return sim_vals_[proj_idx];
}
  
void xreg::ImgSimMetric2DCombineAddition::compute()
{
  this->sim_vals_.assign(this->num_projs_per_sim_metric_, 0);

  // e.g. for each view
  for (size_type sim_idx = 0; sim_idx < this->num_sim_metrics_; ++sim_idx)
  {
    // we may be computing several candidate poses for each view
    for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
    {
      this->sim_vals_[proj_idx] += this->sim_objs_[sim_idx]->sim_val(proj_idx);
    }
  }
}
  
void xreg::ImgSimMetric2DCombineMean::compute()
{
  this->sim_vals_.assign(this->num_projs_per_sim_metric_, 0);

  // e.g. for each view
  for (size_type sim_idx = 0; sim_idx < this->num_sim_metrics_; ++sim_idx)
  {
    // we may be computing several candidate poses for each view
    for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
    {
      this->sim_vals_[proj_idx] += this->sim_objs_[sim_idx]->sim_val(proj_idx);
    }
  }
  
  // compute mean
  for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
  {
    this->sim_vals_[proj_idx] /= this->num_sim_metrics_;
  }
}
  
void xreg::ImgSimMetric2DCombineSumOfSquares::compute()
{
  this->sim_vals_.assign(this->num_projs_per_sim_metric_, 0);

  // e.g. for each view
  for (size_type sim_idx = 0; sim_idx < this->num_sim_metrics_; ++sim_idx)
  {
    // we may be computing several candidate poses for each view
    for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
    {
      Scalar x = this->sim_objs_[sim_idx]->sim_val(proj_idx);

      this->sim_vals_[proj_idx] += x * x;
    }
  }
}
  
void xreg::ImgSimMetric2DCombineGaussian::compute()
{
  this->sim_vals_.assign(this->num_projs_per_sim_metric_, 0);

  // we may be computing several candidate poses for each view
  for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
  {
    // e.g. for each view
    for (size_type sim_idx = 0; sim_idx < this->num_sim_metrics_; ++sim_idx)
    {
      this->sim_vals_[proj_idx] -= (this->sim_objs_[sim_idx]->sim_val(proj_idx) * this->sim_objs_[sim_idx]->sim_val(proj_idx));
    }

    this->sim_vals_[proj_idx] = -std::exp(this->sim_vals_[proj_idx]);
  }
}
  
void xreg::ImgSimMetric2DCombineExp::compute()
{
  this->sim_vals_.assign(this->num_projs_per_sim_metric_, 0);

  // we may be computing several candidate poses for each view
  for (size_type proj_idx = 0; proj_idx < this->num_projs_per_sim_metric_; ++proj_idx)
  {
    // e.g. for each view
    for (size_type sim_idx = 0; sim_idx < this->num_sim_metrics_; ++sim_idx)
    {
      this->sim_vals_[proj_idx] += this->sim_objs_[sim_idx]->sim_val(proj_idx);
    }

    this->sim_vals_[proj_idx] = std::exp(this->sim_vals_[proj_idx]);
  }
}

