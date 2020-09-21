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

#ifndef XREGIMGSIMMETRIC2DCOMBINE_H_
#define XREGIMGSIMMETRIC2DCOMBINE_H_

#include "xregImgSimMetric2D.h"

namespace xreg
{

/// \brief Interface for combining a collection of similarity metric scores.
///
/// As an example, the simplest case is addition of the scores, however with
/// some additional knowledge about the similarity metrics another combination
/// may be more desirable.
/// NOTE: It is up to the user to ensure that the number of projections is
///       consistent across the collection of similarity metrics.
class ImgSimMetric2DCombine
{
public:
  using Scalar     = ImgSimMetric2D::Scalar;
  using ScalarList = ImgSimMetric2D::ScalarList;
  
  /// \brief Contructor - performs no work
  ImgSimMetric2DCombine() = default;

  // no copying
  ImgSimMetric2DCombine(const ImgSimMetric2DCombine&) = delete;
  ImgSimMetric2DCombine& operator=(const ImgSimMetric2DCombine&) = delete;

  /// \brief Allocates storage for similarity metric object storage and
  ///        similarity scores.
  ///
  /// This should be called after setting the number of similarity metrics and
  /// the number of projections per similarity metric.
  void allocate_resources();

  /// \brief Sets the number of similarity metrics to combine.
  ///
  /// Must be called prior to allocate_resources()
  void set_num_sim_metrics(const size_type num_sim_metrics);

  /// \brief Sets the number of projections each similarity metric object
  ///        has values for.
  ///
  /// Must be called prior to allocate_resources()
  void set_num_projs_per_sim_metric(const size_type num_projs_per_sim_metric);

  /// \brief Sets the similarity metric to be used at a specific index.
  void set_sim_metric(const size_type sim_idx, ImgSimMetric2D* sim);

  /// \brief Reference to the list of combined similarity scores.
  ///
  /// The list is indiced by projection index.
  const ScalarList& sim_vals() const;

  /// \brief Retrieve the combine similarity score for a specific projection index.
  const Scalar sim_val(const size_type proj_idx) const;

  /// \brief Perform the similarity combination computation - must be implemented
  ///        by a child class.
  virtual void compute() = 0;

protected:
  size_type num_sim_metrics_ = 0;

  size_type num_projs_per_sim_metric_ = 0;

  ScalarList sim_vals_;

  std::vector<ImgSimMetric2D*> sim_objs_;
};

/// \brief Combine similarity metrics with addition.
class ImgSimMetric2DCombineAddition : public ImgSimMetric2DCombine
{
public:
  ImgSimMetric2DCombineAddition() = default;

  void compute();
};

/// \brief Combine similarity metrics as the mean of the values.
///
/// This is easier to reason with when we know bounds on each view's similarity score.
class ImgSimMetric2DCombineMean : public ImgSimMetric2DCombine
{
public:
  ImgSimMetric2DCombineMean() = default;

  void compute();
};

/// \brief Combines similarity metrics by summing the squares of each similarity
///        score.
///
/// NOTE: The user needs to ensure that the global minimum of any similarity
///       metric is zero.
class ImgSimMetric2DCombineSumOfSquares : public ImgSimMetric2DCombine
{
public:
  ImgSimMetric2DCombineSumOfSquares() = default;

  void compute();
};

/// \brief Combines similarity metrics using a, negated, multi-variate normal distribution.
///
/// This treats the similarity metric as random vector drawn from a zero-mean,
/// multi-variate, normal distribution, computes the density and flips value.
/// NOTE: The user needs to ensure that the global minimum of any similarity
///       metric is zero.
class ImgSimMetric2DCombineGaussian : public ImgSimMetric2DCombine
{
public:
  ImgSimMetric2DCombineGaussian() = default;

  void compute();
};

/// \brief Combines similarity metrics by computing the exponential of their sum.
class ImgSimMetric2DCombineExp : public ImgSimMetric2DCombine
{
public:
  ImgSimMetric2DCombineExp() = default;

  void compute();
};

}  // xreg

#endif

