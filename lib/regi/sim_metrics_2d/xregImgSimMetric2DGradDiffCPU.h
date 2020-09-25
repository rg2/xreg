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

#ifndef XREGIMGSIMMETRIC2DGRADDIFFCPU_H_
#define XREGIMGSIMMETRIC2DGRADDIFFCPU_H_

#include <array>

#include "xregImgSimMetric2DGradImgCPU.h"

namespace xreg
{

/// \brief Gradient Difference Similarity Metric
///
/// As described by Penney 2001
/// This currently optimizes the subproblem with a backtracking Armijo line search with modified Newton step directions..
class ImgSimMetric2DGradDiffCPU : public ImgSimMetric2DGradImgCPU
{
public:
  /// \brief Constructor - trivial, no work performed.
  ImgSimMetric2DGradDiffCPU() = default;

  /// \brief Allocation of other resources required and computation of fixed image gradients
  ///        and their standard deviations.
  ///
  /// This call allocates memory for storing the horizontal and vertical fixed image gradients
  /// and a buffer for storing the gradients of moving images.
  /// This call also computes the fixed image gradients and their standard deviations.
  void allocate_resources() override;

  /// \brief Computation of the similarity metric.
  ///
  /// For each gradient direction, this makes serial calls to cv::Sobel (which may be threaded)
  /// for each moving image, followed by a call to the optimizer to solve for the best gradient scale factor.
  void compute() override;

  size_type num_sub_prob_its() const;

  void set_num_sub_prob_its(const size_type num_its);

  std::array<double,2> sub_prob_init_guess() const;

  void set_sub_prob_init_guess(const double init_guess);

  void set_sub_prob_init_guess(const std::array<double,2>& init_guess);

  bool do_two_guess_symmetric() const;

  void set_do_two_guess_symmetric(const bool do_sym);

  bool track_sub_prob_inits() const;

  void set_track_sub_prob_inits(const bool track_inits);

protected:
  void process_mask() override;

private:
  using PixelBuffer    = std::vector<Scalar>;
  using ImageArray     = Eigen::Array<Scalar,Eigen::Dynamic,1>;
  using ImageArrayMap  = Eigen::Map<ImageArray>;
  using ImageArrayList = std::vector<ImageArray>;

  cv::Mat fixed_grad_img_x_to_use_;
  cv::Mat fixed_grad_img_y_to_use_;

  ImageArray fixed_grad_img_x_vec_;
  ImageArray fixed_grad_img_y_vec_;

  Scalar fixed_grad_x_var_ = 0;
  Scalar fixed_grad_y_var_ = 0;

  PixelBuffer grad_mov_imgs_buf_;

  size_type num_rows_ = 0;
  size_type num_cols_ = 0;
  size_type num_pix_  = 0;

  int open_cv_img_type_;

  // These need to be lists of image arrays to accomodate a multi-threaded
  // implementation. Memory usage could be made a little more conservative, but
  // should not be a big deal for our data.
  ImageArrayList tmp_imgs_vec1_;
  ImageArrayList tmp_imgs_vec2_;
  ImageArrayList tmp_imgs_vec3_;

  // sub-problem configuration:
  size_type num_sub_prob_its_ = 5;
  
  std::array<double,2> sub_prob_init_guess_= std::array<double,2>{ 1.0, 1.0 };

  bool do_two_guess_symmetric_ = false;

  bool track_sub_prob_inits_ = false;

  std::vector<std::array<double,2>> sub_prob_vals_;

  cv::Mat mask_ocv_;
};

}  // xreg

#endif

