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

#include "xregImgSimMetric2DGradOrientCPU.h"

#include "xregOpenCVUtils.h"
#include "xregTBBUtils.h"

bool xreg::ImgSimMetric2DGradOrientCPU::use_median_as_thresh() const
{
  return use_median_as_thresh_;
}

void xreg::ImgSimMetric2DGradOrientCPU::set_use_median_as_thresh(const bool use_med)
{
  use_median_as_thresh_ = use_med;
}

void xreg::ImgSimMetric2DGradOrientCPU::set_fixed_grad_mag_thresh(const Scalar thresh)
{
  fixed_grad_mag_thresh_ = thresh;
}

void xreg::ImgSimMetric2DGradOrientCPU::set_mov_grad_mag_thresh(const Scalar thresh)
{
  mov_grad_mag_thresh_ = thresh;
}

void xreg::ImgSimMetric2DGradOrientCPU::allocate_resources()
{
  ImgSimMetric2DGradImgCPU::allocate_resources();

  fixed_grad_mag_ = cv::Mat::zeros(this->fixed_grad_img_x_.size(),
                                   this->fixed_grad_img_x_.type());

  // compute fixed image grad mag
  cv::magnitude(this->fixed_grad_img_x_, this->fixed_grad_img_y_, fixed_grad_mag_);

  // build mask of fixed image grad magnitudes that above the threshold
  use_fixed_mag_ = cv::Mat::zeros(this->fixed_grad_img_x_.size(), CV_8UC1);
  
  const size_type num_rows = this->fixed_grad_img_x_.rows;
  const size_type num_cols = this->fixed_grad_img_x_.cols;
  
  // allocate buffers/images for moving grad mag
  
  mov_grad_mags_ = AllocContiguousBufferForOpenCVImages<Scalar>(num_rows,
                                                                num_cols,
                                                                this->num_mov_imgs_,
                                                                &mov_grad_mags_buf_);

  if (use_median_as_thresh_)
  {
    // allocate buffers/images for moving grad mag for median computation
    
    mov_grad_mags_for_med_ = AllocContiguousBufferForOpenCVImages<Scalar>(num_rows,
                                                                          num_cols,
                                                                          this->num_mov_imgs_,
                                                                          &mov_grad_mags_for_med_buf_);
  }

  const Scalar thresh_to_use = use_median_as_thresh_ ?
            static_cast<Scalar>(FindMedian(fixed_grad_mag_, &mov_grad_mags_for_med_buf_[0])) :
                                    fixed_grad_mag_thresh_;

  for (size_type r = 0; r < num_rows; ++r)
  {
    unsigned char* use_fixed_mag_row = &use_fixed_mag_.at<unsigned char>(r,0);
    
    const Scalar* fixed_mag_row = &fixed_grad_mag_.at<Scalar>(r,0);

    for (size_type c = 0; c < num_cols; ++c)
    {
      if (fixed_mag_row[c] > thresh_to_use)
      {
        use_fixed_mag_row[c] = 1;
      }
    }
  }
}

void xreg::ImgSimMetric2DGradOrientCPU::compute()
{
  this->pre_compute();

  this->compute_sobel_grads();

  auto grad_orient_fn = [&] (const RangeType& r)
  {
    const size_type num_rows = this->fixed_grad_img_x_.rows;
    const size_type num_cols = this->fixed_grad_img_x_.cols;
      
    const size_type min_num = static_cast<size_type>(0.1 * num_rows * num_cols);

    const bool use_med = this->use_median_as_thresh_;

    for (size_type mov_idx = r.begin(); mov_idx < r.end(); ++mov_idx)
    {
      // compute moving mag
      cv::magnitude(this->mov_grad_imgs_x_[mov_idx],
                    this->mov_grad_imgs_y_[mov_idx],
                    this->mov_grad_mags_[mov_idx]);
      
      const Scalar cur_mov_thresh = use_med ?
                            FindMedian(this->mov_grad_mags_[mov_idx],
              &this->mov_grad_mags_for_med_[mov_idx].at<Scalar>(0,0)) :
                                          this->mov_grad_mag_thresh_;

      size_type num_pix_used = 0;

      Scalar s = 0;

      for (size_type r = 0; r < num_rows; ++r)
      {
        unsigned char* use_fixed_mag_row =
                               &this->use_fixed_mag_.at<unsigned char>(r,0);
        
        const Scalar* fixed_mag_row =
                                  &this->fixed_grad_mag_.at<Scalar>(r,0);

        const Scalar* fixed_grad_x_row =
                               &this->fixed_grad_img_x_.at<Scalar>(r,0);
        const Scalar* fixed_grad_y_row =
                               &this->fixed_grad_img_y_.at<Scalar>(r,0);

        const Scalar* mov_mag_row =
                         &this->mov_grad_mags_[mov_idx].at<Scalar>(r,0);

        const Scalar* mov_grad_x_row =
                        &this->mov_grad_imgs_x_[mov_idx].at<Scalar>(r,0);
        const Scalar* mov_grad_y_row =
                        &this->mov_grad_imgs_y_[mov_idx].at<Scalar>(r,0);

        for (size_type c = 0; c < num_cols; ++c)
        {
          if (use_fixed_mag_row[c] && (mov_mag_row[c] > cur_mov_thresh))
          {
            const Scalar mag_prod = fixed_mag_row[c] * mov_mag_row[c];

            if (mag_prod > Scalar(1.0e-6))
            {
              const Scalar cos_theta = ((fixed_grad_x_row[c] * mov_grad_x_row[c]) +
                                           (fixed_grad_y_row[c] * mov_grad_y_row[c]))
                                              / mag_prod;
              
              // NOTE: This is from the formula in DeSilva 2016, this did not work
              //       for me. Also, they maximize, while we minimize
              //const Scalar cos_theta_clamp = std::min(Scalar(-1),
              //                                    std::max(Scalar(1), cos_theta));

              //s += std::log(std::abs(std::acos(cos_theta_clamp)) + 1);
            
              const Scalar one_minus_cos_theta = 1 - cos_theta;
              //s += one_minus_cos_theta * one_minus_cos_theta;
              s += one_minus_cos_theta;

              ++num_pix_used;
            }
          }
        }
      }

      this->sim_vals_[mov_idx] = s / std::max(min_num, num_pix_used);
    }
  };

  ParallelFor(grad_orient_fn, RangeType(0, this->num_mov_imgs_));
}

