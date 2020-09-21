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

#include "xregImgSimMetric2DGradDiffCPU.h"

#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregBasicStats.h"
#include "xregLineSearchOpt.h"
#include "xregTBBUtils.h"

void xreg::ImgSimMetric2DGradDiffCPU::allocate_resources()
{
  // fixed image sobel gradients are computed here, and buffers for the moving images
  // and moving image gradients are allocated
  ImgSimMetric2DGradImgCPU::allocate_resources();

  num_rows_ = this->fixed_grad_img_x_.rows;
  num_cols_ = this->fixed_grad_img_x_.cols;
  num_pix_  = num_rows_ * num_cols_;

  this->process_updated_mask();

  tmp_imgs_vec1_.resize(this->num_mov_imgs_);
  tmp_imgs_vec2_.resize(this->num_mov_imgs_);
  tmp_imgs_vec3_.resize(this->num_mov_imgs_);

  for (size_type img_idx = 0; img_idx < this->num_mov_imgs_; ++img_idx)
  {
    tmp_imgs_vec1_[img_idx].resize(num_pix_);
    tmp_imgs_vec2_[img_idx].resize(num_pix_);
    tmp_imgs_vec3_[img_idx].resize(num_pix_);
  }

  if (track_sub_prob_inits_)
  {
    sub_prob_vals_.resize(this->num_mov_imgs_);
  }
}

void xreg::ImgSimMetric2DGradDiffCPU::compute()
{
  this->pre_compute();
  
  this->compute_sobel_grads();

  auto grad_diff_fn = [&] (const RangeType& r)
  {
    const bool apply_mask = this->mask_;

    const ImageArray* fixed_grads[2] = { &this->fixed_grad_img_x_vec_,
                                         &this->fixed_grad_img_y_vec_ };
    const Scalar fixed_grad_vars[2] = { this->fixed_grad_x_var_,
                                        this->fixed_grad_y_var_ };

    cvMatList* mov_grad_imgs[2] = { &this->mov_grad_imgs_x_,
                                    &this->mov_grad_imgs_y_ };

    for (size_type mov_idx = r.begin(); mov_idx < r.end(); ++mov_idx)
    {
      this->sim_vals_[mov_idx] = 0;

      for (size_type grad_dir_idx = 0; grad_dir_idx < 2; ++grad_dir_idx)
      {
        cv::Mat& mov_grad_ocv_img = mov_grad_imgs[grad_dir_idx]->at(mov_idx);

        if (apply_mask)
        {
          ApplyMaskToSelf(&mov_grad_ocv_img, this->mask_ocv_);
        }

        const ImageArrayMap mov_grad_vec(&mov_grad_ocv_img.at<Scalar>(0,0),
                                         this->num_pix_);

        using OptScalar  = LineSearchOptimization::Scalar;
        using OptParamPt = LineSearchOptimization::Pt;

        LineSearchOptimization opt;

        const ImageArray& fixed_grad = *fixed_grads[grad_dir_idx];

        const Scalar fixed_grad_var = fixed_grad_vars[grad_dir_idx];
  
        ImageArray& fixed_grad_minus_s_times_mov_grad =
                                            this->tmp_imgs_vec1_[mov_idx];
        ImageArray& fixed_grad_minus_s_times_mov_grad_sq_plus_var =
                                            this->tmp_imgs_vec2_[mov_idx];
        ImageArray& fixed_grad_minus_s_times_mov_grad_sq_plus_var_sq =
                                            this->tmp_imgs_vec3_[mov_idx];

        opt.obj_fn = [&] (const OptParamPt& x, const bool compute_grad, const bool compute_hessian)
        {
          const Scalar s = x(0);

          Scalar F = 0;
          
          OptParamPt grad;
          LineSearchOptimization::Mat hessian;

          if (fixed_grad_var > 1.0e-8)
          {
            fixed_grad_minus_s_times_mov_grad = fixed_grad - (s * mov_grad_vec);

            fixed_grad_minus_s_times_mov_grad_sq_plus_var = fixed_grad_minus_s_times_mov_grad.square() + fixed_grad_var;

            F = -1 * (fixed_grad_minus_s_times_mov_grad_sq_plus_var.inverse() * fixed_grad_var).sum();

            Scalar two_times_fixed_grad_var = 0;

            if (compute_grad || compute_hessian)
            {
              two_times_fixed_grad_var = 2 * fixed_grad_var;
              fixed_grad_minus_s_times_mov_grad_sq_plus_var_sq = fixed_grad_minus_s_times_mov_grad_sq_plus_var.square();
            }

            if (compute_grad)
            {
              grad.resize(1);
              grad(0) = -two_times_fixed_grad_var * ((mov_grad_vec * fixed_grad_minus_s_times_mov_grad) / fixed_grad_minus_s_times_mov_grad_sq_plus_var_sq).sum();
            }

            if (compute_hessian)
            {
              hessian.resize(1,1);
              hessian(0) = -two_times_fixed_grad_var * (mov_grad_vec.square() * (((4 * fixed_grad_minus_s_times_mov_grad.square()) / fixed_grad_minus_s_times_mov_grad_sq_plus_var.cube()) - fixed_grad_minus_s_times_mov_grad_sq_plus_var_sq.inverse())).sum();
            }
          }
          else
          {
            if (compute_grad)
            {
              grad.resize(1);
              grad(0) = 0;
            }
            if (compute_hessian)
            {
              hessian.resize(1,1);
              hessian(0) = 0;
            }
          }

          return std::make_tuple(F,grad,hessian);
        };

        ModNewtonSearchDir search_dir;        
        opt.search_dir_fn = search_dir.callback_fn();

        opt.backtrack_fn = MakeBacktrackingArmijoCallback();

        opt.max_its = this->num_sub_prob_its_;
        opt.grad_tol = 1.0e-6;

        OptParamPt init_s(1);
        OptParamPt final_s;
        OptScalar  min_sim_s;

        init_s(0) = this->sub_prob_init_guess_[mov_idx];
        
        std::tie(std::ignore,final_s,min_sim_s,std::ignore) = opt.solve(init_s);

        // see if we should initialize another solver with the opposite
        // sign for initial guess
        if (this->do_two_guess_symmetric_ &&
            (std::abs(init_s(0)) > 1.0e-6))
        {
          OptParamPt final_s_neg;
          OptScalar  min_sim_s_neg;
        
          init_s(0) *= -1;
          std::tie(std::ignore,final_s_neg,min_sim_s_neg,std::ignore) = opt.solve(init_s);

          if (min_sim_s_neg < min_sim_s)
          {
            min_sim_s = min_sim_s_neg;
            final_s   = final_s_neg;
          }
        }
        
        this->sim_vals_[mov_idx] += min_sim_s;
        
        if (this->track_sub_prob_inits_)
        {
          this->sub_prob_vals_[mov_idx][grad_dir_idx] = final_s(0);
        }
      }
    }
  };

  ParallelFor(grad_diff_fn, RangeType(0, this->num_mov_imgs_));

  if (track_sub_prob_inits_)
  {
    sub_prob_init_guess_ = sub_prob_vals_[std::min_element(this->sim_vals_.begin(),
                                                           this->sim_vals_.end())
                                            - this->sim_vals_.begin()];
  }
}
  
xreg::size_type xreg::ImgSimMetric2DGradDiffCPU::num_sub_prob_its() const
{
  return num_sub_prob_its_;
}

void xreg::ImgSimMetric2DGradDiffCPU::set_num_sub_prob_its(const size_type num_its)
{
  num_sub_prob_its_ = num_its;
}

std::array<double,2> xreg::ImgSimMetric2DGradDiffCPU::sub_prob_init_guess() const
{
  return sub_prob_init_guess_;
}

void xreg::ImgSimMetric2DGradDiffCPU::set_sub_prob_init_guess(const double init_guess)
{
  sub_prob_init_guess_ = { init_guess, init_guess };
}

void xreg::ImgSimMetric2DGradDiffCPU::set_sub_prob_init_guess(const std::array<double,2>& init_guess)
{
  sub_prob_init_guess_ = init_guess;
}

bool xreg::ImgSimMetric2DGradDiffCPU::do_two_guess_symmetric() const
{
  return do_two_guess_symmetric_;
}

void xreg::ImgSimMetric2DGradDiffCPU::set_do_two_guess_symmetric(const bool do_sym)
{
  do_two_guess_symmetric_ = do_sym;
}

bool xreg::ImgSimMetric2DGradDiffCPU::track_sub_prob_inits() const
{
  return track_sub_prob_inits_;
}

void xreg::ImgSimMetric2DGradDiffCPU::set_track_sub_prob_inits(const bool track_inits)
{
  track_sub_prob_inits_ = track_inits;
}

void xreg::ImgSimMetric2DGradDiffCPU::process_mask()
{
  ImgSimMetric2DGradImgCPU::process_mask();

  if (this->mask_)
  {
    mask_ocv_ = ShallowCopyItkToOpenCV(this->mask_.GetPointer());
   
    // deep copy the fixed image gradients, since we'll modify the buffers
    // with the mask 
    fixed_grad_img_x_to_use_ = this->fixed_grad_img_x_.clone();
    fixed_grad_img_y_to_use_ = this->fixed_grad_img_y_.clone();

    // By setting gradient values to zero at masked out locations in the
    // fixed AND moving images, we can still use optimized array computations
    ApplyMaskToSelf(&this->fixed_grad_img_x_to_use_, mask_ocv_);
    ApplyMaskToSelf(&this->fixed_grad_img_y_to_use_, mask_ocv_);
  }
  else
  {
    // shallow copy - will not be modifying the buffers
    fixed_grad_img_x_to_use_ = this->fixed_grad_img_x_;
    fixed_grad_img_y_to_use_ = this->fixed_grad_img_y_;
  }

  // copy the gradient images as Eigen types
  fixed_grad_img_x_vec_ = ImageArrayMap(
                        &this->fixed_grad_img_x_to_use_.at<Scalar>(0,0),
                                        num_pix_).array();
  fixed_grad_img_y_vec_ = ImageArrayMap(
                        &this->fixed_grad_img_y_to_use_.at<Scalar>(0,0),
                                        num_pix_).array();

  // compute variances of the fixed gradient images
  const auto x_std_dev = SampleStdDev(fixed_grad_img_x_vec_);
  fixed_grad_x_var_ = x_std_dev * x_std_dev;

  const auto y_std_dev = SampleStdDev(fixed_grad_img_y_vec_);
  fixed_grad_y_var_ = y_std_dev * y_std_dev;
}
