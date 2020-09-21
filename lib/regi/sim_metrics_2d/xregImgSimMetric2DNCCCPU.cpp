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

#include "xregImgSimMetric2DNCCCPU.h"

#include "xregTBBUtils.h"

namespace  // un-named
{

using namespace xreg;

template <class tMaskMatDerived>
size_type ComputeLenFromMask(const Eigen::MatrixBase<tMaskMatDerived>& mask)
{
  const size_type non_mask_len = mask.size();
  
  size_type len = 0;
  
  for (size_type i = 0; i < non_mask_len; ++i)
  {
    if (mask(i))
    {
      ++len;
    }
  }

  return len; 
}

template <class tMatDerived>
std::tuple<typename tMatDerived::Scalar, typename tMatDerived::Scalar>
ComputeImage2DMeanStdDev(const Eigen::MatrixBase<tMatDerived>& pixel_buf)
{
  using PixelScalar = typename tMatDerived::Scalar;

  const PixelScalar mean = pixel_buf.array().mean();

  const PixelScalar std_dev = std::max(static_cast<PixelScalar>(1.0e-6),
                          std::sqrt((pixel_buf.array() - mean).square().sum() / (pixel_buf.size() - 1)));
  
  return std::make_tuple(mean,std_dev);
}

template <class tMatDerived>
std::tuple<typename tMatDerived::Scalar, typename tMatDerived::Scalar>
ComputeZeroMeanImageAndStats(Eigen::MatrixBase<tMatDerived>* pixel_buf)
{
  const auto mean_and_std = ComputeImage2DMeanStdDev(*pixel_buf);

  pixel_buf->array() -= std::get<0>(mean_and_std);

  return mean_and_std;
}

template <class tMatDerived, class tMaskMatDerived>
std::tuple<typename tMatDerived::Scalar, typename tMatDerived::Scalar>
ComputeImage2DMeanStdDevWithMask(const Eigen::MatrixBase<tMatDerived>& pixel_buf,
                                 const Eigen::MatrixBase<tMaskMatDerived>& mask,
                                 const size_type len)
{
  using PixelScalar = typename tMatDerived::Scalar;

  const size_type non_mask_len = pixel_buf.size();

  PixelScalar mean = 0;
  
  for (size_type i = 0; i < non_mask_len; ++i)
  {
    if (mask(i))
    {
      mean += pixel_buf(i);
    }
  }

  mean /= len;

  PixelScalar std_dev = 0;
  PixelScalar tmp = 0;

  for (size_type i = 0; i < non_mask_len; ++i)
  {
    if (mask(i))
    {
      tmp = pixel_buf(i) - mean;

      std_dev += tmp * tmp;
    }
  }

  std_dev = std::max(static_cast<PixelScalar>(1.0e-6), std::sqrt(std_dev / (len - 1)));

  return std::make_tuple(mean,std_dev);
}

template <class tMatDerived, class tMaskMatDerived>
std::tuple<typename tMatDerived::Scalar, typename tMatDerived::Scalar>
ComputeZeroMeanImageAndStatsWithMask(Eigen::MatrixBase<tMatDerived>* pixel_buf,
                                     const Eigen::MatrixBase<tMaskMatDerived>& mask,
                                     const size_type len)
{
  const auto mean_and_std = ComputeImage2DMeanStdDevWithMask(*pixel_buf, mask, len);

  // we don't care about subtracting from masked pixels... the optimizations
  // in eigen probably out-weight putting in extra branches?
  pixel_buf->array() -= std::get<0>(mean_and_std);
  
  return mean_and_std;
}

}  // un-named

void xreg::ImgSimMetric2DNCCCPU::allocate_resources()
{
  ImgSimMetric2DCPU::allocate_resources();

  const auto itk_size = this->fixed_img_->GetLargestPossibleRegion().GetSize();

  img_num_cols_ = itk_size[0];
  img_num_rows_ = itk_size[1];
  img_num_pix_  = img_num_cols_ * img_num_rows_;

  // allocate the zero-mean fixed image buffer
  zero_mean_fixed_vec_.resize(img_num_pix_);

  // for now, I am going to over-write the input moving image buffers with
  // zero-mean versions
#if 0
  // allocate the zero-mean moving image buffers
  zero_mean_mov_vecs_.resize(this->num_mov_imgs_);

  for (size_type mov_idx = 0; mov_idx < this->num_mov_imgs_; ++mov_idx)
  {
    zero_mean_mov_vecs_[mov_idx].resize(img_num_pix_);
  }
#endif

  // mask processing also includes transforming the fixed image into a zero mean
  // vector
  this->process_updated_mask();
}

void xreg::ImgSimMetric2DNCCCPU::compute()
{
  this->pre_compute();
  
  auto ncc_helper_fn = [&] (const RangeType& r)
  {
    Scalar mov_mean   = 0;
    Scalar mov_stddev = 0;
    Scalar sim_val    = 0;

    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      Eigen::Map<ImageVec> cur_mov_vec(this->mov_imgs_buf_ + (range_idx * img_num_pix_), img_num_pix_);

      if (!this->mask_)
      {
        std::tie(mov_mean,mov_stddev) = ComputeZeroMeanImageAndStats(&cur_mov_vec);
        
        sim_val = zero_mean_fixed_vec_.dot(cur_mov_vec) / (img_num_pix_ * fixed_img_stddev_ * mov_stddev);
      }
      else
      {
        std::tie(mov_mean,mov_stddev) = ComputeZeroMeanImageAndStatsWithMask(&cur_mov_vec,
                                                                             mask_vec_, mask_len_);
    
        sim_val = 0;
        for (size_type i = 0; i < img_num_pix_; ++i)
        {
          if (mask_vec_(i))
          {
            sim_val += zero_mean_fixed_vec_(i) * cur_mov_vec(i);
          }
        }

        sim_val /= mask_len_ * fixed_img_stddev_ * mov_stddev;
      }

      // remap the NCC score from [-1,1] for our purposes of minimization
      //   * Since we are minimizing, the NCC score is negated (good correlations have larger NCC values)
      //   * normalize into [0,1] for convenience
      this->sim_vals_[range_idx] = (1 - sim_val) * Scalar(0.5);
    }
  };

  ParallelFor(ncc_helper_fn, RangeType(0, this->num_mov_imgs_));
}

void xreg::ImgSimMetric2DNCCCPU::process_mask()
{
  ImgSimMetric2DCPU::process_mask();

  // copy the fixed image
  zero_mean_fixed_vec_ = Eigen::Map<ImageVec>(this->fixed_img_->GetBufferPointer(),
                                              img_num_pix_);
  
  if (!this->mask_)
  {
    // compute mean, stddev, and zero-mean fixed image
    std::tie(fixed_img_mean_,fixed_img_stddev_) = ComputeZeroMeanImageAndStats(&zero_mean_fixed_vec_);
    
    mask_len_ = 0;
  }
  else
  {
    mask_vec_ = MappedImageMaskVec(this->mask_->GetBufferPointer(), img_num_pix_);

    mask_len_ = ComputeLenFromMask(mask_vec_);

    // compute mean, stddev, and zero-mean fixed image using MASK
    std::tie(fixed_img_mean_,fixed_img_stddev_) = ComputeZeroMeanImageAndStatsWithMask(&zero_mean_fixed_vec_,
                                                                                       mask_vec_, mask_len_);
  }
}

