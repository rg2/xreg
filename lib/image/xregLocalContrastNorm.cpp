/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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

#include "xregLocalContrastNorm.h"

#include "xregAssert.h"
#include "xregOpenCVUtils.h"
#include "xregNormDist.h"

namespace
{

// The first image returned has the filtered value subtracted from original intensity
// The second image returned contains the sigma values (local sum of the squared values
// in the first image returned)
std::tuple<cv::Mat,cv::Mat> LocalContrastNormJarrettHelper(const cv::Mat& input_img,
                                                           const int win_len_rows,
                                                           const int win_len_cols)
{
  using namespace xreg;

  xregASSERT((win_len_rows % 2) == 1);
  xregASSERT((win_len_cols % 2) == 1);
  xregASSERT(win_len_rows > 0);
  xregASSERT(win_len_cols > 0);
  
  const int half_win_len_rows = win_len_rows / 2;
  const int half_win_len_cols = win_len_cols / 2;

  // TODO: add an assert on float input image
  // TODO: put some asserts here verifying that input_img has sufficient dimensions

  auto get_sigma = [] (const int len)
  {
    return ((((static_cast<float>(len) - 1.0f) * 0.5f) - 1.0f) * 0.3f) + 0.8f;
  };
 
  // compute the guassian weighting filter
  NormalDist2DIndep norm_dist(0, 0, get_sigma(win_len_cols), get_sigma(win_len_rows));
  
  MatMxN wgts(win_len_rows, win_len_cols);

  {  
    float wgts_sum = 0;

    for (int r = -half_win_len_rows; r <= half_win_len_rows; ++r)
    {
      const int wgts_r_idx = r + half_win_len_rows;

      for (int c = -half_win_len_cols; c <= half_win_len_cols; ++c)
      {
        const float w = norm_dist(c, r);

        wgts_sum += w;

        wgts(wgts_r_idx, c + half_win_len_cols) = w;
      }
    }
   
    // filter should sum to one 
    wgts /= wgts_sum;
  }

  // Setup the image that will hold values of subtracting the Gaussian
  // filtered values from the input.
  const int subtr_img_nr = input_img.rows - (win_len_rows - 1);
  const int subtr_img_nc = input_img.cols - (win_len_cols - 1);

  cv::Mat subtr_img = input_img(cv::Rect(
                         half_win_len_cols, half_win_len_rows,
                         subtr_img_nc, subtr_img_nr)).clone();

  cv::Rect roi;
  roi.height = win_len_rows;
  roi.width  = win_len_cols;

  for (int r = 0; r < subtr_img_nr; ++r)
  {
    auto* subtr_row = &subtr_img.at<float>(r,0);
  
    roi.y = r;

    for (int c = 0; c < subtr_img_nc; ++c)
    {
      roi.x = c;

      cv::Mat input_roi = input_img(roi);

      float wgt_sum = 0;
      for (int win_r = 0; win_r < win_len_rows; ++win_r)
      {
        for (int win_c = 0; win_c < win_len_cols; ++win_c)
        {
          wgt_sum += wgts(win_r,win_c) * input_roi.at<float>(win_r,win_c);
        }
      }

      subtr_row[c] = subtr_row[c] - wgt_sum;
    }
  }

  const int v_img_nr = subtr_img_nr - (win_len_rows - 1);
  const int v_img_nc = subtr_img_nc - (win_len_cols - 1);

  cv::Mat v_img = subtr_img(cv::Rect(
                      half_win_len_cols, half_win_len_rows,
                      v_img_nc, v_img_nr)).clone();

  cv::Mat sigma_img(v_img_nr, v_img_nc, v_img.type());
  
  for (int r = 0; r < v_img_nr; ++r)
  {
    auto* v_img_row = &v_img.at<float>(r,0);
    auto* sigma_row = &sigma_img.at<float>(r,0);
  
    roi.y = r;

    for (int c = 0; c < v_img_nc; ++c)
    {
      roi.x = c;

      cv::Mat v_roi = subtr_img(roi);

      float wgt_sum = 0;
      for (int win_r = 0; win_r < win_len_rows; ++win_r)
      {
        for (int win_c = 0; win_c < win_len_cols; ++win_c)
        {
          const auto v = v_roi.at<float>(win_r,win_c);

          wgt_sum += wgts(win_r,win_c) * v * v;
        }
      }

      v_img_row[c] = v_roi.at<float>(half_win_len_rows, half_win_len_cols);

      sigma_row[c] = std::sqrt(wgt_sum);
    }
  }

  return std::make_tuple(v_img, sigma_img);
}

}   // un-named

cv::Mat xreg::LocalContrastNormStdNorm(const cv::Mat& input_img, const int win_len_rows,
                                       const int win_len_cols,
                                       const boost::optional<float>& border_val)
{
  xregASSERT(input_img.channels() == 1);

  xregASSERT((win_len_rows % 2) == 1);
  xregASSERT((win_len_cols % 2) == 1);
  xregASSERT(win_len_rows > 0);
  xregASSERT(win_len_cols > 0);
  
  const int win_half_len_cols = win_len_cols / 2;
  const int win_half_len_rows = win_len_rows / 2;

  const int out_nr = input_img.rows - (win_len_rows - 1);
  const int out_nc = input_img.cols - (win_len_cols - 1);

  const bool has_border = border_val.has_value();

  cv::Mat out(has_border ? input_img.rows : out_nr,
              has_border ? input_img.cols : out_nc,
              CV_32FC1);
 
  if (has_border)
  {
    out.setTo(*border_val);
  }

  const int row_off = has_border ? win_half_len_rows : 0;
  const int col_off = has_border ? win_half_len_cols : 0;

  cv::Rect input_roi;
  input_roi.height = win_len_rows;
  input_roi.width  = win_len_cols;

  for (int r = 0; r < out_nr; ++r)
  {
    auto* row = &out.at<float>(r + row_off, 0);
    
    input_roi.y = r;

    for (int c = 0; c < out_nc; ++c)
    {
      const int cc = c + col_off;

      input_roi.x = c;

      const auto mean_std_dev = MeanStdDev(input_img(input_roi));

      row[cc] = static_cast<float>((row[cc] - std::get<0>(mean_std_dev)) /
                                     std::max(1.0e-6, std::get<1>(mean_std_dev)));
    }
  }

  return out;
}

cv::Mat xreg::LocalContrastNormJarrett(const cv::Mat& input_img,
                                       const int win_len_rows,
                                       const int win_len_cols,
                                       const boost::optional<float>& border_val)
{
  constexpr float sigma_lower_bound = 1.0e-6f;
  
  cv::Mat v_img;
  cv::Mat sigma_img;

  std::tie(v_img, sigma_img) = LocalContrastNormJarrettHelper(
                                  input_img, win_len_rows, win_len_cols);

  const int v_img_nr = v_img.rows;
  const int v_img_nc = v_img.cols;

  for (int r = 0; r < v_img_nr; ++r)
  {
    auto* v_img_row = &v_img.at<float>(r,0);
    
    const auto* sigma_row = &sigma_img.at<float>(r,0);
    
    for (int c = 0; c < v_img_nc; ++c)
    {
      v_img_row[c] /= std::max(sigma_lower_bound, sigma_row[c]);
    }
  }
  
  cv::Mat out_img;

  if (border_val)
  {
    out_img = cv::Mat(input_img.rows, input_img.cols, v_img.type());
    
    out_img.setTo(*border_val);
   
    v_img.copyTo(out_img(cv::Rect((input_img.cols - v_img.cols) / 2,
                     (input_img.rows - v_img.rows) / 2,
                     v_img.cols, v_img.rows)));
  }
  else
  {
    out_img = v_img;
  }

  return out_img;
}

