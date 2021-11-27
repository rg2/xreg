/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#include "xregOpenCVUtils.h"

#include <cmath>

#include "xregExceptionUtils.h"
#include "xregStringUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregITKIOUtils.h"
#include "xregTBBUtils.h"
#include "xregHDF5.h"

#define xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(OCV_TYPE, SCALAR_TYPE) \
  case OCV_TYPE: \
    { \
      itk::Image<SCALAR_TYPE,2>::Pointer itk_img = xreg::ShallowCopyOpenCVToItk<SCALAR_TYPE>(img); \
      WriteITKImageRemap8bpp(itk_img.GetPointer(), path); \
    } \
    break;

void xreg::WriteImageRemap8bpp(cv::Mat& img, const std::string& path)
{
  xregASSERT(img.channels() == 1);

  switch (img.type())
  {
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_8U, unsigned char);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_8S, char);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_16U, unsigned short);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_16S, short);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_32S, int);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_32F, float);
  xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE(CV_64F, double);
  default:
    xregThrow("unsupported OpenCV Image type!");
  }
}

#undef xregOCV_TO_ITK_IMAGE_REMAP_AND_WRITE_CASE

cv::Mat xreg::OverlayEdges(const cv::Mat& base_img, const cv::Mat& edge_img,
                         const unsigned int edge_channel)
{
  const int nr = base_img.rows;
  const int nc = base_img.cols;

  cv::Mat dst_img;

  if (base_img.type() == CV_8UC3)
  {
    dst_img = base_img.clone();
  }
  else
  {
    // TODO: error checking!

    dst_img = cv::Mat(nr, nc, CV_8UC3);
    cv::cvtColor(base_img, dst_img, CV_GRAY2BGR);
  }

  for (int r = 0; r < nr; ++r)
  {
    unsigned char* dst_row = &dst_img.at<unsigned char>(r,0);
    const unsigned char* edge_row = &edge_img.at<unsigned char>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      if (edge_row[c])
      {
        const int off = 3 * c;

        dst_row[off + 0] = (edge_channel == 0) ? 255 : 0;    // B
        dst_row[off + 1] = (edge_channel == 1) ? 255 : 0;    // G
        dst_row[off + 2] = (edge_channel == 2) ? 255 : 0;  // R
      }
    }
  }

  return dst_img;
}

cv::Mat xreg::OverlayRectBoundary(const cv::Mat& base_img, const cv::Rect& rect,
                                  const unsigned int edge_channel)
{
  const int nr = base_img.rows;
  const int nc = base_img.cols;

  xregASSERT((rect.x >= 0) && (rect.y >= 0)   &&
             ((rect.x + rect.width - 1) < nc) &&
             ((rect.y + rect.height - 1) < nr));

  cv::Mat dst_img;

  if (base_img.type() == CV_8UC3)
  {
    dst_img = base_img.clone();
  }
  else
  {
    // TODO: error checking!

    dst_img = cv::Mat(nr, nc, CV_8UC3);
    cv::cvtColor(base_img, dst_img, CV_GRAY2BGR);
  }

  const unsigned char blue_val  = (edge_channel == 0) ? 255 : 0;
  const unsigned char green_val = (edge_channel == 1) ? 255 : 0;
  const unsigned char red_val   = (edge_channel == 2) ? 255 : 0;

  // top edge
  {
    unsigned char* dst_row = &dst_img.at<unsigned char>(rect.y, rect.x * 3);

    for (int c = 0; c < rect.width; ++c)
    {
      const int off = 3 * c;
      dst_row[off + 0] = blue_val;
      dst_row[off + 1] = green_val;
      dst_row[off + 2] = red_val;
    }
  }

  // left and right edges
  for (int r = 1; r < (rect.height - 1); ++r)
  {
    const int rr = rect.y + r;

    unsigned char* dst_row = &dst_img.at<unsigned char>(rr, rect.x * 3);
      
    dst_row[0] = blue_val;
    dst_row[1] = green_val;
    dst_row[2] = red_val;
    
    dst_row = &dst_img.at<unsigned char>(rr, (rect.x + rect.width - 1) * 3);
      
    dst_row[0] = blue_val;
    dst_row[1] = green_val;
    dst_row[2] = red_val;
  }

  // bottom edge
  {
    unsigned char* dst_row = &dst_img.at<unsigned char>(rect.y + rect.height - 1, rect.x * 3);

    for (int c = 0; c < rect.width; ++c)
    {
      const int off = 3 * c;
      dst_row[off + 0] = blue_val;
      dst_row[off + 1] = green_val;
      dst_row[off + 2] = red_val;
    }
  }

  return dst_img;
}

std::vector<cv::Mat> xreg::CreateSummaryTiledImages(const std::vector<cv::Mat>& src_imgs,
                                                    const std::vector<std::string>& src_names,
                                                    const size_type num_tile_rows,
                                                    const size_type num_tile_cols,
                                                    const int border_thickness)
{
  const bool has_strs = !src_names.empty();

  const size_type num_src_imgs = src_imgs.size();

  size_type roi_num_rows = 0;
  size_type roi_num_cols = 0;

  for (size_type i = 0; i < num_src_imgs; ++i)
  {
    const size_type cur_nr = static_cast<size_type>(src_imgs[i].rows);
    const size_type cur_nc = static_cast<size_type>(src_imgs[i].cols);

    if (roi_num_rows < cur_nr)
    {
      roi_num_rows = cur_nr;
    }

    if (roi_num_cols < cur_nc)
    {
      roi_num_cols = cur_nc;
    }
  }

  const size_type summary_img_num_rows = num_tile_rows * roi_num_rows;
  const size_type summary_img_num_cols = num_tile_cols * roi_num_cols;

  const size_type num_summary_imgs = static_cast<size_type>(std::ceil(static_cast<double>(num_src_imgs) / (num_tile_rows * num_tile_cols)));

  std::vector<cv::Mat> dst_imgs(num_summary_imgs);

  size_type src_img_idx = 0;
  for (size_type summary_img_idx = 0; summary_img_idx < num_summary_imgs; ++summary_img_idx)
  {
    // this is BGR
    cv::Mat sum_img(summary_img_num_rows, summary_img_num_cols, CV_8UC3);

    cv::Rect roi;

    sum_img.setTo(0);

    for (size_type tile_row = 0; (tile_row < num_tile_rows) && (src_img_idx < num_src_imgs); ++tile_row)
    {
      roi.y = tile_row * roi_num_rows;

      for (size_type tile_col = 0; (tile_col < num_tile_cols) && (src_img_idx < num_src_imgs); ++tile_col)
      {
        roi.x = tile_col * roi_num_cols;

        cv::Mat cur_img;

        const size_type cur_img_nr = src_imgs[src_img_idx].rows;
        const size_type cur_img_nc = src_imgs[src_img_idx].cols;

        if (src_imgs[src_img_idx].type() == CV_8UC3)
        {
          cur_img = src_imgs[src_img_idx];
        }
        else  // Assuming everything else is grayscale!!!
        {
          cur_img = cv::Mat(cur_img_nr, cur_img_nc, CV_8UC3);
          cv::cvtColor(src_imgs[src_img_idx], cur_img, CV_GRAY2BGR);
        }

        // it may be that the image is smaller than the tile
        roi.width  = cur_img_nc;
        roi.height = cur_img_nr;
        cur_img.copyTo(sum_img(roi));

        // to draw the yellow rectangle, we want to use the full tile size
        roi.width  = roi_num_cols;
        roi.height = roi_num_rows;
        cv::Mat tile_roi_img = sum_img(roi);

        if (border_thickness > 0)
        {
          cv::rectangle(tile_roi_img, cv::Rect(0, 0, roi_num_cols, roi_num_rows),
                        cv::Scalar(0, 255, 255), border_thickness);
        }

        if (has_strs)
        {
          cv::putText(tile_roi_img, src_names[src_img_idx], cv::Point(10, 50),
                      cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
        }

        ++src_img_idx;
      }
    }

    dst_imgs[summary_img_idx] = sum_img;
  }

  return dst_imgs;
}

cv::Mat xreg::Create2CMV(const cv::Mat& img_red, const cv::Mat& img_cyan)
{
  xregASSERT(img_red.type() == CV_8UC1);
  xregASSERT(img_cyan.type() == img_red.type());

  const int nr = img_red.rows;
  const int nc = img_red.cols;

  xregASSERT(nr == img_cyan.rows);
  xregASSERT(nc == img_cyan.cols);

  // bgr image
  cv::Mat cmv_img(img_red.size(), CV_8UC3);

  for (int r = 0; r < nr; ++r)
  {
    unsigned char* dst_row = &cmv_img.at<unsigned char>(r,0);

    const unsigned char* src_red  = &img_red.at<unsigned char>(r,0);
    const unsigned char* src_cyan = &img_cyan.at<unsigned char>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      unsigned char* cur_dst = dst_row + (3 * c);

      cur_dst[0] = src_cyan[c];
      cur_dst[1] = src_cyan[c];
      cur_dst[2] = src_red[c];
    }
  }

  return cmv_img;
}

cv::Mat xreg::AddFlowVecs(const cv::Mat& img_bgr, const cv::Mat& flow,
                          const int flow_subsample, const float vec_scale,
                          const int /*img_upsample*/)
{
  cv::Mat dst_img = img_bgr.clone();

  const int sub_sample = std::max(1, flow_subsample);

  for (int r = 0; r < dst_img.rows; r += sub_sample)
  {
    const float* flow_row = &flow.at<float>(r, 0);

    for (int c = 0; c < dst_img.cols; c += sub_sample)
    {
      const int off = c * 2;

      if (false)
      {
        cv::line(dst_img,
                 cv::Point(c,r),
                 cv::Point(c + static_cast<int>(std::round(flow_row[off] * vec_scale)),
                           r + static_cast<int>(std::round(flow_row[off + 1] * vec_scale))),
                 cv::Scalar(0,255,0));
      }
      else
      {
        cv::arrowedLine(dst_img,
                        cv::Point(c,r),
                        cv::Point(c + static_cast<int>(std::round(flow_row[off] * vec_scale)),
                                  r + static_cast<int>(std::round(flow_row[off + 1] * vec_scale))),
                        cv::Scalar(0,255,0));
      }
    }
  }
  
  return dst_img;
}

cv::Mat xreg::OverlayPtsAsCircles(const cv::Mat& img, const Pt2List& pts, const AnyParamMap& other_params)
{
  const int nr = img.rows;
  const int nc = img.cols;

  cv::Mat dst_img;

  if (img.type() == CV_8UC3)
  {
    dst_img = img.clone();
  }
  else
  {
    // TODO: error checking!

    dst_img = cv::Mat(nr, nc, CV_8UC3);
    cv::cvtColor(img, dst_img, CV_GRAY2BGR);
  }

  const int num_pts = pts.size();

  int radius = 5;

  auto radius_it = other_params.find("radius");
  if (radius_it != other_params.end())
  {
    radius = boost::any_cast<int>(radius_it->second);
  }

  int thickness = 1;

  auto thick_it = other_params.find("thickness");
  if (thick_it != other_params.end())
  {
    thickness = boost::any_cast<int>(thick_it->second);
  }

  const auto& color_lut = OpenCVColorNameToScalar();

  std::vector<cv::Scalar> colors = { color_lut.at("green") };
  
  auto color_it = other_params.find("color");

  if (color_it != other_params.end())
  {
    const auto& t = color_it->second.type();

    if (t == typeid(const char*))
    {
      colors = { color_lut.at(boost::any_cast<const char*>(color_it->second)) };
    }
    else if (t == typeid(std::string))
    {
      colors = { color_lut.at(boost::any_cast<std::string>(color_it->second)) };
    }
    else if (t == typeid(std::vector<std::string>))
    {
      const auto& color_strs = boost::any_cast<std::vector<std::string>>(color_it->second);
      
      colors.clear();
      for (const auto& s : color_strs)
      {
        colors.push_back(color_lut.at(s));
      }
    }
    else
    {
      xregThrow("unsupported color type!!");
    }
  }

  const int num_colors = colors.size();

  for (int i = 0; i < num_pts; ++i)
  {
    const auto& p = pts[i];

    cv::Point ocv_pt;
    ocv_pt.x = p[0];
    ocv_pt.y = p[1];
    
    cv::circle(dst_img, ocv_pt, radius, colors[i % num_colors], thickness);
  }

  return dst_img;
}

const std::unordered_map<std::string, cv::Scalar>& xreg::OpenCVColorNameToScalar()
{
  // TODO: add more
  static std::unordered_map<std::string, cv::Scalar> lut =
    {
      { "red",     cv::Scalar(0,0,255)     },
      { "r",       cv::Scalar(0,0,255)     },
      { "green",   cv::Scalar(0,255,0)     },
      { "g",       cv::Scalar(0,255,0)     },
      { "blue",    cv::Scalar(255,0,0)     },
      { "b",       cv::Scalar(255,0,0)     },
      { "yellow",  cv::Scalar(0,255,255)   },
      { "y",       cv::Scalar(0,255,255)   },
      { "magenta", cv::Scalar(255,0,255)   },
      { "m",       cv::Scalar(255,0,255)   },
      { "cyan",    cv::Scalar(255,255,0)   },
      { "c",       cv::Scalar(255,255,0)   },
      { "white",   cv::Scalar(255,255,255) },
      { "w",       cv::Scalar(255,255,255) },
      { "black",   cv::Scalar(0,0,0)       },
      { "k",       cv::Scalar(0,0,0)       },
      { "orange",  cv::Scalar(0,127,255)   }
    };
  
  return lut;
}

cv::Mat xreg::OverlayPts(const cv::Mat& img, const Pt2List& pts,
                         const AnyParamMap& other_params)
{
  const int nr = img.rows;
  const int nc = img.cols;

  cv::Mat dst_img;

  if (img.type() == CV_8UC3)
  {
    dst_img = img.clone();
  }
  else
  {
    // TODO: error checking!

    dst_img = cv::Mat(nr, nc, CV_8UC3);
    cv::cvtColor(img, dst_img, CV_GRAY2BGR);
  }

  const int num_pts = pts.size();

  int radius = 5;

  auto radius_it = other_params.find("radius");
  if (radius_it != other_params.end())
  {
    radius = boost::any_cast<int>(radius_it->second);
  }

  int thickness = 1;

  auto thick_it = other_params.find("thickness");
  if (thick_it != other_params.end())
  {
    thickness = boost::any_cast<int>(thick_it->second);
  }

  const auto& color_lut = OpenCVColorNameToScalar();

  std::vector<cv::Scalar> colors = { color_lut.at("green") };
  
  auto color_it = other_params.find("color");

  if (color_it != other_params.end())
  {
    const auto& t = color_it->second.type();

    if (t == typeid(const char*))
    {
      colors = { color_lut.at(boost::any_cast<const char*>(color_it->second)) };
    }
    else if (t == typeid(std::string))
    {
      colors = { color_lut.at(boost::any_cast<std::string>(color_it->second)) };
    }
    else if (t == typeid(std::vector<std::string>))
    {
      const auto& color_strs = boost::any_cast<std::vector<std::string>>(color_it->second);
      
      colors.clear();
      for (const auto& s : color_strs)
      {
        colors.push_back(color_lut.at(s));
      }
    }
    else
    {
      xregThrow("unsupported color type!!");
    }
  }

  const int num_colors = colors.size();

  std::vector<std::string> marker_type_names = { "o" };  // default to circle

  auto marker_type_it = other_params.find("marker");

  if (marker_type_it != other_params.end())
  {
    const auto& t = marker_type_it->second.type();

    if (t == typeid(const char*))
    {
      marker_type_names = { boost::any_cast<const char*>(marker_type_it->second) };
    }
    else if (t == typeid(std::string))
    {
      marker_type_names = { boost::any_cast<std::string>(marker_type_it->second) };
    }
    else if (t == typeid(std::vector<std::string>))
    {
      marker_type_names = boost::any_cast<std::vector<std::string>>(marker_type_it->second);
    }
    else
    {
      xregThrow("unsupported marker string type!");
    }
  }

  const int num_marker_types = marker_type_names.size();
  
  const std::unordered_map<std::string,cv::MarkerTypes> kMARKER_NAME_TO_ENUM = {
    { "+",        cv::MARKER_CROSS         },
    { "cross",    cv::MARKER_CROSS         },
    { "x",        cv::MARKER_TILTED_CROSS  },
    { "*",        cv::MARKER_STAR          },
    { "star",     cv::MARKER_STAR          },
    { "d",        cv::MARKER_DIAMOND       },
    { "diamond",  cv::MARKER_DIAMOND       },
    { "s",        cv::MARKER_SQUARE        },
    { "square",   cv::MARKER_SQUARE        },
    { "t",        cv::MARKER_TRIANGLE_UP   },
    { "tri-up",   cv::MARKER_TRIANGLE_UP   },
    { "tri-down", cv::MARKER_TRIANGLE_DOWN }
  };

  for (int i = 0; i < num_pts; ++i)
  {
    const auto& p = pts[i];

    cv::Point ocv_pt;
    ocv_pt.x = p[0];
    ocv_pt.y = p[1];

    const auto& cur_color = colors[i % num_colors];

    const std::string cur_marker_type = ToLowerCase(marker_type_names[i % num_marker_types]);

    if ((cur_marker_type == "o") || (cur_marker_type == "circle"))
    {
      cv::circle(dst_img, ocv_pt, radius, cur_color, thickness);
    }
    else
    {
      const auto mark_it = kMARKER_NAME_TO_ENUM.find(cur_marker_type);
      xregASSERT(mark_it != kMARKER_NAME_TO_ENUM.end());

      const auto& m = mark_it->second;

      cv::drawMarker(dst_img, ocv_pt, cur_color, m,
                     ((m != cv::MARKER_TILTED_CROSS) || (m != cv::MARKER_DIAMOND)) ? (radius * 2) : radius,
                     thickness);
      
    }
  }

  return dst_img;
}

namespace
{

using namespace xreg;

template <class T>
void AbsImgHelper(cv::Mat& img)
{
  const int num_rows = img.rows;
  const int num_cols = img.cols;
  
  auto abs_fn = [] (const T& x)
  {
    return std::abs(x);
  };

  for (int r = 0; r < num_rows; ++r)
  {
    T* row_buf = &img.at<T>(r,0);
    //std::transform(row_buf, row_buf + num_cols, row_buf, abs_fn);
    ParallelTransform(row_buf, row_buf + num_cols, row_buf, abs_fn);
  }
}

template <class T>
void AbsImgHelper(const cv::Mat& src, cv::Mat& dst)
{
  const int num_rows = src.rows;
  const int num_cols = src.cols;

  auto abs_fn = [] (const T& x)
  {
    return std::abs(x);
  };

  for (int r = 0; r < num_rows; ++r)
  {
    const T* src_row_buf = &src.at<T>(r,0);

    T* dst_row_buf = &dst.at<T>(r,0);

    ParallelTransform(src_row_buf, src_row_buf + num_cols, dst_row_buf, abs_fn);
  }
}

template <class T>
void ScaleTo8bppWithMaxHelper(const cv::Mat& src, cv::Mat& dst)
{
  const size_type nr = src.rows;
  const size_type nc = src.cols;
  const size_type num_pix = nr * nc;

  Eigen::Map<Eigen::Array<T,Eigen::Dynamic,1> > src_vec(const_cast<T*>(&src.at<T>(0,0)), num_pix);
  Eigen::Map<Eigen::Array<unsigned char,Eigen::Dynamic,1> > dst_vec(&dst.at<unsigned char>(0,0), num_pix);

  dst_vec = (src_vec * (T(255) / src_vec.maxCoeff())).template cast<unsigned char>();
}

template <class T>
void ScaleTo8bppWithMinMaxHelper(const cv::Mat& src, cv::Mat& dst)
{
  const size_type nr = src.rows;
  const size_type nc = src.cols;
  const size_type num_pix = nr * nc;

  Eigen::Map<Eigen::Array<T,Eigen::Dynamic,1> > src_vec(const_cast<T*>(&src.at<T>(0,0)), num_pix);
  Eigen::Map<Eigen::Array<unsigned char,Eigen::Dynamic,1> > dst_vec(&dst.at<unsigned char>(0,0), num_pix);

  const T max_val = src_vec.maxCoeff();
  const T min_val = src_vec.minCoeff();

  dst_vec = ((src_vec - min_val) * (T(255) / (max_val - min_val))).template cast<unsigned char>();
}

template <class T>
cv::Mat CropAndMaskEllipseHelper(const cv::Mat& src, const size_type x_axis_len_px, const size_type y_axis_len_px)
{
  const size_type half_src_nr = src.rows / 2;
  const size_type half_src_nc = src.cols / 2;

  cv::Rect roi;
  roi.x = half_src_nc - (x_axis_len_px / 2);
  roi.width = x_axis_len_px;
  roi.y = half_src_nr - (y_axis_len_px / 2);
  roi.height = y_axis_len_px;

  cv::Mat dst = src(roi).clone();

  const size_type half_dst_nr = y_axis_len_px / 2;
  const size_type half_dst_nc = x_axis_len_px / 2;

  const size_type num_channels = dst.channels();

  double y = 0;
  double x = 0;

  for (size_type r = 0; r < y_axis_len_px; ++r)
  {
    T* dst_buf = &dst.at<T>(r,0);

    y = static_cast<double>(r - half_dst_nr) / y_axis_len_px;
    y *= y;

    for (long c = 0; c < x_axis_len_px; ++c)
    {
      x = static_cast<double>(c - half_dst_nc) / x_axis_len_px;
      x *= x;

      if ((y + x) > 1.0)
      {
        for (long ch = 0; ch < num_channels; ++ch)
        {
          dst_buf[ch] = 0;
        }
      }

      dst_buf += num_channels;
    }
  }
  
  return dst;
}

template <class tMaskPixelType>
cv::Mat CreateCenteredEllipseMaskHelper(const cv::Mat& src, const double x_axis_len_px, const double y_axis_len_px)
{
  using MaskPixelType = tMaskPixelType;

  const size_type nr = src.rows;
  const size_type nc = src.cols;

  const double half_nr = nr / 2.0;
  const double half_nc = nc / 2.0;

  cv::Mat mask(nr, nc, cv::DataType<MaskPixelType>::type);
  mask.setTo(static_cast<MaskPixelType>(0));

  double y = 0;
  double x = 0;

  for (size_type r = 0; r < nr; ++r)
  {
    MaskPixelType* dst_buf = &mask.at<MaskPixelType>(r,0);

    y = (r - half_nr) / y_axis_len_px;
    y *= y;

    for (size_type c = 0; c < nc; ++c)
    {
      x = (c - half_nc) / x_axis_len_px;
      x *= x;

      if ((y + x) <= 1.0)
      {
        dst_buf[c] = 1;
      }
    }
  }

  return mask;
}

template <class tPixelType, class tMaskPixelType>
void ApplyMaskToSelfHelper(cv::Mat* img, const cv::Mat& mask, const tPixelType masked_val)
{
  using PixelType     = tPixelType;
  using MaskPixelType = tMaskPixelType;

  const size_type nr = img->rows;
  const size_type nc = img->cols;

  xregASSERT(nr == static_cast<size_type>(mask.rows));
  xregASSERT(nc == static_cast<size_type>(mask.cols));
  xregASSERT(img->channels() == 1);

  for (size_type r = 0; r < nr; ++r)
  {
    PixelType* cur_img_row = &img->at<PixelType>(r,0);

    const MaskPixelType* cur_mask_row = &mask.at<MaskPixelType>(r,0);

    for (size_type c = 0; c < nc; ++c)
    {
      if (!cur_mask_row[c])
      {
        cur_img_row[c] = masked_val;
      }
    }
  }
}

template <class tPixelType>
void FlipImageColumnsHelper(cv::Mat* img)
{
  using PixelType = tPixelType;

  const size_type nr = img->rows;
  const size_type nc = img->cols;
  
  const size_type num_chan = img->channels();

  const size_type half_nc = nc / 2;

  PixelType tmp_px = 0;

  for (size_type r = 0; r < nr; ++r)
  {
    PixelType* cur_row = &img->at<PixelType>(r,0);

    size_type other_c = nc - 1;
    for (size_type c = 0; c < half_nc; ++c, --other_c)
    {
      const size_type off1 = c * num_chan;
      const size_type off2 = other_c * num_chan;

      for (size_type ch = 0; ch < num_chan; ++ch)
      {
        tmp_px = cur_row[off2 + ch];
        cur_row[off2 + ch] = cur_row[off1 + ch];
        cur_row[off1 + ch] = tmp_px;
      }
    }
  }
}

template <class tPixelType>
void FlipImageRowsHelper(cv::Mat* img)
{
  using PixelType = tPixelType;

  const size_type nr = img->rows;
  const size_type nc = img->cols;

  const size_type num_chan = img->channels();

  const size_type num_elems_wide = nc * num_chan;

  const size_type half_nr = nr / 2;

  PixelType tmp_px = 0;

  for (size_type r = 0; r < half_nr; ++r)
  {
    PixelType* top_row = &img->at<PixelType>(r,0);
    PixelType* bot_row = &img->at<PixelType>(nr - 1 - r, 0);

    for (size_type c = 0; c < num_elems_wide; ++c)
    {
      tmp_px = top_row[c];
      top_row[c] = bot_row[c];
      bot_row[c] = tmp_px;
    }
  }
}

template <class tPixelType>
void FindPixelsWithAdjacentZeroIntensityHelper(const cv::Mat& img, cv::Mat* edges)
{
  // Not an efficient implementation, but I am currently only using this for debug
  // purposes.
  //
  // I am also assuming that PixelType is a floating point type.

  using PixelType = tPixelType;
  static_assert(std::is_same<PixelType,float>::value || std::is_same<PixelType,double>::value,
                "Only floating point types are supported.");

  constexpr PixelType kTOL = 1.0e-6;

  xregASSERT(edges->type() == CV_8U);

  const int nr = img.rows;
  const int nc = img.cols;

  xregASSERT((nr == edges->rows) && (nc == edges->cols));

  edges->setTo(0);

  for (int r = 0; r < nr; ++r)
  {
    const PixelType* prev_src_row = (r > 0) ? &img.at<PixelType>(r-1, 0) : 0;
    const PixelType* cur_src_row  = &img.at<PixelType>(r, 0);
    const PixelType* next_src_row = (r < (nr-1)) ? &img.at<PixelType>(r+1, 0) : 0;

    unsigned char* edge_row = &edges->at<unsigned char>(r, 0);

    for (int c = 0; c < nc; ++c)
    {
      bool is_edge = false;

      if (std::abs(cur_src_row[c]) > kTOL)
      {
        if (r > 0)
        {
          if (c > 0)
          {
            is_edge = std::abs(prev_src_row[c-1]) < kTOL;
          }

          if (!is_edge && (std::abs(prev_src_row[c]) < kTOL))
          {
            is_edge = true;
          }
          
          if ((c < (nc-1)) && !is_edge && (std::abs(prev_src_row[c+1]) < kTOL))
          {
            is_edge = true;
          } 
        }

        if (!is_edge)
        {
          if (c > 0)
          {
            is_edge = std::abs(cur_src_row[c-1]) < kTOL;
          }

          if (!is_edge && (std::abs(cur_src_row[c]) < kTOL))
          {
            is_edge = true;
          }
          
          if ((c < (nc-1)) && !is_edge && (std::abs(cur_src_row[c+1]) < kTOL))
          {
            is_edge = true;
          } 
         
          if (!is_edge && (r < (nr-1)))
          {
            if (c > 0)
            {
              is_edge = std::abs(next_src_row[c-1]) < kTOL;
            }

            if (!is_edge && (std::abs(next_src_row[c]) < kTOL))
            {
              is_edge = true;
            }

            if ((c < (nc-1)) && !is_edge && (std::abs(next_src_row[c+1]) < kTOL))
            {
              is_edge = true;
            } 
          }
        }

        if (is_edge)
        {
          edge_row[c] = 1;
        }
      }
    }
  } 
}

template <class tPixelType>
void FindPixelsWithAdjacentMaxIntensityHelper(const cv::Mat& img, cv::Mat* edges, const bool edges_pre_alloc)
{
  using PixelType = tPixelType;

  const int nr = img.rows;
  const int nc = img.cols;

  if (!edges_pre_alloc)
  {
    *edges = cv::Mat_<unsigned char>(nr,nc);
  }
  
  xregASSERT(edges->type() == CV_8U);

  const PixelType max_val = std::numeric_limits<PixelType>::max();

  for (int r = 0; r < nr; ++r)
  {
    const PixelType* prev_src_row = (r > 0) ? &img.at<PixelType>(r-1, 0) : 0;
    const PixelType* cur_src_row  = &img.at<PixelType>(r, 0);
    const PixelType* next_src_row = (r < (nr-1)) ? &img.at<PixelType>(r+1, 0) : 0;

    unsigned char* edge_row = &edges->at<unsigned char>(r, 0);

    for (int c = 0; c < nc; ++c)
    {
      bool is_edge = false;

      if (cur_src_row[c] != max_val)
      {
        if (r > 0)
        {
          if (c > 0)
          {
            is_edge = prev_src_row[c-1] == max_val;
          }

          if (!is_edge && (prev_src_row[c] == max_val))
          {
            is_edge = true;
          }
          
          if ((c < (nc-1)) && !is_edge && (prev_src_row[c+1]  == max_val))
          {
            is_edge = true;
          } 
        }

        if (!is_edge)
        {
          if (c > 0)
          {
            is_edge = cur_src_row[c-1] == max_val;
          }

          if (!is_edge && (cur_src_row[c] == max_val))
          {
            is_edge = true;
          }
          
          if ((c < (nc-1)) && !is_edge && (cur_src_row[c+1] == max_val))
          {
            is_edge = true;
          } 
         
          if (!is_edge && (r < (nr-1)))
          {
            if (c > 0)
            {
              is_edge = next_src_row[c-1] == max_val;
            }

            if (!is_edge && (next_src_row[c] == max_val))
            {
              is_edge = true;
            }

            if ((c < (nc-1)) && !is_edge && (next_src_row[c+1] == max_val))
            {
              is_edge = true;
            } 
          }
        }
      }
      
      edge_row[c] = is_edge ? 1 : 0;
    }
  } 
}

template <class tPixelType>
cv::Mat FindPixelsWithLessThanMaxIntensityHelper(const cv::Mat& img)
{
  using Scalar = tPixelType;

  const int nr = img.rows;
  const int nc = img.cols;

  const Scalar max_val = std::numeric_limits<Scalar>::max();

  cv::Mat labels = cv::Mat_<unsigned char>::zeros(nr, nc);

  for (int r = 0; r < nr; ++r)
  {
    const Scalar* src_row = &img.at<Scalar>(r,0);

    unsigned char* labels_row = &labels.at<unsigned char>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      if (src_row[c] < max_val)
      {
        labels_row[c] = 1;
      }
    }
  }

  return labels;
}

template <class tPixelType>
cv::Mat FindPixelsWithMaxIntensityHelper(const cv::Mat& img)
{
  using Scalar = tPixelType;

  const int nr = img.rows;
  const int nc = img.cols;

  const Scalar max_val = std::numeric_limits<Scalar>::max();

  cv::Mat labels = cv::Mat_<unsigned char>::zeros(nr, nc);

  for (int r = 0; r < nr; ++r)
  {
    const Scalar* src_row = &img.at<Scalar>(r,0);

    unsigned char* labels_row = &labels.at<unsigned char>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      if (src_row[c] == max_val)
      {
        labels_row[c] = 1;
      }
    }
  }

  return labels;
}

template <class tPixelType>
cv::Mat RemapGradImgToBGRHelper(const cv::Mat& img)
{
  using PixelType = tPixelType;

  PixelType max_abs_val = 0;

  const int nr = img.rows;
  const int nc = img.cols;

  for (int r = 0; r < nr; ++r)
  {
    const PixelType* cur_row = &img.at<PixelType>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      max_abs_val = std::max(max_abs_val, std::abs(cur_row[c]));
    }
  }

  cv::Mat dst(nr, nc, CV_8UC3);

  for (int r = 0; r < nr; ++r)
  {
    const PixelType* cur_row = &img.at<PixelType>(r,0);

    unsigned char* dst_row = &dst.at<unsigned char>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      const int off = c * 3;
      unsigned char& b = dst_row[off];
      unsigned char& g = dst_row[off + 1];
      unsigned char& r = dst_row[off + 2];
      
      const unsigned char scale_val = static_cast<unsigned char>(
                                        std::abs(cur_row[c] / max_abs_val) * 255);
      
      if (cur_row[c] > 0)
      {
        b = 0;
        g = 0;
        r = scale_val;
      }
      else
      {
        b = scale_val;
        g = scale_val;
        r = 0;
      }
    } 
  }

  return dst;
}

template <class tPixelType>
tPixelType FindMedianHelper(const cv::Mat& img, void* tmp_sort_arr)
{
  using PixelType = tPixelType;

  const int img_len = img.rows * img.cols;

  PixelType* sort_img_vals_buf = nullptr;
  std::vector<PixelType> tmp_buf;
  if (tmp_sort_arr)
  {
    sort_img_vals_buf = static_cast<PixelType*>(tmp_sort_arr);
  }
  else
  {
    tmp_buf.resize(img_len);
    sort_img_vals_buf = &tmp_buf[0];
  }

  if (img.isContinuous())
  {
    const PixelType* src_buf = &img.template at<PixelType>(0,0);
    std::copy(src_buf, src_buf + img_len, sort_img_vals_buf);
  }
  else
  {
    int off = 0;
    for (int r = 0; r < img.rows; ++r, off += img.cols)
    {
      const PixelType* src_row = &img.template at<PixelType>(r,0);
      std::copy(src_row, src_row + img.cols, sort_img_vals_buf + off);
    }
  }

  const int med_idx = img_len / 2;

  std::nth_element(sort_img_vals_buf, sort_img_vals_buf + med_idx,
                   sort_img_vals_buf + img_len);

  return sort_img_vals_buf[med_idx];
}

template <class tPixelType>
tPixelType FindNonZeroMedianHelper(const cv::Mat& img, void* tmp_sort_arr)
{
  using PixelType = tPixelType;

  const int img_len = img.rows * img.cols;

  PixelType* sort_img_vals_buf = nullptr;
  std::vector<PixelType> tmp_buf;
  if (tmp_sort_arr)
  {
    sort_img_vals_buf = static_cast<PixelType*>(tmp_sort_arr);
  }
  else
  {
    tmp_buf.resize(img_len);
    sort_img_vals_buf = &tmp_buf[0];
  }

  int img_non_zero_len = 0;

  if (img.isContinuous())
  {
    const PixelType* src_buf = &img.template at<PixelType>(0,0);

    std::for_each(src_buf, src_buf + img_len,
                  [&img_non_zero_len,&sort_img_vals_buf] (const PixelType& p)
                  {
                    if (p != PixelType(0))
                    {
                      sort_img_vals_buf[img_non_zero_len] = p;
                      ++img_non_zero_len;
                    }
                  });
  }
  else
  {
    int off = 0;
    for (int r = 0; r < img.rows; ++r, off += img.cols)
    {
      const PixelType* src_row = &img.template at<PixelType>(r,0);
      
      std::for_each(src_row, src_row + img.cols,
                    [&img_non_zero_len,&sort_img_vals_buf] (const PixelType& p)
                    {
                      if (p != PixelType(0))
                      {
                        sort_img_vals_buf[img_non_zero_len] = p;
                        ++img_non_zero_len;
                      }
                    });
    }
  }

  const int med_idx = img_non_zero_len / 2;

  std::nth_element(sort_img_vals_buf, sort_img_vals_buf + med_idx,
                   sort_img_vals_buf + img_non_zero_len);

  return sort_img_vals_buf[med_idx];
}

template <class tScalar>
std::tuple<double,double> MeanStdDevHelper(const cv::Mat& img)
{
  using Scalar = tScalar;

  double mean = 0;

  for (int r = 0; r < img.rows; ++r)
  {
    const Scalar* row = &img.at<Scalar>(r,0);
    
    for (int c = 0; c < img.cols; ++c)
    {
      mean += row[c];
    }
  }

  const long num_pix = img.rows * img.cols;
  
  mean /= num_pix;

  double var = 0;

  for (int r = 0; r < img.rows; ++r)
  {
    const Scalar* row = &img.at<Scalar>(r,0);
    
    for (int c = 0; c < img.cols; ++c)
    {
      const double d = row[c] - mean;
      var += d * d;
    }
  }

  var /= (num_pix - 1);

  return std::make_tuple(mean, std::sqrt(var));
}

}  // un-named

void xreg::AbsImg(cv::Mat* img)
{
  switch (img->type())
  {
  case CV_8S:
    AbsImgHelper<char>(*img);
    break;
  case CV_16S:
    AbsImgHelper<short>(*img);
    break;
  case CV_32S:
    AbsImgHelper<int>(*img);
    break;
  case CV_32F:
    AbsImgHelper<float>(*img);
    break;
  case CV_64F:
    AbsImgHelper<double>(*img);
    break;
  case CV_8U:  // should not be calling for unsigned
  case CV_16U:
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::AbsImg(const cv::Mat& src, cv::Mat* dst)
{
  xregASSERT(src.type() == dst->type());

  switch (src.type())
  {
  case CV_8S:
    AbsImgHelper<char>(src, *dst);
    break;
  case CV_16S:
    AbsImgHelper<short>(src, *dst);
    break;
  case CV_32S:
    AbsImgHelper<int>(src, *dst);
    break;
  case CV_32F:
    AbsImgHelper<float>(src, *dst);
    break;
  case CV_64F:
    AbsImgHelper<double>(src, *dst);
    break;
  case CV_8U:  // should not be calling for unsigned
  case CV_16U:
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::ComputeGradMag(cv::Mat& src_img, cv::Mat& grad_img, cv::Mat& tmp_img)
{
  cv::Sobel(src_img, grad_img, -1, 1, 0);

  cv::Sobel(src_img, tmp_img, -1, 0, 1);

  cv::magnitude(grad_img, tmp_img, grad_img);
}

void xreg::SmoothAndGradMag(cv::Mat& src_img, cv::Mat& smooth_img, cv::Mat& grad_img,
                            cv::Mat& tmp_img, const int smooth_kernel_width_pix)
{
  xregASSERT(smooth_kernel_width_pix > 0);
  xregASSERT((smooth_kernel_width_pix % 2) == 1);

  cv::GaussianBlur(src_img, smooth_img,
                   cv::Size(smooth_kernel_width_pix, smooth_kernel_width_pix),
                   0, 0);

  ComputeGradMag(smooth_img, grad_img, tmp_img);
}

void xreg::ScaleTo8bppWithMax(const cv::Mat& src, cv::Mat* dst)
{
  xregASSERT(src.type() == dst->type());

  switch (src.type())
  {
  case CV_8U:
    ScaleTo8bppWithMaxHelper<unsigned char>(src, *dst);
    break;
  case CV_8S:
    ScaleTo8bppWithMaxHelper<char>(src, *dst);
    break;
  case CV_16U:
    ScaleTo8bppWithMaxHelper<unsigned short>(src, *dst);
    break;
  case CV_16S:
    ScaleTo8bppWithMaxHelper<short>(src, *dst);
    break;
  case CV_32S:
    ScaleTo8bppWithMaxHelper<int>(src, *dst);
    break;
  case CV_32F:
    ScaleTo8bppWithMaxHelper<float>(src, *dst);
    break;
  case CV_64F:
    ScaleTo8bppWithMaxHelper<double>(src, *dst);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::ScaleTo8bppWithMinMax(const cv::Mat& src, cv::Mat* dst)
{
  xregASSERT(src.type() == dst->type());

  switch (src.type())
  {
  case CV_8U:
    ScaleTo8bppWithMinMaxHelper<unsigned char>(src, *dst);
    break;
  case CV_8S:
    ScaleTo8bppWithMinMaxHelper<char>(src, *dst);
    break;
  case CV_16U:
    ScaleTo8bppWithMinMaxHelper<unsigned short>(src, *dst);
    break;
  case CV_16S:
    ScaleTo8bppWithMinMaxHelper<short>(src, *dst);
    break;
  case CV_32S:
    ScaleTo8bppWithMinMaxHelper<int>(src, *dst);
    break;
  case CV_32F:
    ScaleTo8bppWithMinMaxHelper<float>(src, *dst);
    break;
  case CV_64F:
    ScaleTo8bppWithMinMaxHelper<double>(src, *dst);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

cv::Mat xreg::CropAndMaskEllipse(const cv::Mat& src, const size_type x_axis_len_px, const size_type y_axis_len_px)
{
  cv::Mat dst;

  switch (src.type())
  {
  case CV_8U:
    dst = CropAndMaskEllipseHelper<unsigned char>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_8S:
    dst = CropAndMaskEllipseHelper<char>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_16U:
    dst = CropAndMaskEllipseHelper<unsigned short>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_16S:
    dst = CropAndMaskEllipseHelper<short>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_32S:
    dst = CropAndMaskEllipseHelper<int>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_32F:
    dst = CropAndMaskEllipseHelper<float>(src, x_axis_len_px, y_axis_len_px);
    break;
  case CV_64F:
    dst = CropAndMaskEllipseHelper<double>(src, x_axis_len_px, y_axis_len_px);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }

  return dst;
}

cv::Mat xreg::CropAndMaskCircle(const cv::Mat& src, const long radius_px)
{
  return CropAndMaskEllipse(src, radius_px, radius_px);
}

cv::Mat xreg::CreateCenteredEllipseMask(const cv::Mat& src, const double x_axis_len_px, const double y_axis_len_px)
{
  return CreateCenteredEllipseMaskHelper<unsigned char>(src, x_axis_len_px, y_axis_len_px);
}

cv::Mat xreg::CreateCenteredCircleMask(const cv::Mat& src, const double radius_px)
{
  return CreateCenteredEllipseMask(src, radius_px, radius_px);
}

void xreg::ApplyMaskToSelf(cv::Mat* img, const cv::Mat& mask, const double masked_val)
{
  xregASSERT(mask.type() == CV_8U);

  switch (img->type())
  {
  case CV_8U:
    ApplyMaskToSelfHelper<unsigned char,unsigned char>(img, mask, static_cast<unsigned char>(masked_val));
    break;
  case CV_8S:
    ApplyMaskToSelfHelper<char,unsigned char>(img, mask, static_cast<char>(masked_val));
    break;
  case CV_16U:
    ApplyMaskToSelfHelper<unsigned short,unsigned char>(img, mask, static_cast<unsigned short>(masked_val));
    break;
  case CV_16S:
    ApplyMaskToSelfHelper<short,unsigned char>(img, mask, static_cast<short>(masked_val));
    break;
  case CV_32S:
    ApplyMaskToSelfHelper<int,unsigned char>(img, mask, static_cast<int>(masked_val));
    break;
  case CV_32F:
    ApplyMaskToSelfHelper<float,unsigned char>(img, mask, static_cast<float>(masked_val));
    break;
  case CV_64F:
    ApplyMaskToSelfHelper<double,unsigned char>(img, mask, masked_val);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

std::tuple<cv::Mat,cv::Mat,cv::Rect>
xreg::TightCropAboutMask(const cv::Mat& src_img, const cv::Mat& src_mask)
{
  using MaskPixelType = unsigned char;

  xregASSERT(src_mask.type() == CV_8U);

  const size_type nr = src_img.rows;
  const size_type nc = src_img.cols;

  xregASSERT(nr == static_cast<size_type>(src_mask.rows));
  xregASSERT(nc == static_cast<size_type>(src_mask.cols));

  size_type first_non_zero_col = nc;
  size_type last_non_zero_col  = 0;
  size_type first_non_zero_row = nr;
  size_type last_non_zero_row  = 0;

  for (size_type r = 0; r < nr; ++r)
  {
    const MaskPixelType* mask_row = &src_mask.at<MaskPixelType>(r,0);

    for (size_type c = 0; c < nc; ++c)
    {
      if (mask_row[c])
      {
        if (c < first_non_zero_col)
        {
          first_non_zero_col = c;
        }

        if (c > last_non_zero_col)
        {
          last_non_zero_col = c;
        }

        if (r < first_non_zero_row)
        {
          first_non_zero_row = r;
        }

        if (r > last_non_zero_row)
        {
          last_non_zero_row = r;
        }
      }
    }
  }

  const size_type dst_nr = (last_non_zero_row >= first_non_zero_row) ? (last_non_zero_row - first_non_zero_row + 1) : 0;
  const size_type dst_nc = (last_non_zero_col >= first_non_zero_col) ? (last_non_zero_col - first_non_zero_col + 1) : 0;
  
  cv::Mat dst_img;
  cv::Mat dst_mask;
  cv::Rect mask_roi_in_src;

  if (dst_nr && dst_nc)
  {
    cv::Rect roi;
    roi.x = first_non_zero_col;
    roi.width = dst_nc;
    roi.y = first_non_zero_row;
    roi.height = dst_nr;

    dst_img  = src_img(roi).clone();
    dst_mask = src_mask(roi).clone();
    mask_roi_in_src = roi;
  }
  else
  {
    // Mask did not have any inclusive values - return empty images
  }

  return std::make_tuple(dst_img, dst_mask, mask_roi_in_src);
}

void xreg::FlipImageColumns(cv::Mat* img)
{
  switch (img->type())
  {
  case CV_8UC1:
  case CV_8UC2:
  case CV_8UC3:
  case CV_8UC4:
    FlipImageColumnsHelper<unsigned char>(img);
    break;
  case CV_8SC1:
  case CV_8SC2:
  case CV_8SC3:
  case CV_8SC4:
    FlipImageColumnsHelper<char>(img);
    break;
  case CV_16UC1:
  case CV_16UC2:
  case CV_16UC3:
  case CV_16UC4:
    FlipImageColumnsHelper<unsigned short>(img);
    break;
  case CV_16SC1:
  case CV_16SC2:
  case CV_16SC3:
  case CV_16SC4:
    FlipImageColumnsHelper<short>(img);
    break;
  case CV_32SC1:
  case CV_32SC2:
  case CV_32SC3:
  case CV_32SC4:
    FlipImageColumnsHelper<int>(img);
    break;
  case CV_32FC1:
  case CV_32FC2:
  case CV_32FC3:
  case CV_32FC4:
    FlipImageColumnsHelper<float>(img);
    break;
  case CV_64FC1:
  case CV_64FC2:
  case CV_64FC3:
  case CV_64FC4:
    FlipImageColumnsHelper<double>(img);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::FlipImageRows(cv::Mat* img)
{
  switch (img->type())
  {
  case CV_8UC1:
  case CV_8UC2:
  case CV_8UC3:
  case CV_8UC4:
    FlipImageRowsHelper<unsigned char>(img);
    break;
  case CV_8SC1:
  case CV_8SC2:
  case CV_8SC3:
  case CV_8SC4:
    FlipImageRowsHelper<char>(img);
    break;
  case CV_16UC1:
  case CV_16UC2:
  case CV_16UC3:
  case CV_16UC4:
    FlipImageRowsHelper<unsigned short>(img);
    break;
  case CV_16SC1:
  case CV_16SC2:
  case CV_16SC3:
  case CV_16SC4:
    FlipImageRowsHelper<short>(img);
    break;
  case CV_32SC1:
  case CV_32SC2:
  case CV_32SC3:
  case CV_32SC4:
    FlipImageRowsHelper<int>(img);
    break;
  case CV_32FC1:
  case CV_32FC2:
  case CV_32FC3:
  case CV_32FC4:
    FlipImageRowsHelper<float>(img);
    break;
  case CV_64FC1:
  case CV_64FC2:
  case CV_64FC3:
  case CV_64FC4:
    FlipImageRowsHelper<double>(img);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::FindPixelsWithAdjacentZeroIntensity(const cv::Mat& img, cv::Mat* edges)
{
  switch (img.type())
  {
  case CV_32F:
    FindPixelsWithAdjacentZeroIntensityHelper<float>(img, edges);
    break;
  case CV_64F:
    FindPixelsWithAdjacentZeroIntensityHelper<double>(img, edges);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

void xreg::FindPixelsWithAdjacentMaxIntensity(const cv::Mat& img, cv::Mat* edges, const bool edges_pre_alloc)
{
  switch (img.type())
  {
  case CV_8U:
    FindPixelsWithAdjacentMaxIntensityHelper<unsigned char>(img, edges, edges_pre_alloc);
    break;
  case CV_8S:
    FindPixelsWithAdjacentMaxIntensityHelper<char>(img, edges, edges_pre_alloc);
    break;
  case CV_16U:
    FindPixelsWithAdjacentMaxIntensityHelper<unsigned short>(img, edges, edges_pre_alloc);
    break;
  case CV_16S:
    FindPixelsWithAdjacentMaxIntensityHelper<short>(img, edges, edges_pre_alloc);
    break;
  case CV_32S:
    FindPixelsWithAdjacentMaxIntensityHelper<int>(img, edges, edges_pre_alloc);
    break;
  case CV_32F:
    FindPixelsWithAdjacentMaxIntensityHelper<float>(img, edges, edges_pre_alloc);
    break;
  case CV_64F:
    FindPixelsWithAdjacentMaxIntensityHelper<double>(img, edges, edges_pre_alloc);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

cv::Mat xreg::FindPixelsWithLessThanMaxIntensity(const cv::Mat& img)
{
  cv::Mat dst;

  switch (img.type())
  {
  case CV_8U:
    dst = FindPixelsWithLessThanMaxIntensityHelper<unsigned char>(img);
    break;
  case CV_8S:
    dst = FindPixelsWithLessThanMaxIntensityHelper<char>(img);
    break;
  case CV_16U:
    dst = FindPixelsWithLessThanMaxIntensityHelper<unsigned short>(img);
    break;
  case CV_16S:
    dst = FindPixelsWithLessThanMaxIntensityHelper<short>(img);
    break;
  case CV_32S:
    dst = FindPixelsWithLessThanMaxIntensityHelper<int>(img);
    break;
  case CV_32F:
    dst = FindPixelsWithLessThanMaxIntensityHelper<float>(img);
    break;
  case CV_64F:
    dst = FindPixelsWithLessThanMaxIntensityHelper<double>(img);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }

  return dst;
}

cv::Mat xreg::FindPixelsWithMaxIntensity(const cv::Mat& img)
{
  cv::Mat dst;

  switch (img.type())
  {
  case CV_8U:
    dst = FindPixelsWithMaxIntensityHelper<unsigned char>(img);
    break;
  case CV_8S:
    dst = FindPixelsWithMaxIntensityHelper<char>(img);
    break;
  case CV_16U:
    dst = FindPixelsWithMaxIntensityHelper<unsigned short>(img);
    break;
  case CV_16S:
    dst = FindPixelsWithMaxIntensityHelper<short>(img);
    break;
  case CV_32S:
    dst = FindPixelsWithMaxIntensityHelper<int>(img);
    break;
  case CV_32F:
    dst = FindPixelsWithMaxIntensityHelper<float>(img);
    break;
  case CV_64F:
    dst = FindPixelsWithMaxIntensityHelper<double>(img);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }

  return dst;
}

namespace  // un-named
{

template <class T>
struct FindPixelsWithAdjacentIntensityComp
{
  bool operator()(const T& x, const T& y) const
  {
    return x == y;
  }
};

template <>
struct FindPixelsWithAdjacentIntensityComp<float>
{
  bool operator()(const float& x, const float& y) const
  {
    return std::abs(x - y) < 1.0e-8f;
  }
};

template <>
struct FindPixelsWithAdjacentIntensityComp<double>
{
  bool operator()(const double& x, const double& y) const
  {
    return std::abs(x - y) < 1.0e-12f;
  }
};

template <class tPixelScalar>
void FindPixelsWithAdjacentIntensityHelper(const cv::Mat& img, cv::Mat* edges,
                                           const tPixelScalar val, const bool edges_pre_alloc)
{
  using PixelScalar = tPixelScalar;

  FindPixelsWithAdjacentIntensityComp<PixelScalar> comp;
  
  const int nr = img.rows;
  const int nc = img.cols;

  if (!edges_pre_alloc)
  {
    *edges = cv::Mat_<unsigned char>(nr,nc);
  }
  
  xregASSERT(edges->type() == CV_8U);

  for (int r = 0; r < nr; ++r)
  {
    const PixelScalar* prev_src_row = (r > 0) ? &img.at<PixelScalar>(r-1, 0) : nullptr;
    const PixelScalar* cur_src_row  = &img.at<PixelScalar>(r, 0);
    const PixelScalar* next_src_row = (r < (nr-1)) ? &img.at<PixelScalar>(r+1, 0) : nullptr;

    unsigned char* edge_row = &edges->at<unsigned char>(r, 0);

    for (int c = 0; c < nc; ++c)
    {
      bool is_edge = false;

      if (!comp(cur_src_row[c], val))
      {
        if (r > 0)
        {
          if (c > 0)
          {
            is_edge = comp(prev_src_row[c-1], val);
          }

          if (!is_edge && comp(prev_src_row[c], val))
          {
            is_edge = true;
          }
          
          if ((c < (nc-1)) && !is_edge && comp(prev_src_row[c+1], val))
          {
            is_edge = true;
          } 
        }

        if (!is_edge)
        {
          if (c > 0)
          {
            is_edge = comp(cur_src_row[c-1], val);
          }

          // we already tested the current pixel to not be equal to val
          //if (!is_edge && comp(cur_src_row[c], val))
          //{
          //  is_edge = true;
          //}
          
          if ((c < (nc-1)) && !is_edge && comp(cur_src_row[c+1], val))
          {
            is_edge = true;
          } 
         
          if (!is_edge && (r < (nr-1)))
          {
            if (c > 0)
            {
              is_edge = comp(next_src_row[c-1], val);
            }

            if (!is_edge && comp(next_src_row[c], val))
            {
              is_edge = true;
            }

            if ((c < (nc-1)) && !is_edge && comp(next_src_row[c+1], val))
            {
              is_edge = true;
            } 
          }
        }
      }
      
      edge_row[c] = is_edge ? 1 : 0;
    }
  } 
}

}  // un-named

void xreg::FindPixelsWithAdjacentIntensity(const cv::Mat& img, cv::Mat* edges,
                                           const double val, const bool edges_pre_alloc)
{
  switch (img.type())
  {
  case CV_8U:
    FindPixelsWithAdjacentIntensityHelper<unsigned char>(img, edges, static_cast<unsigned char>(val), edges_pre_alloc);
    break;
  case CV_8S:
    FindPixelsWithAdjacentIntensityHelper<char>(img, edges, static_cast<char>(val), edges_pre_alloc);
    break;
  case CV_16U:
    FindPixelsWithAdjacentIntensityHelper<unsigned short>(img, edges, static_cast<unsigned short>(val), edges_pre_alloc);
    break;
  case CV_16S:
    FindPixelsWithAdjacentIntensityHelper<short>(img, edges, static_cast<short>(val), edges_pre_alloc);
    break;
  case CV_32S:
    FindPixelsWithAdjacentIntensityHelper<int>(img, edges, static_cast<int>(val), edges_pre_alloc);
    break;
  case CV_32F:
    FindPixelsWithAdjacentIntensityHelper<float>(img, edges, static_cast<float>(val), edges_pre_alloc);
    break;
  case CV_64F:
    FindPixelsWithAdjacentIntensityHelper<double>(img, edges, val, edges_pre_alloc);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

cv::Mat xreg::RemapGradImgToBGR(const cv::Mat& img)
{
  cv::Mat dst;

  switch (img.type())
  {
  case CV_32F:
    dst = RemapGradImgToBGRHelper<float>(img);
    break;
  case CV_64F:
    dst = RemapGradImgToBGRHelper<double>(img);
    break;
  case CV_8U:
  case CV_8S:
  case CV_16U:
  case CV_16S:
  case CV_32S:
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }

  return dst;
}

double xreg::FindMedian(const cv::Mat& img, void* tmp_sort_arr)
{
  double med = 0;

  switch (img.type())
  {
  case CV_8U:
    med = FindMedianHelper<unsigned char>(img, tmp_sort_arr);
    break;
  case CV_8S:
    med = FindMedianHelper<char>(img, tmp_sort_arr);
    break;
  case CV_16U:
    med = FindMedianHelper<unsigned short>(img, tmp_sort_arr);
    break;
  case CV_16S:
    med = FindMedianHelper<short>(img, tmp_sort_arr);
    break;
  case CV_32S:
    med = FindMedianHelper<int>(img, tmp_sort_arr);
    break;
  case CV_32F:
    med = FindMedianHelper<float>(img, tmp_sort_arr);
    break;
  case CV_64F:
    med = FindMedianHelper<double>(img, tmp_sort_arr);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
  
  return med;
}

double xreg::FindNonZeroMedian(const cv::Mat& img, void* tmp_sort_arr)
{
  double med = 0;

  switch (img.type())
  {
  case CV_8U:
    med = FindNonZeroMedianHelper<unsigned char>(img, tmp_sort_arr);
    break;
  case CV_8S:
    med = FindNonZeroMedianHelper<char>(img, tmp_sort_arr);
    break;
  case CV_16U:
    med = FindNonZeroMedianHelper<unsigned short>(img, tmp_sort_arr);
    break;
  case CV_16S:
    med = FindNonZeroMedianHelper<short>(img, tmp_sort_arr);
    break;
  case CV_32S:
    med = FindNonZeroMedianHelper<int>(img, tmp_sort_arr);
    break;
  case CV_32F:
    med = FindNonZeroMedianHelper<float>(img, tmp_sort_arr);
    break;
  case CV_64F:
    med = FindNonZeroMedianHelper<double>(img, tmp_sort_arr);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
  
  return med;
}

std::tuple<double,double> xreg::MeanStdDev(const cv::Mat& img)
{
  std::tuple<double,double> mean_std_dev;
  
  switch (img.type())
  {
  case CV_8U:
    mean_std_dev = MeanStdDevHelper<unsigned char>(img);
    break;
  case CV_8S:
    mean_std_dev = MeanStdDevHelper<char>(img);
    break;
  case CV_16U:
    mean_std_dev = MeanStdDevHelper<unsigned short>(img);
    break;
  case CV_16S:
    mean_std_dev = MeanStdDevHelper<short>(img);
    break;
  case CV_32S:
    mean_std_dev = MeanStdDevHelper<int>(img);
    break;
  case CV_32F:
    mean_std_dev = MeanStdDevHelper<float>(img);
    break;
  case CV_64F:
    mean_std_dev = MeanStdDevHelper<double>(img);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }

  return mean_std_dev;
}

namespace  // un-named
{

template <class tPixelScalar>
void WriteImgsAsMultiChannelH5Helper(const std::vector<cv::Mat>& imgs,
                                     const std::string& h5_dataset_name,
                                     H5::Group* h5,
                                     const bool compress)
{
  using PixelScalar = tPixelScalar;

  const size_type num_imgs = imgs.size();

  const hsize_t nr = imgs[0].rows;
  const hsize_t nc = imgs[0].cols;

  for (size_type i = 1; i < num_imgs; ++i)
  {
    xregASSERT((nr == imgs[i].rows) && (nc == imgs[i].cols));
    xregASSERT(imgs[i].isContinuous());
  }

  std::array<hsize_t,3> multi_chan_dims = { num_imgs, nr, nc };

  H5::DataSpace data_space(multi_chan_dims.size(), multi_chan_dims.data());
    
  // 1 image channel in memory (lets us write imgs[i] contiguously)
  std::array<hsize_t,3> mem_dims = { 1, nr, nc };

  H5::DSetCreatPropList props;
  props.copy(H5::DSetCreatPropList::DEFAULT);

  if (compress)
  {
    // chunk of 1 image channel - reuse memory dims
    props.setChunk(mem_dims.size(), mem_dims.data());
    props.setDeflate(9);
  }

  H5::DataSet multi_chan_ds = h5->createDataSet(h5_dataset_name,
                                                LookupH5DataType<PixelScalar>(),
                                                data_space, props);
  
  H5::DataSpace ds_m(mem_dims.size(), mem_dims.data());

  for (size_type i = 0; i < num_imgs; ++i)
  {
    const std::array<hsize_t,3> f_start = { i, 0, 0 };

    H5::DataSpace ds_f = multi_chan_ds.getSpace();
    ds_f.selectHyperslab(H5S_SELECT_SET, mem_dims.data(), f_start.data());
    
    multi_chan_ds.write(imgs[i].data, LookupH5DataType<PixelScalar>(), ds_m, ds_f);
  }
}

}  // un-named

void xreg::WriteImgsAsMultiChannelH5(const std::vector<cv::Mat>& imgs,
                                     const std::string& h5_dataset_name,
                                     H5::Group* h5,
                                     const bool compress)
{
  const size_type num_imgs = imgs.size();
  xregASSERT(num_imgs);

  const auto img_type = imgs[0].type();

  for (size_type i = 1; i < num_imgs; ++i)
  {
    xregASSERT(img_type == imgs[i].type());
  }
  
  switch (img_type)
  {
  case CV_8U:
    WriteImgsAsMultiChannelH5Helper<unsigned char>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_8S:
    WriteImgsAsMultiChannelH5Helper<char>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_16U:
    WriteImgsAsMultiChannelH5Helper<unsigned short>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_16S:
    WriteImgsAsMultiChannelH5Helper<short>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_32S:
    WriteImgsAsMultiChannelH5Helper<int>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_32F:
    WriteImgsAsMultiChannelH5Helper<float>(imgs, h5_dataset_name, h5, compress);
    break;
  case CV_64F:
    WriteImgsAsMultiChannelH5Helper<double>(imgs, h5_dataset_name, h5, compress);
    break;
  default:
    xregThrow("unsupported OpenCV Mat Type!");
    break;
  }
}

