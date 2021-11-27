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

#ifndef XREGOPENCVUTILS_H_
#define XREGOPENCVUTILS_H_

#include <opencv2/core/core.hpp>

#include "xregCommon.h"
#include "xregAssert.h"

// Forward Declarations
namespace H5
{
class Group;
}

namespace xreg
{

/// \brief Writes a 2D OpenCV image object to disk after remapping the scalar values
///        to be in 8bpp.
///
/// The file format is determined by the file extension
void WriteImageRemap8bpp(cv::Mat& img, const std::string& path);


/// \brief Overlays edges onto an image.
///
/// Edge pixels are set to maximum intensity (255) in a single channel.
/// The channel is customizable via edge_channel, but defaults to red (index 2 in BGR).
cv::Mat OverlayEdges(const cv::Mat& base_img, const cv::Mat& edge_img,
                     const unsigned int edge_channel = 2);

cv::Mat OverlayRectBoundary(const cv::Mat& base_img, const cv::Rect& r,
                            const unsigned int edge_channel = 2);

/// \brief Create tiled summary images from a collection of images
///
/// This is useful when attempting to summarize a dataset for understanding or
/// presentation.
/// The src_imgs list must be populated with cv::Mat objects
/// The dst_imgs list will be populated with BGR cv::Mat objects
/// If the src_names list is empty, then no titles will be overlaid
std::vector<cv::Mat> CreateSummaryTiledImages(const std::vector<cv::Mat>& src_imgs,
                                              const std::vector<std::string>& src_names,
                                              const size_type num_tile_rows,
                                              const size_type num_tile_cols,
                                              const int border_thickness = 5);

/// \brief Create a basic two-color-multiview image from two gray-level images.
cv::Mat Create2CMV(const cv::Mat& img_red, const cv::Mat& img_cyan);

/// \brief Overlay optical flow vectors on an image.
cv::Mat AddFlowVecs(const cv::Mat& img_bgr, const cv::Mat& flow,
                    const int flow_subsample = 1, const float vec_scale = 1,
                    const int img_upsample = 1);

cv::Mat OverlayPtsAsCircles(const cv::Mat& img, const Pt2List& pts,
                            const AnyParamMap& other_params = AnyParamMap());

const std::unordered_map<std::string, cv::Scalar>& OpenCVColorNameToScalar();

cv::Mat OverlayPts(const cv::Mat& img, const Pt2List& pts,
                   const AnyParamMap& other_params = AnyParamMap());

/// \brief Computes the pixelwise absolute value of an image
///
/// This is in-place; it overwrites the input
void AbsImg(cv::Mat* img);

/// \brief Computes the pixelwise absolute value of an image
///
/// This is out-of-place; it preserves the input and writes the output in another buffer
void AbsImg(const cv::Mat& src, cv::Mat* dst);

/// \brief Computes the gradient magnitude of an image.
///
/// Sobel derivatives are computed in both directions and the sum of their
/// absolute values defines gradient magnitude in this routine.
/// A temporary buffer image is required.
void ComputeGradMag(cv::Mat& src_img, cv::Mat& grad_img, cv::Mat& tmp_img);

/// \brief Smooths an image and then computes the gradient magnitude of the
///        smoothed image.
void SmoothAndGradMag(cv::Mat& src_img, cv::Mat& smooth_img, cv::Mat& grad_img,
                      cv::Mat& tmp_img, const int smooth_kernel_width_pix = 3);

/// \brief Scales an image to 8bpp (unsigned char) using ONLY the maximum
///        value of the input.
///
/// Basically: Output(r,c) = Input(r,c) * 255 / max(Input)
void ScaleTo8bppWithMax(const cv::Mat& src, cv::Mat* dst);

/// \brief Scales an image to 8bpp (unsigned char) using the maximum and
///        minimum values of the input.
///
/// Basically: Output(r,c) = (Input(r,c) - min(Input)) * 255 / (max(Input) - min(Input))
void ScaleTo8bppWithMinMax(const cv::Mat& src, cv::Mat* dst);

/// \brief Mask pixels according to an ellipse, and then perform a tight cropping
///        about the ellipse.
///
/// Pixels inside the ellipse are not masked (kept/not modified), pixels outside
/// the ellipse will be set to 0 (TODO: allow customization).
cv::Mat CropAndMaskEllipse(const cv::Mat& src, const size_type x_axis_len_px, const size_type y_axis_len_px);

/// \brief Mask pixels according to a circle, and then perform a tight cropping
///        about the circle.
///
/// Pixels inside the circle are not masked (kept/not modified), pixels outside
/// the circle will be set to 0 (TODO: allow customization).
/// \see CropAndMaskEllipse
cv::Mat CropAndMaskCircle(const cv::Mat& src, const long radius_px);

/// \brief Creates an ellipse shaped mask centered on an input image
cv::Mat CreateCenteredEllipseMask(const cv::Mat& src, const double x_axis_len_px, const double y_axis_len_px);

/// \brief Creates a circle shaped mask centered on an input image.
cv::Mat CreateCenteredCircleMask(const cv::Mat& src, const double radius_px);

/// \brief Applies a mask in-place to an image.
void ApplyMaskToSelf(cv::Mat* img, const cv::Mat& mask, const double masked_val = 0);

/// \brief Crop an image and mask with a tight rectangular bound about the mask.
std::tuple<cv::Mat,cv::Mat,cv::Rect>
TightCropAboutMask(const cv::Mat& src_img, const cv::Mat& src_mask);

/// \brief Flips the columns in an image (e.g. about the center y-axis)
void FlipImageColumns(cv::Mat* img);

/// \brief Flips the rows in an image (e.g. about the center x-axis)
void FlipImageRows(cv::Mat* img);

///
/// edges has pixel type unsigned char, img has pixel type tPixelType
void FindPixelsWithAdjacentZeroIntensity(const cv::Mat& img, cv::Mat* edges);

void FindPixelsWithAdjacentMaxIntensity(const cv::Mat& img, cv::Mat* edges, const bool edges_pre_alloc = false);

cv::Mat FindPixelsWithLessThanMaxIntensity(const cv::Mat& img);

cv::Mat FindPixelsWithMaxIntensity(const cv::Mat& img);

void FindPixelsWithAdjacentIntensity(const cv::Mat& img, cv::Mat* edges,
                                     const double val, const bool edges_pre_alloc = false);

cv::Mat RemapGradImgToBGR(const cv::Mat& img);

double FindMedian(const cv::Mat& img, void* tmp_sort_arr = nullptr);

double FindNonZeroMedian(const cv::Mat& img, void* tmp_sort_arr = nullptr);

std::tuple<double,double> MeanStdDev(const cv::Mat& img);

template <class tPixelScalar>
Eigen::Matrix<tPixelScalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>
ConvertOpenCVMatToEigen(cv::Mat& img)
{
  using PixelScalar = tPixelScalar;
  using Mat         = Eigen::Matrix<PixelScalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;

  xregASSERT(img.isContinuous());
  xregASSERT(cv::DataType<PixelScalar>::type == img.type());

  return Eigen::Map<Mat>(&img.at<PixelScalar>(0,0), img.rows, img.cols);
}

/// \brief Creates a single, contiguous, buffer to store a collection of images.
template <class tPixelType>
std::vector<cv::Mat> AllocContiguousBufferForOpenCVImages(const size_type num_rows,
                                                          const size_type num_cols,
                                                          const size_type num_imgs,
                                                          std::vector<tPixelType>* pix_buf)
{
  using PixelType = tPixelType;

  pix_buf->assign(num_rows * num_cols * num_imgs, PixelType(0));

  std::vector<cv::Mat> ocv_imgs(num_imgs);

  for (unsigned long i = 0; i < num_imgs; ++i)
  {
    ocv_imgs[i] = cv::Mat(num_rows, num_cols, cv::DataType<PixelType>::type,
                          &pix_buf->operator[](num_rows * num_cols * i));
  }
  
  return ocv_imgs;
}

void WriteImgsAsMultiChannelH5(const std::vector<cv::Mat>& imgs,
                               const std::string& h5_dataset_name,
                               H5::Group* h5,
                               const bool compress = true);

}  // xreg

#endif
