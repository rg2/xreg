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

#ifndef XREGITKOPENCVUTILS_H_
#define XREGITKOPENCVUTILS_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>  // canny

#include "xregITKRemapUtils.h"
#include "xregExceptionUtils.h"

namespace xreg
{

/// \brief Create a shallow reference to a 2D ITK image stored as an OpenCV
///        matrix object.
///
/// The pixel type template parameter of the ITK image must be a scalar type.
template <class T>
cv::Mat ShallowCopyItkToOpenCV(itk::Image<T,2>* itk_img)
{
  const auto itk_size = itk_img->GetLargestPossibleRegion().GetSize();

  return cv::Mat(itk_size[1], itk_size[0], cv::DataType<T>::type, itk_img->GetBufferPointer());
}

/// \brief Create a shallow reference to a 2D OpenCV matrix object stored as a
///        2D ITK image object.
///
/// The pixel type template parameter of the ITK image must be a scalar type.
template <class T>
typename itk::Image<T,2>::Pointer ShallowCopyOpenCVToItk(cv::Mat& ocv_img)
{
  using ItkImage              = itk::Image<T,2>;
  using ItkImageRegionType    = typename ItkImage::RegionType;
  using ItkPixelContainerType = typename ItkImage::PixelContainer;

  xregASSERT(cv::DataType<T>::type == ocv_img.type());
  xregASSERT(ocv_img.isContinuous());

  auto itk_img = ItkImage::New();

  auto px_cont = ItkPixelContainerType::New();

  px_cont->SetImportPointer(reinterpret_cast<T*>(ocv_img.data), ocv_img.total(), false);

  itk_img->SetPixelContainer(px_cont);

  ItkImageRegionType itk_reg;
  itk_reg.SetIndex(0, 0);
  itk_reg.SetIndex(1, 0);
  itk_reg.SetSize(0, ocv_img.cols);
  itk_reg.SetSize(1, ocv_img.rows);

  itk_img->SetRegions(itk_reg);

  return itk_img;
}

namespace detail
{

template <class T, class U>
typename itk::Image<T,2>::Pointer CopyAndCastOpenCVToITKHelper(const cv::Mat& ocv_img)
{
  using PixelType       = T;
  using ImageType       = itk::Image<PixelType,2>;
  using ImageRegionType = typename ImageType::RegionType;
  using size_type       = typename ImageType::SizeValueType;

  auto itk_img = ImageType::New();

  const size_type num_rows = ocv_img.rows;
  const size_type num_cols = ocv_img.cols;

  ImageRegionType itk_region;
  itk_region.SetIndex(0,0);
  itk_region.SetIndex(1,0);
  itk_region.SetSize(0, num_cols);
  itk_region.SetSize(1, num_rows);

  itk_img->SetRegions(itk_region);
  itk_img->Allocate();

  PixelType* itk_buf = itk_img->GetBufferPointer();

  for (size_type r = 0; r < num_rows; ++r, itk_buf += num_cols)
  {
    const U* ocv_row = &ocv_img.at<U>(r,0);

    for (size_type c = 0; c < num_cols; ++c)
    {
      itk_buf[c] = static_cast<PixelType>(ocv_row[c]);  // TODO: Use a better cast!
    }
  }

  return itk_img;
}

}  // detail

/// \brief Performs a deep-copying cast from an OpenCV image to an ITK image.
///
/// Currently implemented using a static_cast, so there may be issues moving from
/// OpenCV floating point images to ITK integer images, however that's a rare
/// use case.
template <class T>
typename itk::Image<T,2>::Pointer CopyAndCastOpenCVToITK(const cv::Mat& ocv_img)
{
  typename itk::Image<T,2>::Pointer itk_img;

  if (ocv_img.type() == CV_8UC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,unsigned char>(ocv_img);
  }
  else if (ocv_img.type() == CV_8SC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,char>(ocv_img);
  }
  else if (ocv_img.type() == CV_16UC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,unsigned short>(ocv_img);
  }
  else if (ocv_img.type() == CV_16SC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,short>(ocv_img);
  }
  else if (ocv_img.type() == CV_32SC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,int>(ocv_img);
  }
  else if (ocv_img.type() == CV_32FC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,float>(ocv_img);
  }
  else if (ocv_img.type() == CV_64FC1)
  {
    itk_img = detail::CopyAndCastOpenCVToITKHelper<T,double>(ocv_img);
  }
  else
  {
    xregThrow("unsupported opencv type!");
  }

  return itk_img;
}

/// \brief Remaps an image to 8-bpp and performs Canny edge detection.
///
/// Reuses existing ITK objects to perform min/max and windowing computations.
template <class T>
void RemapAndComputeEdges(const itk::Image<T,2>* src_img, cv::Mat* edge_img,
                          const double thresh_low, const double thresh_high,
                          itk::MinimumMaximumImageCalculator<itk::Image<T,2>>* min_max_calc,
                          itk::IntensityWindowingImageFilter<itk::Image<T,2>,
                                                             itk::Image<unsigned char,2>>* window_filter)
{
  itk::Image<unsigned char,2>::Pointer remapped_img = ITKImageRemap8bpp(src_img, min_max_calc, window_filter);

  cv::Mat disp_img = ShallowCopyItkToOpenCV(remapped_img.GetPointer());

  cv::GaussianBlur(disp_img, *edge_img, cv::Size(9,9), 0);

  // &src == &dst is allowed
  cv::Canny(*edge_img, *edge_img, thresh_low, thresh_high);
}

/// \brief Remaps an image to 8-bpp and performs Canny edge detection.
template <class T>
void RemapAndComputeEdges(const itk::Image<T,2>* src_img, cv::Mat* edge_img,
                          const double thresh_low, const double thresh_high)
{
  using MinMaxCalc        = itk::MinimumMaximumImageCalculator<itk::Image<T,2>>;
  using IntensityRemapper = itk::IntensityWindowingImageFilter<itk::Image<T,2>,itk::Image<unsigned char,2>>;

  auto min_max_calc = MinMaxCalc::New();
  auto window_filter = IntensityRemapper::New();

  RemapAndComputeEdges(src_img, edge_img, thresh_low, thresh_high,
                       min_max_calc.GetPointer(), window_filter.GetPointer());
}

template <class tScalar>
cv::Mat CopyITKRGBToOpenCVBGR(const itk::Image<itk::RGBPixel<tScalar>,2>* src)
{
  using Scalar = tScalar;

  const auto itk_size = src->GetLargestPossibleRegion().GetSize();

  const int nr = static_cast<int>(itk_size[1]);
  const int nc = static_cast<int>(itk_size[0]);
  
  cv::Mat dst(nr, nc, CV_MAKETYPE(cv::DataDepth<Scalar>::value, 3));

  const auto* src_buf = src->GetBufferPointer();

  for (int r = 0; r < nr; ++r, src_buf += nc)
  {
    Scalar* dst_row = &dst.at<Scalar>(r,0);

    for (int c = 0; c < nc; ++c)
    {
      const auto& sp = src_buf[c];

      const int off = 3 * c;
      
      dst_row[off]     = sp.GetBlue();
      dst_row[off + 1] = sp.GetGreen();
      dst_row[off + 2] = sp.GetRed();
    }
  }

  return dst;
}

}  // xreg

#endif

