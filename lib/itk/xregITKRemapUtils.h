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

#ifndef XREGITKREMAPUTILS_H_
#define XREGITKREMAPUTILS_H_

#include <vector>

#include <itkImage.h>
#include <itkIntensityWindowingImageFilter.h>
#include <itkMinimumMaximumImageCalculator.h>
#include <itkRGBPixel.h>
#include <itkImageRegionIterator.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionConstIteratorWithIndex.h>

#include "xregITKBasicImageUtils.h"

namespace xreg
{

std::vector<itk::RGBPixel<unsigned char>> GenericAnatomyLUT();

/// \brief Remaps the intensities of an image to fit in an 8-bpp image.
///
/// Reuses existing ITK objects to perform min/max and windowing computations.
template <class T, unsigned int tN>
typename itk::Image<unsigned char,tN>::Pointer
ITKImageRemap8bpp(const itk::Image<T,tN>* img,
                  itk::MinimumMaximumImageCalculator<itk::Image<T,tN>>* min_max_calc,
                  itk::IntensityWindowingImageFilter<itk::Image<T,tN>,itk::Image<unsigned char,tN>>* window_filter)
{
  min_max_calc->SetImage(img);
  min_max_calc->Compute();

  window_filter->SetInput(img);
  window_filter->SetWindowMinimum(min_max_calc->GetMinimum());
  window_filter->SetWindowMaximum(min_max_calc->GetMaximum());
  window_filter->SetOutputMinimum(0);
  window_filter->SetOutputMaximum(255);
  window_filter->Update();

  return window_filter->GetOutput();
}
 
/// \brief Remaps the intensities of an image to fit in an 8-bpp image.
template <class T, unsigned int tN>
typename itk::Image<unsigned char,tN>::Pointer ITKImageRemap8bpp(const itk::Image<T,tN>* img)
{
  using ImageType     = itk::Image<T,tN>;
  using Image8bppType = itk::Image<unsigned char,tN>;

  using MinMaxCalcType   = itk::MinimumMaximumImageCalculator<ImageType>;
  using WindowFilterType = itk::IntensityWindowingImageFilter<ImageType,Image8bppType>;

  auto min_max_calc  = MinMaxCalcType::New();
  auto window_filter = WindowFilterType::New();

  return ITKImageRemap8bpp(img, min_max_calc.GetPointer(), window_filter.GetPointer());
}

/// \brief Remaps the intensities of an image to fit in an 8-bpp image (uses pre-computed min/max window values)
template <class T, unsigned int tN>
typename itk::Image<unsigned char,tN>::Pointer
ITKImageRemap8bpp(const itk::Image<T,tN>* img, const T min_val, const T max_val)
{
  using WindowFn = itk::IntensityWindowingImageFilter<itk::Image<T,tN>,itk::Image<unsigned char,tN>>;
  
  auto window_filter = WindowFn::New();
  
  window_filter->SetInput(img);
  window_filter->SetWindowMinimum(min_val);
  window_filter->SetWindowMaximum(max_val);
  window_filter->SetOutputMinimum(0);
  window_filter->SetOutputMaximum(255);
  window_filter->Update();

  return window_filter->GetOutput();
}

template <class tLabelScalar, unsigned int tN, class tDstScalar>
typename itk::Image<tDstScalar,tN>::Pointer
RemapITKLabelMap(const itk::Image<tLabelScalar,tN>* labels,
                 const std::vector<tDstScalar>& lut)
{
  using LabelScalar = tLabelScalar;
  using DstScalar   = tDstScalar;
  using DstImg      = itk::Image<DstScalar,tN>;

  const auto img_size = labels->GetLargestPossibleRegion().GetSize();

  size_type num_pix = 1;
  for (unsigned int i = 0; i < tN; ++i)
  {
    num_pix *= img_size[i];
  }
  
  auto dst_img = MakeITKNDVol<DstScalar>(labels->GetLargestPossibleRegion());

  dst_img->SetDirection(labels->GetDirection());
  dst_img->SetSpacing(labels->GetSpacing());
  dst_img->SetOrigin(labels->GetOrigin());

  const LabelScalar* labels_buf = labels->GetBufferPointer();

  std::transform(labels_buf, labels_buf + num_pix, dst_img->GetBufferPointer(),
                 [&lut] (const LabelScalar& l)
                 {
                   return lut[l];
                 });

  return dst_img;
}

/// \brief Replace pixels in locations corresponding to a label with values from
///        another image.
///
/// \param img1 The original image - can be though of as background
/// \param label_img The label map
/// \param img2 The image with values that will be replaced into img1
/// \param label The label that indicates "foreground"
template <class tPixelType, class tLabelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ITKImageReplacePixelsCorrespondingToLabel(const itk::Image<tPixelType,tN>* img1,
                                          const itk::Image<tLabelType,tN>* label_img,
                                          const itk::Image<tPixelType,tN>* img2,
                                          const tLabelType label)
{
  constexpr unsigned int kDIM = tN;

  using LabelType      = tLabelType;
  using LabelImageType = itk::Image<LabelType,kDIM>;
  using LabelIt        = itk::ImageRegionConstIteratorWithIndex<LabelImageType>;

  auto dst_img = ITKImageDeepCopy(img1);

  LabelIt it(label_img, label_img->GetLargestPossibleRegion());
  for (it.GoToBegin(); !it.IsAtEnd(); ++it)
  {
    if (it.Value() == label)
    {
      dst_img->SetPixel(it.GetIndex(), img2->GetPixel(it.GetIndex()));
    }
  }

  return dst_img;
}

/// \brief Create a new copy of img1 where locations are updated to have
///        new values, when that location matches a pair of values in img1
///        and img2.
///
/// Useful for updating air voxels to "cut" voxels in a hand segmented fragment label map
/// given, also, the original full pelvis segmentation.
template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ChangePixelsUsingTwoImages(const itk::Image<tPixelType,tN>* img1,
                           const itk::Image<tPixelType,tN>* img2,
                           const tPixelType img1_val,
                           const tPixelType img2_val,
                           const tPixelType new_val)
{
  using PixelType  = tPixelType;
  using Img        = itk::Image<PixelType,tN>;
  using ImgConstIt = itk::ImageRegionConstIteratorWithIndex<Img>;
  using ImgIt      = itk::ImageRegionIteratorWithIndex<Img>;

  auto dst_img = ITKImageDeepCopy(img1);
  
  ImgIt dst_it(dst_img, dst_img->GetLargestPossibleRegion());
  
  ImgConstIt img2_it(img2, img2->GetLargestPossibleRegion());
  
  for (dst_it.GoToBegin(), img2_it.GoToBegin(); !dst_it.IsAtEnd(); ++dst_it, ++img2_it)
  {
    xregASSERT(!img2_it.IsAtEnd());
  
    if ((dst_it.Value() == img1_val) && (img2_it.Value() == img2_val))
    {
      dst_it.Value() = new_val;
    }
  } 

  return dst_img;
}

template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ChangeImageIntValues(const itk::Image<tPixelType,tN>* img, const std::vector<tPixelType>& m)
{
  using PixelType = tPixelType;
  using Img       = itk::Image<tPixelType,tN>;
  using ImgPtr    = typename Img::Pointer;
  
  static_assert(std::is_same<PixelType,unsigned char>::value || std::is_same<PixelType,unsigned short>::value,
                "Only unsigned char and unsigned short are supported.");

  ImgPtr dst_img = ITKImageDeepCopy(img);

  itk::ImageRegionConstIterator<Img> src_it(img, img->GetLargestPossibleRegion());
  itk::ImageRegionIterator<Img>      dst_it(dst_img, dst_img->GetLargestPossibleRegion());

  for (; !src_it.IsAtEnd(); ++src_it, ++dst_it)
  {
    dst_it.Value() = m[src_it.Value()];
  }

  return dst_img;
}

/// This is really only appropriate to use for unsigned char and unsigned short
template <class tPixelType, unsigned int tN>
std::vector<tPixelType>
CreateImageIntConsecutiveLabelsMap(const itk::Image<tPixelType,tN>* img, const bool keep_zero_fixed = true)
{
  using PixelType = tPixelType;

  static_assert(std::is_same<PixelType,unsigned char>::value || std::is_same<PixelType,unsigned short>::value,
                "Only unsigned char and unsigned short are supported.");

  const size_type num_vals = static_cast<size_type>(std::numeric_limits<PixelType>::max()) + 1;

  std::vector<bool> mapped(num_vals, false);
  std::vector<PixelType> lut(num_vals, 0);

  PixelType cur_map = 0;

  if (keep_zero_fixed)
  {
    ++cur_map;
    lut[0]    = 0;
    mapped[0] = true;
  }

  itk::ImageRegionConstIterator<itk::Image<PixelType,tN>> img_it(img, img->GetLargestPossibleRegion()); 

  for (; !img_it.IsAtEnd(); ++img_it)
  {
    const PixelType cur_val = img_it.Value();

    if (!mapped[cur_val])
    {
      lut[cur_val] = cur_map;
      ++cur_map;

      mapped[cur_val] = true;
    }
  }

  return lut;
}


template <class tScalar, unsigned int tN>
std::array<typename itk::Image<tScalar,tN>::Pointer,3>
ITKSplitRGB(const itk::Image<itk::RGBPixel<tScalar>,tN>* rgb_img)
{
  constexpr size_type kDIM = tN;

  using Scalar     = tScalar;
  using RGBPixel   = itk::RGBPixel<Scalar>;
  using RGBImg     = itk::Image<RGBPixel,kDIM>;
  using GrayImg    = itk::Image<Scalar,kDIM>;
  using GrayImgPtr = typename GrayImg::Pointer;

  std::array<GrayImgPtr,3> split_imgs;

  for (size_type i = 0; i < 3; ++i)
  {
    auto& img = split_imgs[i];
    
    img = GrayImg::New();
    img->SetDirection(rgb_img->GetDirection());
    img->SetSpacing(rgb_img->GetSpacing());
    img->SetOrigin(rgb_img->GetOrigin());
    img->SetRegions(rgb_img->GetLargestPossibleRegion());
    img->Allocate();
  }

  auto& r_img = split_imgs[0];
  auto& g_img = split_imgs[1];
  auto& b_img = split_imgs[2];

  itk::ImageRegionIterator<GrayImg> r_it(r_img, r_img->GetLargestPossibleRegion());
  itk::ImageRegionIterator<GrayImg> g_it(g_img, g_img->GetLargestPossibleRegion());
  itk::ImageRegionIterator<GrayImg> b_it(b_img, b_img->GetLargestPossibleRegion());

  itk::ImageRegionConstIterator<RGBImg> rgb_it(rgb_img, rgb_img->GetLargestPossibleRegion());

  while (!rgb_it.IsAtEnd())
  {
    const auto& cur_rgb = rgb_it.Value();

    r_it.Set(cur_rgb.GetRed()); 
    g_it.Set(cur_rgb.GetGreen()); 
    b_it.Set(cur_rgb.GetBlue()); 
    
    ++rgb_it;

    ++r_it;
    ++g_it;
    ++b_it;
  }

  return split_imgs;
}

template <class tScalar, unsigned int tN>
typename itk::Image<itk::RGBPixel<tScalar>,tN>::Pointer
ITKCombineIntoRGB(const itk::Image<tScalar,tN>* r_img,
                  const itk::Image<tScalar,tN>* g_img,
                  const itk::Image<tScalar,tN>* b_img)
{
  using Scalar   = tScalar;
  using GrayImg  = itk::Image<Scalar,tN>;
  using RGBPixel = itk::RGBPixel<Scalar>;
  using RGBImg   = itk::Image<RGBPixel,tN>;

  xregASSERT(ImagesHaveSameCoords(r_img, g_img));
  xregASSERT(ImagesHaveSameCoords(r_img, b_img));

  typename RGBImg::Pointer rgb_img = RGBImg::New();
  
  rgb_img->SetDirection(r_img->GetDirection());
  rgb_img->SetSpacing(r_img->GetSpacing());
  rgb_img->SetOrigin(r_img->GetOrigin());
  rgb_img->SetRegions(r_img->GetLargestPossibleRegion());
  rgb_img->Allocate();

  itk::ImageRegionConstIterator<GrayImg> r_it(r_img, r_img->GetLargestPossibleRegion());
  itk::ImageRegionConstIterator<GrayImg> g_it(g_img, g_img->GetLargestPossibleRegion());
  itk::ImageRegionConstIterator<GrayImg> b_it(b_img, b_img->GetLargestPossibleRegion());

  itk::ImageRegionIterator<RGBImg> rgb_it(rgb_img, rgb_img->GetLargestPossibleRegion());

  while (!rgb_it.IsAtEnd())
  {
    auto& p = rgb_it.Value();

    p.SetRed(r_it.Value());
    p.SetGreen(g_it.Value());
    p.SetBlue(b_it.Value());

    ++rgb_it;

    ++r_it;
    ++g_it;
    ++b_it;
  }

  return rgb_img;
}

}  // xreg

#endif

