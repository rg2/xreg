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

#ifndef XREGITKMATHOPS_H_
#define XREGITKMATHOPS_H_

#include <tuple>

#include <itkMinimumMaximumImageCalculator.h>
#include <itkImageRegionConstIteratorWithIndex.h>
#include <itkSubtractImageFilter.h>
#include <itkAddImageFilter.h>

namespace xreg
{

/// \brief Find the minimum and maximum pixel values in an ITK image
template <class tPixelType, unsigned int tN>
std::tuple<tPixelType,tPixelType> GetITKImageMinMax(const itk::Image<tPixelType,tN>* img)
{
  using PixelType  = tPixelType;
  using ImageType  = itk::Image<PixelType,tN>;
  using MinMaxCalc = itk::MinimumMaximumImageCalculator<ImageType>;

  auto min_max_calc = MinMaxCalc::New();

  min_max_calc->SetImage(img);
  min_max_calc->Compute();

  return std::make_tuple(min_max_calc->GetMinimum(), min_max_calc->GetMaximum());
}

/// \brief Find the minimum pixel values in an ITK image
template <class tPixelType, unsigned int tN>
tPixelType GetITKImageMin(const itk::Image<tPixelType,tN>* img)
{
  using ImageType  = itk::Image<tPixelType,tN>;
  using MinMaxCalc = itk::MinimumMaximumImageCalculator<ImageType>;

  auto min_max_calc = MinMaxCalc::New();

  min_max_calc->SetImage(img);
  min_max_calc->ComputeMinimum();

  return min_max_calc->GetMinimum();
}

/// \brief Find the maximum pixel values in an ITK image
template <class tPixelType, unsigned int tN>
tPixelType GetITKImageMax(const itk::Image<tPixelType,tN>* img)
{
  using ImageType  = itk::Image<tPixelType,tN>;
  using MinMaxCalc = itk::MinimumMaximumImageCalculator<ImageType>;

  auto min_max_calc = MinMaxCalc::New();

  min_max_calc->SetImage(img);
  min_max_calc->ComputeMaximum();

  return min_max_calc->GetMaximum();
}

/// \brief Find the minimum positive pixel value in an ITK image.
///
/// Returns zero if no posivite values found.
template <class tPixelType, unsigned int tN>
tPixelType GetITKImageMinPositive(const itk::Image<tPixelType,tN>* img)
{
  using PixelType = tPixelType;
  using ImageType = itk::Image<PixelType,tN>;
  using ItType    = itk::ImageRegionConstIteratorWithIndex<ImageType>;

  ItType it(img, img->GetLargestPossibleRegion());

  PixelType cur_min = 0;

  // first scan for a positive value
  for (it.GoToBegin(); (cur_min == 0) && !it.IsAtEnd(); ++it)
  {
    const PixelType& cur_val = it.Value();

    if (cur_val > 0)
    {
      cur_min = cur_val;
      // this will cause the loop condition to not be met - e.g. break
    }
  }

  if (cur_min > 0)
  {
    // a positive value was found, now check against the remainder of positive
    // values
    for (; !it.IsAtEnd(); ++it)
    {
      const PixelType& cur_val = it.Value();
      if ((cur_val > 0) && (cur_val < cur_min))
      {
        cur_min = cur_val;
      }
    }
  }
  // else we've searched through the entire image and no positive value was found.

  return cur_min;
}

/// \brief Subtracts one image from another (pixel-wise)
///
/// Returns img1 - img2
template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ITKSubtractImages(const itk::Image<tPixelType,tN>* img1, const itk::Image<tPixelType,tN>* img2)
{
  using ImgType      = itk::Image<tPixelType,tN>;
  using SubtractFilt = itk::SubtractImageFilter<ImgType>;

  auto sub_filt = SubtractFilt::New();
  sub_filt->SetInput1(img1);
  sub_filt->SetInput2(img2);
  sub_filt->Update();

  return sub_filt->GetOutput();
}

/// \brief Subtracts each image in a list from the corresponding image in
///        the other list.
///
/// dst_imgs[i] = imgs1[i] - imgs2[i]
template <class tPixelType, unsigned int tN>
std::vector<typename itk::Image<tPixelType,tN>::Pointer>
ITKSubtractImgLists(const std::vector<typename itk::Image<tPixelType,tN>::Pointer>& imgs1,
                    const std::vector<typename itk::Image<tPixelType,tN>::Pointer>& imgs2)
{
  using Img       = itk::Image<tPixelType,tN>;
  using ImgList   = std::vector<typename itk::Image<tPixelType,tN>::Pointer>;
  using size_type = typename ImgList::size_type;

  const size_type num_imgs = imgs1.size();

  xregASSERT(imgs2.size() == num_imgs);

  ImgList dst_imgs(num_imgs);

  for (size_type i = 0; i < num_imgs; ++i)
  {
    dst_imgs[i] = ITKSubtractImages(imgs1[i].GetPointer(), imgs2[i].GetPointer());
  }
}

/// \brief Adds two images together (pixel-wise)
template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ITKAddImages(const itk::Image<tPixelType,tN>* img1, const itk::Image<tPixelType,tN>* img2)
{
  using Img   = itk::Image<tPixelType,tN>;
  using AddFn = itk::AddImageFilter<Img>;

  auto add_fn = AddFn::New();

  add_fn->SetInput1(img1);
  add_fn->SetInput2(img2);
  add_fn->Update();

  return add_fn->GetOutput();
}

}  // xreg

#endif

