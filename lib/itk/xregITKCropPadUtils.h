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

#ifndef XREGITKCROPPADUTILS_H_
#define XREGITKCROPPADUTILS_H_

#include <array>

#include <itkRegionOfInterestImageFilter.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>

#include "xregCommon.h"

namespace xreg
{

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<char,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<char,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<unsigned char,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<unsigned char,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<short,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<short,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<unsigned short,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<unsigned short,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<float,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<float,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

/// \brief Crop a volume using a bounding box defined by a center point and dimensions
///        in physical units.
///
/// The dimensions are "radius," e.g. half of the full dimensions.
itk::Image<double,3>::Pointer
CropImageWithBoundBoxPhysPts(const itk::Image<double,3>* src_img,
                             const Pt3& center_pt, const Pt3& phys_dims);

template <class tPixelType>
typename itk::Image<tPixelType,2>::Pointer
CropImage2DBoundary(const itk::Image<tPixelType,2>* src_img, const unsigned long boundary_width)
{
  using Img       = itk::Image<tPixelType,2>;
  using ROIFilter = itk::RegionOfInterestImageFilter<Img,Img>;
  
  const auto src_img_size = src_img->GetLargestPossibleRegion().GetSize();

  typename Img::SizeType roi_size;
  roi_size[0] = src_img_size[0] - (2 * boundary_width);
  roi_size[1] = src_img_size[1] - (2 * boundary_width);

  typename Img::IndexType roi_start;
  roi_start[0] = boundary_width;
  roi_start[1] = boundary_width;

  typename Img::RegionType roi;
  roi.SetSize(roi_size);
  roi.SetIndex(roi_start);

  auto img_roi_filter = ROIFilter::New();
  img_roi_filter->SetInput(src_img);
  img_roi_filter->SetRegionOfInterest(roi);
  img_roi_filter->Update();
  
 return img_roi_filter->GetOutput();
}

template <class tPixelScalar, unsigned int tN, class tSizeScalar>
typename itk::Image<tPixelScalar,tN>::Pointer
ITKPadImage(const itk::Image<tPixelScalar,tN>* src_img,
            const std::array<tSizeScalar,static_cast<unsigned long>(tN)>& start_pad,
            const std::array<tSizeScalar,static_cast<unsigned long>(tN)>& end_pad,
            const tPixelScalar& fill_val)
{
  using PixelScalar = tPixelScalar;
  using size_type   = tSizeScalar;
  
  constexpr unsigned int kDIM = tN;

  using Img    = itk::Image<PixelScalar,kDIM>;
  using ImgPtr = typename Img::Pointer;

  using ITKReg = typename Img::RegionType;

  const auto src_img_reg = src_img->GetLargestPossibleRegion();

  const auto src_img_size = src_img_reg.GetSize();

  auto pad_vol_size = src_img_size;

  for (unsigned int i = 0; i < kDIM; ++i)
  {
    pad_vol_size[i] += start_pad[i] + end_pad[i];
  }
  
  ITKReg pad_reg(pad_vol_size);

  itk::ContinuousIndex<double,kDIM> new_zero_idx_wrt_old_idx;
  for (unsigned int i = 0; i < kDIM; ++i)
  {
    new_zero_idx_wrt_old_idx[i] = -static_cast<double>(start_pad[i]);
  }

  typename Img::PointType new_zero_idx_wrt_phys;
  
  src_img->TransformContinuousIndexToPhysicalPoint(new_zero_idx_wrt_old_idx, new_zero_idx_wrt_phys);
  
  ImgPtr dst_img = Img::New();

  dst_img->SetDirection(src_img->GetDirection());
  dst_img->SetSpacing(src_img->GetSpacing());
  dst_img->SetOrigin(new_zero_idx_wrt_phys);
  dst_img->SetRegions(pad_reg);
  dst_img->Allocate();
  dst_img->FillBuffer(fill_val);
        
  // copy pixels from src_img
  itk::ImageRegionConstIterator<Img> src_img_it(src_img, src_img_reg);

  ITKReg dst_reg_in_pad;
  dst_reg_in_pad.SetIndex(0, start_pad[0]);
  dst_reg_in_pad.SetIndex(1, start_pad[1]);
  dst_reg_in_pad.SetIndex(2, start_pad[2]);
  dst_reg_in_pad.SetSize(src_img_size);
  
  itk::ImageRegionIterator<Img> pad_img_it(dst_img, dst_reg_in_pad);

  while (!src_img_it.IsAtEnd())
  {
    pad_img_it.Set(src_img_it.Get());

    ++pad_img_it;
    ++src_img_it;
  }

  return dst_img;
}


}  // xreg

#endif

