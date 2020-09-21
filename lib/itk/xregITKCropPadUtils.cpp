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

#include "xregITKCropPadUtils.h"

namespace
{

using namespace xreg;

template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
CropImageWithBoundBoxPhysPtsHelper(const itk::Image<tPixelType,tN>* src_img,
                                   const Eigen::Matrix<CoordScalar,int(tN),1>& center_pt,
                                   const Eigen::Matrix<CoordScalar,int(tN),1>& phys_dims)
{
  using ImageType  = itk::Image<tPixelType,tN>;
  using Point      = itk::Point<double,tN>;
  using ContInd    = itk::ContinuousIndex<double,tN>;
  using RegionType = typename ImageType::RegionType;
  using SizeType   = typename ImageType::SizeType;
  using ROIFilter  = itk::RegionOfInterestImageFilter<ImageType,ImageType>;

  const SizeType src_size = src_img->GetLargestPossibleRegion().GetSize();

  Point pt1_itk;
  Point pt2_itk;
  for (unsigned int i = 0; i < tN; ++i)
  {
    pt1_itk[i] = center_pt[i] - (phys_dims[i]);
    pt2_itk[i] = center_pt[i] + (phys_dims[i]);
  }

  ContInd ind1;
  src_img->TransformPhysicalPointToContinuousIndex(pt1_itk, ind1);  
  
  ContInd ind2;
  src_img->TransformPhysicalPointToContinuousIndex(pt2_itk, ind2);

  RegionType roi;

  for (unsigned int i = 0; i < tN; ++i)
  {
    const long start_ind = std::max(std::lround(std::min(ind1[i], ind2[i])), static_cast<long>(0));
    const long stop_ind  = std::min(std::lround(std::max(ind1[i], ind2[i])), static_cast<long>(src_size[i]-1));
  
    roi.SetIndex(i, start_ind);
    roi.SetSize(i, stop_ind - start_ind + 1);
  }

  auto roi_filt = ROIFilter::New();
  roi_filt->SetInput(src_img);
  roi_filt->SetRegionOfInterest(roi);
  roi_filt->Update();

  return roi_filt->GetOutput(); 
}

}  // un-named

itk::Image<char,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<char,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}

itk::Image<unsigned char,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<unsigned char,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}

itk::Image<short,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<short,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}


itk::Image<unsigned short,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<unsigned short,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}


itk::Image<float,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<float,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}

itk::Image<double,3>::Pointer
xreg::CropImageWithBoundBoxPhysPts(const itk::Image<double,3>* src_img,
                                   const Pt3& center_pt, const Pt3& phys_dims)
{
  return CropImageWithBoundBoxPhysPtsHelper(src_img, center_pt, phys_dims);
}

