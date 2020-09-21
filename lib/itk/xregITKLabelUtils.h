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

#ifndef XREGITKLABELUTILS_H_
#define XREGITKLABELUTILS_H_

#include <unordered_set>

#include <itkRegionOfInterestImageFilter.h>
#include <itkLabelMapMaskImageFilter.h>
#include <itkLabelMap.h>
#include <itkLabelObject.h>
#include <itkLabelImageToLabelMapFilter.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkImageRegionConstIteratorWithIndex.h>

namespace xreg
{

  /// \brief Finds the index-based bounding box about all locations with a certain label.
///
/// TODO: Handle dimensionality besides 3
template <class tLabelType>
typename itk::Image<tLabelType,3>::RegionType
FindBoundBoxAboutLabel(const itk::Image<tLabelType,3>* img, const tLabelType label)
{
  using LabelType      = tLabelType;
  using LabelImageType = itk::Image<LabelType,3>;
  using SizeType       = typename LabelImageType::SizeType;
  using SizeValueType  = typename LabelImageType::SizeValueType;
  using RegionType     = typename LabelImageType::RegionType;

  const SizeType img_size = img->GetLargestPossibleRegion().GetSize();

  SizeType min_vals;
  min_vals[0] = img_size[0] - 1;
  min_vals[1] = img_size[1] - 1;
  min_vals[2] = img_size[2] - 1;

  SizeType max_vals;
  max_vals.Fill(0);

  const LabelType* label_ptr = img->GetBufferPointer();

  for (SizeValueType k = 0; k < img_size[2]; ++k)
  {
    const bool k_is_min = k < min_vals[2];
    const bool k_is_max = k > max_vals[2];

    for (SizeValueType j = 0; j < img_size[1]; ++j)
    {
      const bool j_is_min = j < min_vals[1];
      const bool j_is_max = j > max_vals[1];

      for (SizeValueType i = 0; i < img_size[0]; ++i, ++label_ptr)
      {
        if (*label_ptr == label)
        {
          if (i < min_vals[0])
          {
            min_vals[0] = i;
          }
          if (i > max_vals[0])
          {
            max_vals[0] = i;
          }

          if (j_is_min)
          {
            min_vals[1] = j;
          }
          if (j_is_max)
          {
            max_vals[1] = j;
          }

          if (k_is_min)
          {
            min_vals[2] = k;
          }
          if (k_is_max)
          {
            max_vals[2] = k;
          }
        }
      }
    }
  }

  RegionType bb;
  bb.SetIndex(0, min_vals[0]);
  bb.SetIndex(1, min_vals[1]);
  bb.SetIndex(2, min_vals[2]);
  bb.SetSize(0, max_vals[0] - min_vals[0] + 1);
  bb.SetSize(1, max_vals[1] - min_vals[1] + 1);
  bb.SetSize(2, max_vals[2] - min_vals[2] + 1);

  return bb;
}

/// \brief Finds the index-based bounding box about all locations equal to any certain labels in a collection.
///
/// TODO: Handle dimensionality besides 3
template <class tLabelType>
typename itk::Image<tLabelType,3>::RegionType
FindBoundBoxAboutLabels(const itk::Image<tLabelType,3>* img, const std::unordered_set<tLabelType>& labels)
{
  using LabelType      = tLabelType;
  using LabelImageType = itk::Image<LabelType,3>;
  using SizeType       = typename LabelImageType::SizeType;
  using SizeValueType  = typename LabelImageType::SizeValueType;
  using RegionType     = typename LabelImageType::RegionType;
  using LabelSet       = typename std::decay<decltype(labels)>::type;

  xregASSERT(!labels.empty());

  const SizeType img_size = img->GetLargestPossibleRegion().GetSize();

  SizeType min_inds;
  min_inds[0] = img_size[0] - 1;
  min_inds[1] = img_size[1] - 1;
  min_inds[2] = img_size[2] - 1;

  SizeType max_inds;
  max_inds.Fill(0);

  for (typename LabelSet::const_iterator lit = labels.begin(); lit != labels.end(); ++lit)
  {
    const RegionType cur_label_reg = FindBoundBoxAboutLabel(img, *lit);
    min_inds[0] = std::min(min_inds[0], static_cast<SizeValueType>(cur_label_reg.GetIndex(0)));
    min_inds[1] = std::min(min_inds[1], static_cast<SizeValueType>(cur_label_reg.GetIndex(1)));
    min_inds[2] = std::min(min_inds[2], static_cast<SizeValueType>(cur_label_reg.GetIndex(2)));

    max_inds[0] = std::max(max_inds[0], static_cast<SizeValueType>(cur_label_reg.GetIndex(0)) + cur_label_reg.GetSize(0) - 1);
    max_inds[1] = std::max(max_inds[1], static_cast<SizeValueType>(cur_label_reg.GetIndex(1)) + cur_label_reg.GetSize(1) - 1);
    max_inds[2] = std::max(max_inds[2], static_cast<SizeValueType>(cur_label_reg.GetIndex(2)) + cur_label_reg.GetSize(2) - 1);
  }

  RegionType bb;
  bb.SetIndex(0, min_inds[0]);
  bb.SetIndex(1, min_inds[1]);
  bb.SetIndex(2, min_inds[2]);
  bb.SetSize(0, max_inds[0] - min_inds[0] + 1);
  bb.SetSize(1, max_inds[1] - min_inds[1] + 1);
  bb.SetSize(2, max_inds[2] - min_inds[2] + 1);

  return bb;
}

/// \brief Applies a masking operation to a volume using a specific label from
///        a label map.
///
/// \param tight_crop true indicates that the output volume should be tightly
///                   cropped about the region containing label pixels; the
///                   cropped output will have consistent physical coordinates.
template <class tPixelType, class tLabelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ApplyMaskToITKImage(const itk::Image<tPixelType,tN>* img, const itk::Image<tLabelType,tN>* label_img,
                    const tLabelType keep_label, const tPixelType bg_val,
                    const bool tight_crop = false)
{
  using PixelType = tPixelType;
  using LabelType = tLabelType;
  
  constexpr unsigned int kDIM = tN;
  
  using ImageType         = itk::Image<PixelType,kDIM>;
  using ImagePointer      = typename ImageType::Pointer;
  using LabelImageType    = itk::Image<LabelType,kDIM>;
  using LabelImagePointer = typename LabelImageType::Pointer;

  using LabelROIFilter = itk::RegionOfInterestImageFilter<LabelImageType,LabelImageType>;
  using ImageROIFilter = itk::RegionOfInterestImageFilter<ImageType,ImageType>;

  using LabelMapType   = itk::LabelMap<itk::LabelObject<LabelType,kDIM>>;
  using LabelImageToLabelMapFilterType = itk::LabelImageToLabelMapFilter<LabelImageType,LabelMapType>;
  using MaskFilterType = itk::LabelMapMaskImageFilter<LabelMapType,ImageType>;

  const ImageType* src_img = img;
  const LabelImageType* src_labels = label_img;

  ImagePointer cropped_img;
  LabelImagePointer cropped_labels;

  if (tight_crop)
  {
    auto crop_region = FindBoundBoxAboutLabel(label_img, keep_label);

    auto img_roi_filt = ImageROIFilter::New();
    img_roi_filt->SetInput(img);
    img_roi_filt->SetRegionOfInterest(crop_region);
    img_roi_filt->Update();
    cropped_img = img_roi_filt->GetOutput();
    src_img = cropped_img.GetPointer();

    auto label_roi_filt = LabelROIFilter::New();
    label_roi_filt->SetInput(label_img);
    label_roi_filt->SetRegionOfInterest(crop_region);
    label_roi_filt->Update();
    cropped_labels = label_roi_filt->GetOutput();
    src_labels = cropped_labels.GetPointer();
  }

  auto label_img_to_map_filter = LabelImageToLabelMapFilterType::New();
  label_img_to_map_filter->SetInput(src_labels);
  label_img_to_map_filter->Update();

  auto masker = MaskFilterType::New();
  masker->SetBackgroundValue(bg_val);
  masker->SetLabel(keep_label);
  masker->SetInput(label_img_to_map_filter->GetOutput());
  masker->SetFeatureImage(src_img);
  masker->Update();

  return masker->GetOutput();
}

/// \brief Applies a masking operation to a volume using a specific collection
///        of label from a label map.
///
/// \param tight_crop true indicates that the output volume should be tightly
///                   cropped about the region containing label pixels; the
///                   cropped output will have consistent physical coordinates.
template <class tPixelType, class tLabelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ApplyMaskToITKImageKeepLabels(itk::Image<tPixelType,tN>* img, itk::Image<tLabelType,tN>* label_img,
                              const std::unordered_set<tLabelType>& keep_labels, const tPixelType bg_val,
                              const bool tight_crop = false)
{
  constexpr unsigned int kDIM = tN;

  using PixelType    = tPixelType;
  using ImageType    = itk::Image<PixelType,kDIM>;
  using ImagePointer = typename ImageType::Pointer;

  using ImageIt = itk::ImageRegionIteratorWithIndex<ImageType>;

  using LabelType         = tLabelType;
  using LabelImageType    = itk::Image<LabelType,kDIM>;
  using LabelImagePointer = typename LabelImageType::Pointer;

  using LabelROIFilter = itk::RegionOfInterestImageFilter<LabelImageType,LabelImageType>;
  using ImageROIFilter = itk::RegionOfInterestImageFilter<ImageType,ImageType>;

  ImagePointer src_img = img;
  LabelImagePointer src_labels = label_img;

  if (tight_crop)
  {
    auto crop_region = FindBoundBoxAboutLabels(label_img, keep_labels);

    auto img_roi_filt = ImageROIFilter::New();
    img_roi_filt->SetInput(img);
    img_roi_filt->SetRegionOfInterest(crop_region);
    img_roi_filt->Update();
    src_img = img_roi_filt->GetOutput();

    auto label_roi_filt = LabelROIFilter::New();
    label_roi_filt->SetInput(src_labels);
    label_roi_filt->SetRegionOfInterest(crop_region);
    label_roi_filt->Update();
    src_labels = label_roi_filt->GetOutput();
  }

  auto dst_img = ITKImageDeepCopy(src_img.GetPointer());

  ImageIt img_it(dst_img, dst_img->GetLargestPossibleRegion());
  for (img_it.GoToBegin(); !img_it.IsAtEnd(); ++img_it)
  {
    if (keep_labels.find(src_labels->GetPixel(img_it.GetIndex())) == keep_labels.end())
    {
      img_it.Value() = bg_val;
    }
  }

  return dst_img;
}

/// \brief Replace the voxels in an image that correspond to a certain set of
///        voxels with a specific value.
template <class tPixelType, class tLabelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
ITKImageMaskOutLabels(const itk::Image<tPixelType,tN>* img, const itk::Image<tLabelType,tN>* label_img,
                      const std::unordered_set<tLabelType>& labels_to_mask, const tPixelType masked_value)
{
  constexpr unsigned int kDIM = tN;

  using PixelType = tPixelType;
  using ImageType = itk::Image<PixelType,kDIM>;

  using ImageIt = itk::ImageRegionIteratorWithIndex<ImageType>;

  auto dst_img = ITKImageDeepCopy(img);

  ImageIt img_it(dst_img, dst_img->GetLargestPossibleRegion());
  for (img_it.GoToBegin(); !img_it.IsAtEnd(); ++img_it)
  {
    if (labels_to_mask.find(label_img->GetPixel(img_it.GetIndex())) != labels_to_mask.end())
    {
      img_it.Value() = masked_value;
    }
  }

  return dst_img;
}

/// \brief Retrieves a list of indices that correspond to a specific label in a
///        label map.
template <class tLabelType, unsigned tN>
std::vector<typename itk::Image<tLabelType,tN>::IndexType>
GetIndsForSpecificLabel(const itk::Image<tLabelType,tN>* img, const tLabelType l)
{
  using Img       = itk::Image<tLabelType,tN>;
  using IndexType = typename Img::IndexType;
  using ImgIt     = itk::ImageRegionConstIteratorWithIndex<Img>;
 
  std::vector<IndexType> inds;

  ImgIt it(img, img->GetLargestPossibleRegion());
  
  for (it.GoToBegin(); !it.IsAtEnd(); ++it)
  {
    if (it.Value() == l)
    {
      inds.push_back(it.GetIndex());
    }
  }

  return inds;
}

std::vector<itk::Image<unsigned char,3>::Pointer>
MakeVolListFromVolAndLabels(const itk::Image<unsigned char,3>* vol,
                            const itk::Image<unsigned char,3>* labels,
                            const std::vector<unsigned char>& labels_to_use,
                            const unsigned char masked_out_val);

std::vector<itk::Image<float,3>::Pointer>
MakeVolListFromVolAndLabels(const itk::Image<float,3>* vol,
                            const itk::Image<unsigned char,3>* labels,
                            const std::vector<unsigned char>& labels_to_use,
                            const float masked_out_val);

}  // xreg

#endif

