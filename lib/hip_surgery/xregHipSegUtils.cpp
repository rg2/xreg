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

#include "xregHipSegUtils.h"

#include <itkImageRegionConstIteratorWithIndex.h>

#include "xregITKBasicImageUtils.h"
#include "xregITKMathOps.h"
#include "xregITKLabelUtils.h"
#include "xregSpatialPrimitives.h"
#include "xregPAOCuts.h"

namespace
{

template <class tLabelScalar>
std::tuple<tLabelScalar,tLabelScalar,tLabelScalar>
GuessPelvisLeftRightFemurLabelsHelper(const itk::Image<tLabelScalar,3>* label_img)
{
  using LabelScalar = tLabelScalar;
  using LabelHist   = std::vector<unsigned long>;
  
  static_assert(std::numeric_limits<LabelScalar>::is_integer,
                "label type must be an integer");
  static_assert(!std::numeric_limits<LabelScalar>::is_signed,
                "label type must be unsigned");
  
  using LabelMap   = itk::Image<LabelScalar,3>;
  using LabelMapIt = itk::ImageRegionConstIteratorWithIndex<LabelMap>;
  
  LabelHist label_hist(std::numeric_limits<LabelScalar>::max(), 0);

  LabelMapIt it(label_img, label_img->GetLargestPossibleRegion());

  // create the histogram
  for (it.GoToBegin(); !it.IsAtEnd(); ++it)
  {
    ++label_hist[it.Value()];
  }

  label_hist[0] = 0;  // make sure bg is ignored.

  const LabelScalar pelvis_label = static_cast<LabelScalar>(
      std::max_element(label_hist.begin(), label_hist.end()) - label_hist.begin());
  
  label_hist[pelvis_label] = 0;

  LabelScalar left_femur_label = static_cast<LabelScalar>(
      std::max_element(label_hist.begin(), label_hist.end()) - label_hist.begin());
 
  const unsigned long num_left_femur_labels = label_hist[left_femur_label]; 
  label_hist[left_femur_label] = 0;
  
  LabelScalar right_femur_label = static_cast<LabelScalar>(
      std::max_element(label_hist.begin(), label_hist.end()) - label_hist.begin());

  const unsigned long num_right_femur_labels = label_hist[right_femur_label];

  // get the average X values of the left/right femur candidates
  
  double left_x_avg  = 0;
  double right_x_avg = 0;
 
  itk::Point<double,3> cur_pt;

  for (it.GoToBegin(); !it.IsAtEnd(); ++it)
  {
    const LabelScalar& cur_label = it.Value();

    const bool is_left  = cur_label == left_femur_label;
    const bool is_right = cur_label == right_femur_label;

    if (is_left || is_right)
    {
      label_img->TransformIndexToPhysicalPoint(it.GetIndex(), cur_pt);
      
      if (is_left)
      {
        left_x_avg += cur_pt[0];
      }
      else if (is_right)
      {
        right_x_avg += cur_pt[0];
      }
    }
  }

  if ((left_x_avg / num_left_femur_labels) < (right_x_avg / num_right_femur_labels))
  {
    // The left femur has an average X coordinate less than the average X coordinate
    // of the right femur, but X increases from right to left, so they are flipped.
    std::swap(left_femur_label, right_femur_label);
  }

  return std::make_tuple(pelvis_label, left_femur_label, right_femur_label);

}

}  // un-named

std::tuple<unsigned char,unsigned char,unsigned char>
xreg::GuessPelvisLeftRightFemurLabels(const itk::Image<unsigned char,3>* label_img)
{
  return GuessPelvisLeftRightFemurLabelsHelper(label_img);
}

std::tuple<unsigned short,unsigned short,unsigned short>
xreg::GuessPelvisLeftRightFemurLabels(const itk::Image<unsigned short,3>* label_img)
{
  return GuessPelvisLeftRightFemurLabelsHelper(label_img);
}

namespace
{

using namespace xreg;

template <class tLabelType>
std::tuple<tLabelType,tLabelType,tLabelType>
GuessPelvisPAOFragCutLabelsHelper(const itk::Image<tLabelType,3>* labels,
                                  const bool labels_has_frag,
                                  const bool labels_has_cut)
{
  const bool pelvis_label = GetITKImageMinPositive(labels);

  const auto max_label = GetITKImageMax(labels);

  return std::make_tuple(pelvis_label,
                         labels_has_frag ? 
                           (labels_has_cut ? (max_label - 1) : max_label) :
                           (max_label + 1),
                         labels_has_cut ?
                           max_label :
                           (labels_has_frag ? (max_label + 1) : (max_label + 2)));
}

}  // un-named

std::tuple<unsigned char,unsigned char,unsigned char>
xreg::GuessPelvisPAOFragCutLabels(const itk::Image<unsigned char,3>* labels,
                                  const bool labels_has_frag,
                                  const bool labels_has_cut)
{
  return GuessPelvisPAOFragCutLabelsHelper(labels,
                                           labels_has_frag,
                                           labels_has_cut);
}

std::tuple<unsigned short,unsigned short,unsigned short>
xreg::GuessPelvisPAOFragCutLabels(const itk::Image<unsigned short,3>* labels,
                                  const bool labels_has_frag,
                                  const bool labels_has_cut)
{
  return GuessPelvisPAOFragCutLabelsHelper(labels,
                                           labels_has_frag,
                                           labels_has_cut);
}

namespace
{

template <class tLabelType>
std::tuple<tLabelType,tLabelType,tLabelType,tLabelType>
GuessPelvisFemurPAOFragLabelsHelper(const itk::Image<tLabelType,3>* labels,
                                    const Pt3& femur_pt,
                                    const bool labels_has_frag,
                                    const bool labels_has_cut)
{
  using LabelType  = tLabelType;
  using LabelImage = itk::Image<LabelType,3>;
  using LabelIndex = typename LabelImage::IndexType;
  using LabelPoint = typename LabelImage::PointType;

  const auto pelvis_frag_cut_labels = GuessPelvisPAOFragCutLabelsHelper(labels, labels_has_frag, labels_has_cut);
  
  LabelIndex femur_idx;
  LabelPoint femur_pt_itk;
      
  femur_pt_itk[0] = femur_pt[0];
  femur_pt_itk[1] = femur_pt[1];
  femur_pt_itk[2] = femur_pt[2];

  labels->TransformPhysicalPointToIndex(femur_pt_itk, femur_idx);
  
  return std::make_tuple(std::get<0>(pelvis_frag_cut_labels),
                         labels->GetPixel(femur_idx),
                         std::get<1>(pelvis_frag_cut_labels),
                         std::get<2>(pelvis_frag_cut_labels));
}

}  // un-named

std::tuple<unsigned char,unsigned char,unsigned char,unsigned char>
xreg::GuessPelvisFemurPAOFragLabels(const itk::Image<unsigned char,3>* labels,
                                    const Pt3& femur_pt,
                                    const bool labels_has_frag,
                                    const bool labels_has_cut)
{
  return GuessPelvisFemurPAOFragLabelsHelper(labels, femur_pt, labels_has_frag, labels_has_cut);
}

std::tuple<unsigned short,unsigned short,unsigned short,unsigned short>
xreg::GuessPelvisFemurPAOFragLabels(const itk::Image<unsigned short,3>* labels,
                                    const Pt3& femur_pt,
                                    const bool labels_has_frag,
                                    const bool labels_has_cut)
{
  return GuessPelvisFemurPAOFragLabelsHelper(labels, femur_pt, labels_has_frag, labels_has_cut);
}

// TODO: I want to replace looking for cut voxels with looking at all voxels
//       and checking distance to plane is within cut width, this has the
//       problem of finding things outisde of the connected component
void xreg::PAOExtract3DCutPointsForEachCut(const itk::Image<unsigned char,3>* labels,
                                           const unsigned char cut_label,
                                           const PAOCutPlanes& cut_defs,
                                           const FrameTransform& vol_to_app,
                                           Pt3List* ilium_cut_pts,
                                           Pt3List* ischium_cut_pts,
                                           Pt3List* pubis_cut_pts,
                                           Pt3List* post_cut_pts,
                                           std::vector<itk::Image<unsigned char,3>::IndexType>* ilium_cut_inds,
                                           std::vector<itk::Image<unsigned char,3>::IndexType>* ischium_cut_inds,
                                           std::vector<itk::Image<unsigned char,3>::IndexType>* pubis_cut_inds,
                                           std::vector<itk::Image<unsigned char,3>::IndexType>* post_cut_inds)
{
  using LabelType  = unsigned char;
  using LabelImage = itk::Image<LabelType,3>;
  using LabelsIt   = itk::ImageRegionConstIteratorWithIndex<LabelImage>;
  using DstInd     = LabelsIt::IndexType;
  using IndexList  = std::vector<DstInd>;

  const Plane3* cut_planes[4] = { &cut_defs.ilium, &cut_defs.ischium,
                                  &cut_defs.pubis, &cut_defs.post };
  Pt3List* cut_pts[4] = { ilium_cut_pts, ischium_cut_pts,
                          pubis_cut_pts, post_cut_pts };

  IndexList* cut_inds[4] =  { ilium_cut_inds, ischium_cut_inds,
                              pubis_cut_inds, post_cut_inds };

  DstInd tmp_dst_idx;

  for (size_type cut_idx = 0; cut_idx < 4; ++cut_idx)
  {
    if (cut_pts[cut_idx])
    {
      cut_pts[cut_idx]->clear();
    }

    if (cut_inds[cut_idx])
    {
      cut_inds[cut_idx]->clear();
    }
  }
  
  CoordScalar dists[4] = { 0, 0, 0, 0 };

  LabelsIt it(labels, labels->GetLargestPossibleRegion());

  Pt3 tmp_idx;
  Pt3 tmp_pt_wrt_vol;
  Pt3 tmp_pt_wrt_app;

  const FrameTransform itk_idx_to_phys = ITKImagePhysicalPointTransformsAsEigen(labels);

  for (it.GoToBegin(); !it.IsAtEnd(); ++it)
  {
    if (it.Value() == cut_label)
    {
      // map index to volume physical point
      const auto& cur_idx = it.GetIndex();
      tmp_idx[0] = cur_idx[0];
      tmp_idx[1] = cur_idx[1];
      tmp_idx[2] = cur_idx[2];

      tmp_pt_wrt_vol = itk_idx_to_phys * tmp_idx;
    
      // map volume physical point to APP where the cutting planes are defined
      tmp_pt_wrt_app = vol_to_app * tmp_pt_wrt_vol;

      // find the distances to each plane

      CoordScalar cur_min = std::numeric_limits<CoordScalar>::max();

      for (size_type cut_idx = 0; cut_idx < 4; ++cut_idx)
      {
        dists[cut_idx] = DistToPlane(tmp_pt_wrt_app, *cut_planes[cut_idx]);
    
        if (dists[cut_idx] < cur_min)
        {
          cur_min = dists[cut_idx];
        }
      }

      // Assign volume point to a plane if the point is equal to the minimum distance
      // (at the intersection of planes a point may belong to several planes)
      for (size_type cut_idx = 0; cut_idx < 4; ++cut_idx)
      {
        if (std::abs(dists[cut_idx] - cur_min) < 1.0e-6)
        {
          if (cut_pts[cut_idx])
          {
            cut_pts[cut_idx]->push_back(tmp_pt_wrt_vol);
          }

          if (cut_inds[cut_idx])
          {
            // could do an optimization here if the label image index type
            // is the same as this index.
            
            tmp_dst_idx[0] = cur_idx[0];
            tmp_dst_idx[1] = cur_idx[1];
            tmp_dst_idx[2] = cur_idx[2];

            cut_inds[cut_idx]->push_back(tmp_dst_idx);
          }
        }
      }
    }
  }
}

namespace
{

using namespace xreg;

template <class tPixelType, class tLabelType>
std::tuple<typename itk::Image<tPixelType,3>::Pointer,
           typename itk::Image<tPixelType,3>::Pointer,
           typename itk::Image<tPixelType,3>::Pointer>
SplitIntoPelvisFemurFragVolsHelper(const itk::Image<tPixelType,3>* img,
                                   const itk::Image<tLabelType,3>* labels,
                                   const tLabelType pelvis_label,
                                   const tLabelType femur_label,
                                   const tLabelType frag_label,
                                   const Pt3* femur_pt)
{
  using PixelType = tPixelType;
  using LabelType = tLabelType;

  const bool need_to_find_pelvis_label = !pelvis_label;
  const bool need_to_find_frag_label   = !frag_label;
  const bool need_to_find_femur_label  = !femur_label;

  LabelType pelvis_label_to_use = pelvis_label;
  LabelType femur_label_to_use  = femur_label;
  LabelType frag_label_to_use   = frag_label;

  if (need_to_find_pelvis_label || need_to_find_frag_label ||
      need_to_find_femur_label)
  {
    const auto guessed_labels = GuessPelvisFemurPAOFragLabels(labels, *femur_pt, true, true);
    
    if (need_to_find_pelvis_label)
    {
      pelvis_label_to_use = std::get<0>(guessed_labels);
    }
    
    if (need_to_find_femur_label)
    {
      femur_label_to_use = std::get<1>(guessed_labels);
    }

    if (need_to_find_pelvis_label)
    {
      frag_label_to_use = std::get<2>(guessed_labels);
    }
  }

  constexpr bool kTIGHT_CROP_VOLS = true;

  return std::make_tuple(
      ApplyMaskToITKImage(img, labels, pelvis_label_to_use,
                          PixelType(0), kTIGHT_CROP_VOLS),
      ApplyMaskToITKImage(img, labels, femur_label_to_use,
                          PixelType(0), kTIGHT_CROP_VOLS),
      ApplyMaskToITKImage(img, labels, frag_label_to_use,
                          PixelType(0), kTIGHT_CROP_VOLS));
}

}  // un-named

std::tuple<itk::Image<float,3>::Pointer,
           itk::Image<float,3>::Pointer,
           itk::Image<float,3>::Pointer>
xreg::SplitIntoPelvisFemurFragVols(const itk::Image<float,3>* img,
                                   const itk::Image<unsigned char,3>* labels,
                                   const unsigned char pelvis_label,
                                   const unsigned char femur_label,
                                   const unsigned char frag_label,
                                   const Pt3* femur_pt)
{
  return SplitIntoPelvisFemurFragVolsHelper(img, labels,
                                            pelvis_label, femur_label, frag_label,
                                            femur_pt);
}

