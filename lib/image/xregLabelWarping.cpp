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

#include "xregLabelWarping.h"

#include <itkImageRegionConstIteratorWithIndex.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkNearestNeighborInterpolateImageFunction.h>

#include "xregRigidUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregAssert.h"

bool xreg::CheckCollisionBetweenLabelSets(const itk::Image<unsigned char,3>* labels,
                                          const boost::container::flat_set<unsigned char>& mov_labels,
                                          const boost::container::flat_set<unsigned char>& fixed_labels,
                                          const FrameTransform& delta_xform_inv)
{
  using LabelScalar  = unsigned char;
  using LabelImage   = itk::Image<LabelScalar,3>;
  using ConstIt      = itk::ImageRegionConstIteratorWithIndex<LabelImage>;
  using ITKPointType = LabelImage::PointType;
  using ITKIndexType = LabelImage::IndexType;

  ITKPointType tmp_itk_pt;
  ITKIndexType tmp_itk_idx;

  Pt3 tmp_pt1;
  Pt3 tmp_pt2;

  bool collides = false;

  ConstIt src_it(labels, labels->GetLargestPossibleRegion());
  for (src_it.GoToBegin(); !collides && !src_it.IsAtEnd(); ++src_it)
  {
    if (fixed_labels.find(src_it.Value()) != fixed_labels.end())
    {
      // this voxel is a fixed label, let's use an inverse warp to see if it
      // would "pull-in" any moving labels, which would indicated a collision.
      labels->TransformIndexToPhysicalPoint(src_it.GetIndex(), tmp_itk_pt);
      tmp_pt1[0] = tmp_itk_pt[0];
      tmp_pt1[1] = tmp_itk_pt[1];
      tmp_pt1[2] = tmp_itk_pt[2];

      tmp_pt2 = delta_xform_inv * tmp_pt1;
      tmp_itk_pt[0] = tmp_pt2[0];
      tmp_itk_pt[1] = tmp_pt2[1];
      tmp_itk_pt[2] = tmp_pt2[2];

      // this returns true if the index is within bounds, if it's out of bounds,
      // we assume no collision
      if (labels->TransformPhysicalPointToIndex(tmp_itk_pt, tmp_itk_idx))
      {
        if (mov_labels.find(labels->GetPixel(tmp_itk_idx)) != mov_labels.end())
        {
          collides = true;
        }
      }
    }
  }

  return collides;
}

xreg::SampleValidLabelWarpsFn::SampleValidLabelWarpsFn()
{
  labels = nullptr;

  num_xforms = 0;

  check_for_collision = true;

  std::random_device rand_dev;
  rng_eng.seed(rand_dev());

  param_sampler.reset(new SampleRepoParams);
}
  
void xreg::SampleValidLabelWarpsFn::operator()()
{
  param_sampler->setup(&rng_eng);

  const FrameTransform vol_to_inter = inter_to_vol_xform.inverse();

  delta_inter_xforms.resize(num_xforms);

  xform_vals.assign(num_xforms, CoordScalarList(6,0));

  num_samples = 0;

  CoordScalar rot_x = 0;
  CoordScalar rot_y = 0;
  CoordScalar rot_z = 0;

  CoordScalar trans_x = 0;
  CoordScalar trans_y = 0;
  CoordScalar trans_z = 0;

  for (size_type xform_idx = 0; xform_idx < num_xforms; ++xform_idx)
  {
    // for debugging/checking progress
    //if (!(xform_idx % 100))
    //{
    //  std::cout << xform_idx << std::endl;
    //}

    FrameTransform delta_inter;
    FrameTransform delta_vol;

    do
    {
      // sample delta_inter
      delta_inter.setIdentity();

      rot_x = param_sampler->rot_x();
      rot_y = param_sampler->rot_y();
      rot_z = param_sampler->rot_z();

      trans_x = param_sampler->trans_x();
      trans_y = param_sampler->trans_y();
      trans_z = param_sampler->trans_z();

      delta_inter.matrix() = EulerRotX4x4(rot_x)
                             * EulerRotY4x4(rot_y)
                             * EulerRotZ4x4(rot_z);

      delta_inter.matrix()(0,3) = trans_x;
      delta_inter.matrix()(1,3) = trans_y;
      delta_inter.matrix()(2,3) = trans_z;

      delta_vol = inter_to_vol_xform * delta_inter * vol_to_inter;
    
      ++num_samples;
    }
    while (check_for_collision &&
           CheckCollisionBetweenLabelSets(labels, mov_labels, fixed_labels,
                                          delta_vol.inverse()));

    xform_vals[xform_idx][0] = rot_x;
    xform_vals[xform_idx][1] = rot_y;
    xform_vals[xform_idx][2] = rot_z;

    xform_vals[xform_idx][3] = trans_x;
    xform_vals[xform_idx][4] = trans_y;
    xform_vals[xform_idx][5] = trans_z;

    delta_inter_xforms[xform_idx] = delta_inter;
  }
}
  
double xreg::SampleValidLabelWarpsFn::rejection_prob() const
{
  return 1.0 - (static_cast<double>(num_xforms) / num_samples);
}

void xreg::SampleValidLabelWarpsFn::SampleRepoParams::setup(RNGEng*)
{ }

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::rot_x()
{
  return 0;
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::rot_y()
{
  return 0;
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::rot_z()
{
  return 0;
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::trans_x()
{
  return 0;
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::trans_y()
{
  return 0;
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParams::trans_z()
{
  return 0;
}
    
void xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::setup(RNGEng* rng_eng_arg)
{
  rng_eng = rng_eng_arg;

  rot_x_dist = NormalDist(rot_mean(0), rot_std_dev(0));
  rot_y_dist = NormalDist(rot_mean(1), rot_std_dev(1));
  rot_z_dist = NormalDist(rot_mean(2), rot_std_dev(2));

  trans_x_dist = NormalDist(trans_mean(0), trans_std_dev(0));
  trans_y_dist = NormalDist(trans_mean(1), trans_std_dev(1));
  trans_z_dist = NormalDist(trans_mean(2), trans_std_dev(2));
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::rot_x()
{
  return rot_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::rot_y()
{
  return rot_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::rot_z()
{
  return rot_z_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::trans_x()
{
  return trans_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::trans_y()
{
  return trans_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllNormal::trans_z()
{
  return trans_z_dist(*rng_eng);
}
    
void xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::setup(RNGEng* rng_eng_arg)
{
  rng_eng = rng_eng_arg;

  rot_x_dist = UniDist(rot_lower(0), rot_upper(0));
  rot_y_dist = UniDist(rot_lower(1), rot_upper(1));
  rot_z_dist = UniDist(rot_lower(2), rot_upper(2));

  trans_x_dist = UniDist(trans_lower(0), trans_upper(0));
  trans_y_dist = UniDist(trans_lower(1), trans_upper(1));
  trans_z_dist = UniDist(trans_lower(2), trans_upper(2));
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::rot_x()
{
  return rot_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::rot_y()
{
  return rot_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::rot_z()
{
  return rot_z_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::trans_x()
{
  return trans_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::trans_y()
{
  return trans_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsAllUniform::trans_z()
{
  return trans_z_dist(*rng_eng);
}
    
void xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::setup(RNGEng* rng_eng_arg)
{
  rng_eng = rng_eng_arg;

  rot_x_dist = LogNormDist(rot_m(0), rot_s(0));
  rot_y_dist = LogNormDist(rot_m(1), rot_s(1));
  rot_z_dist = LogNormDist(rot_m(2), rot_s(2));

  trans_x_dist = NormalDist(trans_mean(0), trans_std_dev(0));
  trans_y_dist = NormalDist(trans_mean(1), trans_std_dev(1));
  trans_z_dist = NormalDist(trans_mean(2), trans_std_dev(2));
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::rot_x()
{
  return (negate_rot_x ? -1 : 1) * rot_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::rot_y()
{
  return (negate_rot_y ? -1 : 1) * rot_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::rot_z()
{
  return (negate_rot_z ? -1 : 1) * rot_z_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::trans_x()
{
  return trans_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::trans_y()
{
  return trans_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFrag::trans_z()
{
  return trans_z_dist(*rng_eng);
}
    
void xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFemur::setup(RNGEng* rng_eng_arg)
{
  rng_eng = rng_eng_arg;

  rot_x_dist = LogNormDist(rot_m(0), rot_s(0));
  rot_y_dist = NormalDist(rot_m(1), rot_s(1));
  rot_z_dist = NormalDist(rot_m(2), rot_s(2));
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFemur::rot_x()
{
  return (negate_rot_x ? -1 : 1) * rot_x_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFemur::rot_y()
{
  return rot_y_dist(*rng_eng);
}

xreg::CoordScalar xreg::SampleValidLabelWarpsFn::SampleRepoParamsLogNormalMixFemur::rot_z()
{
  return rot_z_dist(*rng_eng);
}

namespace
{

using namespace xreg;

template <class tLabelType, class tPixelType>
typename itk::Image<tPixelType,3>::Pointer
WarpImageLimitedToLabelHelper(const itk::Image<tLabelType,3>* labels,
                              const tLabelType label_to_warp,
                              const itk::Image<tPixelType,3>* img_to_warp,
                              const FrameTransform& warp_xform_inv,
                              const itk::InterpolateImageFunction<itk::Image<tPixelType,3>>* interp_fn,
                              const tPixelType other_val)
{
  using LabelType = tLabelType;
  using PixelType = tPixelType;

  using LabelImage     = itk::Image<LabelType,3>;
  using IntensityImage = itk::Image<PixelType,3>;

  using ContinuousIndexType = itk::ContinuousIndex<double,3>;

  //using Interp = itk::InterpolateImageFunction<IntensityImage>;
  using NNInterp = itk::NearestNeighborInterpolateImageFunction<LabelImage>;

  //const bool pixel_type_is_lable_type = std::is_same<LabelType,PixelType>::val;
  //using DefaultInterpToUse = typename std::conditional<pixel_type_is_lable_type,NNInterp,Interp>::type;
  //const DefaultInterpToUse* interp_fn_to_use = interp_fn ? interp_fn :

  using DstIt = itk::ImageRegionIteratorWithIndex<IntensityImage>;

  xregASSERT(ImagesHaveSameCoords(labels, img_to_warp));

  FrameTransform itk_idx_to_phys_pt = ITKImagePhysicalPointTransformsAsEigen(img_to_warp);

  const FrameTransform phys_pt_to_itk_idx = itk_idx_to_phys_pt.inverse();

  // just do a deep copy to get all the metadata correct - TODO: change
  auto dst_img = ITKImageDeepCopy(img_to_warp);

  auto label_interp = NNInterp::New();
  label_interp->SetInputImage(labels);

  Pt3 tmp_idx;

  ContinuousIndexType tmp_itk_idx;

  DstIt dst_it(dst_img, dst_img->GetLargestPossibleRegion());
  for (dst_it.GoToBegin(); !dst_it.IsAtEnd(); ++dst_it)
  {
    const auto& cur_idx = dst_it.GetIndex();
    tmp_idx[0] = cur_idx[0];
    tmp_idx[1] = cur_idx[1];
    tmp_idx[2] = cur_idx[2];

    tmp_idx = phys_pt_to_itk_idx * warp_xform_inv * itk_idx_to_phys_pt * tmp_idx;

    tmp_itk_idx[0] = tmp_idx[0];
    tmp_itk_idx[1] = tmp_idx[1];
    tmp_itk_idx[2] = tmp_idx[2];

    if (label_interp->IsInsideBuffer(tmp_itk_idx) &&
        (label_interp->EvaluateAtContinuousIndex(tmp_itk_idx) == label_to_warp))
    {
      dst_it.Value() = interp_fn->EvaluateAtContinuousIndex(tmp_itk_idx);
    }
    else
    {
      dst_it.Value() = other_val;
    }
  }

  return dst_img;
}

}  // un-named

itk::Image<float,3>::Pointer
xreg::WarpImageLimitedToLabel(const itk::Image<unsigned char,3>* labels,
                              const unsigned char label_to_warp,
                              const itk::Image<float,3>* img_to_warp,
                              const FrameTransform& warp_xform_inv,
                              const itk::InterpolateImageFunction<itk::Image<float,3>>* interp_fn,
                              const float other_val)
{
  return WarpImageLimitedToLabelHelper(labels, label_to_warp, img_to_warp, warp_xform_inv, interp_fn, other_val);
}

