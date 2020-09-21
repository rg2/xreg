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

#include "xregPAOVolAfterRepo.h"

#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkBSplineInterpolateImageFunction.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkBinaryBallStructuringElement.h>
#include <itkGrayscaleDilateImageFilter.h>

#include "xregAssert.h"
#include "xregITKBasicImageUtils.h"
#include "xregITKLabelUtils.h"
#include "xregSampleUtils.h"
#include "xregMetalObjSampling.h"
#include "xregITKCropPadUtils.h"
#include "xregITKResampleUtils.h"

void xreg::UpdateVolAfterRepos::operator()()
{
  using ITKIndex = Vol::IndexType;
  using ContinuousIndexType = itk::ContinuousIndex<double,3>;

  using NNInterp = itk::NearestNeighborInterpolateImageFunction<LabelVol>;

  using VolInterp = itk::BSplineInterpolateImageFunction<Vol>;

  using VolIt = itk::ImageRegionIteratorWithIndex<Vol>;
  using LabelVolIt = itk::ImageRegionIteratorWithIndex<LabelVol>;

  xregASSERT(ImagesHaveSameCoords(labels.GetPointer(), src_vol.GetPointer()));

  std::mt19937 rng_eng;
  std::normal_distribution<VolScalar> norm_dist(0,add_rand_to_default_val_std_dev);

  const bool add_rand = add_rand_to_default_val_std_dev > 0;
  if (add_rand)
  {
    SeedRNGEngWithRandDev(&rng_eng);
  }
 
  const VolScalar tmp_default_val = default_val; 
  auto replacement_intensity = [add_rand, &rng_eng,
                                &norm_dist,
                                tmp_default_val] ()
  {
    return tmp_default_val + (add_rand ? norm_dist(rng_eng) :
                                         VolScalar(0));
  };

  std::normal_distribution<VolScalar> air_normal_dist(0,10);
  auto air_intensity = [add_rand,&rng_eng,&air_normal_dist]()
  {
    return -1000 + (add_rand ? air_normal_dist(rng_eng) : VolScalar(0));
  };

  const FrameTransform itk_idx_to_phys_pt = ITKImagePhysicalPointTransformsAsEigen(src_vol.GetPointer());

  const FrameTransform phys_pt_to_itk_idx = itk_idx_to_phys_pt.inverse();

  auto vol_interp_fn = VolInterp::New();
  vol_interp_fn->SetSplineOrder(3);
    
  dst_vol = ITKImageDeepCopy(src_vol.GetPointer());

  if (!labels_of_air.empty())
  {
    for (const auto l : labels_of_air)
    {
      VolIt dst_it(dst_vol, dst_vol->GetLargestPossibleRegion());
      LabelVolIt label_it(labels, labels->GetLargestPossibleRegion());

      for (dst_it.GoToBegin(), label_it.GoToBegin(); !dst_it.IsAtEnd(); ++dst_it, ++label_it)
      {
        if (label_it.Value() == l)
        {
          dst_it.Value() = air_intensity();
        }
      }
    }
   
    // we want to interpolate as if the cuts have already been made 
    vol_interp_fn->SetInputImage(ITKImageDeepCopy(dst_vol.GetPointer()));
  }
  else
  {
    // no cuts incorporated, just use the source volume
    vol_interp_fn->SetInputImage(src_vol);
  }

  const bool do_dilate = labels_dilate_rad;
  
  auto label_interp = NNInterp::New();
  
  if (!do_dilate)
  {
    // we can look directly into the original labelmap if we are not
    // dilating it
    label_interp->SetInputImage(labels);
  }

  Pt3 tmp_idx;

  ContinuousIndexType tmp_itk_idx;

  const unsigned long num_repo_objs = labels_of_repo_objs.size();
  xregASSERT(num_repo_objs == repo_objs_xforms.size());

  for (unsigned long obj_idx = 0; obj_idx < num_repo_objs; ++obj_idx)
  {
    const LabelScalar cur_label = labels_of_repo_objs[obj_idx];

    if (do_dilate)
    {
      using MorphKernel  = itk::BinaryBallStructuringElement<LabelScalar,3>;
      using DilateFilter = itk::GrayscaleDilateImageFilter<LabelVol,LabelVol,MorphKernel>;
   
      LabelVolPtr cur_labels = ApplyMaskToITKImage(labels.GetPointer(), labels.GetPointer(),
                                                   cur_label, LabelScalar(0));

      MorphKernel kern;
      kern.SetRadius(labels_dilate_rad);
      kern.CreateStructuringElement();

      auto dilate_fn = DilateFilter::New();
      dilate_fn->SetInput(cur_labels);
      dilate_fn->SetKernel(kern);

      dilate_fn->Update();

      label_interp->SetInputImage(dilate_fn->GetOutput());
    }

    const FrameTransform obj_warp = repo_objs_xforms[obj_idx].inverse();

    VolIt dst_it(dst_vol, dst_vol->GetLargestPossibleRegion());
    LabelVolIt label_it(labels, labels->GetLargestPossibleRegion());

    for (dst_it.GoToBegin(), label_it.GoToBegin(); !dst_it.IsAtEnd(); ++dst_it, ++label_it)
    {
      // First find out if this location now belongs to the current repositioned object
      const ITKIndex& cur_idx = dst_it.GetIndex();
      tmp_idx[0] = cur_idx[0];
      tmp_idx[1] = cur_idx[1];
      tmp_idx[2] = cur_idx[2];

      // continuous index before repositioning
      tmp_idx = phys_pt_to_itk_idx * obj_warp * itk_idx_to_phys_pt * tmp_idx;

      tmp_itk_idx[0] = tmp_idx[0];
      tmp_itk_idx[1] = tmp_idx[1];
      tmp_itk_idx[2] = tmp_idx[2];

      if (label_interp->IsInsideBuffer(tmp_itk_idx) &&
          (label_interp->EvaluateAtContinuousIndex(tmp_itk_idx) == cur_label))
      {
        // this location should be set to an intensity value from the repositioned object
        dst_it.Value() = vol_interp_fn->EvaluateAtContinuousIndex(tmp_itk_idx);
      }
      else if (label_it.Value() == cur_label)
      {
        // this location was, but no longer corresponds to the repositioned object,
        // fill the intensity with a default value, e.g. air
        dst_it.Value() = replacement_intensity();
      }
    }
  }
}

namespace
{

using namespace xreg;

struct PtInObjVisitor : public boost::static_visitor<bool>
{
  Pt3 p;

  bool operator()(const NaiveScrewModel& s) const
  {
    return PointInNaiveScrew(s, p);
  }

  bool operator()(const NaiveKWireModel& w) const
  {
    return PointInNaiveKWire(w, p);
  }
};

}  // un-named

void xreg::AddPAOScrewKWireToVol::operator()()
{
  const size_type num_objs = obj_start_pts.size();
  
  xregASSERT(num_objs == obj_end_pts.size());

  screw_models.clear();
  screw_models.reserve(num_objs);
  screw_poses_wrt_vol.clear();
  screw_poses_wrt_vol.reserve(num_objs);

  kwire_models.clear();
  kwire_models.reserve(num_objs);
  kwire_poses_wrt_vol.clear();
  kwire_poses_wrt_vol.reserve(num_objs);

  CreateRandScrew create_rand_screws;
  CreateRandKWire create_rand_kwires;

  create_rand_kwires.set_debug_output_stream(*this);

  std::uniform_real_distribution<double> obj_dist(0,1);

  std::uniform_real_distribution<PixelScalar> screw_hu_dist(14000, 16000);
  std::uniform_real_distribution<PixelScalar> kwire_hu_dist(14000, 26000);

  using ObjVar = boost::variant<NaiveScrewModel,NaiveKWireModel>;
  using ObjVarList = std::vector<ObjVar>;

  ObjVarList objs;
  objs.reserve(num_objs);

  FrameTransformList obj_to_vol_poses;
  obj_to_vol_poses.reserve(num_objs);

  std::vector<PixelScalar> obj_hu_vals;
  obj_hu_vals.reserve(num_objs);

  std::vector<BoundBox3> obj_bbs;
  obj_bbs.reserve(num_objs);

  // for each object 
  for (size_type obj_idx = 0; obj_idx < num_objs; ++obj_idx)
  {
    this->dout() << "creating object model: " << obj_idx << std::endl;

    if (obj_dist(create_rand_screws.rng_eng) < prob_screw)
    {
      this->dout() << "  a screw..." << std::endl;

      obj_hu_vals.push_back(screw_hu_dist(create_rand_screws.rng_eng));

      NaiveScrewModel s;
      FrameTransform screw_to_vol_xform;

      // create random screw model and get pose in volume
      this->dout() << "creating random screw model..." << std::endl;
      std::tie(s,screw_to_vol_xform) = create_rand_screws(obj_start_pts[obj_idx],
                                                          obj_end_pts[obj_idx]);
     
      screw_models.push_back(s);
      screw_poses_wrt_vol.push_back(screw_to_vol_xform);
      
      obj_bbs.push_back(ComputeBoundingBox(s));

      objs.push_back(s);
      obj_to_vol_poses.push_back(screw_to_vol_xform);
    }
    else
    {
      this->dout() << "a K-Wire..." << std::endl;

      // insert a K-Wire
      obj_hu_vals.push_back(kwire_hu_dist(create_rand_screws.rng_eng));

      NaiveKWireModel s;
      FrameTransform kwire_to_vol_xform;

      // create random screw model and get pose in volume
      this->dout() << "creating random K-Wire model..." << std::endl;
      std::tie(s,kwire_to_vol_xform) = create_rand_kwires(obj_start_pts[obj_idx],
                                                          obj_end_pts[obj_idx]);
     
      kwire_models.push_back(s);
      kwire_poses_wrt_vol.push_back(kwire_to_vol_xform);
 
      obj_bbs.push_back(ComputeBoundingBox(s));

      objs.push_back(s);
      obj_to_vol_poses.push_back(kwire_to_vol_xform);
    }
  }  // end for each obj

  BoundBox3 all_objs_bb_wrt_vol = TransformBoundBox(obj_bbs[0], obj_to_vol_poses[0]);

  for (size_type obj_idx = 1; obj_idx < num_objs; ++obj_idx)
  {
    all_objs_bb_wrt_vol = CombineBoundBoxes(all_objs_bb_wrt_vol,
                                            TransformBoundBox(obj_bbs[obj_idx], obj_to_vol_poses[obj_idx]));
  }

  orig_vol_start_pad = { 0, 0, 0 };
  orig_vol_end_pad   = { 0, 0, 0 };

  {
    this->dout() << "determing if additional padding is needed to fit objects in volume..." << std::endl;

    const FrameTransform orig_vol_inds_to_phys_pts = ITKImagePhysicalPointTransformsAsEigen(orig_vol.GetPointer());

    const FrameTransform phys_pts_to_orig_vol_inds = orig_vol_inds_to_phys_pts.inverse();

    const BoundBox3 all_objs_bb_wrt_vol_idx = TransformBoundBox(all_objs_bb_wrt_vol,
                                                                            phys_pts_to_orig_vol_inds);
 
    const auto orig_vol_itk_reg  = orig_vol->GetLargestPossibleRegion();
    const auto orig_vol_itk_size = orig_vol_itk_reg.GetSize();

    for (size_type i = 0; i < 3; ++i)
    {
      const CoordScalar lower_round = std::floor(all_objs_bb_wrt_vol_idx.lower(i));

      orig_vol_start_pad[i] = (lower_round < 0) ? static_cast<size_type>(-lower_round) : size_type(0);
     
      const CoordScalar upper_round = std::ceil(all_objs_bb_wrt_vol_idx.upper(i));
      
      orig_vol_end_pad[i] = (upper_round > orig_vol_itk_size[i]) ?
                                static_cast<size_type>(upper_round - orig_vol_itk_size[i]) :
                                size_type(0);
    }

    bool need_to_pad = false;
    
    for (size_type i = 0; i < 3; ++i)
    {
      if (orig_vol_start_pad[i] || orig_vol_end_pad[i])
      {
        need_to_pad = true;
        break;
      }
    }

    if (need_to_pad)
    {
      this->dout() << "padding volume..." << std::endl;

      orig_vol_pad = ITKPadImage(orig_vol.GetPointer(),
                                 orig_vol_start_pad,
                                 orig_vol_end_pad,
                                 PixelScalar(-1000));
    }
    else
    {
      this->dout() << "no padding required..." << std::endl;
      orig_vol_pad = ITKImageDeepCopy(orig_vol.GetPointer());
    }
  }
  
  VolPtr vol_sup;
  
  if (std::abs(super_sample_factor - 1.0) > 1.0e-3)
  {
    // super sample the original volume
    this->dout() << "super sampling (padded) original volume..." << std::endl;
    vol_sup = DownsampleImageNNInterp(orig_vol_pad.GetPointer(), super_sample_factor, 0.0);
  }
  else
  {
    this->dout() << "no super-sampling..." << std::endl;
    vol_sup = ITKImageDeepCopy(orig_vol_pad.GetPointer());
  }
  
  const FrameTransform vol_inds_to_phys_pts = ITKImagePhysicalPointTransformsAsEigen(vol_sup.GetPointer());

  const FrameTransform phys_pts_to_vol_inds = vol_inds_to_phys_pts.inverse();

  // keep track of voxels that are marked as belonging to a screw, we'll downsample
  // this back to the original resolution and only update the screw voxels
  // in the final output
  VolPtr voxels_modified_sup = MakeITKNDVol<PixelScalar>(vol_sup->GetLargestPossibleRegion());

  const auto itk_size = vol_sup->GetLargestPossibleRegion().GetSize();

  // for each object 
  for (size_type obj_idx = 0; obj_idx < num_objs; ++obj_idx)
  {
    this->dout() << "inserting object: " << obj_idx << std::endl;

    const FrameTransform& obj_to_vol_xform = obj_to_vol_poses[obj_idx];

    const FrameTransform obj_to_vol_idx = phys_pts_to_vol_inds * obj_to_vol_xform;

    const auto obj_bb_wrt_inds = TransformBoundBox(obj_bbs[obj_idx], obj_to_vol_idx);

    const std::array<size_type,3> start_inds = {
                  static_cast<size_type>(std::max(CoordScalar(0), std::floor(obj_bb_wrt_inds.lower(0)))),
                  static_cast<size_type>(std::max(CoordScalar(0), std::floor(obj_bb_wrt_inds.lower(1)))),
                  static_cast<size_type>(std::max(CoordScalar(0), std::floor(obj_bb_wrt_inds.lower(2)))) };
   

    const std::array<size_type,3> stop_inds = {
            static_cast<size_type>(std::min(CoordScalar(itk_size[0]), std::ceil(obj_bb_wrt_inds.upper(0)))),
            static_cast<size_type>(std::min(CoordScalar(itk_size[1]), std::ceil(obj_bb_wrt_inds.upper(1)))),
            static_cast<size_type>(std::min(CoordScalar(itk_size[2]), std::ceil(obj_bb_wrt_inds.upper(2)))) };

    const std::array<size_type,3> num_inds_to_check = {
                                    stop_inds[0] - start_inds[0] + 1,
                                    stop_inds[1] - start_inds[1] + 1,
                                    stop_inds[2] - start_inds[2] + 1 };

    const size_type xy_num_inds_to_check = num_inds_to_check[0] *
                                           num_inds_to_check[1];

    const size_type tot_num_inds_to_check = xy_num_inds_to_check * 
                                            num_inds_to_check[2];

    // now fill intersecting voxels   

    const FrameTransform vol_to_obj_xform = obj_to_vol_xform.inverse();

    const FrameTransform vol_idx_to_obj_pts = vol_to_obj_xform * vol_inds_to_phys_pts;

    const PixelScalar obj_hu = obj_hu_vals[obj_idx];

    ObjVar& obj = objs[obj_idx];

    auto insert_obj_intens_fn = [&] (const RangeType& r)
    {
      Vol::IndexType itk_idx;

      PtInObjVisitor pt_in_obj;

      Pt3 idx;

      size_type cur_idx_1d = r.begin();

      // these indices need to be offset by the start_inds[]        
      size_type       z_idx = cur_idx_1d / xy_num_inds_to_check;
      const size_type tmp   = cur_idx_1d - (z_idx * xy_num_inds_to_check);
      size_type       y_idx = tmp / num_inds_to_check[0];
      size_type       x_idx = tmp - (y_idx * num_inds_to_check[0]);
      
      for (; cur_idx_1d < r.end(); ++cur_idx_1d)
      {
        itk_idx[0] = start_inds[0] + x_idx;
        itk_idx[1] = start_inds[1] + y_idx;
        itk_idx[2] = start_inds[2] + z_idx;

        idx[0] = itk_idx[0];
        idx[1] = itk_idx[1];
        idx[2] = itk_idx[2];
        
        pt_in_obj.p = vol_idx_to_obj_pts * idx;

        if (boost::apply_visitor(pt_in_obj, obj))
        {
          vol_sup->SetPixel(itk_idx, obj_hu);

          voxels_modified_sup->SetPixel(itk_idx, 1);
        }

        // increment
        ++x_idx;
        if (x_idx == num_inds_to_check[0])
        {
          x_idx = 0;

          ++y_idx;
          if (y_idx == num_inds_to_check[1])
          {
            y_idx = 0;

            ++z_idx;
          }
        }
      }
    };

    this->dout() << "running parallel for over all indices to check..." << std::endl;
    ParallelFor(insert_obj_intens_fn, RangeType(0, tot_num_inds_to_check));
  }  // end for each obj

  // downsample back to original resolution

  this->dout() << "downsampling from super-sampled..." << std::endl;

  // -1 --> smooth before downsampling and choose the kernel size automatically 
  VolPtr vol_tmp_ds = DownsampleImageNNInterp(vol_sup.GetPointer(),
                                              1.0 / super_sample_factor, -1.0);

  VolPtr modified_ds = DownsampleImageLinearInterp(voxels_modified_sup.GetPointer(),
                                                   1.0 / super_sample_factor, -1.0); 

  // adding in screw voxels in the original volume resolution

  this->dout() << "updating original volume intensities where necessary and creating label map..." << std::endl;

  obj_vol = ITKImageDeepCopy(orig_vol_pad.GetPointer());

  obj_labels = LabelVol::New();
  obj_labels->SetDirection(obj_vol->GetDirection());
  obj_labels->SetSpacing(obj_vol->GetSpacing());
  obj_labels->SetOrigin(obj_vol->GetOrigin());
  obj_labels->SetRegions(obj_vol->GetLargestPossibleRegion());
  obj_labels->Allocate();
  obj_labels->FillBuffer(0);

  itk::ImageRegionIteratorWithIndex<Vol> mod_ds_it(modified_ds, modified_ds->GetLargestPossibleRegion());
  
  while (!mod_ds_it.IsAtEnd())
  {
    if (mod_ds_it.Value() > 1.0e-6)
    {
      const auto idx = mod_ds_it.GetIndex();

      obj_vol->SetPixel(idx, vol_tmp_ds->GetPixel(idx));
      obj_labels->SetPixel(idx, 1);
    }
    
    ++mod_ds_it;
  }
}
