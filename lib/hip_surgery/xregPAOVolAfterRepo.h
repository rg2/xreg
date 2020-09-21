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

#ifndef XREGPAOVOLAFTERREPO_H_
#define XREGPAOVOLAFTERREPO_H_

#include "xregCommon.h"
#include "xregMetalObjs.h"
#include "xregObjWithOStream.h"

namespace xreg
{

struct UpdateVolAfterRepos
{
  using VolScalar   = float;
  using LabelScalar = unsigned char;

  using Vol    = itk::Image<VolScalar,3>;
  using VolPtr = Vol::Pointer;

  using LabelVol    = itk::Image<LabelScalar,3>;
  using LabelVolPtr = LabelVol::Pointer;

  using LabelList = std::vector<LabelScalar>;

  VolPtr src_vol;

  LabelVolPtr labels;

  LabelList labels_of_air;

  LabelList labels_of_repo_objs;

  // 0 -> no dilation
  unsigned long labels_dilate_rad = 0;

  // These transforms points in the original volume to the volume with repositioned
  // objects
  FrameTransformList repo_objs_xforms;

  VolScalar default_val = -1000;  // assumes that the volume is HU and air is default 

  // if <= 0, then no additional random values are added to the default val
  // otherwise zero mean with a standard deviation of this value noise is
  // added
  VolScalar add_rand_to_default_val_std_dev = -1;

  VolPtr dst_vol;

  void operator()();

};

struct AddPAOScrewKWireToVol : public ObjWithOStream
{
  using PixelScalar = float;
  using Vol         = itk::Image<PixelScalar,3>;
  using VolPtr      = Vol::Pointer;
  using LabelScalar = unsigned char;
  using LabelVol    = itk::Image<LabelScalar,3>;
  using LabelVolPtr = LabelVol::Pointer;
  
  VolPtr orig_vol;

  Pt3List obj_start_pts;
  Pt3List obj_end_pts;

  // probability of choosing to insert a screw
  // probability of inserting a K-Wire is 1 minus this value
  double prob_screw = 1.0;

  double super_sample_factor = 4.0;

  // outputs
  
  std::vector<NaiveScrewModel> screw_models;
  FrameTransformList screw_poses_wrt_vol;

  std::vector<NaiveKWireModel> kwire_models;
  FrameTransformList kwire_poses_wrt_vol;

  VolPtr obj_vol;
  
  LabelVolPtr obj_labels;
    
  VolPtr orig_vol_pad;
    
  std::array<size_type,3> orig_vol_start_pad;
  std::array<size_type,3> orig_vol_end_pad;

  void operator()();
};

}  // xreg

#endif

