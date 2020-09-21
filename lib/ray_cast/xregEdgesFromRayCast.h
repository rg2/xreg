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

#ifndef XREGEDGESFROMRAYCAST_H_
#define XREGEDGESFROMRAYCAST_H_

#include "xregObjWithOStream.h"
#include "xregRayCastInterface.h"

namespace xreg
{

struct EdgesFromRayCast : ObjWithOStream
{
  using VolPtr = RayCaster::VolPtr;
 
  using LabelScalar = unsigned char;
  using LabelVol    = itk::Image<LabelScalar,3>;
  using LabelVolPtr = LabelVol::Pointer;
  using EdgeProj    = itk::Image<LabelScalar,2>;
  using EdgeProjPtr = EdgeProj::Pointer;

  bool do_canny    = true;
  bool do_boundary = false;
  bool do_occ      = true;
 
  // The Gaussian kernel width/height to be used for smoothing an image 
  // prior to Canny edge detection. <= 1 will do no smoothing. 
  int canny_smooth_width = 3;
  // Lower threshold to be used for Canny edge detection.
  int canny_low_thresh   = 0;
  // Upper threshold to be used for Canny edge detection.
  int canny_high_thresh  = 48;

  // Amount of dilation to apply to the edge map to make it easier to see.
  // <= 1 will not enlarge.
  int edge_dilate_width = 1;

  // Collision threshold (HU)
  double thresh = 150.0;

  // Threshold, in degrees, away from 90 deg that allows a ray to indicate 
  // an occluding contour.
  double occ_ang_deg = 15.0;

  CameraModel cam;

  VolPtr vol;

  LabelVolPtr label_vol;

  std::shared_ptr<RayCaster> line_int_ray_caster;
  std::shared_ptr<RayCaster> boundary_ray_caster;
  std::shared_ptr<RayCaster> occ_ray_caster;
  
  FrameTransformList cam_wrt_vols;

  std::vector<LabelScalar> obj_label_vals;

  EdgeProjPtr final_edge_img;

  void operator()();
};

}  // xreg

#endif

