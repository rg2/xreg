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

#ifndef XREGPROJ3DLABELSTO2D_H_
#define XREGPROJ3DLABELSTO2D_H_

#include "xregRayCastInterface.h"
#include "xregObjWithOStream.h"

namespace xreg
{

struct Proj3DLabelsTo2D : ObjWithOStream
{
  using LabelScalar = unsigned char;
  using LabelVol    = itk::Image<unsigned char,3>;
  using LabelVolPtr = LabelVol::Pointer;
  
  using LabelImg2D    = itk::Image<unsigned char,2>;
  using LabelImg2DPtr = LabelImg2D::Pointer;

  std::shared_ptr<RayCaster> depth_ray_caster;
  
  LabelVolPtr label_vol;

  std::vector<LabelScalar> labels_to_proj;

  CameraModel cam;

  // Each label's extracted volume will be treated as a separate object
  FrameTransformList obj_poses;

  // seg_channels is passed into the first arg, so remember to account for the bg channel at 0
  // single_chan_seg is passed into the second arg
  std::function<void(const std::vector<cv::Mat>&,LabelImg2D*)> convert_to_single_chan_seg_fn;

  // The following members should be set before calling init()
  //   * depth_ray_caster
  //   * label_vol
  //   * labels_to_proj
  //   * cam
  //   * convert_to_single_chan_seg_fn
  void init();

  // obj_poses should be set prior to this call
  void run();

  // OUTPUTS:

  // seg_channels[0] is the background channel (no objects intersected)
  // seg_channels[0](row,col) = 1 if no objects intersect,
  //                          = 0 otherwise
  // seg_channels[i], for i > 0 is the channel for object of labels_to_proj[i-1]
  // seg_channels[i](row,col) = 1 / (# objs. intersected) when (# objs. intersected) > 0
  //                          = 0 otherwise
  std::vector<cv::Mat> seg_channels;

  LabelImg2DPtr single_chan_seg;
};

}  // xreg

#endif

