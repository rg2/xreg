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

#ifndef XREGLANDMARK2D3DREGI_H_
#define XREGLANDMARK2D3DREGI_H_

#include "xregCommon.h"
#include "xregPerspectiveXform.h"

namespace xreg
{

// NOTE: This now has multi-view support, however it has a severe
//       limitation, each view must have all 3D landmarks visible!
class Landmark2D3DRegi
{
public:
  using CamModelList = std::vector<CameraModel>;

  using ListOfPt2Lists = std::vector<Pt2List>;

  struct UnsupportedOperationException { };

  Landmark2D3DRegi() = default;

  virtual ~Landmark2D3DRegi() = default;

  Landmark2D3DRegi(const Landmark2D3DRegi&) = delete;
  Landmark2D3DRegi& operator=(const Landmark2D3DRegi&) = delete;

  void set_cam(const CameraModel& cam);

  void set_cams(const CamModelList& cams);

  void set_inds_2d(const Pt2List& inds_2d);

  void set_inds_2d(const ListOfPt2Lists& inds_2d);

  const ListOfPt2Lists& inds_2d() const;

  void set_world_pts_3d(const Pt3List& pts_3d);
  
  const Pt3List& world_pts_3d() const;

  void set_inds_2d_and_world_pts_3d(const LandMap2& inds_2d, const LandMap3& pts_3d);
  
  void set_inds_2d_and_world_pts_3d(const std::vector<LandMap2>& inds_2d, const LandMap3& pts_3d);

  void set_init_cam_to_world(const FrameTransform& init_cam_to_world);

  const FrameTransform& regi_cam_to_world() const;

  void set_ref_frame(const FrameTransform& ref_frame, const bool maps_world_to_ref);

  void run();

  virtual int num_pts_required();

  void set_use_ref_frame_as_pts_3d_centroid(const bool use_ref_frame_as_pts_3d_centroid);
  
  virtual bool uses_ref_frame() const = 0;

protected:
  virtual void run_impl() = 0;

  CamModelList cams_;

  ListOfPt2Lists inds_2d_;

  Pt3List world_pts_3d_;

  FrameTransform init_cam_to_world_ = FrameTransform::Identity(); 
  FrameTransform regi_cam_to_world_ = FrameTransform::Identity();

  FrameTransform ref_frame_     = FrameTransform::Identity();
  FrameTransform ref_frame_inv_ = FrameTransform::Identity();
  bool ref_frame_maps_world_to_ref_ = true;

  bool use_ref_frame_as_pts_3d_centroid_ = true;
};

}  // xreg

#endif

