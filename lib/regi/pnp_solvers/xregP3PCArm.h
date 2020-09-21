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

#ifndef XREGP3PCARM_H_
#define XREGP3PCARM_H_

#include "xregLandmark2D3DRegi.h"
#include "xregNDRange.h"

namespace xreg
{

// ref_pt 3D Location/estimate of a reference point with respect to the C-Arm coordinate frame
// src_pt 3D Location of the X-ray source with respect to the C-Arm frame
// det_pt 3D Location of another/query point on the detector, with respect to the C-Arm frame
// l      Known distance between the reference point and the query point in 3D
// dist_ratio_lower Lower bound to accept 3D estimates of the point of ratio: ||ref_pt - est_pt|| / l
// dist_ratio_upper Upper bound to accept 3D estimates of the point of ratio: ||ref_pt - est_pt|| / l
Pt3List FindPossPtsAlongLineForMatch(const Pt3& ref_pt, const Pt3& src_pt, const Pt3& det_pt, const CoordScalar l,
                                     const CoordScalar dist_ratio_lower = 0.99,
                                     const CoordScalar dist_ratio_upper = 1.01);


class CArmP3P : public Landmark2D3DRegi
{
public:
  using PoseFoundCallBack = std::function<void(const CArmP3P*,
                                               const int,
                                               const FrameTransform&,
                                               const Pt3List&,
                                               const Pt3List&)>;

  CArmP3P() = default;

  void set_ref_idx(const int ref_idx);

  void set_eps(const CoordScalar eps);
  
  void set_cluster_poses(const bool cluster_poses);
  
  void set_sort_reproj(const bool sort_reproj);

  void set_ref_pt_depths(const CoordScalarList& ref_pt_depths);

  void set_ref_pt_depths(const CoordScalar start, const CoordScalar stop,
                         const CoordScalar inc);

  void set_pose_found_call_back_fn(PoseFoundCallBack& pose_found_call_back_fn);

  const FrameTransformList& poss_poses() const;
  
  int num_pts_required() override;

  bool uses_ref_frame() const override;

protected:
  void run_impl() override;

private:
  // < 0 --> run P3P for each possibility of ref. point
  int ref_idx_ = -1;
  
  CoordScalar eps_ = 0.01;

  bool cluster_poses_ = true;
  
  bool sort_reproj_ = true;

  CoordScalarList ref_pt_depths_ = ConstSpacedRange({0.5f, 1.0f, 0.01f}).vals();

  PoseFoundCallBack pose_found_call_back_fn_;

  FrameTransformList poss_poses_;
};

}  // xreg

#endif

