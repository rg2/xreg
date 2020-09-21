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

#ifndef XREGREGI2D3DPENALTYFNPELVISAP_H_
#define XREGREGI2D3DPENALTYFNPELVISAP_H_

#include "xregRegi2D3DPenaltyFn.h"

namespace xreg
{

struct Regi2D3DPenaltyFnPelvisAP : public Regi2D3DPenaltyFn
{
  FrameTransform vol_to_app;

  size_type pelvis_obj_idx = 0;

  CoordScalar intersect_pt_on_src_det_line = 0.8;

  FrameTransformList app_wrt_cam_gts;

  FrameTransform init_app_to_cam;

  // In APP:
  // X Axis: LR
  // Y Axis: IS
  // Z Axis: AP

  CoordScalar rot_x_std_dev_rad = 30.0 * kDEG2RAD;
  CoordScalar rot_y_std_dev_rad = 15 * kDEG2RAD;
  CoordScalar rot_z_std_dev_rad = 15 * kDEG2RAD;

  CoordScalar trans_x_std_dev = 15.0;
  CoordScalar trans_y_std_dev = 15.0;
  CoordScalar trans_z_std_dev = 100.0;

  bool pat_is_right_side_up = true;

  void compute(const ListOfFrameTransformLists& cams_wrt_objs,
               const size_type num_projs,
               const CamList& cams,
               const CamAssocList& cam_assocs,
               const std::vector<bool>& intermediate_frames_wrt_vol,
               const FrameTransformList& intermediate_frames,
               const FrameTransformList& regi_xform_guesses,
               const ListOfFrameTransformLists* xforms_from_opt);
};

}  // xreg

#endif

