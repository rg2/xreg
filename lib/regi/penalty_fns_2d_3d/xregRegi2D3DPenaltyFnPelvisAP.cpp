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

#include "xregRegi2D3DPenaltyFnPelvisAP.h"

#include "xregAnatCoordFrames.h"
#include "xregRigidUtils.h"
#include "xregTBBUtils.h"

void xreg::Regi2D3DPenaltyFnPelvisAP::compute(
             const ListOfFrameTransformLists& cams_wrt_objs,
             const size_type num_projs,
             const CamList& cams,
             const CamAssocList& cam_assocs,
             const std::vector<bool>& intermediate_frames_wrt_vol,
             const FrameTransformList& intermediate_frames,
             const FrameTransformList& regi_xform_guesses,
             const ListOfFrameTransformLists* xforms_from_opt)
{
  const FrameTransformList& cam_wrt_pelvis_vols = cams_wrt_objs[pelvis_obj_idx];

  const size_type num_cams = cams.size();

  xregASSERT(num_projs <= cam_wrt_pelvis_vols.size());
  this->reg_vals_.resize(num_projs);

  app_wrt_cam_gts.resize(num_cams);
  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    app_wrt_cam_gts[cam_idx] = CreateAPViewOfAPP(cams[cam_idx], intersect_pt_on_src_det_line,
                                                 true, pat_is_right_side_up).inverse();
  }

  const CoordScalar rot_x_std_dev_sq = rot_x_std_dev_rad * rot_x_std_dev_rad;
  const CoordScalar rot_y_std_dev_sq = rot_y_std_dev_rad * rot_y_std_dev_rad;
  const CoordScalar rot_z_std_dev_sq = rot_z_std_dev_rad * rot_z_std_dev_rad;

  const CoordScalar trans_x_std_dev_sq = trans_x_std_dev * trans_x_std_dev;
  const CoordScalar trans_y_std_dev_sq = trans_y_std_dev * trans_y_std_dev;
  const CoordScalar trans_z_std_dev_sq = trans_z_std_dev * trans_z_std_dev;

  auto compute_reg_for_projs_fn = [&] (const RangeType& r)
  {
    CoordScalar rot_x   = 0;
    CoordScalar rot_y   = 0;
    CoordScalar rot_z   = 0;
    CoordScalar trans_x = 0;
    CoordScalar trans_y = 0;
    CoordScalar trans_z = 0;
    
    for (size_type i = r.begin(); i < r.end(); ++i)
    {
      const FrameTransform cur_cam_to_app = vol_to_app * cam_wrt_pelvis_vols[i];

      const FrameTransform cur_delta_for_rot = cur_cam_to_app * app_wrt_cam_gts[cam_assocs[i]];
      
      std::tie(rot_x,rot_y,rot_z,trans_x,trans_y,trans_z) = RigidXformToEulerXYZAndTrans(cur_delta_for_rot);

      const FrameTransform cur_delta_for_trans = cur_cam_to_app * init_app_to_cam;
      
      trans_x = cur_delta_for_trans.matrix()(0,3);
      trans_y = cur_delta_for_trans.matrix()(1,3);
      trans_z = cur_delta_for_trans.matrix()(2,3);

      this->reg_vals_[i] = CoordScalar(0.5) *
                            (((rot_x * rot_x) / rot_x_std_dev_sq) +
                             ((rot_y * rot_y) / rot_y_std_dev_sq) +
                             ((rot_z * rot_z) / rot_z_std_dev_sq) +
                             ((trans_x * trans_x) / trans_x_std_dev_sq) +
                             ((trans_y * trans_y) / trans_y_std_dev_sq) +
                             ((trans_z * trans_z) / trans_z_std_dev_sq));
    }
  };

  ParallelFor(compute_reg_for_projs_fn, RangeType(0,num_projs));

  if (this->compute_probs_)
  {
    constexpr CoordScalar two_pi = 2.0 * 3.141592653589793;
    
    const CoordScalar log_norm_const = std::log(two_pi * two_pi * two_pi *
                                            rot_x_std_dev_rad * rot_y_std_dev_rad * rot_z_std_dev_rad *
                                            trans_x_std_dev   * trans_y_std_dev   * trans_z_std_dev);

    this->log_probs_.resize(num_projs);

    for (size_type i = 0; i < num_projs; ++i)
    {
      this->log_probs_[i] = -this->reg_vals_[i] - log_norm_const;
    }
  }
}
