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

#include "xregRegi2D3DPenaltyFnSE3Mag.h"

#include "xregPerspectiveXform.h"
#include "xregDistInterface.h"
#include "xregRigidUtils.h"
#include "xregTBBUtils.h"
 
void xreg::Regi2D3DPenaltyFnSE3Mag::compute(
             const ListOfFrameTransformLists& cams_wrt_objs,
             const size_type num_projs,
             const CamList& cams,
             const CamAssocList& cam_assocs,
             const std::vector<bool>& intermediate_frames_wrt_vol,
             const FrameTransformList& intermediate_frames,
             const FrameTransformList& regi_xform_guesses,
             const ListOfFrameTransformLists* xforms_from_opt)
{
  const size_type num_objs = cams_wrt_objs.size();

  xregASSERT(intermediate_frames_wrt_vol.size() == num_objs);
  xregASSERT(intermediate_frames.size() == num_objs);
  xregASSERT(regi_xform_guesses.size() == num_objs);

  xregASSERT(num_objs == rot_pdfs_per_obj.size());
  xregASSERT(num_objs == trans_pdfs_per_obj.size());

  const bool apply_reg_for_all = apply_reg_for_obj.empty();
  xregASSERT(apply_reg_for_all || (apply_reg_for_obj.size() == num_objs));

  this->reg_vals_.assign(num_projs, 0);

  // using this for intermediate storage even when the user does not need log probs
  this->log_probs_.assign(num_projs, 0);

  for (size_type obj_idx = 0; obj_idx < num_objs; ++obj_idx)
  {
    if (apply_reg_for_all || apply_reg_for_obj[obj_idx])
    {
      const auto& cur_cams_wrt_obj = cams_wrt_objs[obj_idx];
      xregASSERT(num_projs <= cur_cams_wrt_obj.size());
      
      auto&   rot_pdf = *rot_pdfs_per_obj[obj_idx];
      auto& trans_pdf = *trans_pdfs_per_obj[obj_idx];

      const bool inter_wrt_vol = intermediate_frames_wrt_vol[obj_idx];
    
      const FrameTransform& inter_frame     = intermediate_frames[obj_idx];
      const FrameTransform  inter_frame_inv = inter_frame.inverse();
      const FrameTransform& init_cam_to_vol = regi_xform_guesses[obj_idx];
      const FrameTransform  init_vol_to_cam = init_cam_to_vol.inverse();

      const FrameTransform init_X_to_inter = inter_frame_inv * (inter_wrt_vol ? init_cam_to_vol : init_vol_to_cam);

      auto compute_reg_for_projs_fn = [&] (const RangeType& r)
      {
        CoordScalar tmp_rot_err   = 0;
        CoordScalar tmp_trans_err = 0;
        
        PtN tmp_dist_in(1);

        for (size_type proj_idx = r.begin(); proj_idx < r.end(); ++proj_idx)
        {
          const auto& cur_cam_wrt_obj = cur_cams_wrt_obj[proj_idx];

          const FrameTransform cur_inter_to_X = (inter_wrt_vol ? FrameTransform(cur_cam_wrt_obj.inverse())
                                                               : cur_cam_wrt_obj) * inter_frame;

          std::tie(tmp_rot_err,tmp_trans_err) = ComputeRotAngTransMag(init_X_to_inter * cur_inter_to_X);
         
          tmp_dist_in[0] = tmp_rot_err;
          const CoordScalar cur_rot_log_prob = rot_pdf.log_density(tmp_dist_in);
         
          tmp_dist_in[0] = tmp_trans_err;
          const CoordScalar cur_trans_log_prob = trans_pdf.log_density(tmp_dist_in);

          this->log_probs_[proj_idx] += cur_rot_log_prob + cur_trans_log_prob;
          
          this->reg_vals_[proj_idx] += (rot_pdf.log_norm_const() - cur_rot_log_prob +
                                        trans_pdf.log_norm_const() - cur_trans_log_prob);
        }
      };
      
      ParallelFor(compute_reg_for_projs_fn, RangeType(0, num_projs));
    }
  }
}

