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

#include "xregRegi2D3DPenaltyFnRelPose.h"

#include "xregDistInterface.h"
#include "xregRigidUtils.h"
#include "xregTBBUtils.h"

void xreg::Regi2D3DPenaltyFnSE3RelPose::compute(
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
  xregASSERT(num_objs > 1);

  xregASSERT(vol_idx_1 < num_objs);
  xregASSERT(vol_idx_2 < num_objs);

  // Currently we only use these transforms directly from optimization space
  xregASSERT(xforms_from_opt && xforms_from_opt->size() == num_objs);

  // TODO: Add support for using these too
  xregASSERT(intermediate_frames_wrt_vol.size() == num_objs);
  xregASSERT(intermediate_frames.size() == num_objs);
  xregASSERT(regi_xform_guesses.size() == num_objs);

  this->reg_vals_.assign(num_projs, 0);

  // using this for intermediate storage even when the user does not need log probs
  this->log_probs_.assign(num_projs, 0);

  const auto& xforms_from_opt_vol_1 = xforms_from_opt->operator[](vol_idx_1);
  const auto& xforms_from_opt_vol_2 = xforms_from_opt->operator[](vol_idx_2);
  
  xregASSERT(num_projs <= xforms_from_opt_vol_1.size());
  xregASSERT(num_projs <= xforms_from_opt_vol_2.size());

  FrameTransform delta_xform;
  CoordScalar tmp_rot_err   = 0;
  CoordScalar tmp_trans_err = 0;
  
  const CoordScalar rot_pdf_log_norm_const   = rot_pdf->log_norm_const();
  const CoordScalar trans_pdf_log_norm_const = trans_pdf->log_norm_const();
 
  PtN tmp_dist_in(1);

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    std::tie(tmp_rot_err,tmp_trans_err) = ComputeRotAngTransMag(
                          xforms_from_opt_vol_1[proj_idx].inverse() * xforms_from_opt_vol_2[proj_idx]);
   
    tmp_dist_in[0] = tmp_rot_err;
    const CoordScalar cur_rot_log_prob = rot_pdf->log_density(tmp_dist_in);
    
    tmp_dist_in[0] = tmp_trans_err;
    const CoordScalar cur_trans_log_prob = trans_pdf->log_density(tmp_dist_in);

    this->log_probs_[proj_idx] += cur_rot_log_prob + cur_trans_log_prob;
    
    this->reg_vals_[proj_idx] += (rot_pdf_log_norm_const - cur_rot_log_prob +
                                  trans_pdf_log_norm_const - cur_trans_log_prob);
  }
}

