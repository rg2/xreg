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

#include "xregRegi2D3DPenaltyFnSE3EulerDecomp.h"

#include "xregDistInterface.h"
#include "xregRigidUtils.h"
#include "xregTBBUtils.h"

void xreg::Regi2D3DPenaltyFnSE3EulerDecomp::compute(
             const ListOfFrameTransformLists& cams_wrt_objs,
             const size_type num_projs,
             const CamList& cams,
             const CamAssocList& cam_assocs,
             const std::vector<bool>& intermediate_frames_wrt_vol,
             const FrameTransformList& intermediate_frames,
             const FrameTransformList& regi_xform_guesses,
             const ListOfFrameTransformLists* xforms_from_opt)
{
  xregASSERT(!use_xforms_from_opt || (use_xforms_from_opt && xforms_from_opt));
 
  const size_type num_objs = cams_wrt_objs.size();

  xregASSERT(obj_idx < num_objs);
  
  xregASSERT(intermediate_frames_wrt_vol.size() == num_objs);
  xregASSERT(intermediate_frames.size() == num_objs);
  xregASSERT(regi_xform_guesses.size() == num_objs);

  this->reg_vals_.assign(num_projs, 0);

  // using this for intermediate storage even when the user does not need log probs
  this->log_probs_.assign(num_projs, 0);
  
  const CoordScalar rot_x_pdf_log_norm_const = rot_x_pdf->log_norm_const();
  const CoordScalar rot_y_pdf_log_norm_const = rot_y_pdf->log_norm_const();
  const CoordScalar rot_z_pdf_log_norm_const = rot_z_pdf->log_norm_const();
  
  const CoordScalar trans_x_pdf_log_norm_const = trans_x_pdf->log_norm_const();
  const CoordScalar trans_y_pdf_log_norm_const = trans_y_pdf->log_norm_const();
  const CoordScalar trans_z_pdf_log_norm_const = trans_z_pdf->log_norm_const();
  
  {
    const auto& cur_cams_wrt_obj = use_xforms_from_opt ? xforms_from_opt->operator[](obj_idx)
                                                       : cams_wrt_objs[obj_idx];
    xregASSERT(num_projs <= cur_cams_wrt_obj.size());
    
    const bool           inter_wrt_vol   = intermediate_frames_wrt_vol[obj_idx];
    const FrameTransform inter_frame     = intermediate_frames[obj_idx];
    const FrameTransform inter_frame_inv = inter_frame.inverse();
    const FrameTransform init_cam_to_vol = regi_xform_guesses[obj_idx];
    const FrameTransform init_vol_to_cam = init_cam_to_vol.inverse();

    const FrameTransform init_X_to_inter = inter_frame_inv * (inter_wrt_vol ? init_cam_to_vol : init_vol_to_cam);

    auto compute_reg_for_projs_fn = [&] (const RangeType& r)
    {
      CoordScalar tmp_rot_x   = 0;
      CoordScalar tmp_rot_y   = 0;
      CoordScalar tmp_rot_z   = 0;
      CoordScalar tmp_trans_x = 0;
      CoordScalar tmp_trans_y = 0;
      CoordScalar tmp_trans_z = 0;

      PtN tmp_dist_in(1);

      FrameTransform xform_to_decomp;
      
      for (size_type proj_idx = r.begin(); proj_idx < r.end(); ++proj_idx)
      {
        if (use_xforms_from_opt)
        {
          xform_to_decomp = cur_cams_wrt_obj[proj_idx];
        }
        else
        {
          const auto& cur_cam_wrt_obj = cur_cams_wrt_obj[proj_idx];

          const FrameTransform cur_inter_to_X = (inter_wrt_vol ? FrameTransform(cur_cam_wrt_obj.inverse())
                                                               : cur_cam_wrt_obj) * inter_frame;
          
          xform_to_decomp = init_X_to_inter * cur_inter_to_X;
        }
        
        std::tie(tmp_rot_x,tmp_rot_y,tmp_rot_z,tmp_trans_x,tmp_trans_y,tmp_trans_z) =
                                                      RigidXformToEulerXYZAndTrans(xform_to_decomp);
      
        tmp_dist_in[0] = tmp_rot_x; 
        const CoordScalar cur_rot_x_log_prob = rot_x_pdf->log_density(tmp_dist_in);
        
        tmp_dist_in[0] = tmp_rot_y; 
        const CoordScalar cur_rot_y_log_prob = rot_y_pdf->log_density(tmp_dist_in);
        
        tmp_dist_in[0] = tmp_rot_z; 
        const CoordScalar cur_rot_z_log_prob = rot_z_pdf->log_density(tmp_dist_in);
        
        tmp_dist_in[0] = tmp_trans_x; 
        const CoordScalar cur_trans_x_log_prob = trans_x_pdf->log_density(tmp_dist_in);
        
        tmp_dist_in[0] = tmp_trans_y; 
        const CoordScalar cur_trans_y_log_prob = trans_y_pdf->log_density(tmp_dist_in);
        
        tmp_dist_in[0] = tmp_trans_z; 
        const CoordScalar cur_trans_z_log_prob = trans_z_pdf->log_density(tmp_dist_in);

        this->log_probs_[proj_idx] += cur_rot_x_log_prob   + cur_rot_y_log_prob   + cur_rot_z_log_prob +
                                      cur_trans_x_log_prob + cur_trans_y_log_prob + cur_trans_z_log_prob;
        
        this->reg_vals_[proj_idx] += (rot_x_pdf_log_norm_const   - cur_rot_x_log_prob +
                                      rot_y_pdf_log_norm_const   - cur_rot_y_log_prob +
                                      rot_z_pdf_log_norm_const   - cur_rot_z_log_prob +
                                      trans_x_pdf_log_norm_const - cur_trans_x_log_prob +
                                      trans_y_pdf_log_norm_const - cur_trans_y_log_prob +
                                      trans_z_pdf_log_norm_const - cur_trans_z_log_prob);
      }
    };

    ParallelFor(compute_reg_for_projs_fn, RangeType(0, num_projs));
  }
}
