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

#include "xregIntensity2D3DRegiHillClimb.h"

#include <fmt/format.h>

#include <opencv2/imgcodecs.hpp>

#include "xregIntensity2D3DRegiDebug.h"
#include "xregSE3OptVars.h"
#include "xregImgSimMetric2D.h"
#include "xregFilesystemUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"

xreg::Intensity2D3DRegiHillClimb::Intensity2D3DRegiHillClimb()
{
  num_step_levels_ = 1;
}

void xreg::Intensity2D3DRegiHillClimb::setup()
{
  this->num_projs_per_view_ = max_num_projs_per_view_per_iter();

  Intensity2D3DRegi::setup();
}

void xreg::Intensity2D3DRegiHillClimb::set_num_step_levels(const size_type num_step_levels)
{
  num_step_levels_ = num_step_levels;
}

void xreg::Intensity2D3DRegiHillClimb::set_init_steps(const ScalarList& init_steps)
{
  init_steps_ = init_steps;
}

void xreg::Intensity2D3DRegiHillClimb::run()
{
  xregASSERT(num_step_levels_);

  constexpr Scalar kDEFAULT_STEP_LEN = 1;

  const size_type nv = this->num_vols();

  const size_type num_params_per_xform = this->opt_vars_->num_params();

  const size_type tot_num_params = num_params_per_xform * nv;

  ScalarList cur_steps = init_steps_;
  if (cur_steps.empty())
  {
    cur_steps.assign(tot_num_params, kDEFAULT_STEP_LEN);
  }

  xregASSERT(cur_steps.size() == tot_num_params);

  const size_type num_projs_per_view = this->num_projs_per_view_;
  xregASSERT(num_projs_per_view == ((tot_num_params * 2) + 1));

  // similarity metric values computed at each iteration
  ScalarList tmp_sim_vals(num_projs_per_view, 0);

  // cur_params_to_eval[i][j][k] is the kth parameter of the jth SE(3) (projection) element for the ith object/volume
  ListOfListsOfScalarLists cur_params_to_eval(nv,
                                ListOfScalarLists(num_projs_per_view, ScalarList(num_params_per_xform)));

  cur_param_vec_.assign(tot_num_params, 0);

  cur_param_sim_val_ = 0;

  Scalar sim_val_change_neg_dir = 0;
  Scalar sim_val_change_pos_dir = 0;

  std::vector<bool> param_idx_changed(tot_num_params, false);

  size_type iter = 0;

  this->before_first_iteration();

  for (size_type cur_step_level = 0; (cur_step_level < num_step_levels_) && (iter < this->max_num_iters_);
       /* see bottom of loop for increment*/)
  {
    this->begin_of_iteration(cur_param_vec_);

    // populate the list of parameterizations to evaluate

    size_type param_off = 0;
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx, param_off += num_params_per_xform)
    {
      // the first parameterization is just the current guess
      cur_params_to_eval[vol_idx][0].assign(cur_param_vec_.begin() + param_off,
                                            cur_param_vec_.begin() + param_off + num_params_per_xform);

      // the remaining parameterizations are stepped off from the current guess
      //for (size_type param_idx = 0; param_idx < tot_num_params; ++param_idx)
      for (size_type param_idx = 0; param_idx < num_params_per_xform; ++param_idx)
      {
        const size_type off = 1 + (vol_idx * num_params_per_xform) + (param_idx * 2);

        cur_params_to_eval[vol_idx][off].assign(cur_param_vec_.begin() + param_off,
                                                cur_param_vec_.begin() + param_off + num_params_per_xform);
        cur_params_to_eval[vol_idx][off][param_idx] -= cur_steps[param_idx];

        cur_params_to_eval[vol_idx][off + 1].assign(cur_param_vec_.begin() + param_off,
                                                    cur_param_vec_.begin() + param_off + num_params_per_xform);
        cur_params_to_eval[vol_idx][off + 1][param_idx] += cur_steps[param_idx];
      }
    }

    // compute DRRs and similarity metrics
    this->obj_fn(cur_params_to_eval, &tmp_sim_vals);

    // similarity value for the current guess
    cur_param_sim_val_ = tmp_sim_vals[0];

    // keep track of parameters that are updated, when none are the steps
    // are halved
    param_idx_changed.assign(tot_num_params, false);

    // update the parameterization
    for (size_type param_idx = 0; param_idx < tot_num_params; ++param_idx)
    {
      const size_type off = 1 + (param_idx * 2);

      sim_val_change_neg_dir = cur_param_sim_val_ - tmp_sim_vals[off];
      sim_val_change_pos_dir = cur_param_sim_val_ - tmp_sim_vals[off + 1];

      // we are minimizing so we're looking for large positive changes
      // (according to the difference above)

      const bool use_neg = sim_val_change_neg_dir > sim_val_change_pos_dir;

      const Scalar sim_val_change_to_use = use_neg ? sim_val_change_neg_dir : sim_val_change_pos_dir;

      // must be a positive change (similarity value decreased) if the parameter
      // is to be updated
      if (sim_val_change_to_use > 1.0e-6)
      {
        // TODO: make the scaling by the amount of similarity value decrease optional
        cur_param_vec_[param_idx] += (use_neg ? -1 : 1) * cur_steps[param_idx] /* (sim_val_change_to_use / std::abs(cur_param_sim_val_))*/;

        param_idx_changed[param_idx] = true;
      }
    }

    // check to see if any params have been updated, if so we continue at this
    // level, otherwise, the steps are halved.
    if (std::find(param_idx_changed.begin(), param_idx_changed.end(), true) == param_idx_changed.end())
    {
      for (size_type param_idx = 0; param_idx < tot_num_params; ++param_idx)
      {
        cur_steps[param_idx] *= 0.5;
      }

      ++cur_step_level;
    }

    this->end_of_iteration();
    
    ++iter;
  }

  this->after_last_iteration();

  FrameTransformList delta_xforms(nv);
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    delta_xforms[vol_idx] = this->opt_vars_->operator()(
      Eigen::Map<PtN>(&cur_param_vec_[0] + (vol_idx * num_params_per_xform),
                      num_params_per_xform));
  }

  this->update_regi_xforms(delta_xforms);

  if (this->src_and_obj_pose_opt_vars_)
  {
    // update regi cams
    this->regi_cam_models_.assign(1,
        this->src_and_obj_pose_opt_vars_->cam(Eigen::Map<PtN>(&cur_param_vec_[0], num_params_per_xform))); 
  }
}

xreg::size_type xreg::Intensity2D3DRegiHillClimb::max_num_projs_per_view_per_iter() const
{
  return (this->opt_vars_->num_params() * this->num_vols() * 2) + 1;
}

void xreg::Intensity2D3DRegiHillClimb::init_opt()
{
  // nothing required here
}

void xreg::Intensity2D3DRegiHillClimb::debug_write_remapped_drrs()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("drr_remap_{:03d}_{:03d}.png", this->num_obj_fn_evals_, view_idx);

    WriteITKImageRemap8bpp(this->ray_caster_->proj(view_idx * this->num_projs_per_view_).GetPointer(),
                           dst_path.string());
  }
}

void xreg::Intensity2D3DRegiHillClimb::debug_write_raw_drrs()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("drr_raw_{:03d}_{:03d}.nii.gz", this->num_obj_fn_evals_, view_idx);

    WriteITKImageToDisk(this->ray_caster_->proj(view_idx * this->num_projs_per_view_).GetPointer(),
                        dst_path.string());
  }
}

void xreg::Intensity2D3DRegiHillClimb::debug_write_fixed_img_edge_overlays()
{
  const size_type num_sim_metrics = this->sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    auto cur_proj = this->ray_caster_->proj(view_idx * this->num_projs_per_view_);

    const auto img_2d_size = cur_proj->GetLargestPossibleRegion().GetSize();

    cv::Mat edge_img(img_2d_size[1], img_2d_size[0], cv::DataType<unsigned char>::type);
    RemapAndComputeEdges(cur_proj.GetPointer(), &edge_img, 12, 75);

    auto fixed_u8 = ITKImageRemap8bpp(this->sim_metrics_[view_idx]->fixed_image().GetPointer());

    cv::Mat edge_overlay = OverlayEdges(ShallowCopyItkToOpenCV(fixed_u8.GetPointer()), edge_img, 1 /* 1 -> green edges */);

    Path dst_path = this->debug_output_dir_path_;
    dst_path += fmt::format("edges_{:03d}_{:03d}.png", this->num_obj_fn_evals_, view_idx);
    cv::imwrite(dst_path.string(), edge_overlay);
  }
}

void xreg::Intensity2D3DRegiHillClimb::debug_write_comb_sim_score()
{
  if (this->debug_save_iter_debug_info_)
  {
    this->dout() << fmt::format("{:+20.6f}\n", cur_param_sim_val_);
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    this->debug_info_->sims.push_back(cur_param_sim_val_);
  }
}

void xreg::Intensity2D3DRegiHillClimb::debug_write_opt_pose_vars()
{
  const size_type nv = this->num_vols();
  const size_type num_params_per_xform = this->opt_vars_->num_params();

  if (this->write_opt_vars_to_stream_)
  {
    size_type param_idx = 0;

    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      for (; param_idx < num_params_per_xform; ++param_idx)
      {
        this->dout() << fmt::format("{:+14.6f}, ", cur_param_vec_[param_idx]);
      }
      this->dout() << "| ";
    }
    this->dout() << '\n';
  }

  if (this->debug_save_iter_debug_info_ && this->num_obj_fn_evals_)
  {
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      this->debug_info_->iter_vars[vol_idx].push_back(
            Eigen::Map<PtN>(const_cast<Scalar*>(&cur_param_vec_[vol_idx * num_params_per_xform]),
            num_params_per_xform));
    }
  }
}
