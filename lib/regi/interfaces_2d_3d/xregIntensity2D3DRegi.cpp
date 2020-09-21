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

#include "xregIntensity2D3DRegi.h"

#include <fmt/format.h>

#include <opencv2/imgcodecs.hpp>

#include "xregAssert.h"
#include "xregImgSimMetric2D.h"
#include "xregSE3OptVars.h"
#include "xregImgSimMetric2DCombine.h"
#include "xregRegi2D3DPenaltyFn.h"
#include "xregIntensity2D3DRegiDebug.h"
#include "xregFilesystemUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregTBBUtils.h"

void xreg::Intensity2D3DRegi::setup()
{
  xregASSERT(bool(opt_vars_));

  const size_type num_views = sim_metrics_.size();
  
  // currently I can only support a single view (single source) and single object
  // when optimizing over the source position
  xregASSERT(!src_and_obj_pose_opt_vars_ ||
             (src_and_obj_pose_opt_vars_ && (num_views == 1) && (num_vols() == 1)));

  const size_type num_cams = ray_caster_->num_camera_models();
  const size_type tot_num_projs = num_projs_per_view_ * num_views;

  // Assert that we are not optimizing over camera models and the number of cameras is equal to the number of views
  //  -OR-
  // we are optimizing over camera models, and their is a separate camera model for each view
  xregASSERT((!src_and_obj_pose_opt_vars_ && (num_cams == num_views) ||
             (src_and_obj_pose_opt_vars_ && (num_cams == tot_num_projs))));

  ray_caster_->set_num_projs(tot_num_projs);

  if (need_to_alloc_ray_caster_)
  {
    // all other parameters should have already been set.
    ray_caster_->allocate_resources();

    need_to_alloc_ray_caster_ = false;
  }

  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    sim_metrics_[view_idx]->set_save_aux_info(this->sim_metric_debug_save_info_ &&
                                              this->debug_save_iter_debug_info_);
    
    sim_metrics_[view_idx]->set_num_moving_images(num_projs_per_view_);

    // This assumes a view-major ordering of projections in memory
    sim_metrics_[view_idx]->set_mov_imgs_buf_from_ray_caster(ray_caster_.get(), num_projs_per_view_ * view_idx);
  }
  need_to_setup_sim_combiner_ = true;  // This variable has now become useless - TODO: remove!

  if (need_to_alloc_sim_metrics_)
  {
    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      // all other parameters should have already been set.
      sim_metrics_[view_idx]->allocate_resources();
    }

    need_to_alloc_sim_metrics_  = false;
  }

  if (need_to_setup_sim_combiner_)
  {
    sim_metric_combiner_ = std::make_shared<ImgSimMetric2DCombineMean>();
    sim_metric_combiner_->set_num_sim_metrics(num_views);
    sim_metric_combiner_->set_num_projs_per_sim_metric(num_projs_per_view_);
    sim_metric_combiner_->allocate_resources();

    for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
    {
      sim_metric_combiner_->set_sim_metric(view_idx, sim_metrics_[view_idx].get());
    }

    need_to_setup_sim_combiner_ = false;
  }

  if (need_to_init_opt_)
  {
    init_opt();
    need_to_init_opt_ = false;
  }

  num_obj_fn_evals_ = 0;

  tmp_frame_xforms_.assign(num_vols(), FrameTransformList(num_projs_per_view_));

  regi_xforms_.assign(num_vols(), FrameTransform::Identity());
  regi_xform_guesses_.assign(num_vols(), FrameTransform::Identity());

  intermediate_frames_.assign(num_vols(), FrameTransform::Identity());
  intermediate_frames_wrt_vol_.assign(num_vols(), kDEFAULT_INTER_FRAMES_WRT_VOL);

  dyn_ref_frame_fns_.assign(num_vols(), nullptr);

  if (src_and_obj_pose_opt_vars_)
  {
    tmp_cam_models_.resize(num_cams);
  }
}
  
xreg::size_type xreg::Intensity2D3DRegi::num_vols() const
{
  return vol_inds_in_ray_caster_.size();
}

void xreg::Intensity2D3DRegi::set_multi_vol_inds_in_ray_caster(const IndexList& vol_inds)
{
  vol_inds_in_ray_caster_ = vol_inds;
  num_vols_updated();
}

void xreg::Intensity2D3DRegi::set_single_vol_ind_in_ray_caster(const size_type vol_idx)
{
  vol_inds_in_ray_caster_.assign(1, vol_idx);
  num_vols_updated();
}

void xreg::Intensity2D3DRegi::set_use_all_vols_in_ray_caster()
{
  const size_type nv = ray_caster_->num_vols();

  vol_inds_in_ray_caster_.resize(nv);

  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    vol_inds_in_ray_caster_[vol_idx] = vol_idx;
  }

  num_vols_updated();
}

xreg::FrameTransform xreg::Intensity2D3DRegi::regi_xform(const size_type vol_idx) const
{
  return regi_xforms_[vol_idx];
}

void xreg::Intensity2D3DRegi::set_regi_xform_guess(const FrameTransform& regi_xform_guess,
                                                   const size_type vol_idx)
{
  if (vol_idx == kNPOS)
  {
    std::fill(regi_xform_guesses_.begin(), regi_xform_guesses_.end(), regi_xform_guess);
  }
  else
  {
    regi_xform_guesses_[vol_idx] = regi_xform_guess;
  }
}
  
const xreg::CameraModel& xreg::Intensity2D3DRegi::regi_cam() const
{
  xregASSERT(src_and_obj_pose_opt_vars_ && !regi_cam_models_.empty());
  return regi_cam_models_[0];
}

void xreg::Intensity2D3DRegi::set_intermediate_frame(const FrameTransform& inter_frame,
                                                     const bool wrt_vol,
                                                     const size_type vol_idx)
{
  if (vol_idx == kNPOS)
  {
    std::fill(intermediate_frames_.begin(), intermediate_frames_.end(), inter_frame);
    std::fill(intermediate_frames_wrt_vol_.begin(), intermediate_frames_wrt_vol_.end(), wrt_vol);
    
    dyn_ref_frame_fns_.assign(intermediate_frames_.size(), nullptr);
  }
  else
  {
    intermediate_frames_[vol_idx]         = inter_frame;
    intermediate_frames_wrt_vol_[vol_idx] = wrt_vol;
    
    dyn_ref_frame_fns_[vol_idx] = nullptr;
  }
}

void xreg::Intensity2D3DRegi::set_intermediate_frame(DynRefFrameFn dyn_ref_frame_fn,
                                                     const size_type vol_idx)
{
  dyn_ref_frame_fns_[vol_idx] = dyn_ref_frame_fn;
}

void xreg::Intensity2D3DRegi::set_ray_caster(RayCasterPtr ray_caster,
                                             const size_type vol_idx,
                                             const bool need_to_alloc)
{
  ray_caster_ = ray_caster;
  need_to_alloc_ray_caster_ = need_to_alloc;

  if (vol_idx == kNPOS)
  {
    // use all available volumes

    const size_type num_vols = ray_caster->num_vols();

    vol_inds_in_ray_caster_.resize(num_vols);
    for (size_type v = 0; v < num_vols; ++v)
    {
      vol_inds_in_ray_caster_[v] = v;
    }
  }
  else
  {
    xregASSERT(vol_idx < ray_caster->num_vols());
    vol_inds_in_ray_caster_.assign(1, vol_idx);
  }

  num_vols_updated();
}

void xreg::Intensity2D3DRegi::set_ray_caster(RayCasterPtr ray_caster,
                                             const IndexList& vol_inds,
                                             const bool need_to_alloc)
{
  ray_caster_ = ray_caster;
  need_to_alloc_ray_caster_ = need_to_alloc;

  vol_inds_in_ray_caster_ = vol_inds;

  num_vols_updated();
}
 
xreg::Intensity2D3DRegi::RayCasterPtr xreg::Intensity2D3DRegi::ray_caster()
{
  return ray_caster_;
}

void xreg::Intensity2D3DRegi::set_sim_metric(SimMetricPtr sim_metric,
                                             const bool need_to_alloc)
{
  sim_metrics_.assign(1, sim_metric);
  need_to_alloc_sim_metrics_  = need_to_alloc;
  need_to_setup_sim_combiner_ = true;
}

void xreg::Intensity2D3DRegi::set_sim_metrics(const SimMetricList& sim_metrics,
                                              const bool need_to_alloc)
{
  sim_metrics_ = sim_metrics;
  need_to_alloc_sim_metrics_  = need_to_alloc;
  need_to_setup_sim_combiner_ = true;
}
  
xreg::Intensity2D3DRegi::SimMetricList& xreg::Intensity2D3DRegi::sim_metrics()
{
  return sim_metrics_;
}

const xreg::Intensity2D3DRegi::SimMetricList& xreg::Intensity2D3DRegi::sim_metrics() const
{
  return sim_metrics_;
}

xreg::Intensity2D3DRegi::SimMetricPtr xreg::Intensity2D3DRegi::sim_metric(const size_type view_idx)
{
  xregASSERT(view_idx < sim_metrics_.size());
  return sim_metrics_[view_idx];
}

void xreg::Intensity2D3DRegi::use_ray_caster_from_other_regi(Intensity2D3DRegi* other_regi)
{
  ray_caster_ = other_regi->ray_caster_;
  need_to_alloc_ray_caster_ = false;
  vol_inds_in_ray_caster_ = other_regi->vol_inds_in_ray_caster_;

  num_vols_updated();
}

void xreg::Intensity2D3DRegi::use_ray_caster_and_sim_metrics_from_other_regi(Intensity2D3DRegi* other_regi)
{
  use_ray_caster_from_other_regi(other_regi);

  sim_metrics_ = other_regi->sim_metrics_;
  need_to_alloc_sim_metrics_  = false;
  need_to_setup_sim_combiner_ = true;
}

void xreg::Intensity2D3DRegi::set_opt_vars(SE3OptVarsPtr opt_vars)
{
  opt_vars_ = opt_vars;
  need_to_init_opt_ = true;

  src_and_obj_pose_opt_vars_ = dynamic_cast<const CamSourceObjPoseOptVars*>(opt_vars_.get());  
}
  
void xreg::Intensity2D3DRegi::set_opt_obj_fn_tol(const Scalar& tol)
{ }

void xreg::Intensity2D3DRegi::set_opt_x_tol(const Scalar& tol)
{ }

void xreg::Intensity2D3DRegi::set_debug_output_dir_path(const std::string& debug_output_dir_path)
{
  debug_output_dir_path_ = debug_output_dir_path;
}

void xreg::Intensity2D3DRegi::set_debug_write_remapped_drrs(const bool write_remapped_drrs)
{
  write_remapped_drrs_ = write_remapped_drrs;
}

void xreg::Intensity2D3DRegi::set_debug_write_raw_drrs(const bool write_raw_drrs)
{
  write_raw_drrs_ = write_raw_drrs;
}

void xreg::Intensity2D3DRegi::set_debug_write_fixed_img_edge_overlays(const bool write_fixed_img_edge_overlays)
{
  write_fixed_img_edge_overlays_ = write_fixed_img_edge_overlays;
}

void xreg::Intensity2D3DRegi::set_debug_write_combined_sim_scores_to_stream(
                                        const bool write_combined_sim_scores_to_stream)
{
  write_combined_sim_scores_to_stream_ = write_combined_sim_scores_to_stream;
}

void xreg::Intensity2D3DRegi::set_debug_write_opt_vars_to_stream(const bool write_opt_vars_to_stream)
{
  write_opt_vars_to_stream_ = write_opt_vars_to_stream;
}

void xreg::Intensity2D3DRegi::debug_compute_drrs_single_proj_per_view(
                                  const FrameTransformList& xforms,
                                  ProjList* projs,
                                  const bool intermediate_frame,
                                  const CamModelList* cams)
{
  // TODO: replace this code with a call to the function object:
  //       SimpleRayCasterWrapperFn

  const size_type nv = num_vols();

  const size_type num_views = sim_metrics_.size();

  projs->resize(num_views);

  // save off some state from the ray caster
  auto cam_assocs = ray_caster_->camera_model_proj_associations();
  CamModelList orig_cams;
  if (cams)
  {
    xregASSERT(cams->size() == num_views);

    orig_cams = ray_caster_->camera_models();
    ray_caster_->set_camera_models(*cams);
  }

  // now
  ray_caster_->set_num_projs(num_views);

  const bool orig_ray_caster_use_bg_projs = ray_caster_->use_bg_projs();

  // Perform the ray casting of multiple, or one, object(s), by looping over
  // each volume, setting the pose parameters for the current volume, and
  // ray casting for the current volume. The projection buffers are initialized
  // for the first projection and then accumulated afterwards.
  
  if (has_a_static_vol_)
  {
    ray_caster_->set_use_bg_projs(true);
  }
  ray_caster_->use_proj_store_replace_method();
  
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    ray_caster_->distribute_xform_among_cam_models(xforms[vol_idx]);

    if (intermediate_frame)
    {
      // TODO: use dyn_ref_frame_fns_

      if (intermediate_frames_wrt_vol_[vol_idx])
      {
        ray_caster_->pre_multiply_all_xforms(intermediate_frames_[vol_idx]);
        ray_caster_->post_multiply_all_xforms(intermediate_frames_[vol_idx].inverse() * regi_xform_guesses_[vol_idx]);
      }
      else
      {
        ray_caster_->pre_multiply_all_xforms(regi_xform_guesses_[vol_idx] * intermediate_frames_[vol_idx]);
        ray_caster_->post_multiply_all_xforms(intermediate_frames_[vol_idx].inverse());
      }
    }

    ray_caster_->compute(vol_inds_in_ray_caster_[vol_idx]);
    
    if (has_a_static_vol_)
    {
      ray_caster_->set_use_bg_projs(false);
    }
    
    ray_caster_->use_proj_store_accum_method();
  }

  // copy the projections
  for (size_type view_idx = 0; view_idx < num_views; ++view_idx)
  {
    projs->at(view_idx) = ray_caster_->proj(view_idx);
  }

  // restore the previous ray caster state
  ray_caster_->set_num_projs(num_projs_per_view_ * num_views);
  if (cams)
  {
    ray_caster_->set_camera_models(orig_cams);
  }
  ray_caster_->set_camera_model_proj_associations(cam_assocs);

  if (has_a_static_vol_)
  {
    ray_caster_->set_use_bg_projs(orig_ray_caster_use_bg_projs);
  }
}
  
void xreg::Intensity2D3DRegi::debug_compute_drrs_single_proj_per_view(
                                               const FrameTransform& xform,
                                               ProjList* projs,
                                               const bool intermediate_frame,
                                               const CamModelList* cams)
{
  FrameTransformList xforms(num_vols(), xform);

  debug_compute_drrs_single_proj_per_view(xforms, projs, intermediate_frame, cams);
}

void xreg::Intensity2D3DRegi::set_debug_save_iter_debug_info(const bool save_iter_info)
{
  debug_save_iter_debug_info_ = save_iter_info;
}

void xreg::Intensity2D3DRegi::set_sim_metric_debug_save_info(const bool save_sm_info)
{
  sim_metric_debug_save_info_ = save_sm_info;
}
  
xreg::Intensity2D3DRegi::SingleRegiDebugResultsPtr
xreg::Intensity2D3DRegi::debug_info() const
{
  return debug_info_;
}
  
void xreg::Intensity2D3DRegi::set_penalty_fn(PenaltyFnPtr penalty_fn)
{
  penalty_fn_ = penalty_fn;
}

xreg::Intensity2D3DRegi::PenaltyFnPtr xreg::Intensity2D3DRegi::penalty_fn()
{
  return penalty_fn_;
}

// This should only be called after setup
void xreg::Intensity2D3DRegi::set_img_sim_penalty_coefs(const Scalar img_sim_coeff,
                                                        const Scalar penalty_fn_coeff)
{
  coeffs_img_sim_.assign(num_projs_per_view_,     img_sim_coeff);
  coeffs_penalty_fns_.assign(num_projs_per_view_, penalty_fn_coeff);
}

void xreg::Intensity2D3DRegi::set_img_sim_penalty_coefs(const ScalarList& img_sim_coeffs,
                                                        const ScalarList& penalty_fn_coeffs)
{
  xregASSERT(num_projs_per_view_ == img_sim_coeffs.size());
  xregASSERT(num_projs_per_view_ == penalty_fn_coeffs.size());

  coeffs_img_sim_     = img_sim_coeffs;
  coeffs_penalty_fns_ = penalty_fn_coeffs;
}

bool xreg::Intensity2D3DRegi::include_penalty_in_obj_fn() const
{
  return include_penalty_in_obj_fn_;
}

void xreg::Intensity2D3DRegi::set_include_penalty_in_obj_fn(const bool b)
{
  include_penalty_in_obj_fn_ = b;
}

const std::vector<bool>&
xreg::Intensity2D3DRegi::intermediate_frames_wrt_vol() const
{
  return intermediate_frames_wrt_vol_;
}

const xreg::FrameTransformList&
xreg::Intensity2D3DRegi::intermediate_frames() const
{
  return intermediate_frames_;
}

const xreg::FrameTransformList&
xreg::Intensity2D3DRegi::regi_xform_guesses() const
{
  return regi_xform_guesses_;
}

void xreg::Intensity2D3DRegi::add_begin_of_iter_callback(CallbackFn& fn)
{
  begin_of_iter_fns_.push_back(fn);
}

void xreg::Intensity2D3DRegi::reset_begin_of_iter_callbacks()
{
  begin_of_iter_fns_.clear();
}

void xreg::Intensity2D3DRegi::add_end_of_iter_callback(CallbackFn& fn)
{
  end_of_iter_fns_.push_back(fn);
}

void xreg::Intensity2D3DRegi::reset_end_of_iter_callbacks()
{
  end_of_iter_fns_.clear();
}

xreg::size_type xreg::Intensity2D3DRegi::max_num_iters() const
{
  return max_num_iters_;
}

void xreg::Intensity2D3DRegi::set_max_num_iters(const size_type max_iters)
{
  max_num_iters_ = max_iters;
}

bool xreg::Intensity2D3DRegi::has_a_static_vol() const
{
  return has_a_static_vol_;
}

void xreg::Intensity2D3DRegi::set_has_a_static_vol(const bool b)
{
  has_a_static_vol_ = b;
}

void xreg::Intensity2D3DRegi::obj_fn(
                    const ListOfFrameTransformLists& frame_xforms_per_object,
                    const CamModelList* cams_per_proj,
                    ScalarList* sim_vals_ptr)
{
  const bool compute_penalty = penalty_fn_.get();

  const size_type nv = num_vols();

  const auto inter_frame_xforms = apply_inter_transforms_for_obj_fn(frame_xforms_per_object);

  if (cams_per_proj)
  {  
    ray_caster_->set_camera_models(*cams_per_proj);
  }
    
  const bool orig_ray_caster_use_bg_projs = ray_caster_->use_bg_projs();

  // Perform the ray casting of multiple, or one, object(s), by looping over
  // each volume, setting the pose parameters for the current volume, and
  // ray casting for the current volume. The projection buffers are initialized
  // for the first projection and then accumulated afterwards.
  
  if (has_a_static_vol_)
  {
    ray_caster_->set_use_bg_projs(true);
  }
  ray_caster_->use_proj_store_replace_method();
  
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    if (!cams_per_proj)
    {
      // camera models are constant (1 per view), distribute transforms amongst cameras
      ray_caster_->distribute_xforms_among_cam_models(inter_frame_xforms[vol_idx]);
    }
    else
    {
      const CamModelList& cams = *cams_per_proj;
      xregASSERT(num_projs_per_view_ == cams.size());

      // create a separate camera model for each projection
      for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
      {
        ray_caster_->set_proj_cam_model(proj_idx, proj_idx);

        ray_caster_->xform_cam_to_itk_phys(proj_idx) = inter_frame_xforms[vol_idx][proj_idx];
      }
    }

    ray_caster_->compute(vol_inds_in_ray_caster_[vol_idx]);
      
    if (has_a_static_vol_)
    {
      ray_caster_->set_use_bg_projs(false);
    }
    
    ray_caster_->use_proj_store_accum_method();
  }

  // this is equivalent to the number of views
  const size_type num_sim_metrics = sim_metrics_.size();

  // e.g. for each view, compute the similarity scores for each candidate projection
  for (size_type sim_idx = 0; sim_idx < num_sim_metrics; ++sim_idx)
  {
    sim_metrics_[sim_idx]->compute();
  }

  // combine the similarity scores for each candidate projection over all of
  // the views
  ScalarList& sim_vals = *sim_vals_ptr;
  xregASSERT(sim_vals.size() == num_projs_per_view_);

  sim_metric_combiner_->compute();

  for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
  {
    sim_vals[proj_idx] = sim_metric_combiner_->sim_val(proj_idx);
  }

  // Handle regularization if it has been specified
  if (compute_penalty)
  {
    penalty_fn_->compute(inter_frame_xforms, num_projs_per_view_,
                         ray_caster_->camera_models(),
                         ray_caster_->camera_model_proj_associations(),
                         intermediate_frames_wrt_vol_,
                         intermediate_frames_,
                         regi_xform_guesses_,
                         &frame_xforms_per_object);
    
    if (include_penalty_in_obj_fn_)
    {
      auto penalty_vals = penalty_fn_->reg_vals();

      if (!coeffs_img_sim_.empty())
      {
        for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
        {
          sim_vals[proj_idx] *= coeffs_img_sim_[proj_idx];
        }
      }
      
      if (!coeffs_penalty_fns_.empty())
      {
        for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
        {
          penalty_vals[proj_idx] *= coeffs_penalty_fns_[proj_idx];
        }
      }
      
      for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
      {
        sim_vals[proj_idx] += penalty_vals[proj_idx];
      }
    }
  }

  if (has_a_static_vol_)
  {
    ray_caster_->set_use_bg_projs(orig_ray_caster_use_bg_projs);
  }

  ++num_obj_fn_evals_;
}

void xreg::Intensity2D3DRegi::obj_fn(
                    const ListOfListsOfScalarLists& opt_vec_space_vals,
                    ScalarList* sim_vals_ptr)
{
  const SE3OptVars& opt_vars = *opt_vars_;

  const size_type num_params_per_xform = opt_vars.num_params();

  const size_type nv = num_vols();

  // Map from optimization vector space to rigid transformation parameterizations and
  // camera models
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    tmp_frame_xforms_[vol_idx].resize(num_projs_per_view_);

    // compute frame transformations 
    for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
    {
      tmp_frame_xforms_[vol_idx][proj_idx] =
        opt_vars(Eigen::Map<PtN>(const_cast<Scalar*>(&opt_vec_space_vals[vol_idx][proj_idx][0]),
                                         num_params_per_xform));
    }
  }
  
  if (!src_and_obj_pose_opt_vars_)
  {
    obj_fn(tmp_frame_xforms_, nullptr, sim_vals_ptr);
  }
  else
  {
    // this is a current limitation of this implementation
    xregASSERT(nv == 1);

    // create a separate camera model for each projection
    for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
    {
      tmp_cam_models_[proj_idx] = this->src_and_obj_pose_opt_vars_->cam(
          Eigen::Map<PtN>(const_cast<Scalar*>(&opt_vec_space_vals[0][proj_idx][0]),
                                         num_params_per_xform));
    }

    obj_fn(tmp_frame_xforms_, &tmp_cam_models_, sim_vals_ptr);
  }
}
  
void xreg::Intensity2D3DRegi::before_first_iteration()
{
  num_obj_fn_evals_ = 0;
  write_debug();

  if (debug_save_iter_debug_info_)
  {
    debug_info_ = std::make_shared<SingleRegiDebugResults>();

    debug_info_->init(opt_vars_, vol_inds_in_ray_caster_);

    const size_type nv = num_vols();

    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      debug_info_->init_poses[vol_idx] = regi_xform_guesses_[vol_idx];
      debug_info_->inter_frames[vol_idx] = intermediate_frames_[vol_idx];
    }

    // TODO: save dyn ref frames

    debug_info_->inter_frames_wrt_vol = intermediate_frames_wrt_vol_;
  }
    
  if (penalty_fn_)
  {
    penalty_fn_->set_save_debug_info(debug_save_iter_debug_info_);
    penalty_fn_->setup();
  }
}

void xreg::Intensity2D3DRegi::after_last_iteration()
{
  if (debug_save_iter_debug_info_)
  {
    debug_info_->sims_aux.clear();
    
    for (auto& sm : sim_metrics_)
    {
      debug_info_->sims_aux.push_back(sm->aux_info());
    }
    
    if (penalty_fn_)
    {
      debug_info_->pen_fn_debug = penalty_fn_->debug_info();
    }
  }
}

void xreg::Intensity2D3DRegi::begin_of_iteration(const ScalarList& x)
{
  begin_of_iteration(opt_vec_to_frame_transforms(x));
}

void xreg::Intensity2D3DRegi::begin_of_iteration(const FrameTransformList& delta_xforms)
{
  update_regi_xforms(delta_xforms);
  
  for (auto& f : begin_of_iter_fns_)
  {
    f(this);
  }
}

void xreg::Intensity2D3DRegi::end_of_iteration()
{
  write_debug();

  for (auto& f : end_of_iter_fns_)
  {
    f(this);
  }
}

bool xreg::Intensity2D3DRegi::write_debug_requires_drr() const
{
  return write_remapped_drrs_ || write_raw_drrs_ || write_fixed_img_edge_overlays_;
}

void xreg::Intensity2D3DRegi::write_debug()
{
  if (write_remapped_drrs_)
  {
    debug_write_remapped_drrs();
  }

  if (write_raw_drrs_)
  {
    debug_write_raw_drrs();
  }

  if (write_fixed_img_edge_overlays_)
  {
    debug_write_fixed_img_edge_overlays();
  }

  if (write_combined_sim_scores_to_stream_ || write_combined_sim_scores_to_stream_)
  {
    this->dout() << fmt::format("Iteration {:4d}:\n", num_obj_fn_evals_);
  }

  if (debug_save_iter_debug_info_ || write_combined_sim_scores_to_stream_)
  {
    debug_write_comb_sim_score();
  }

  if (debug_save_iter_debug_info_ || write_opt_vars_to_stream_)
  {
    debug_write_opt_pose_vars();
  }
}

void xreg::Intensity2D3DRegi::debug_write_remapped_drrs()
{
  const size_type num_sim_metrics = sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    const size_type off = view_idx * num_projs_per_view_;
    for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
    {
      auto drr_img = ray_caster_->proj(off + proj_idx);

      Path dst_path = debug_output_dir_path_;
      dst_path += fmt::format("drr_remap_{:03d}_{:03d}_{:03d}.png", num_obj_fn_evals_, view_idx, proj_idx);

      WriteITKImageRemap8bpp(drr_img.GetPointer(), dst_path.string());
    }
  }
}

void xreg::Intensity2D3DRegi::debug_write_raw_drrs()
{
  const size_type num_sim_metrics = sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    const size_type off = view_idx * num_projs_per_view_;
    for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
    {
      auto drr_img = ray_caster_->proj(off + proj_idx);

      Path dst_path = debug_output_dir_path_;
      dst_path += fmt::format("drr_raw_{:03d}_{:03d}_{:03d}.nii.gz", num_obj_fn_evals_, view_idx, proj_idx);

      WriteITKImageToDisk(drr_img.GetPointer(), dst_path.string());
    }
  }
}

void xreg::Intensity2D3DRegi::debug_write_fixed_img_edge_overlays()
{
  const size_type num_sim_metrics = sim_metrics_.size();

  for (size_type view_idx = 0; view_idx < num_sim_metrics; ++view_idx)
  {
    const size_type off = view_idx * num_projs_per_view_;
    for (size_type proj_idx = 0; proj_idx < num_projs_per_view_; ++proj_idx)
    {
      auto drr_img = ray_caster_->proj(off + proj_idx);

      const auto img_2d_size = drr_img->GetLargestPossibleRegion().GetSize();

      cv::Mat edge_img(img_2d_size[1], img_2d_size[0], cv::DataType<unsigned char>::type);
      RemapAndComputeEdges(drr_img.GetPointer(), &edge_img, 12, 75);

      auto fixed_u8 = ITKImageRemap8bpp(sim_metrics_[view_idx]->fixed_image().GetPointer());

      cv::Mat edge_overlay = OverlayEdges(ShallowCopyItkToOpenCV(fixed_u8.GetPointer()), edge_img, 1 /* 1 -> green edges */);

      Path dst_path = this->debug_output_dir_path_;
      dst_path += fmt::format("edges_{:03d}_{:03d}_{:03d}.png", num_obj_fn_evals_, view_idx, proj_idx);

      cv::imwrite(dst_path.string(), edge_overlay);
    }
  }
}

void xreg::Intensity2D3DRegi::update_regi_xforms(const FrameTransformList& delta_xforms)
{
  const size_type nv = num_vols();

  regi_xforms_ = inter_transforms_to_regi(delta_xforms);

  if (debug_save_iter_debug_info_)
  {
    debug_info_->final_poses = regi_xforms_;
  }
}
  
void xreg::Intensity2D3DRegi::debug_write_comb_sim_score()
{
  throw UnsupportedOperationException();
}

void xreg::Intensity2D3DRegi::debug_write_opt_pose_vars()
{
  throw UnsupportedOperationException();
}

xreg::FrameTransformList
xreg::Intensity2D3DRegi::opt_vec_to_frame_transforms(const ScalarList& x) const
{
  const size_type nv = num_vols();
  const size_type np = opt_vars_->num_params();

  xregASSERT((np * nv) == x.size());

  FrameTransformList xforms;
  xforms.reserve(nv);

  for (size_type v = 0; v < nv; ++v)
  {
    xforms.push_back(opt_vars_->operator()(
        Eigen::Map<PtN>(const_cast<Scalar*>(&x[np * v]), np)));
  }

  return xforms;
}

xreg::FrameTransformList
xreg::Intensity2D3DRegi::inter_transforms_to_regi(const FrameTransformList& delta_xforms) const
{
  const size_type nv = num_vols();
  xregASSERT(delta_xforms.size() == nv);

  FrameTransform pre_mult;
  FrameTransform post_mult;

  FrameTransformList regi_xforms(nv);

  bool need_dyn_ref_frame = false;

  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    if (!dyn_ref_frame_fns_[vol_idx])
    {
      if (intermediate_frames_wrt_vol_[vol_idx])
      {
        pre_mult  = intermediate_frames_[vol_idx];
        post_mult = intermediate_frames_[vol_idx].inverse() * regi_xform_guesses_[vol_idx];
      }
      else
      {
        pre_mult  = regi_xform_guesses_[vol_idx] * intermediate_frames_[vol_idx];
        post_mult = intermediate_frames_[vol_idx].inverse();
      }

      regi_xforms[vol_idx] = pre_mult * delta_xforms[vol_idx] * post_mult;
    }
    else
    {
      need_dyn_ref_frame = true;
    }
  }
 
  if (need_dyn_ref_frame)
  {
    ListOfFrameTransformLists tmp_regi_xforms(nv);
    ListOfFrameTransformLists tmp_delta_xforms(nv);

    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      tmp_regi_xforms[vol_idx]  = { regi_xforms[vol_idx] };
      tmp_delta_xforms[vol_idx] = { delta_xforms[vol_idx] };
    }

    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      if (dyn_ref_frame_fns_[vol_idx])
      {
        regi_xforms[vol_idx] = dyn_ref_frame_fns_[vol_idx](vol_idx, tmp_delta_xforms, tmp_regi_xforms,
                                                           intermediate_frames_, intermediate_frames_wrt_vol_,
                                                           regi_xform_guesses_)[0];
        
        tmp_regi_xforms[vol_idx] = { regi_xforms[vol_idx] };
      }
    }
  }
  
  return regi_xforms;
}

xreg::Intensity2D3DRegi::ListOfFrameTransformLists
xreg::Intensity2D3DRegi::apply_inter_transforms_for_obj_fn(
                            const ListOfFrameTransformLists& src_frame_xforms_per_object) const
{
  const size_type nv = num_vols();
  
  xregASSERT(nv > 0);
  xregASSERT(src_frame_xforms_per_object.size() == nv);
  
  const size_type num_xforms_per_vol = src_frame_xforms_per_object[0].size();

  ListOfFrameTransformLists dst_xforms(nv, FrameTransformList(num_xforms_per_vol));

  FrameTransform pre_mult;
  FrameTransform post_mult;

  bool need_dyn_ref_frame = false;

  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    if (!dyn_ref_frame_fns_[vol_idx])
    {
      if (intermediate_frames_wrt_vol_[vol_idx])
      {
        pre_mult  = intermediate_frames_[vol_idx];
        post_mult = intermediate_frames_[vol_idx].inverse() * regi_xform_guesses_[vol_idx];
      }
      else
      {
        pre_mult  = regi_xform_guesses_[vol_idx] * intermediate_frames_[vol_idx];
        post_mult = intermediate_frames_[vol_idx].inverse();
      }
     
      const auto& cur_src_xforms = src_frame_xforms_per_object[vol_idx];
      xregASSERT(num_xforms_per_vol == cur_src_xforms.size()); 
      
      auto& cur_dst_xforms = dst_xforms[vol_idx];
      xregASSERT(cur_dst_xforms.size() == num_xforms_per_vol);
      
      auto compute_xforms_fn = [&cur_dst_xforms,&cur_src_xforms,
                                &pre_mult,&post_mult] (const RangeType& r)
      {
        for (size_type xform_idx = r.begin(); xform_idx < r.end(); ++xform_idx)
        {
          cur_dst_xforms[xform_idx] = pre_mult * cur_src_xforms[xform_idx] * post_mult;
        }
      };
      
      ParallelFor(compute_xforms_fn, RangeType(0, num_xforms_per_vol));
    }
    else
    {
      need_dyn_ref_frame = true;
    }
  }
 
  if (need_dyn_ref_frame)
  {
    for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
    {
      if (dyn_ref_frame_fns_[vol_idx])
      {
        auto& cur_dst_xforms = dst_xforms[vol_idx];
        
        cur_dst_xforms = dyn_ref_frame_fns_[vol_idx](vol_idx, src_frame_xforms_per_object, dst_xforms,
                                                     intermediate_frames_, intermediate_frames_wrt_vol_,
                                                     regi_xform_guesses_);
      }
    }
  }

  return dst_xforms;
}

void xreg::Intensity2D3DRegi::num_vols_updated()
{
  const size_type nv = num_vols();
  
  tmp_frame_xforms_.assign(nv, FrameTransformList(num_projs_per_view_));

  regi_xforms_.resize(nv, FrameTransform::Identity());

  regi_xform_guesses_.resize(nv, FrameTransform::Identity());

  intermediate_frames_.resize(nv, FrameTransform::Identity());

  intermediate_frames_wrt_vol_.resize(nv, kDEFAULT_INTER_FRAMES_WRT_VOL);

  dyn_ref_frame_fns_.resize(nv, nullptr);
}

