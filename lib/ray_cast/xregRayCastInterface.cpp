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

#include "xregRayCastInterface.h"

#include "xregAssert.h"
#include "xregTBBUtils.h"

void xreg::RayCaster::set_volume(VolPtr img_vol)
{
  vols_ = { img_vol };

  vols_changed();
}

void xreg::RayCaster::set_volumes(const VolList& vols)
{
  vols_ = vols;
  
  vols_changed();
}

xreg::size_type xreg::RayCaster::num_vols() const
{
  return vols_.size();
}
  
xreg::size_type xreg::RayCaster::num_camera_models() const
{
  return camera_models_.size();
}

void xreg::RayCaster::set_camera_models(const CameraModelList& camera_models)
{
  camera_models_ = camera_models;
  camera_models_changed();
}

void xreg::RayCaster::set_camera_model(const CameraModel& camera_model)
{
  camera_models_ = { camera_model };
  camera_models_changed();
}

const xreg::RayCaster::CameraModelList&
xreg::RayCaster::camera_models() const
{
  return camera_models_;
}

const xreg::CameraModel&
xreg::RayCaster::camera_model(const size_type cam_idx) const
{
  return camera_models_[cam_idx];
}

void xreg::RayCaster::set_proj_cam_model(const size_type proj_idx, const size_type cam_idx)
{
  cam_model_for_proj_[proj_idx] = cam_idx;
}

const xreg::RayCaster::CamModelAssocList&
xreg::RayCaster::camera_model_proj_associations() const
{
  return cam_model_for_proj_;
}
  
void xreg::RayCaster::set_camera_model_proj_associations(const CamModelAssocList& cam_model_for_proj)
{
  xregASSERT(cam_model_for_proj.size() == num_projs_);
  // TODO: also check validity of camera indices

  cam_model_for_proj_ = cam_model_for_proj;
}

void xreg::RayCaster::distribute_xforms_among_cam_models(const FrameTransformList& xforms_cam_to_itk_phys)
{
  const size_type num_passed_xforms = xforms_cam_to_itk_phys.size();

  const size_type num_cams = num_camera_models();

  xregASSERT((num_passed_xforms * num_cams) == num_projs());

  size_type global_proj_idx = 0;
  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    for (size_type passed_proj_idx = 0; passed_proj_idx < num_passed_xforms; ++passed_proj_idx, ++global_proj_idx)
    {
      xforms_cam_to_itk_phys_[global_proj_idx] = xforms_cam_to_itk_phys[passed_proj_idx];
      cam_model_for_proj_[global_proj_idx] = cam_idx;
    }
  }
}

void xreg::RayCaster::distribute_xform_among_cam_models(const FrameTransform& xform_cam_to_itk_phys)
{
  distribute_xforms_among_cam_models(FrameTransformList(1, xform_cam_to_itk_phys));
}

void xreg::RayCaster::set_ray_step_size(const CoordScalar& step_size)
{
  ray_step_size_ = step_size;
}

xreg::CoordScalar xreg::RayCaster::ray_step_size() const
{
  return ray_step_size_;
}

void xreg::RayCaster::set_num_projs(const size_type num_projs)
{
  // verify that we have either have not allocated resources or can fit the
  // requested number of projections
  xregASSERT(!max_num_projs_ || (num_projs_ <= max_num_projs_));
  num_projs_ = num_projs;

  cam_model_for_proj_.resize(num_projs_);
}
  
xreg::size_type xreg::RayCaster::num_projs() const
{
  return num_projs_;
}

void xreg::RayCaster::set_interp_method(const InterpMethod& interp_method)
{
  interp_method_ = interp_method;
}
  
xreg::RayCaster::InterpMethod xreg::RayCaster::interp_method() const
{
  return interp_method_;
}

void xreg::RayCaster::use_linear_interp()
{
  interp_method_ = kRAY_CAST_INTERP_LINEAR;
}

void xreg::RayCaster::use_nn_interp()
{
  interp_method_ = kRAY_CAST_INTERP_NN;
}

void xreg::RayCaster::use_sinc_interp()
{
  interp_method_ = kRAY_CAST_INTERP_SINC;
}

void xreg::RayCaster::use_bspline_interp()
{
  interp_method_ = kRAY_CAST_INTERP_BSPLINE;
}

void xreg::RayCaster::set_xforms_cam_to_itk_phys(const FrameTransformList& xforms)
{
  set_num_projs(xforms.size());
  xforms_cam_to_itk_phys_ = xforms;
}
  
const xreg::FrameTransformList&
xreg::RayCaster::xforms_cam_to_itk_phys() const
{
  return xforms_cam_to_itk_phys_;
}

xreg::FrameTransform&
xreg::RayCaster::xform_cam_to_itk_phys(const size_type proj_idx)
{
  return xforms_cam_to_itk_phys_[proj_idx];
}

const xreg::FrameTransform&
xreg::RayCaster::xform_cam_to_itk_phys(const size_type proj_idx) const
{
  return xforms_cam_to_itk_phys_[proj_idx];
}

xreg::FrameTransform
xreg::RayCaster::xform_img_center_to_itk_phys(const size_type vol_idx) const
{
  // we want to treat image coordinates as having the origin at the volume center,
  // ITK physical coordinates are typically have the origin at the "top left" of the
  // image/volume (or the zero index)

  const auto vol_size = vols_[vol_idx]->GetLargestPossibleRegion().GetSize();

  itk::ContinuousIndex<double,3> vol_center_ind;
  vol_center_ind[0] = vol_size[0] / 2.0;
  vol_center_ind[1] = vol_size[1] / 2.0;
  vol_center_ind[2] = vol_size[2] / 2.0;

  // retrieve the ITK physical point coordinate of the volume center
  itk::Point<double,3> vol_center_pt_wrt_itk;
  vols_[vol_idx]->TransformContinuousIndexToPhysicalPoint(vol_center_ind, vol_center_pt_wrt_itk);

  FrameTransform xform = FrameTransform::Identity();
  xform.matrix()(0,3) = vol_center_pt_wrt_itk[0];
  xform.matrix()(1,3) = vol_center_pt_wrt_itk[1];
  xform.matrix()(2,3) = vol_center_pt_wrt_itk[2];

  return xform;
}

xreg::FrameTransform
xreg::RayCaster::xform_cam_wrt_carm_center_of_rot(const size_type cam_idx) const
{
  FrameTransform xform = FrameTransform::Identity();
  xform.matrix()(2,3) -= camera_models_[cam_idx].focal_len / 2;

  return xform;
}

void xreg::RayCaster::post_multiply_all_xforms(const FrameTransform& post_xform)
{
  auto post_mult_fn = [&] (const RangeType& r)
  {
    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      FrameTransform& cur_frame = xforms_cam_to_itk_phys_[range_idx];
      cur_frame = cur_frame * post_xform;
    }
  };

  ParallelFor(post_mult_fn, RangeType(0, num_projs_));
}

void xreg::RayCaster::pre_multiply_all_xforms(const FrameTransform& pre_xform)
{
  auto pre_mult_fn = [&] (const RangeType& r)
  {
    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      FrameTransform& cur_frame = xforms_cam_to_itk_phys_[range_idx];
      cur_frame = pre_xform * cur_frame;
    }
  };

  ParallelFor(pre_mult_fn, RangeType(0, num_projs_));
}

void xreg::RayCaster::allocate_resources()
{
  xregASSERT(num_projs_);
  xregASSERT(!camera_models_.empty());

  max_num_projs_ = num_projs_;

  xforms_cam_to_itk_phys_.resize(num_projs_);

  cam_model_for_proj_.resize(num_projs_, 0);  // default camera index is 0

  // NOTE: for the gpu, this would be where space would be allocated and the volume memcpy'd

  resources_allocated_ = true;
}

void xreg::RayCaster::set_proj_store_method(const ProjPixelStoreMethod m)
{
  proj_store_meth_ = m;
}

xreg::RayCaster::ProjPixelStoreMethod xreg::RayCaster::proj_store_method() const
{
  return proj_store_meth_;
}

void xreg::RayCaster::use_proj_store_replace_method()
{
  proj_store_meth_ = kRAY_CAST_PIXEL_REPLACE;
}

void xreg::RayCaster::use_proj_store_accum_method()
{
  proj_store_meth_ = kRAY_CAST_PIXEL_ACCUM;
}
  
xreg::RayCastSyncOCLBuf* xreg::RayCaster::to_ocl_buf()
{
  throw UnsupportedOperationException();
}

xreg::RayCastSyncHostBuf* xreg::RayCaster::to_host_buf()
{
  throw UnsupportedOperationException();
}

void xreg::RayCaster::set_use_bg_projs(const bool use_bg_projs)
{
  use_bg_projs_ = use_bg_projs;
}

bool xreg::RayCaster::use_bg_projs() const
{
  return use_bg_projs_;
}

void xreg::RayCaster::set_bg_proj(ProjPtr& proj, const bool use_bg_projs)
{
  bg_projs_for_each_cam_.assign(num_camera_models(), proj);
  use_bg_projs_ = use_bg_projs;

  bg_projs_updated_ = true;
}

void xreg::RayCaster::set_bg_projs(ProjList& projs, const bool use_bg_projs)
{
  xregASSERT(num_camera_models() == projs.size());

  bg_projs_for_each_cam_ = projs;
  use_bg_projs_ = use_bg_projs;

  bg_projs_updated_ = true;
}

xreg::size_type xreg::RayCaster::max_num_projs() const
{
  return max_num_projs_;
}

xreg::RayCaster::PixelScalar2D xreg::RayCaster::default_bg_pixel_val() const
{
  return default_bg_pixel_val_;
}

void xreg::RayCaster::set_default_bg_pixel_val(const PixelScalar2D bg_val)
{
  default_bg_pixel_val_ = bg_val;
}
  
void xreg::RayCasterCollisionParamInterface::set_render_thresh(const PixelScalar t)
{
  collision_params_.thresh = t;
}

void xreg::RayCasterCollisionParamInterface::set_num_backtracking_steps(const size_type num_steps)
{
  collision_params_.num_backtracking_steps = num_steps;
}

xreg::RayCasterCollisionParamInterface::PixelScalar
xreg::RayCasterCollisionParamInterface::render_thresh() const
{
  return collision_params_.thresh;
}

xreg::size_type xreg::RayCasterCollisionParamInterface::num_backtracking_steps() const
{
  return collision_params_.num_backtracking_steps;
}

const xreg::RayCasterCollisionParams&
xreg::RayCasterCollisionParamInterface::collision_params() const
{
  return collision_params_;
}
  
xreg::RayCasterSurRenderParamInterface::RayCasterSurRenderParamInterface()
{
  set_render_thresh(200);
  set_num_backtracking_steps(20);

  sur_render_params_.ambient_reflection_ratio  = 0.25;
  sur_render_params_.diffuse_reflection_ratio  = 0.7;
  sur_render_params_.specular_reflection_ratio = 0.05;
  sur_render_params_.alpha_shininess           = 1.0;
}

void xreg::RayCasterSurRenderParamInterface::set_sur_render_params(
                      const RayCasterSurRenderShadingParams& sur_render_params)
{
  sur_render_params_ = sur_render_params;
}

const xreg::RayCasterSurRenderShadingParams&
xreg::RayCasterSurRenderParamInterface::surface_render_params() const
{
  return sur_render_params_;
}

void xreg::RayCasterSurRenderParamInterface::set_ambient_reflection_ratio(
                                              const PixelScalar amb_ref_ratio)
{
  sur_render_params_.ambient_reflection_ratio = amb_ref_ratio;
}

void xreg::RayCasterSurRenderParamInterface::set_diffuse_reflection_ratio(
                                              const PixelScalar diff_ref_ratio)
{
  sur_render_params_.diffuse_reflection_ratio = diff_ref_ratio;
}

void xreg::RayCasterSurRenderParamInterface::set_specular_reflection_ratio(
                                              const PixelScalar spec_ref_ratio)
{
  sur_render_params_.specular_reflection_ratio = spec_ref_ratio;
}

void xreg::RayCasterSurRenderParamInterface::set_alpha_shininess(const PixelScalar alpha)
{
  sur_render_params_.alpha_shininess = alpha;
}
  
xreg::RayCasterOccludingContours::RayCasterOccludingContours()
{
  this->set_render_thresh(150);
  this->set_num_backtracking_steps(0);
}

xreg::CoordScalar xreg::RayCasterOccludingContours::occlusion_angle_thresh_rad() const
{
  return occlusion_angle_thresh_rad_;
}

void xreg::RayCasterOccludingContours::set_occlusion_angle_thresh_rad(const CoordScalar ang_rad)
{
  occlusion_angle_thresh_rad_ = ang_rad;
}

void xreg::RayCasterOccludingContours::set_occlusion_angle_thresh_deg(const CoordScalar ang_deg)
{
  occlusion_angle_thresh_rad_ = ang_deg * kDEG2RAD;
}

bool xreg::RayCasterOccludingContours::stop_after_collision() const
{
  return stop_after_collision_;
}

void xreg::RayCasterOccludingContours::set_stop_after_collision(const bool stop_after_coll)
{
  stop_after_collision_ = stop_after_coll;
}

void xreg::SimpleRayCasterWrapperFn::operator()()
{
  const bool vols_to_proj_specified = !vols_to_proj.empty();

  const size_type nv = vols_to_proj_specified ? vols_to_proj.size() :
                                                ray_caster->num_vols();

  xregASSERT(cam_world_to_vols.size() == nv);

  const bool inter_frames_specified = !inter_frames.empty();
  xregASSERT(!inter_frames_specified ||
             ((inter_frames.size() == nv) &&
              (nv == inter_frames_wrt_vol.size()) &&
              (nv == ref_frames_cam_world_to_vol.size())));

  const size_type num_cams = ray_caster->num_camera_models();

  projs.resize(num_cams);

  // save off some state from the ray caster
  const size_type prev_num_projs = ray_caster->num_projs();
  const auto cam_assocs = ray_caster->camera_model_proj_associations();

  // now, we'll create a projection for each view/camera
  ray_caster->set_num_projs(num_cams);

  // Perform the ray casting of multiple, or one, object(s), by looping over
  // each volume, setting the pose parameters for the current volume, and
  // ray casting for the current volume. The projection buffers are initialized
  // for the first projection and then accumulated afterwards.
  for (size_type vol_idx = 0; vol_idx < nv; ++vol_idx)
  {
    const size_type ray_caster_vol_idx = vols_to_proj_specified ?
                                                      vols_to_proj[vol_idx] :
                                                      vol_idx;
    xregASSERT(ray_caster_vol_idx < ray_caster->num_vols());

    // TODO: have another case when replacing with another image
    if (vol_idx != 0)
    {
      ray_caster->use_proj_store_accum_method();
    }
    else
    {
      ray_caster->use_proj_store_replace_method();
    }

    ray_caster->distribute_xform_among_cam_models(cam_world_to_vols[vol_idx]);

    if (inter_frames_specified)
    {
      if (inter_frames_wrt_vol[vol_idx])
      {
        ray_caster->pre_multiply_all_xforms(inter_frames[vol_idx]);
        ray_caster->post_multiply_all_xforms(inter_frames[vol_idx].inverse() *
                                             ref_frames_cam_world_to_vol[vol_idx]);
      }
      else
      {
        ray_caster->pre_multiply_all_xforms(ref_frames_cam_world_to_vol[vol_idx] *
                                            inter_frames[vol_idx]);
        ray_caster->post_multiply_all_xforms(inter_frames[vol_idx].inverse());
      }
    }

    ray_caster->compute(ray_caster_vol_idx);
  }

  // copy the projections
  for (size_type cam_idx = 0; cam_idx < num_cams; ++cam_idx)
  {
    projs[cam_idx] = ray_caster->proj(cam_idx);
  }

  // restore the previous ray caster state
  ray_caster->set_num_projs(prev_num_projs);
  ray_caster->set_camera_model_proj_associations(cam_assocs); 
}

xreg::RayCastLineIntKernel xreg::RayCastLineIntParamInterface::kernel_id() const
{
  return kernel_id_;
}

void xreg::RayCastLineIntParamInterface::set_kernel_id(const RayCastLineIntKernel k)
{
  kernel_id_ = k;
}

