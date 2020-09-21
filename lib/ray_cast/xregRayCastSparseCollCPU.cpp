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

#include "xregRayCastSparseCollCPU.h"

#include <itkLinearInterpolateImageFunction.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkWindowedSincInterpolateImageFunction.h>
#include <itkConstantBoundaryCondition.h>
#include <itkBSplineInterpolateImageFunction.h>

#include "xregITKBasicImageUtils.h"
#include "xregTBBUtils.h"
#include "xregSpatialPrimitives.h"
#include "xregExceptionUtils.h"
#include "xregRayCastBaseCPU.h"

namespace  // un-named
{
using namespace xreg;
  
struct RayCastCollFn
{
  using Vol           = RayCaster::Vol;
  using PixelScalar2D = RayCaster::PixelScalar2D;
  using PixelScalar3D = RayCaster::PixelScalar3D;
  
  const Vol* img_vol;  ///< 3D volume we are ray casting through

  const Pt3 img_aabb_min;  ///< Axis-Aligned minimum bounds on the image indices for each index dimension (should be zeros)
  const Pt3 img_aabb_max;  ///< Axis-Aligned maximum bounds on the image indices for each index dimension

  const FrameTransform itk_idx_to_itk_phys_pt_xform;  ///< Transformation from ITK physical points to continuous indices
  
  const FrameTransform itk_phys_pt_to_itk_idx_xform;  ///< Transformation from ITK physical points to continuous indices

  const size_type num_projs;  ///< The number of views, or poses of the volume, that are being computed (needed to recover indices into xforms_cam_to_itk_phys and proj_buf)

  const RayCaster::CameraModelList& camera_models;  ///< The camera models (intrinsics)

  const FrameTransformList& xforms_cam_to_itk_phys;  ///< The collection of camera poses, each representing a view/pose

  const RayCaster::CamModelAssocList& cam_model_for_proj;  ///< The association of view/pose to camera model

  const CoordScalar step_size;  ///< The step size for each ray

  const RayCaster::InterpMethod interp_method;  ///< The interpolation method used for fractional indices into the 3D volume

  PixelScalar3D collision_thresh;

  size_type num_backtracking_steps;

  const RayCasterSparseCollisionCPU::ListOfPt3Lists& pts_wrt_each_cam_ext;

  RayCasterSparseCollisionCPU::ListOfPt3Lists& entry_coll_pts_wrt_vol_for_each_view;

  RayCasterSparseCollisionCPU::ListOfPt3Lists& exit_coll_pts_wrt_vol_for_each_view;

  std::vector<RayCasterSparseCollisionCPU::DistList>& entry_coll_intersect_dists_for_each_view;

  const bool orient_towards_cam_pinhole;

  const bool find_exit_pts;

  const RayCasterSparseCollisionCPU::RayIndLUT& ray_idx_lut;

  /// \brief Computation operator - executes a collection of rays cast
  ///
  /// The collection of line integrals is not necessarily restricted to a single projection
  void operator()(const RangeType& r) const
  {
    using VolInterpType        = itk::InterpolateImageFunction<Vol,CoordScalar>;
    using VolLinearInterpType  = itk::LinearInterpolateImageFunction<Vol,CoordScalar>;
    using VolNNInterpType      = itk::NearestNeighborInterpolateImageFunction<Vol,CoordScalar>;
    using VolBSplineInterpType = itk::BSplineInterpolateImageFunction<Vol,CoordScalar>;
    using VolSincInterpType    = itk::WindowedSincInterpolateImageFunction<
                                         Vol,4,itk::Function::LanczosWindowFunction<4>,
                                         itk::ConstantBoundaryCondition<Vol>,CoordScalar>;

    VolInterpType::Pointer vol_interp;

    switch (interp_method)
    {
      case RayCaster::kRAY_CAST_INTERP_NN:
        vol_interp = VolNNInterpType::New();
        break;
      case RayCaster::kRAY_CAST_INTERP_SINC:
        vol_interp = VolSincInterpType::New();
        break;
      case RayCaster::kRAY_CAST_INTERP_BSPLINE:
      {
        auto bspline_interp = VolBSplineInterpType::New();
        bspline_interp->SetSplineOrder(3);
        vol_interp = bspline_interp;
      }
        break;
      case RayCaster::kRAY_CAST_INTERP_LINEAR:
      default:
        vol_interp = VolLinearInterpType::New();
        break;
    }

    vol_interp->SetInputImage(img_vol);

    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      size_type view_idx = 0;
      size_type ray_idx  = 0;
      std::tie(view_idx,ray_idx) = ray_idx_lut[range_idx];  

      CoordScalar& cur_pt_dist = entry_coll_intersect_dists_for_each_view[view_idx][ray_idx];
      cur_pt_dist = -1;

      const auto& cam = camera_models[cam_model_for_proj[view_idx]];

      // This point is stored with respect to the camera's
      // "world" coordinates
      const Pt3& cur_pt_wrt_cam = pts_wrt_each_cam_ext[view_idx][ray_idx];

      // The current transformation from detector coordinates to ITK indices
      const FrameTransform xform_cam_to_itk_idx = itk_phys_pt_to_itk_idx_xform * xforms_cam_to_itk_phys[view_idx];

      const FrameTransform xform_itk_idx_to_cam = xform_cam_to_itk_idx.inverse();

      const Pt3 cur_pt_wrt_itk_idx = xform_cam_to_itk_idx * cur_pt_wrt_cam;

      // Position of the X-Ray source / pinhole point in ITK indices
      const Pt3 pinhole_wrt_itk_idx = xform_cam_to_itk_idx * cam.pinhole_pt;

      const Pt3 look_unit_vec_wrt_cam = (orient_towards_cam_pinhole ? CoordScalar(-1) : CoordScalar(1))
                                                 * (cur_pt_wrt_cam - cam.pinhole_pt).normalized();

      const Pt3 look_vec_wrt_itk_idx = (orient_towards_cam_pinhole ? CoordScalar(-1) : CoordScalar(1))
                                                * (cur_pt_wrt_itk_idx - pinhole_wrt_itk_idx);

      const Pt3 look_unit_vec_wrt_itk_idx = look_vec_wrt_itk_idx.normalized();

      Pt3 start_pt_wrt_itk_idx = orient_towards_cam_pinhole ? cur_pt_wrt_itk_idx : pinhole_wrt_itk_idx;

      CoordScalar t_start = 0;
      CoordScalar t_stop  = 0;
      
      constexpr CoordScalar kVOL_BB_STEP_INC_TOL = RayCasterCPU::kVOL_BB_STEP_INC_TOL;
      
      bool inter_vol = false;

      std::tie(inter_vol,t_start,t_stop) = RayRectIntersect(img_aabb_min, img_aabb_max,
                                                            start_pt_wrt_itk_idx, look_vec_wrt_itk_idx,
                                                            false);  // false -> do not limit to line segment

      // Besides intersecting, need to be able to nudge inward a bit to avoid an
      // ITK crash when interpolating on edge
      if (inter_vol && ((t_stop - t_start) > CoordScalar(2 * kVOL_BB_STEP_INC_TOL)))
      {
        t_start += kVOL_BB_STEP_INC_TOL;
        t_stop  -= kVOL_BB_STEP_INC_TOL;

        // the first index on the line from source to detector that lies within the volume bounds
        start_pt_wrt_itk_idx += t_start * look_vec_wrt_itk_idx;

        const CoordScalar look_vec_len_wrt_itk_idx = look_vec_wrt_itk_idx.norm();
        const CoordScalar intersect_len_wrt_itk_idx = (t_stop - t_start) * look_vec_len_wrt_itk_idx;

        // Rotate and scale the step vector to get it wrt ITK indices
        const CoordScalar step_len_wrt_itk_idx = (xform_cam_to_itk_idx.matrix().block(0,0,3,3) * (look_unit_vec_wrt_cam * step_size)).norm();

        const size_type num_steps = static_cast<size_type>(intersect_len_wrt_itk_idx / step_len_wrt_itk_idx);

        // We'll use the ITK objects now, since that is the easiest interface with itk::Image and itk interpolation
        itk::ContinuousIndex<CoordScalar,3> cur_cont_vol_idx;
        cur_cont_vol_idx[0] = start_pt_wrt_itk_idx[0];
        cur_cont_vol_idx[1] = start_pt_wrt_itk_idx[1];
        cur_cont_vol_idx[2] = start_pt_wrt_itk_idx[2];

        const CoordScalar scale_to_step = step_len_wrt_itk_idx / look_vec_len_wrt_itk_idx;
        itk::Vector<CoordScalar,3> tmp_step_vec_wrt_itk_idx;
        tmp_step_vec_wrt_itk_idx[0] = look_vec_wrt_itk_idx[0] * scale_to_step;
        tmp_step_vec_wrt_itk_idx[1] = look_vec_wrt_itk_idx[1] * scale_to_step;
        tmp_step_vec_wrt_itk_idx[2] = look_vec_wrt_itk_idx[2] * scale_to_step;

        PixelScalar3D cur_vol_val = 0;

        bool looking_for_entry_pt = true;

        for (size_type step_idx = 0; step_idx <= num_steps; ++step_idx)
        {
          //xregASSERT(vol_interp->IsInsideBuffer(cur_cont_vol_idx));
          cur_vol_val = vol_interp->EvaluateAtContinuousIndex(cur_cont_vol_idx);
           
          if ((looking_for_entry_pt && (cur_vol_val >= collision_thresh)) ||
              (!looking_for_entry_pt && find_exit_pts && (cur_vol_val < collision_thresh)))
          {
            // we have either collided with the surface/volume on entry -OR-
            // left the surface volume

            // perform some binary search/back-tracking
            // to determine a more accurate location of where the threshold is crossed
     
            itk::Vector<CoordScalar,3> tmp_back_track_step = tmp_step_vec_wrt_itk_idx;

            for (size_type sur_bin_step_idx = 0;
                 sur_bin_step_idx < num_backtracking_steps;
                 ++sur_bin_step_idx)
            {
              tmp_back_track_step *= 0.5;
              
              if (looking_for_entry_pt)
              {
                cur_cont_vol_idx -=
                  (cur_vol_val >= collision_thresh) ?
                                   tmp_back_track_step : -tmp_back_track_step;
              }
              else
              {
                cur_cont_vol_idx -=
                  (cur_vol_val <= collision_thresh) ?
                                   tmp_back_track_step : -tmp_back_track_step;
              }

              //xregASSERT(vol_interp->IsInsideBuffer(cur_cont_vol_idx));
              cur_vol_val = vol_interp->EvaluateAtContinuousIndex(cur_cont_vol_idx);
            }
            // end backtracking

            // compute depth in the camera frame
            Pt3 tmp_vol_idx;
            tmp_vol_idx[0] = cur_cont_vol_idx[0];
            tmp_vol_idx[1] = cur_cont_vol_idx[1];
            tmp_vol_idx[2] = cur_cont_vol_idx[2];
            
            if (looking_for_entry_pt)
            {
              entry_coll_pts_wrt_vol_for_each_view[view_idx][ray_idx] = itk_idx_to_itk_phys_pt_xform * tmp_vol_idx;

              cur_pt_dist = ((xform_itk_idx_to_cam * tmp_vol_idx) - cur_pt_wrt_cam).norm();

              if (find_exit_pts)
              {
                looking_for_entry_pt = false;
              }
              else
              {
                break;
              }
            }
            else if (find_exit_pts)
            {
              exit_coll_pts_wrt_vol_for_each_view[view_idx][ray_idx] = itk_idx_to_itk_phys_pt_xform * tmp_vol_idx;
            }
          }  // end if (cur_vol_val >= collision_thresh)

          cur_cont_vol_idx += tmp_step_vec_wrt_itk_idx;
        }  // for step
      }  // if RayRectIntersect
    }  // for range_idx
  }  // end operator()(Range)
};

}  // un-named

void xreg::RayCasterSparseCollisionCPU::allocate_resources()
{
  RayCaster::allocate_resources();

  pts_wrt_each_cam_ext_.resize(this->camera_models_.size());

  entry_coll_pts_wrt_vol_for_each_view_.resize(this->num_projs_);
  entry_coll_intersect_dists_for_each_view_.resize(this->num_projs_); 

  if (find_exit_pts_)
  {
    exit_coll_pts_wrt_vol_for_each_view_.resize(this->num_projs_);
  }
}

void xreg::RayCasterSparseCollisionCPU::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);
  
  // Get the index bounding box (axis-aligned in the index space) of the volume
  Pt3 img_aabb_min;
  Pt3 img_aabb_max;
  std::tie(img_aabb_min,img_aabb_max) = ITKImageIndexBoundsAsEigen(this->vols_[vol_idx].GetPointer());

  // Compute the frame transform from ITK physical space to index space (sR + t)
  const FrameTransform itk_idx_to_itk_phys_pt_xform = ITKImagePhysicalPointTransformsAsEigen(this->vols_[vol_idx].GetPointer());

  const FrameTransform itk_phys_pt_to_itk_idx_xform = itk_idx_to_itk_phys_pt_xform.inverse();

  // resize the output buffers as required
  size_type tot_rays = 0;
  for (size_type view_idx = 0; view_idx < this->num_projs_; ++view_idx)
  {
    const size_type num_rays_this_view = pts_wrt_each_cam_ext_[this->cam_model_for_proj_[view_idx]].size();
  
    entry_coll_pts_wrt_vol_for_each_view_[view_idx].resize(num_rays_this_view);
    entry_coll_intersect_dists_for_each_view_[view_idx].resize(num_rays_this_view);

    if (find_exit_pts_)
    {
      exit_coll_pts_wrt_vol_for_each_view_[view_idx].resize(num_rays_this_view);
    }

    tot_rays += num_rays_this_view;
  }

  {
    // TODO: there is probably a more efficient way to do this
    ray_idx_lut_.resize(tot_rays);

    size_type global_ray_idx = 0;
    for (size_type view_idx = 0; view_idx < this->num_projs_; ++view_idx)
    {
      const size_type num_rays_this_view = pts_wrt_each_cam_ext_[this->cam_model_for_proj_[view_idx]].size();
    
      for (size_type local_ray_idx = 0; local_ray_idx < num_rays_this_view; ++local_ray_idx, ++global_ray_idx)
      {
        ray_idx_lut_[global_ray_idx] = std::make_tuple(view_idx, local_ray_idx);
      }
    }
  }

  RayCastCollFn ray_cast_fn = { this->vols_[vol_idx],
                                img_aabb_min,
                                img_aabb_max,
                                itk_idx_to_itk_phys_pt_xform,
                                itk_phys_pt_to_itk_idx_xform,
                                this->num_projs_,
                                this->camera_models_,
                                this->xforms_cam_to_itk_phys_,
                                this->cam_model_for_proj_,
                                this->ray_step_size_,
                                this->interp_method_,
                                this->render_thresh(),
                                this->num_backtracking_steps(),
                                pts_wrt_each_cam_ext_,
                                entry_coll_pts_wrt_vol_for_each_view_,
                                exit_coll_pts_wrt_vol_for_each_view_,
                                entry_coll_intersect_dists_for_each_view_,
                                orient_towards_cam_pinhole_,
                                find_exit_pts_,
                                ray_idx_lut_
                              };

  // Cast the rays
  ParallelFor(ray_cast_fn, RangeType(0, tot_rays));
}

xreg::RayCasterSparseCollisionCPU::ProjPtr
xreg::RayCasterSparseCollisionCPU::proj(const size_type proj_idx)
{
  throw UnsupportedOperationException();
}

cv::Mat xreg::RayCasterSparseCollisionCPU::proj_ocv(const size_type proj_idx)
{
  throw UnsupportedOperationException();
}

xreg::RayCasterSparseCollisionCPU::PixelScalar2D*
xreg::RayCasterSparseCollisionCPU::raw_host_pixel_buf()
{
  throw UnsupportedOperationException();
}

void xreg::RayCasterSparseCollisionCPU::use_external_host_pixel_buf(void* buf)
{
  throw UnsupportedOperationException();
}

xreg::size_type
xreg::RayCasterSparseCollisionCPU::max_num_projs_possible() const
{
  return ~size_type(0);
}

void xreg::RayCasterSparseCollisionCPU::use_other_proj_buf(RayCaster* other_ray_caster)
{
  throw UnsupportedOperationException();
}

bool xreg::RayCasterSparseCollisionCPU::orient_rays_towards_pinhole() const
{
  return orient_towards_cam_pinhole_;
}

void xreg::RayCasterSparseCollisionCPU::set_orient_rays_towards_pinhole(const bool orient_towards_pinhole)
{
  orient_towards_cam_pinhole_ = orient_towards_pinhole;
}

bool xreg::RayCasterSparseCollisionCPU::find_exit_pts() const
{
  return find_exit_pts_;
}

void xreg::RayCasterSparseCollisionCPU::set_find_exit_pts(const bool find_exit_pts)
{
  find_exit_pts_ = find_exit_pts;
}

void xreg::RayCasterSparseCollisionCPU::set_pts_for_cam(const Pt3List& pts_wrt_cam, const size_type cam_idx)
{
  xregASSERT(this->resources_allocated_);
  xregASSERT(cam_idx < this->camera_models_.size());

  pts_wrt_each_cam_ext_[cam_idx] = pts_wrt_cam;
}

const xreg::Pt3List&
xreg::RayCasterSparseCollisionCPU::entry_coll_pts_wrt_vol(const size_type view_idx) const
{
  xregASSERT(this->resources_allocated_);
  xregASSERT(view_idx < this->num_projs_);

  return entry_coll_pts_wrt_vol_for_each_view_[view_idx];
}

const xreg::Pt3List&
xreg::RayCasterSparseCollisionCPU::exit_coll_pts_wrt_vol(const size_type view_idx) const
{
  xregASSERT(this->resources_allocated_);
  xregASSERT(view_idx < this->num_projs_);

  return exit_coll_pts_wrt_vol_for_each_view_[view_idx];
}

const xreg::RayCasterSparseCollisionCPU::DistList&
xreg::RayCasterSparseCollisionCPU::intersect_dists(const size_type view_idx) const
{
  xregASSERT(this->resources_allocated_);
  xregASSERT(view_idx < this->num_projs_);

  return entry_coll_intersect_dists_for_each_view_[view_idx];
} 
