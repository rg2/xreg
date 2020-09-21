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

#include "xregRayCastOccContourCPU.h"

#include <itkLinearInterpolateImageFunction.h>
#include <itkNearestNeighborInterpolateImageFunction.h>
#include <itkWindowedSincInterpolateImageFunction.h>
#include <itkConstantBoundaryCondition.h>
#include <itkBSplineInterpolateImageFunction.h>

#include "xregITKBasicImageUtils.h"
#include "xregTBBUtils.h"
#include "xregSpatialPrimitives.h"
#include "xregExceptionUtils.h"

namespace  // un-named
{

using namespace xreg;

/// \brief Computation task for evaluating whether a 2D pixel has a contour.
struct ContourFn
{
  using Vol           = RayCaster::Vol;
  using PixelScalar2D = RayCaster::PixelScalar2D;
  using PixelScalar3D = RayCaster::PixelScalar3D;
  
  const Vol* img_vol;  ///< 3D volume we are ray casting through

  const Pt3 img_aabb_min;  ///< Axis-Aligned minimum bounds on the image indices for each index dimension (should be zeros)
  const Pt3 img_aabb_max;  ///< Axis-Aligned maximum bounds on the image indices for each index dimension

  const FrameTransform itk_phys_pt_to_itk_idx_xform;  ///< Transformation from ITK physical points to continuous indices

  const size_type num_projs;  ///< The number of projection images that are being computed (needed to recover indices into xforms_cam_to_itk_phys and proj_buf)

  const RayCaster::CameraModelList& camera_models;  ///< The camera models (intrinsics)

  const FrameTransformList& xforms_cam_to_itk_phys;  ///< The collection of camera poses, each representing a projection

  const RayCaster::CamModelAssocList& cam_model_for_proj;  ///< The association of projection to camera model

  const CoordScalar step_size;  ///< The step size for each ray

  const RayCaster::InterpMethod interp_method;  ///< The interpolation method used for fractional indices into the 3D volume

  PixelScalar2D* proj_buf;  ///< The large buffer used for storing all projection results

  const PixelScalar3D collision_thresh;

  const size_type num_backtracking_steps;

  const CoordScalar angle_thresh_rad;

  const bool stop_after_collision;

  /// \brief Computation operator - executes a collection of contour pixel computations
  ///
  /// The collection of contours is not necessarily restricted to a single projection
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
    
    const size_type num_drr_px = camera_models[0].num_det_rows *
                                              camera_models[0].num_det_cols;
    
    for (size_type range_idx = r.begin(); range_idx < r.end(); ++range_idx)
    {
      // recover the original projection, row, column indices
      const size_type proj_idx    = range_idx / num_drr_px;
      const size_type off_in_proj = range_idx - (num_drr_px * proj_idx);
      const size_type row_idx     = off_in_proj / camera_models[0].num_det_cols;
      const size_type col_idx     = off_in_proj - (camera_models[0].num_det_cols * row_idx);

      const auto& cam = camera_models[cam_model_for_proj[proj_idx]];

      // This point is stored with respect to the camera's
      // "world" coordinates
      const Pt3 cur_det_pt_wrt_cam = cam.ind_pt_to_phys_det_pt(Pt2{static_cast<CoordScalar>(col_idx),
                                                                   static_cast<CoordScalar>(row_idx)});

      // The current transformation from detector coordinates to ITK indices
      const FrameTransform xform_cam_to_itk_idx = itk_phys_pt_to_itk_idx_xform * xforms_cam_to_itk_phys[proj_idx];

      // Position of the X-Ray source / pinhole point in ITK indices
      const Pt3 pinhole_wrt_itk_idx = xform_cam_to_itk_idx * cam.pinhole_pt;

      // Position of the current detector element in volume coordinates
      const Pt3 pinhole_to_det_wrt_itk_idx = (xform_cam_to_itk_idx * cur_det_pt_wrt_cam) - pinhole_wrt_itk_idx;

      const Pt3 pinhole_to_det_unit_wrt_itk_idx = pinhole_to_det_wrt_itk_idx.normalized();

      CoordScalar t_start = 0;
      CoordScalar t_stop  = 0;

      constexpr CoordScalar kVOL_BB_STEP_INC_TOL = RayCasterOccludingContoursCPU::kVOL_BB_STEP_INC_TOL;

      bool inter_vol = false;

      std::tie(inter_vol,t_start,t_stop) = RayRectIntersect(img_aabb_min, img_aabb_max,
                                                            pinhole_wrt_itk_idx, pinhole_to_det_wrt_itk_idx,
                                                            false);  // false -> do not limit to line segment

      // Besides intersecting, need to be able to nudge inward a bit to avoid an
      // ITK crash when interpolating on edge
      if (inter_vol && ((t_stop - t_start) > CoordScalar(2 * kVOL_BB_STEP_INC_TOL)))
      {
        t_start += kVOL_BB_STEP_INC_TOL;
        t_stop  -= kVOL_BB_STEP_INC_TOL;

        // the first index on the line from source to detector that lies within the volume bounds
        const Pt3 start_pt_wrt_itk_idx = pinhole_wrt_itk_idx + (t_start * pinhole_to_det_wrt_itk_idx);

        const CoordScalar pinhole_to_det_len_wrt_itk_idx = pinhole_to_det_wrt_itk_idx.norm();
        const CoordScalar intersect_len_wrt_itk_idx = (t_stop - t_start) * pinhole_to_det_len_wrt_itk_idx;

        // Rotate and scale the step vector to get it wrt ITK indices
        const CoordScalar step_len_wrt_itk_idx = (xform_cam_to_itk_idx.matrix().block(0,0,3,3) * ((cur_det_pt_wrt_cam - cam.pinhole_pt).normalized() * step_size)).norm();

        const size_type num_steps = static_cast<size_type>(intersect_len_wrt_itk_idx / step_len_wrt_itk_idx);

        // We'll use the ITK objects now, since that is the easiest interface with itk::Image and itk interpolation
        itk::ContinuousIndex<CoordScalar,3> cur_cont_vol_idx;
        cur_cont_vol_idx[0] = start_pt_wrt_itk_idx[0];
        cur_cont_vol_idx[1] = start_pt_wrt_itk_idx[1];
        cur_cont_vol_idx[2] = start_pt_wrt_itk_idx[2];

        // Index to be used for retrieving values to perform gradient computation
        itk::ContinuousIndex<CoordScalar,3> tmp_cont_vol_idx_grad;

        const CoordScalar scale_to_step = step_len_wrt_itk_idx / pinhole_to_det_len_wrt_itk_idx;
        itk::Vector<CoordScalar,3> tmp_step_vec_wrt_itk_idx;
        tmp_step_vec_wrt_itk_idx[0] = pinhole_to_det_wrt_itk_idx[0] * scale_to_step;
        tmp_step_vec_wrt_itk_idx[1] = pinhole_to_det_wrt_itk_idx[1] * scale_to_step;
        tmp_step_vec_wrt_itk_idx[2] = pinhole_to_det_wrt_itk_idx[2] * scale_to_step;

        PixelScalar3D cur_vol_val = 0;
        PixelScalar3D cur_vol_val_for_grad[2] = { 0, 0 };

        // First, this will be used for gradient calcluation, of which the
        // negative estimates the surface normal. Next, it will be used to esetimate
        // the vector at which light "perfectly" reflects.
        Eigen::Matrix<PixelScalar3D,3,1> sur_grad_vec;

        PixelScalar3D tmp_dot = 0;

        bool is_edge = false;
        for (size_type step_idx = 0; !is_edge && (step_idx <= num_steps); ++step_idx)
        {
          cur_vol_val = vol_interp->EvaluateAtContinuousIndex(cur_cont_vol_idx);
          if (cur_vol_val >= collision_thresh)
          {
            if (t_start > 1.0e-6)
            {
              // we're not at the source, perform some binary search/back-tracking
              // to determine a more accurate location of where the threshold is crossed
        
              // backtracking step vector - repeatedly cut in half
              itk::Vector<CoordScalar,3> backtrack_step_vec_wrt_itk_idx = tmp_step_vec_wrt_itk_idx;

              for (size_type sur_bin_step_idx = 0;
                   sur_bin_step_idx < num_backtracking_steps;
                   ++sur_bin_step_idx)
              {
                backtrack_step_vec_wrt_itk_idx *= 0.5;
                cur_cont_vol_idx -=
                  (cur_vol_val >= collision_thresh) ?
                                  backtrack_step_vec_wrt_itk_idx : -backtrack_step_vec_wrt_itk_idx;

                cur_vol_val = vol_interp->EvaluateAtContinuousIndex(cur_cont_vol_idx);
              }
              // end backtracking

              // approximate the derivative at this point with finite differencing adjacent indices
              // NOTE: these are wrt image (index) axes.

              // Gradient in X direction
              tmp_cont_vol_idx_grad = cur_cont_vol_idx;
              tmp_cont_vol_idx_grad[0] -= 1;

              cur_vol_val_for_grad[0] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);
              tmp_cont_vol_idx_grad[0] += 2;
              cur_vol_val_for_grad[1] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);

              sur_grad_vec[0] = cur_vol_val_for_grad[1] - cur_vol_val_for_grad[0];

              // Gradient in Y direction
              tmp_cont_vol_idx_grad[0] = cur_cont_vol_idx[0];
              tmp_cont_vol_idx_grad[1] -= 1;

              cur_vol_val_for_grad[0] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);
              tmp_cont_vol_idx_grad[1] += 2;
              cur_vol_val_for_grad[1] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);

              sur_grad_vec[1] = cur_vol_val_for_grad[1] - cur_vol_val_for_grad[0];

              // Gradient in Z direction
              tmp_cont_vol_idx_grad[1] = cur_cont_vol_idx[1];
              tmp_cont_vol_idx_grad[2] -= 1;

              cur_vol_val_for_grad[0] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);
              tmp_cont_vol_idx_grad[2] += 2;
              cur_vol_val_for_grad[1] = vol_interp->EvaluateAtContinuousIndex(tmp_cont_vol_idx_grad);

              sur_grad_vec[2] = cur_vol_val_for_grad[1] - cur_vol_val_for_grad[0];

              sur_grad_vec.normalize();
              
              constexpr CoordScalar pi_over_2 = 1.5707963267948966;
              is_edge = std::abs(std::acos(sur_grad_vec.dot(pinhole_to_det_unit_wrt_itk_idx.cast<PixelScalar3D>())) - pi_over_2) < angle_thresh_rad;
              // this will terminate the step loop
            }
            else
            {
              // we're at the source, every pixel in the projection will be saturated --> no edges (or all edges?)
            }

            // once we collide on this ray, we can stop moving along it.
            // TODO: maybe this could be customized to get apparent contours in X-Ray?
            if (stop_after_collision)
            {
              break;
            }
          }  // if (cur_vol_val >= collision_thresh)

          cur_cont_vol_idx += tmp_step_vec_wrt_itk_idx;
        }  // for step

        if (is_edge)
        {
          proj_buf[range_idx] += PixelScalar2D(1);
        }
      }  // if RayRectIntersect
    }  // for range_idx
  }
};


}  // un-named

void xreg::RayCasterOccludingContoursCPU::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);
  
  // Get the index bounding box (axis-aligned in the index space) of the volume
  Pt3 img_aabb_min;
  Pt3 img_aabb_max;
  std::tie(img_aabb_min,img_aabb_max) = ITKImageIndexBoundsAsEigen(this->vols_[vol_idx].GetPointer());

  // Compute the frame transform from ITK physical space to index space (sR + t)
  const FrameTransform itk_idx_to_itk_phys_pt_xform = ITKImagePhysicalPointTransformsAsEigen(this->vols_[vol_idx].GetPointer());

  const FrameTransform itk_phys_pt_to_itk_idx_xform = itk_idx_to_itk_phys_pt_xform.inverse();

  this->pre_compute();

  ContourFn contour_fn = { this->vols_[vol_idx],
                           img_aabb_min,
                           img_aabb_max,
                           itk_phys_pt_to_itk_idx_xform,
                           this->num_projs_,
                           this->camera_models_,
                           this->xforms_cam_to_itk_phys_,
                           this->cam_model_for_proj_,
                           this->ray_step_size_,
                           this->interp_method_,
                           this->pixel_buf_to_use(),
                           this->render_thresh(),
                           this->num_backtracking_steps(),
                           this->occlusion_angle_thresh_rad(),
                           this->stop_after_collision()
                         };

  // For every single projection pixel
  ParallelFor(contour_fn,
              RangeType(0, this->num_projs_ * this->camera_models_[0].num_det_rows *
                           this->camera_models_[0].num_det_cols));

  this->sync_to_ocl_.set_modified();
  this->sync_to_host_.set_modified();
}

