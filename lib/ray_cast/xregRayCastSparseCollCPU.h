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

#ifndef XREGRAYCASTSPARSECOLLCPU_H_
#define XREGRAYCASTSPARSECOLLCPU_H_

#include "xregRayCastInterface.h"

namespace xreg
{

/// \brief Ray caster from a sparse collection of points in the camera coordinate
///        and testing for collision with a volume.
///
/// This does not create "projections" like most of the other ray casters, so some
/// methods that return images, etc. will throw exceptions. Perhaps a more accurate
/// term would be view instead of projection.
/// Each camera model may have a list of points in its extrinsic frame.
/// For each projection/view the points associated with the assigned camera model
/// are cast through the volume and check for collision with intensity above a 
/// threshold. Additionally, if there is an intersection/collision, casting may
/// continue until the ray does not exceed the intensity threshold.
/// The intersection rays can be oriented towards the camera model pinhole point,
/// or away from it.
class RayCasterSparseCollisionCPU : public RayCaster, public RayCasterCollisionParamInterface
{
public:
  using DistList       = std::vector<CoordScalar>;
  using RayIndLUT      = std::vector<std::tuple<size_type,size_type>>;
  using ListOfPt3Lists = std::vector<Pt3List>;
  
  RayCasterSparseCollisionCPU() = default;

  /// \brief Allocate resources required for computing each ray cast.
  ///
  /// This will allocate data structures in host memory large enough
  /// to store all intersection distances and entrance/exit points.
  /// The number of projections and camera models should be set prior to calling this.
  void allocate_resources() override;

  /// \brief Perform each ray cast.
  void compute(const size_type vol_idx = 0) override;
  
  /// \brief No projection images computed in this class - throws
  ProjPtr proj(const size_type proj_idx) override;

  /// \brief No projection images computed in this class - throws
  cv::Mat proj_ocv(const size_type proj_idx) override;

  /// \brief No projection images computed in this class - throws
  PixelScalar2D* raw_host_pixel_buf() override;

  /// \brief No projection images computed in this class - throws
  void use_external_host_pixel_buf(void* buf) override;

  /// \brief The maximum number of projections that is possible to allocate
  ///        resources for (as limited by hardware or the system).
  ///
  /// For the CPU, we assume that unlimted memory is available and this returns
  /// the largest possible value of size_type.
  size_type max_num_projs_possible() const override;

  /// \brief No projection images computed in this class - throws
  void use_other_proj_buf(RayCaster* other_ray_caster) override;

  bool orient_rays_towards_pinhole() const;

  void set_orient_rays_towards_pinhole(const bool orient_towards_pinhole);

  bool find_exit_pts() const;

  void set_find_exit_pts(const bool find_exit_pts);

  void set_pts_for_cam(const Pt3List& pts_wrt_cam, const size_type cam_idx = 0);

  const Pt3List& entry_coll_pts_wrt_vol(const size_type view_idx = 0) const;

  const Pt3List& exit_coll_pts_wrt_vol(const size_type view_idx = 0) const;

  const DistList& intersect_dists(const size_type view_idx = 0) const;

private:
  // global ray index -> (view index, view ray index)
  RayIndLUT ray_idx_lut_;

  ListOfPt3Lists pts_wrt_each_cam_ext_;

  ListOfPt3Lists entry_coll_pts_wrt_vol_for_each_view_;
  
  ListOfPt3Lists exit_coll_pts_wrt_vol_for_each_view_;

  std::vector<DistList> entry_coll_intersect_dists_for_each_view_;

  bool orient_towards_cam_pinhole_ = true;

  bool find_exit_pts_ = false;
};

}  // xreg

#endif

