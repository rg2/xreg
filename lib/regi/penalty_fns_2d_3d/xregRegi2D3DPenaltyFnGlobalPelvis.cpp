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

#include "xregRegi2D3DPenaltyFnGlobalPelvis.h"

#include "xregTBBUtils.h"
#include "xregPerspectiveXform.h"
#include "xregExceptionUtils.h"

namespace  // un-named
{

using namespace xreg;

CoordScalar AmountProjOutsideBoundsSquared(const CameraModel& cam,
                                           const Pt3& p)
{
  const Pt2 proj_pt = cam.phys_pt_to_ind_pt(p).head(2);

  const bool small_col = proj_pt(0) < -1.0e-6;
  const bool small_row = proj_pt(1) < -1.0e-6;
  const bool big_col   = proj_pt(0) > (cam.num_det_cols - (1 - 1.0e-6));
  const bool big_row   = proj_pt(1) > (cam.num_det_rows - (1 - 1.0e-6));

  const CoordScalar col_dist = small_col ? proj_pt(0) :
                            (big_col ? (proj_pt(0) - cam.num_det_cols + 1) :
                                       CoordScalar(0));
  const CoordScalar row_dist = small_row ? proj_pt(1) :
                            (big_row ? (proj_pt(1) - cam.num_det_rows + 1) :
                                       CoordScalar(0));

  return (col_dist * col_dist) + (row_dist * row_dist);
}

CoordScalar AmountIofAboveAsisSquared(const CameraModel& cam,
                                      const Pt3& asis_3d,
                                      const Pt3& iof_3d,
                                      const bool pat_is_up)
{
  const Pt2 asis_proj = cam.phys_pt_to_ind_pt(asis_3d).head(2);
  const Pt2 iof_proj  = cam.phys_pt_to_ind_pt(iof_3d).head(2);

  // bottom of image has large row values
  // top has low row values
  // so we want the ASIS to have a smaller row value than the IOF
  // UNLESS the image is rotated 180 degrees so the patient is not
  // "up" but "down;" flip the signs in this case - we get this info
  // from the CIOS metadata

  // minus closest_dist_allowed since we don't want the ASIS close to the IOF even when it
  // lies above the IOF
  //const CoordScalar closest_dist_allowed = 2 / cam.det_row_spacing;  // 2 mm
  const CoordScalar closest_dist_allowed = 0;

  // positive values are good
  const CoordScalar row_diff = ((pat_is_up ? 1 : -1) * (iof_proj(1) - asis_proj(1))) - closest_dist_allowed;

  return (row_diff < 1.0e-6) ? (row_diff * row_diff) : CoordScalar(0);
}

CoordScalar DistanceFromAllowableDepthSquared(const CameraModel& cam,
                                              const Pt3& p)
{
  // using extrins rotation to get these vectors in projective frame with z-axis as depth
  const Pt3 src_to_pt_vec  = cam.extrins.matrix().block(0,0,3,3) * (p - cam.pinhole_pt);
  const Pt3 src_to_det_vec = cam.extrins.matrix().block(0,0,3,3) * (cam.proj_pt_to_det_pt(p) - cam.pinhole_pt);
  
  const CoordScalar depth_ratio = (std::abs(src_to_det_vec(2)) > 1.0e-6) ?
                                     (src_to_pt_vec(2) / src_to_det_vec(2)) : CoordScalar(0);
  
  CoordScalar dist = 0;
  
  if (depth_ratio > 0.999999)
  {
    dist = depth_ratio + 1.0e-6;
  }
  else if (depth_ratio < 0.699999)
  {
    dist = (0.7 - depth_ratio) * 10;
  }
  
  return dist * dist;
}

}  // un-named

void xreg::Regi2D3DPenaltyFnGlobalPelvis::compute(
             const ListOfFrameTransformLists& cams_wrt_objs,
             const size_type num_projs,
             const CamList& cams, const CamAssocList& cam_assocs,
             const std::vector<bool>& intermediate_frames_wrt_vol,
             const FrameTransformList& intermediate_frames,
             const FrameTransformList& regi_xform_guesses,
             const ListOfFrameTransformLists* xforms_from_opt)
{
  const FrameTransformList& cam_wrt_vols = cams_wrt_objs[vol_idx];

  xregASSERT(num_projs <= cam_wrt_vols.size());
  this->reg_vals_.resize(num_projs);

  auto compute_reg_for_projs_fn = [&] (const RangeType& r)
  {
    for (size_type i = r.begin(); i < r.end(); ++i)
    { 
      auto& reg_val = this->reg_vals_[i];

      reg_val = 0;

      const FrameTransform cur_vol_wrt_cam = cam_wrt_vols[i].inverse();
      
      const auto& cam = cams[cam_assocs[i]];
    
      const Pt3 left_fh_wrt_cam  = cur_vol_wrt_cam * left_fh_wrt_vol;
      const Pt3 right_fh_wrt_cam = cur_vol_wrt_cam * right_fh_wrt_vol;

      const CoordScalar left_fh_out_dist  = AmountProjOutsideBoundsSquared(
                                              cam, left_fh_wrt_cam);
      const CoordScalar right_fh_out_dist = AmountProjOutsideBoundsSquared(
                                              cam, right_fh_wrt_cam);
      
      reg_val += 2 * (left_fh_out_dist * right_fh_out_dist);

      const CoordScalar left_iof_bad = AmountIofAboveAsisSquared(cam,
                                         cur_vol_wrt_cam * left_asis_wrt_vol,
                                         cur_vol_wrt_cam * left_iof_wrt_vol,
                                         pat_is_up);
      const CoordScalar right_iof_bad = AmountIofAboveAsisSquared(cam,
                                          cur_vol_wrt_cam * right_asis_wrt_vol,
                                          cur_vol_wrt_cam * right_iof_wrt_vol,
                                          pat_is_up);
      
      reg_val += left_iof_bad + right_iof_bad;

      const CoordScalar left_fh_bad_depth  = DistanceFromAllowableDepthSquared(
                                                cam, left_fh_wrt_cam);
      const CoordScalar right_fh_bad_depth = DistanceFromAllowableDepthSquared(
                                                cam, right_fh_wrt_cam);

      reg_val += 2 * (left_fh_bad_depth + right_fh_bad_depth);
    }
  };

  ParallelFor(compute_reg_for_projs_fn, RangeType(0, num_projs));

  if (this->compute_probs_)
  {
    xregThrow("Prob. Not Currently Supported!");
  } 
}

