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

#include "xregAnatCoordFrames.h"

#include "xregAssert.h"

namespace
{

using namespace xreg;

template <class Itr>
void ConvertRASToLPSPtListHelper(Itr begin_it, Itr end_it)
{
  for (Itr it = begin_it; it != end_it; ++it)
  {
    ConvertRASToLPS(&*it);
  }
}

template <class Itr>
void ConvertRASToLPSLandMapHelper(Itr begin_it, Itr end_it)
{
  for (Itr it = begin_it; it != end_it; ++it)
  {
    ConvertRASToLPS(&it->second);
  }
}

}  // un-named

void xreg::ConvertRASToLPS(Pt3List* pts)
{
  ConvertRASToLPSPtListHelper(pts->begin(), pts->end());
}

void xreg::ConvertRASToLPS(LandMap3* pts)
{
  ConvertRASToLPSLandMapHelper(pts->begin(), pts->end());
}

void xreg::ConvertRASToLPS(LandMultiMap3* pts)
{
  ConvertRASToLPSLandMapHelper(pts->begin(), pts->end());
}

void xreg::ConvertRASToLPS(Pt3* p)
{
  p->operator[](0) *= -1;
  p->operator[](1) *= -1;
}

xreg::FrameTransform xreg::AnteriorPelvicPlaneFromLandmarks(const Pt3& left_asis,
                                                            const Pt3& right_asis,
                                                            const Pt3& left_aps,
                                                            const Pt3& right_aps)
{
  constexpr CoordScalar kTOL = 1.0e-6;
  
  xregASSERT((left_asis - right_asis).norm() > kTOL);

  const Pt3 origin = (left_asis + right_asis) / 2;
  const Pt3 x_axis = (left_asis - right_asis).normalized();

  Pt3 y_axis = (origin - ((left_aps + right_aps) / 2)).normalized();
  // It could be that x and y are not orthogonal - remove the x components from y
  y_axis -= y_axis.dot(x_axis) * x_axis;
  y_axis.normalize();
  xregASSERT(std::abs(y_axis.dot(x_axis)) < kTOL);

  const Pt3 z_axis = x_axis.cross(y_axis);

  FrameTransform app_to_world = FrameTransform::Identity();
  app_to_world.matrix().block(0,0,3,1) = x_axis;
  app_to_world.matrix().block(0,1,3,1) = y_axis;
  app_to_world.matrix().block(0,2,3,1) = z_axis;
  app_to_world.matrix().block(0,3,3,1) = origin;
  
  return app_to_world;
}

xreg::FrameTransform
xreg::AnteriorPelvicPlaneFromLandmarksMap(const LandMap3& landmarks_map,
                                          const APPCenterPt origin_pt)
{
  auto asis_l_it = landmarks_map.find("ASIS-l");
  auto asis_r_it = landmarks_map.find("ASIS-r");
  auto ips_l_it  = landmarks_map.find("IPS-l");
  auto ips_r_it  = landmarks_map.find("IPS-r");

  xregASSERT(asis_l_it != landmarks_map.end());
  xregASSERT(asis_r_it != landmarks_map.end());
  xregASSERT(ips_l_it  != landmarks_map.end());
  xregASSERT(ips_r_it  != landmarks_map.end());

  FrameTransform app_to_world = AnteriorPelvicPlaneFromLandmarks(asis_l_it->second, asis_r_it->second,
                                                                 ips_l_it->second, ips_r_it->second);

  // determine if, and where, we should change the origin to.
  auto fh_it = landmarks_map.end();

  switch (origin_pt)
  {
  case kAPP_ORIGIN_RIGHT_FH:
    fh_it = landmarks_map.find("FH-r");
    xregASSERT(fh_it != landmarks_map.end());
    break;
  case kAPP_ORIGIN_LEFT_FH:
    fh_it = landmarks_map.find("FH-l");
    xregASSERT(fh_it != landmarks_map.end());
    break;
  case kAPP_ORIGIN_MEDIAN_ILIAC_CREST:
  default:
    // origin already set here
    break;
  }

  if (fh_it != landmarks_map.end())
  {
    app_to_world.matrix().block(0,3,3,1) = fh_it->second;
  }
  
  return app_to_world;
}

xreg::FrameTransform
xreg::CreateAPViewOfLPSVol(const CoordScalar off_from_cam_origin_dist,
                           const Pt3& intersect_pt_wrt_vol,
                           const bool src_is_posterior,
                           const bool pat_is_up,
                           const CameraModel::CameraCoordFrame coord_frame_type)
{
  // This is what the camera z-axis and y-axis will map to
  Pt3 cam_z_axis_wrt_vol = Pt3::Zero();
  Pt3 cam_y_axis_wrt_vol = Pt3::Zero();

  if (coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
  {
    cam_z_axis_wrt_vol(1) = src_is_posterior ? -1 : 1;
    cam_y_axis_wrt_vol(2) = pat_is_up ? -1 : 1;
  }
  else if (coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
  {
    cam_z_axis_wrt_vol(1) = src_is_posterior ? 1 : -1;
    cam_y_axis_wrt_vol(2) = pat_is_up ? -1 : 1;
  }
  else
  {
    // origin on detector, positive z to source
    cam_z_axis_wrt_vol(1) = src_is_posterior ? 1 : -1;
    cam_y_axis_wrt_vol(2) = pat_is_up ? 1 : -1;
  }

  const Pt3 off_from_cam_origin_vec = cam_z_axis_wrt_vol * off_from_cam_origin_dist;

  // This is the translation
  const Pt3 cam_origin_wrt_vol = intersect_pt_wrt_vol - off_from_cam_origin_vec;

  // right handedness
  const Pt3 cam_x_axis_wrt_vol = cam_y_axis_wrt_vol.cross(cam_z_axis_wrt_vol);

  FrameTransform cam_to_vol = FrameTransform::Identity();
  cam_to_vol.matrix().block(0,0,3,1) = cam_x_axis_wrt_vol;
  cam_to_vol.matrix().block(0,1,3,1) = cam_y_axis_wrt_vol;
  cam_to_vol.matrix().block(0,2,3,1) = cam_z_axis_wrt_vol;
  cam_to_vol.matrix().block(0,3,3,1) = cam_origin_wrt_vol;

  return cam_to_vol;
}

xreg::FrameTransform
xreg::CreateAPViewOfLPSVol(const CameraModel& cam,
                           const Pt3& intersect_pt_wrt_vol,
                           const CoordScalar intersect_pt_loc_on_src_det_line,
                           const bool src_is_posterior,
                           const bool pat_is_up,
                           const Pt2& intersect_pt_proj_idx)
{
  FrameTransform cam_to_vol = CreateAPViewOfLPSVol(0,  // we'll handle the translation next
                                                   intersect_pt_wrt_vol,
                                                   src_is_posterior,
                                                   pat_is_up,
                                                   cam.coord_frame_type);
 
  const Pt3 inter_pt_wrt_cam = cam.extrins * (cam.pinhole_pt +
                                 (intersect_pt_loc_on_src_det_line *
                                    (cam.ind_pt_to_phys_det_pt(intersect_pt_proj_idx) - cam.pinhole_pt)));
  
  cam_to_vol.matrix().block(0,3,3,1) = intersect_pt_wrt_vol -
                                          (cam_to_vol.matrix().block(0,0,3,3) * inter_pt_wrt_cam);

  // we really want the pose of the camera world frame with respect to the volume,
  // therefore the extra extrinsic multiplication
  return cam_to_vol * cam.extrins;
}

xreg::FrameTransform
xreg::CreateAPViewOfLPSVol(const CameraModel& cam,
                           const Pt3& intersect_pt_wrt_vol,
                           const CoordScalar intersect_pt_loc_on_src_det_line,
                           const bool src_is_posterior,
                           const bool pat_is_up)
{
  Pt2 center_idx;
  center_idx(0) = cam.num_det_cols / 2.0;
  center_idx(1) = cam.num_det_rows / 2.0;
  
  return CreateAPViewOfLPSVol(cam, intersect_pt_wrt_vol, intersect_pt_loc_on_src_det_line,
                              src_is_posterior, pat_is_up, center_idx);
}

xreg::FrameTransform
xreg::CreateAPViewOfAPP(const CameraModel& cam,
                        const CoordScalar intersect_pt_loc_on_src_det_line,
                        const bool src_is_posterior,
                        const bool pat_up,
                        const Pt3& ref_pt_wrt_app,
                        const Pt2& ref_pt_2d_idx)
{
  FrameTransform cam_to_app = FrameTransform::Identity();
  
  Pt3 cam_y_axis_wrt_app = Pt3::Zero();
  Pt3 cam_z_axis_wrt_app = Pt3::Zero();
  
  if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
  {
    // If we want the patient superior to be in the top of the image, and
    // inferior towards the bottom of the image, this is the opposite
    // direction of the camera coordinate Y-Axis, flip it. 
    cam_y_axis_wrt_app(1) = pat_up ? -1 : 1;
    
    // When the source is posterior, the source to detector vector needs to
    // point posterior -> anterior, which is consistent with the APP Z-Axis,
    // when the source is anterior, the direction flips.
    cam_z_axis_wrt_app(2) = src_is_posterior ? 1 : -1;
  }
  else if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
  {
    // If we want the patient superior to be in the top of the image, and
    // inferior towards the bottom of the image, this is the opposite
    // direction of the camera coordinate Y-Axis, flip it. 
    cam_y_axis_wrt_app(1) = pat_up ? -1 : 1;
    
    cam_z_axis_wrt_app(2) = src_is_posterior ? -1 : 1;
  }
  else
  {
    // TODO: explain why this is correct
    cam_y_axis_wrt_app(1) = pat_up ? -1 : 1;
    cam_z_axis_wrt_app(2) = src_is_posterior ? -1 : 1;
  }

  // populate the rotation matrix
  cam_to_app.matrix().block(0,0,3,1) = cam_y_axis_wrt_app.cross(cam_z_axis_wrt_app);  // ensure RH frame
  cam_to_app.matrix().block(0,1,3,1) = cam_y_axis_wrt_app;
  cam_to_app.matrix().block(0,2,3,1) = cam_z_axis_wrt_app;

  // now compute the translation component

  const Pt3 pinhole_pt_wrt_cam = cam.extrins * cam.pinhole_pt;

  const Pt3 ref_pt_wrt_cam = pinhole_pt_wrt_cam +
                                (intersect_pt_loc_on_src_det_line *
                                  ((cam.extrins * cam.ind_pt_to_phys_det_pt(ref_pt_2d_idx)) - pinhole_pt_wrt_cam));
  
  cam_to_app.matrix().block(0,3,3,1) = ref_pt_wrt_app - (cam_to_app.matrix().block(0,0,3,3) * ref_pt_wrt_cam);

  // I want cam_extrins_to_app = cam_to_app * cam_extrins_to_cam
  return cam_to_app * cam.extrins;
}

xreg::FrameTransform
xreg::CreateAPViewOfAPP(const CameraModel& cam,
                        const CoordScalar intersect_pt_loc_on_src_det_line,
                        const bool src_is_posterior,
                        const bool pat_up)
{
  Pt2 center_idx_2d;
  center_idx_2d(0) = cam.num_det_cols / 2.0;
  center_idx_2d(1) = cam.num_det_rows / 2.0;

  return CreateAPViewOfAPP(cam, intersect_pt_loc_on_src_det_line,
                           src_is_posterior, pat_up, Pt3::Zero(), center_idx_2d);
}

