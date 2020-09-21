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

#ifndef XREGANATCOORDFRAMES_H_
#define XREGANATCOORDFRAMES_H_

/// \file
/// \brief Utilities for dealing with transformations related to anatomical coordinate frames.

#include "xregCommon.h"
#include "xregPerspectiveXform.h"

namespace xreg
{

/// \brief Convert Right-Anterior-Superior (RAS) points to Left-Posterior-Superior (LPS) Points.
///
/// This negates the first two components of each point.
void ConvertRASToLPS(Pt3List* pts);

void ConvertRASToLPS(LandMap3* pts);

void ConvertRASToLPS(LandMultiMap3* pts);

void ConvertRASToLPS(Pt3* p);

/// \brief Compute the Anterior Pelvic Plane coordinate frame using anatomical
///        landmarks.
///
/// The paper by Nikou, et al. calls for the following axes:
///   X-Axis: Points to the patient's left, parallel to the line joining  the iliac spine points
///   Y-Axis: Points to the superior, perpendicular to the other axes (but on the plane), so really just the superior vector projected on the plane
///   Z-Axis: x cross y
///   Origin: The median of the iliac crest points
/// The computed transform maps points in the APP frame to the world frame the
/// anatomical landmarks are defined in.
/// Eigen matrix and transform types are assumed.
FrameTransform AnteriorPelvicPlaneFromLandmarks(const Pt3& left_asis,
                                                const Pt3& right_asis,
                                                const Pt3& left_aps,
                                                const Pt3& right_aps);

/// \brief Identifier for the point to use as the origin in an Anterior Pelvic
///        Plane coordinate frame.
enum APPCenterPt
{
  kAPP_ORIGIN_MEDIAN_ILIAC_CREST,
  kAPP_ORIGIN_RIGHT_FH,
  kAPP_ORIGIN_LEFT_FH
};

/// \brief Compte the Anterior Pelvic Plane coordinate frame using a mapping from
///        landmark names to points.
///
/// To compute the APP, left and right Anterior Superior Iliac Spines ("ASIS-{l,r}")
/// and left and right Inferior Pubis Symphisis ("IPS-{l,r}") are required.
/// For shifting the origin to the center of a femural head, the left or right
/// femural heads are required ("FH-{l,r}").
/// \see AnteriorPelvicPlaneFromLandmarks
FrameTransform AnteriorPelvicPlaneFromLandmarksMap(const LandMap3& landmarks_map,
                                                   const APPCenterPt origin_pt = kAPP_ORIGIN_MEDIAN_ILIAC_CREST);

/// \brief Create an AP view of a volume with LPS coordinate frame.
///
/// \see CameraModel for camera coordinate frame definition.
/// This differs from the version that takes a camera model, in that this
/// transformation is that the output transformation assumes identity extrinsic
/// and takes a generic distance that does not rely on focal length of the
/// camera (e.g. for an optical camera with small focal distance, this is more
/// useful).
FrameTransform
CreateAPViewOfLPSVol(const CoordScalar off_from_cam_origin_dist,
                     const Pt3& intersect_pt_wrt_vol,
                     const bool src_is_posterior = true,
                     const bool pat_is_up = true,
                     const CameraModel::CameraCoordFrame coord_frame_type =
                                    CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z);

/// \brief Create an AP X-Ray view of a volume with LPS coordinate frame.
///
/// \see CameraModel for camera coordinate frame definition.
/// \param intersect_pt_loc_on_src_det_line is in [0,1] and indicates how far
///        along the intersection point is on the source to detector line.
///        e.g. 0.5 indicates midway, 0.25 indicates it is closer to the source
///        than detector, and 0.75 indicates it is closer to the detector than
///        the source. The intersect_pt_proj_idx indicates where the intersection
///        point in 3D should project to in 2D.
FrameTransform
CreateAPViewOfLPSVol(const CameraModel& cam,
                     const Pt3& intersect_pt_wrt_vol,
                     const CoordScalar intersect_pt_loc_on_src_det_line,
                     const bool src_is_posterior,
                     const bool pat_is_up,
                     const Pt2& intersect_pt_proj_idx);

/// \brief Create an AP X-Ray view of a volume with LPS coordinate frame.
///
/// \see CameraModel for camera coordinate frame definition.
/// \param intersect_pt_loc_on_src_det_line is in [0,1] and indicates how far
///        along the intersection point is on the source to detector line.
///        e.g. 0.5 indicates midway, 0.25 indicates it is closer to the source
///        than detector, and 0.75 indicates it is closer to the detector than
///        the source. This projects the 3D intersection point to the center of
///        the detector.
FrameTransform
CreateAPViewOfLPSVol(const CameraModel& cam,
                     const Pt3& intersect_pt_wrt_vol,
                     const CoordScalar intersect_pt_loc_on_src_det_line,
                     const bool src_is_posterior = true,
                     const bool pat_is_up = true);

/// \brief Create an AP view of an Anterior Pelvic Plane (APP) coordinate frame
///
/// A 3D reference point is used to project/align to a 2D reference index. The camera/C-Arm
/// source to detector axis will be aligned with the APP Z-axis, and the image
/// is oriented so that the patient superior is on the top of the image.
/// \see CreateAPViewOfLPSVol for a description of intersect_pt_loc_on_src_det_line.
/// This returns the camera extrinsic frame with respect to the APP; e.g. maps camera
/// world coordinates to the APP.
FrameTransform
CreateAPViewOfAPP(const CameraModel& cam,
                  const CoordScalar intersect_pt_loc_on_src_det_line,
                  const bool src_is_posterior,
                  const bool pat_up,
                  const Pt3& ref_pt_wrt_app,
                  const Pt2& ref_pt_2d_idx);

/// \brief Create an AP view of an Anterior Pelvic Plane (APP) coordinate frame
///
/// The origin of the APP frame will be aligned to the center of the detector
/// See version with reference points for more details.
FrameTransform
CreateAPViewOfAPP(const CameraModel& cam,
                  const CoordScalar intersect_pt_loc_on_src_det_line,
                  const bool src_is_posterior = true,
                  const bool pat_up = true);

}  // xreg

#endif

