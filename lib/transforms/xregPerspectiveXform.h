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

#ifndef XREGPERSPECTIVEXFORM_H_
#define XREGPERSPECTIVEXFORM_H_

#include "xregCommon.h"

/// \file
/// \brief Utilities for dealing with perspective transforms and perspective camera models.

namespace xreg
{

/// \brief Decomposes a 3x4 projection matrix into the 3x3 intrinsic and 4x4
///        extrinsic matrices.
/// The user may also specify the positivity of the scaling factor rho; this
/// determines the direction of the camera z-axis (the difference is a rotation
/// of 180 degrees about the camera y-axis).
/// Optionally, the scaling factor rho may be returned so that the input
/// projection matrix may be recovered as P = 1/rho * K * T.
/// In practice, this is not useful as homogeneous pixel coordinates are recovered
/// by scaling the third component to be one, which cancels the multiplication
/// by rho.
/// E.g. [qx, qy, s] = K * T * [x,y,z,1] --> [px, py, 1] = [qx, qy, s] / s
std::tuple<Mat3x3,Mat4x4,CoordScalar>
DecompProjMat(const Mat3x4& P, const bool use_pos_rho);

/// \brief Decompose a 3x4 projection matrix using a QR factorization and some permutations
std::tuple<Mat3x3,Mat4x4,CoordScalar>
DecompProjMatQR(const Mat3x4& P);

/// \brief Compute focal length based on camera intrinsic matrix and known pixel
///        spacings.
///
/// When yps is < 0 (default when not provided) pixels are assumed to be square.
CoordScalar FocalLenFromIntrins(const Mat3x3& K, CoordScalar xps, CoordScalar yps = -1);

/// \brief Computes a 3x4 projection matrix given an intrinsic matrix and extrinsic matrix.
///
/// This computes: [ intrins, 0 ] * extrins
Mat3x4 ProjMat3x4FromIntrinsExtrins(const Mat3x3& intrins, const Mat4x4& extrins);

/// \brief Create a camera intrinsic/calibration matrix in a naive manner based
///        on some simple metadata.
///
/// The principal point is assumed to be in the center of the image, and there
/// is no skew/shear.
/// Pass true for z_is_neg if you want the detector to source vector to be
/// positive in the z direction, otherwise the source to detector vector
/// is positive in the z direction.
Mat3x3 MakeNaiveIntrins(const CoordScalar focal_len,
                        const unsigned long num_rows,
                        const unsigned long num_cols,
                        const CoordScalar pixel_row_spacing,
                        const CoordScalar pixel_col_spacing,
                        const bool z_is_neg = false);

/// \brief Data used to define a pinhole camera and the projection geometry
///
/// This model is capable of representing three different camera coordinate frames:
/// 1) (DEFAULT) Origin at focal point, z axis pointing away from the detector and
///    intersecting with the center, x axis aligned with detector column direction
///    and y axis aligned with detector row direction. The camera origin index is 
///    in the top left of the image. 
/// 2) Origin at focal point, z axis towards the detector and intersecting
///    with the center, x axis aligned with detector column direction and y axis
///    aligned with detector row direction. The camera origin index is in the top
///    left of the image. 
/// 3) Origin on the detector, z axis towards the focal point and
///    intersecting it, axis aligned with detector column direction and y axis
///    aligned with the detector row direction. The camera origin index is in the
///    bottom left of the image.
/// This is controlled via the coord_frame_type member, which should be set before
/// calling the setup methods described below.
///
/// The model should be setup by calling any of the setup methods.
/// One setup method initializes the camera geometry by taking focal length,
/// pixel spacings, and number of pixels, and creates a trivial geometry.
/// In this case, the principal point is defined to be at the
/// center of the detector and that there is zero shear.
/// The second setup method initializes the camera geometry with an input
/// 3x4 projection matrix and the number of pixels and pixel spacings.
/// The third method is similar to the second, but takes the intrinsic and
/// extrinsic matrices instead of the projection matrix; this avoids the
/// ambiguity in sign when decomposing a projection matrix.
struct CameraModel
{
  // Using an Eigen::Array of Points is OK, so long as no numerical operations are performed; e.g. the *= operator
  using Point3DGrid = Eigen::Array<Pt3,Eigen::Dynamic,Eigen::Dynamic>;

  /// \brief Determines the layout of the camera coordinate frame
  ///
  /// kORIGIN_AT_FOCAL_PT_DET_POS_Z: origin at the focal point, z axis towards the detector
  /// and intersecting with the center, x axis aligned with detector column direction
  /// and y axis aligned with detector row direction. The camera origin index is in 
  /// the top left of the image.
  ///
  /// kORIGIN_AT_FOCAL_PT_DET_NEG_Z: origin at the focal point, negative z axis towards the detector
  /// and intersecting with the center, x axis aligned with detector column direction
  /// and y axis aligned with detector row direction. This is useful for our CIOS Fusion.
  ///
  /// kORIGIN_ON_DETECTOR: origin on the detector, z axis 
  /// towards the focal point and intersecting it, axis aligned with detector column
  /// direction and y axis aligned with the detector row direction. The camera origin
  /// index is in the bottom left of the image.
  enum CameraCoordFrame
  {
    kORIGIN_AT_FOCAL_PT_DET_POS_Z,
    kORIGIN_AT_FOCAL_PT_DET_NEG_Z,
    kORIGIN_ON_DETECTOR
  };

  /// \brief Transforms normalized camera coordinates to homogeneous pixel coordinates
  ///
  /// The user should remember to scale the input, or output, so that the z-component
  /// is equal to one.
  /// This has the structure:
  /// [ focal_len / det_col_spacing,                skew,          column principal point;
  ///              0,                focal_len / det_col_spacing,   row principal point;
  ///              0,                                 0,                    1]
  Mat3x3 intrins = Mat3x3(Mat3x3::Identity());

  /// \brief Transforms normalized (homogeneous) pixel coordinates into camera coordinates.
  ///
  /// Need to scale output by the focal length to get physical point on the detector.
  /// e.g. K^-1 * [px; py; 1] * f -> [x,y,z] (where z == f)
  /// Can scale by arbitrary z to get the coordinate on the epipolar line with
  /// that z coordinate value.
  /// The above is valid for the origin located at the focal point.
  /// The scale by f changes to f-z for the origin on the detector, and the z component 
  /// must be negated and f subtracted:
  /// e.g. K^-1 * [px; py; 1] * (f-z) -> [x,y,f-z] -> [x,y,z].
  Mat3x3 intrins_inv = Mat3x3(Mat3x3::Identity());

  FrameTransform extrins     = FrameTransform::Identity();  ///< Transforms "world" coordinates to camera coordinates
  FrameTransform extrins_inv = FrameTransform::Identity();  ///< Transforms camera coordinates to "world" coordinates

  Pt3 pinhole_pt = Pt3(Pt3::Zero());   ///< Location of the camera pinhole in "world" coordinates

  CoordScalar focal_len = 0;   ///< Distance between the focal/pinhole point and the principal point on the detector

  size_type num_det_rows = 0;  ///< Number of detector rows (number of vertical pixels; e.g. height)
  size_type num_det_cols = 0;  ///< Number detector columns (number of horizontal pixels; e.g. width)

  CoordScalar det_row_spacing = 0;  ///< The spacing between detector rows/pixels (Equivalent to pixel size when the spacing is zero).
  CoordScalar det_col_spacing = 0;  ///< The spacing between detector columns/pixels (Equivalent to pixel size when the spacing is zero).

  /// \brief Defines where the camera origin and axes are.
  ///
  /// \see CameraCoordFrame
  CameraCoordFrame coord_frame_type = kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  /// \brief Initialize a basic camera model.
  ///
  /// Useful when we do not have digital information; e.g. from a projection matrix
  /// TODO: This can be made more general by having optional parameters for the
  ///       principal point and shear.
  void setup(const CoordScalar focal_len_arg, const size_type nr, const size_type nc,
             const CoordScalar rs, const CoordScalar cs);

  /// \brief Initialize a camera model using a projection matrix.
  ///
  /// This is consistent with the projection information provided in a PRS file,
  /// obtained from the prototype Siemens flat panel.
  void setup(const Mat3x4& proj_mat, const size_type nr, const size_type nc,
             const CoordScalar rs, const CoordScalar cs,
             const bool use_extrins = true);

  /// \brief Initialize a camera model using existing intrinsic and extrinsic
  ///        matrices.
  void setup(const Mat3x3& intrins_mat, const Mat4x4& extrins_mat,
             const size_type nr, const size_type nc,
             const CoordScalar rs, const CoordScalar cs);

  /// \brief Project a single point onto the detector plane.
  ///
  /// This projects a point in "world" coordinates, and returns a point in
  /// "world"  coordinates.
  /// For the projected output, with respect to the camera, the z component
  /// will be equal to the focal length, or zero depending on coord_frame_type,
  /// however the output is with respect to "world" coordinates and may have
  /// an arbitrary z component value.
  /// NOTE: This does not account for any shearing in the projection.
  Pt3 proj_pt_to_det_pt(const Pt3& src_pt) const;

  /// \brief Projects a collection of points onto the detector plane.
  ///
  /// The input points and output points are in "world" coordinates.
  /// \see proj_pt_to_det_pt
  Pt3List proj_pts_to_det(const Pt3List& src_pts) const;

  /// \brief Convert a physical point to the continuous index into the array of detectors
  ///
  /// The physical point is in "world" coordinates.
  /// For the output point, the first component corresponds to column (analog to x in
  /// physical), the second component corresponds to row (analog to y in physical), and
  /// the third component is always set to one, for homogeneous 2D indices.
  /// Essentially, this method performs the projection of a camera world 3D point to a
  /// 2D index in the image.
  Pt3 phys_pt_to_ind_pt(const Pt3& phys_pt) const;

  /// \brief Convert a collection of physical points to their corresponding continuous indices
  ///
  /// The physical points are in "world" coordinates.
  /// \see phys_pt_to_ind_pt
  Pt3List phys_pts_to_ind_pts(const Pt3List& phys_pts) const;

  /// \brief Convert a continuous index point to a physical point (on the detector)
  ///
  /// The physical point will be in "world" coordinates.
  /// ind_pt[0] -> col, ind_pt[1] -> row
  /// phys_pt[0] -> x, phys_pt[1] -> y
  /// The output z component will be equal to the focal length, or zero depending on
  /// coord_frame_type, with respect to
  /// the camera coordinate frame, however it may be different in the camera
  /// "world frame."
  Pt3 ind_pt_to_phys_det_pt(const Pt2& ind_pt) const;

  /// \brief Convert a continuous index point to a physical point (on the detector)
  ///
  /// The continuous index should be specified as a 2D homogeneous point
  /// (e.g. third component equal to one)
  /// The physical point will be in "world" coordinates.
  /// ind_pt[0] -> col, ind_pt[1] -> row
  /// phys_pt[0] -> x, phys_pt[1] -> y
  /// The output z component will be equal to the focal length, or zero depending on
  /// coord_frame_type, with respect to
  /// the camera coordinate frame, however it may be different in the camera
  /// "world frame."
  Pt3 ind_pt_to_phys_det_pt(const Pt3& ind_pt) const;

  /// \brief Convert a collection of continuous indices into physical points.
  ///
  /// The physical point outputs will be in "world" coordinates.
  /// \see ind_pt_to_phys_det_pt
  Pt3List ind_pts_to_phys_det_pts(const Pt3List& ind_pts) const;

  // TODO: given point on detector, find line to source

  bool operator==(const CameraModel& other) const;

  bool operator!=(const CameraModel& other) const;
  
  /// \brief Given valid detector parameters already set, this creates the grid
  ///        of 3D detector points.
  ///
  /// This is a "virtual" camera focal plane in the sense that it lies in "front"
  /// of the camera, so that the image does not need to be flipped.
  /// The points are with respect to "world" coordinates.
  Point3DGrid detector_grid() const;
};

/// \brief Creates a new camera model that corresponds to a ROI in an existing
///        model, but has the same projection geometry.
///
/// Unless the ROI is centered in the input model, then the new model will generally
/// have a non-center, perhaps outside of the new image, principal point location. 
/// The ROI may have extent outside of the existing physical extent of the image,
/// using negative start col/row, or larger ending col/row's than the current size.
/// The start col/row must be less than the ending col/row.
CameraModel UpdateCameraModelFor2DROI(const CameraModel& src_cam,
                                      const int roi_start_col,
                                      const int roi_start_row,
                                      const int roi_end_col,
                                      const int roi_end_row);

/// \brief Given a set of 3D points in camera world coordinates, compute a
///        a bounding box about their projections in the 2D detector plane.
///
/// This bounding box may lie outside of the 2D image bounds; it is up to the
/// user to clamp values, etc.
/// Returns a tuple representing the bounding box; first element is the top
/// left point, second element is the bottom right
std::tuple<Pt2,Pt2>
GetBoundingBox2DProjPts(const CameraModel& cam, const Pt3List& pts_3d);

/// \brief Given a set of 3D points in camera world coordinates, compute a
///        camera model that will only project to a bounding box about
///        the projected 3D points.
///
/// pad_rows and pad_cols indicate a number of pixels (in 2D rows/cols) that
/// should be used to enlarge the computed bounding box.
CameraModel UpdateCameraModelTightBoundsForProjPts(const CameraModel& cam,
                                                   const Pt3List& pts_3d,
                                                   const CoordScalar pad_cols = 0,
                                                   const CoordScalar pad_rows = 0);

/// \brief Simulate the movement of a focal point source, but keeping the
///        detector in the same physical location.
///
/// The src_delta is with respect to camera coordinate axes.
///
/// So you can get something like this (exaggerated):
///       ________
///        /
///       /
///      /
///     *
CameraModel MoveFocalPointUpdateCam(const CameraModel& cam,
                                    const Pt3 src_delta);

/// \brief compute the adjustment to camera 1's source position to get the intrinsics in camera 2
Pt3 CalcSourcePositionDelta(const CameraModel& cam1, const CameraModel& cam2);

/// \brief "Downsamples" a camera model - e.g. updates the parameters that would
///        produce a downsampled image.
///
/// A factor of 1 retains the original size, a factor less than 1 downsamples,
/// and a factor greater than 1 upsamples.
CameraModel DownsampleCameraModel(const CameraModel& src_cam, const CoordScalar ds_factor,
                                  const bool force_even_dims = false);

/// \brief Create a new camera world (extrinsic) frame based on a collection of
///        frame transforms from each camera to the new frame.
///
/// An example may be to use an object, e.g. pelvis volume, as a fiducial in a
/// series of single-view registrations and then use that objects coordinate frame
/// as the new camera world frame.
std::vector<CameraModel>
CreateCameraWorldUsingFiducial(const std::vector<CameraModel>& orig_cams,
                               const std::vector<FrameTransform>& cams_to_fid);

/// \brief Print Camera parameters to an output stream; useful for debugging.
void PrintCam(std::ostream& out, const CameraModel& cam);

}  // xreg

#endif
