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

#ifndef XREGCIOSFUSIONDICOM_H_
#define XREGCIOSFUSIONDICOM_H_

#include <tuple>

#include <boost/optional.hpp>

#include "xregCommon.h"
#include "xregPerspectiveXform.h"

// Forward declaration
namespace H5
{
class CommonFG;
}
// End Forward declaration

namespace xreg
{

struct CIOSFusionDICOMInfo
{
  // these could be used for sync'ing with an external device,
  // e.g. gyro, tracker, etc.
  double study_time;
  double series_time;
  double acquisition_time;

  std::string pat_name;
  std::string pat_id;

  boost::optional<double> kvp;

  // This is the distance in mm from the source to the
  // center of the detector
  double dist_src_to_det;

  boost::optional<double> tube_current;
  
  boost::optional<double> exposure;

  boost::optional<double> exposure_mu_As;

  // These are defined to be the spacings between the centers of
  // pixels on the front plane of the image receptor housing.
  double pixel_row_spacing;
  double pixel_col_spacing;

  unsigned long fov_origin_row_off;
  unsigned long fov_origin_col_off;

  enum FOVRot
  {
    kZERO = 0,
    kNINETY = 90,
    kONE_EIGHTY = 180,
    kTWO_SEVENTY = 270
  };

  FOVRot fov_rot = kZERO;

  bool fov_horizontal_flip = false;

  double grid_focal_dist;

  boost::optional<double> tube_current_muA;

  unsigned long rows;
  unsigned long cols;

  double window_center;
  double window_width;
};

/// \brief Read CIOS Fusion DICOM metadata.
CIOSFusionDICOMInfo ReadCIOSFusionDICOMMetadata(const std::string& path);

Mat4x4 CIOSFusionCBCTExtrins();

/// \brief Populate a CIOS Fusion metadata struct with some basic params that match
///        a typical Digital Radiograph (DR) shot.
///
/// The source to detector distance, pixel spacings, and rows/columns are all set to
/// nominal values. Not rotation/flipping flags are set.
CIOSFusionDICOMInfo MakeNaiveCIOSFusionMetaDR();

/// \brief Create a naive camera model using nominal values in the CIOS Fusion DICOM metadata.
///
/// The extrinsic coordinate frame is the same as the camera coordinate frame
/// (e.g. the transformation is identity), unless the second argument is passed true, in which
/// case the origin will lie at the isocenter of the c-arm (previously computed offline).
CameraModel NaiveCamModelFromCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                        const bool iso_center_at_origin = false);

///  \brief Create a naive camera model using nominal values in the CIOS Fusion DICOM metadata and Extrinsic xform.
/// 
/// The extrinsic coordinate frame is read in from program
CameraModel NaiveCamModelFromCIOSFusionExtrins(const CIOSFusionDICOMInfo& meta,
                                               const Mat4x4 extrins);

/// \brief Updates a 2D image with any rotating/flipping flags present
///        in a CIOS Fusion header.
///
/// This is useful for converting label maps in the original DICOM space
/// and updating them to match a 2D/3D registration performed on the CIOS
/// Fusion DICOM.
void ModifyImageWithCIOSFusionRotFlipFlags(const CIOSFusionDICOMInfo& meta,
                                           itk::Image<unsigned short,2>* img);

/// \brief Updates a 2D image with any rotating/flipping flags present
///        in a CIOS Fusion header.
///
/// This is useful for converting label maps in the original DICOM space
/// and updating them to match a 2D/3D registration performed on the CIOS
/// Fusion DICOM.
void ModifyImageWithCIOSFusionRotFlipFlags(const CIOSFusionDICOMInfo& meta,
                                           itk::Image<float,2>* img);

/// \brief Reads a CIOS Fusion DICOM from disk, returning an ITK image object and
///        another metadata structure.
std::tuple<itk::Image<unsigned short,2>::Pointer,CIOSFusionDICOMInfo>
ReadCIOSFusionDICOMUShort(const std::string& path, const bool no_rot_or_flip_based_on_meta = false);

/// \brief Reads a CIOS Fusion DICOM from disk, returning an ITK image object and
///        another metadata structure.
std::tuple<itk::Image<float,2>::Pointer,CIOSFusionDICOMInfo>
ReadCIOSFusionDICOMFloat(const std::string& path, const bool no_rot_or_flip_based_on_meta = false);

/// \brief Updates a landmark mapping from ITK physical points to continuous indices.
void UpdateLandmarkMapForCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                    LandMap2* pts,
                                    const bool no_rot_or_flip = false);

/// \brief Updates a landmark mapping from ITK physical points to continuous indices.
void UpdateLandmarkMapForCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                    LandMap3* pts,
                                    const bool no_rot_or_flip = false);

void WriteCIOSMetaH5(const CIOSFusionDICOMInfo& meta, H5::CommonFG* h5);

CIOSFusionDICOMInfo ReadCIOSMetaH5(const H5::CommonFG& h5);

}  // xreg

#endif

