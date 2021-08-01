/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#ifndef XREGDICOMUTILS_H_
#define XREGDICOMUTILS_H_

#include <string>
#include <vector>
#include <ostream>
#include <unordered_map>
#include <array>

#include <boost/optional.hpp>

#include "xregCommon.h"
#include "xregFilesystemUtils.h"
#include "xregObjWithOStream.h"
#include "xregPerspectiveXform.h"
#include "xregProjData.h"

// Forward declaration
namespace H5
{
class Group;
}
// End Forward declaration

namespace xreg
{

/// \brief An incomplete set of DICOM fields that are useful for organizing files.
struct DICOMFIleBasicFields
{
  std::string file_path;

  std::string patient_id;
  std::string series_uid;
  std::string study_uid;
  
  std::string patient_name;
  
  double study_time;
  boost::optional<double> series_time;
  boost::optional<double> acquisition_time;
  boost::optional<double> content_time;

  std::string modality;

  Pt3 img_pos_wrt_pat;

  Pt3 row_dir;
  Pt3 col_dir;

  CoordScalar row_spacing;
  CoordScalar col_spacing;

  unsigned long num_rows;
  unsigned long num_cols;

  boost::optional<std::string> pat_pos;

  boost::optional<std::array<std::string,2>> pat_orient;

  boost::optional<std::string> study_desc;

  boost::optional<std::string> series_desc;

  boost::optional<std::vector<std::string>> image_type;

  std::string manufacturer;

  boost::optional<std::string> institution_name;

  boost::optional<std::string> department_name;

  boost::optional<std::string> manufacturers_model_name;

  boost::optional<std::string> sec_cap_dev_manufacturer;

  boost::optional<std::string> sec_cap_dev_software_versions;

  boost::optional<std::vector<std::string>> software_versions;

  boost::optional<std::string> vol_props;

  boost::optional<unsigned long> num_frames;

  boost::optional<std::string> proto_name;

  boost::optional<std::string> conv_kernel;

  // Fields that we would like to use from 2D radiographs/fluoro:

  boost::optional<std::string> body_part_examined;

  boost::optional<std::string> view_position;

  boost::optional<double> dist_src_to_det_mm;
  
  boost::optional<double> dist_src_to_pat_mm;
  
  boost::optional<double> kvp;

  boost::optional<double> tube_current_mA;

  boost::optional<double> exposure_mAs;

  boost::optional<double> exposure_muAs;

  boost::optional<double> exposure_time_ms;

  // units are dGy * cm * cm
  boost::optional<double> dose_area_product_dGy_cm_sq;

  boost::optional<std::string> fov_shape;
  
  boost::optional<std::vector<unsigned long>> fov_dims;

  boost::optional<std::array<unsigned long,2>> fov_origin_off;
  
  enum FOVRot
  {
    kZERO = 0,
    kNINETY = 90,
    kONE_EIGHTY = 180,
    kTWO_SEVENTY = 270
  };
  
  boost::optional<FOVRot> fov_rot;

  boost::optional<bool> fov_horizontal_flip;

  boost::optional<double> intensifier_diameter_mm;

  // This is usally populated for 2D X-ray images, e.g. when the standard
  // pixel spacing fields are not appropriate as they are required to be
  // in "patient space."
  // row spacing , col spacing
  boost::optional<std::array<CoordScalar,2>> imager_pixel_spacing;
  
  boost::optional<double> grid_focal_dist_mm;
  
  boost::optional<double> window_center;
  boost::optional<double> window_width;
};

using DICOMFIleBasicFieldsList = std::vector<DICOMFIleBasicFields>;

/// \brief Populates a set of basic DICOM fields from a DICOM file.
DICOMFIleBasicFields ReadDICOMFileBasicFields(const std::string& dcm_path);

/// \brief Prints a set of basic DICOM fields
void PrintDICOMFileBasicFields(const DICOMFIleBasicFields& dcm_info, std::ostream& out,
                               const std::string& indent = "");

bool IsLocalizer(const DICOMFIleBasicFields& dcm_info);

bool IsMRLocalizer(const DICOMFIleBasicFields& dcm_info);

bool IsVolDICOMFile(const DICOMFIleBasicFields& dcm_info);

bool IsMultiFrameDICOMFile(const DICOMFIleBasicFields& dcm_info);

bool IsSecondaryDICOMFile(const DICOMFIleBasicFields& dcm_info);

bool IsDerivedDICOMFile(const DICOMFIleBasicFields& dcm_info);

/// \brief Stores paths to DICOM files organized by patient ID, study UID, and
///        series UID.
struct OrganizedDICOMFiles
{
  using SeriesUIDToFilePathsMap     = std::unordered_map<std::string,PathStringList>;
  using StudyUIDToSeriesUIDInfosMap = std::unordered_map<std::string,SeriesUIDToFilePathsMap>;
  using PatientIDToStudyUIDInfosMap = std::unordered_map<std::string,StudyUIDToSeriesUIDInfosMap>;

  std::string root_dir;

  PatientIDToStudyUIDInfosMap patient_infos;
};

/// \brief Basic printing of an organized DICOM data set.
void PrintOrganizedDICOMFiles(const OrganizedDICOMFiles& org_dcm, std::ostream& out,
                              const bool print_meta = false,
                              const std::string& indent = " ");

/// \brief Starting in a root directory, find all sub-directories (including the root)
///        which contain at least a single DICOM file.
void GetDICOMDirs(const std::string& root_dir_path, PathStringList* dir_paths);

/// \brief Gets a collection of path strings for each DICOM file in a directory
void GetDICOMFileNamesInDir(const std::string& dir, PathStringList* dcm_file_names);

/// \brief Gets a collection of path objects for each DICOM file in a directory.
void GetDICOMFilePathObjsInDir(const std::string& dir, PathList* dcm_paths);

/// \brief Starting in a root directory, find all DICOM files in the directory
///        hierarchy and store in a hierarchy of data structures.
///
/// Organized as Patient ID -> Studies for each Patient ID -> Series for each study
/// When the modalities argument is non-empty, then only the specified modalities will
/// be included in the output.
void GetOrgainizedDICOMInfos(const std::string& root_dir_path,
                             OrganizedDICOMFiles* org_dcm,
                             const bool inc_localizer = false,
                             const bool inc_multi_frame_files = false,
                             const bool inc_secondary = true,
                             const bool inc_derived = true,
                             const std::vector<std::string>& modalities = std::vector<std::string>());

/// \brief Get basic information structs for every DICOM file in a single directory
///
/// This does not make any attempt to separate different patients, studies, series, etc.
void ReadDICOMInfosFromDir(const std::string& dir_path, DICOMFIleBasicFieldsList* dcm_infos);

/// \brief Reorders a collection of DICOM slices into ascending order in patient space
///        and checks some metadata fields to ensure that it is usable.
struct ReorderAndCheckDICOMInfos : public ObjWithOStream
{
  // These are return value validity flags
  bool const_in_plane_spacings = false;
  bool const_in_plane_dims     = false;

  bool single_out_of_plane_axis   = false;
  bool const_out_of_plane_spacing = false;
  CoordScalar out_of_plane_spacing_const_val = 0;

  size_type out_of_plane_axis_dim = 0;

  // tolerance used to compare spacings between slices and determine
  // if the slice spacing is constant - default 1.0e-6
  CoordScalar out_of_plane_spacing_tol = 1.0e-6;

  // Perform the reordering and verification.
  // The input list of DICOM fields is preserved and the sorted output is populated
  // in a separate list via a pointer supplied by the caller.
  // This routine returns true when the data is determined to be valid and false otherwise.
  // When false is returned, the output sorted list should not be considered to be valid in any way.
  bool operator()(const DICOMFIleBasicFieldsList& src_infos,
                  DICOMFIleBasicFieldsList* dst_infos);
};

void WriteDICOMFieldsH5(const DICOMFIleBasicFields& dcm_info, H5::Group* h5);

DICOMFIleBasicFields ReadDICOMFieldsH5(const H5::Group& h5);

}  // xreg

#endif
