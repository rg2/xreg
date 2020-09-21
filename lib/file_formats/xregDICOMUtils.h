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

#ifndef XREGDICOMUTILS_H_
#define XREGDICOMUTILS_H_

#include <string>
#include <vector>
#include <ostream>
#include <unordered_map>
#include <array>

#include "xregCommon.h"
#include "xregFilesystemUtils.h"
#include "xregObjWithOStream.h"

namespace xreg
{

/// \brief An incomplete set of DICOM fields that are useful for organizing files.
struct DICOMFIleBasicFields
{
  std::string file_path;

  std::string patient_id;
  std::string series_uid;
  std::string study_uid;

  std::string modality;

  Pt3 img_pos_wrt_pat;

  Pt3 row_dir;
  Pt3 col_dir;

  CoordScalar row_spacing;
  CoordScalar col_spacing;

  unsigned long num_rows;
  unsigned long num_cols;

  bool pat_pos_valid;
  std::string pat_pos;

  bool pat_orient_valid;
  std::array<std::string,2> pat_orient;

  bool study_desc_valid;
  std::string study_desc;

  bool series_desc_valid;
  std::string series_desc;

  bool image_type_valid;
  std::vector<std::string> image_type;

  bool sec_cap_dev_manufacturer_valid;
  std::string sec_cap_dev_manufacturer;

  bool sec_cap_dev_software_versions_valid;
  std::string sec_cap_dev_software_versions;

  bool software_versions_valid;
  std::vector<std::string> software_versions;

  bool vol_props_valid;
  std::string vol_props;

  bool num_frames_valid;
  unsigned long num_frames;

  bool proto_name_valid;
  std::string proto_name;

  bool conv_kernel_valid;
  std::string conv_kernel;
};

using DICOMFIleBasicFieldsList = std::vector<DICOMFIleBasicFields>;

/// \brief Populates a set of basic DICOM fields from a DICOM file.
void ReadDICOMFileBasicFields(const std::string& dcm_path, DICOMFIleBasicFields* dcm_info);

/// \brief Prints a set of basic DICOM fields
void PrintDICOMFileBasicFields(const DICOMFIleBasicFields& dcm_info, std::ostream& out,
                               const std::string& indent = "");

bool IsLocalizer(const DICOMFIleBasicFields& dcm_info);

bool IsMRLocalizer(const DICOMFIleBasicFields& dcm_info);

bool IsVolDICOMFile(const DICOMFIleBasicFields& dcm_info);

bool IsMultiFrameDICOMFile(const DICOMFIleBasicFields& dcm_info);

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
void GetOrgainizedDICOMInfos(const std::string& root_dir_path,
                             OrganizedDICOMFiles* org_dcm,
                             const bool inc_localizer = false,
                             const bool inc_multi_frame_files = false);

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

  bool operator()(const DICOMFIleBasicFieldsList& src_infos,
                  DICOMFIleBasicFieldsList* dst_infos);
};

}  // xreg

#endif
