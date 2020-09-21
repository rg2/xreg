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

#include "xregDICOMUtils.h"

#include <algorithm>
#include <set>

#include <gdcmReader.h>
#include <gdcmAttribute.h>
//#include <gdcmVersion.h>

#include <fmt/printf.h>

#include "xregAssert.h"
#include "xregStringUtils.h"

void xreg::ReadDICOMFileBasicFields(const std::string& dcm_path, DICOMFIleBasicFields* dcm_info)
{
  gdcm::Reader dcm_reader;
  dcm_reader.SetFileName(dcm_path.c_str());
  if (dcm_reader.CanRead())
  {
    using PatientIDAttr = gdcm::Attribute<0x0010,0x0020>;
    using StudyUIDAttr  = gdcm::Attribute<0x0020,0x000D>;
    using SeriesUIDAttr = gdcm::Attribute<0x0020,0x000E>;
    using ModalityAttr  = gdcm::Attribute<0x0008,0x0060>;

    using ImgPosPatAttr    = gdcm::Attribute<0x0020,0x0032>;
    using ImgOrientPatAttr = gdcm::Attribute<0x0020,0x0037>;
    using RowsAttr         = gdcm::Attribute<0x0028,0x0010>;
    using ColsAttr         = gdcm::Attribute<0x0028,0x0011>;
    using PixelSpacingAttr = gdcm::Attribute<0x0028,0x0030>;

    //using PixelDataAttr = gdcm::Attribute<0x7FE0,0x0010>;

    using PatPosAttr    = gdcm::Attribute<0x0018,0x5100>;
    using PatOrientAttr = gdcm::Attribute<0x0020,0x0020>;

    using StudyDescAttr  = gdcm::Attribute<0x0008,0x1030>;
    using SeriesDescAttr = gdcm::Attribute<0x0008,0x103E>;
    
    using ImageTypeAttr = gdcm::Attribute<0x0008,0x0008>;

    using SecCapDevManAttr    = gdcm::Attribute<0x0018,0x1016>;
    using SecCapDevSWVersAttr = gdcm::Attribute<0x0018,0x1019>;

    using SWVersionsAttr = gdcm::Attribute<0x0018,0x1020>;

    using VolPropsAttr  = gdcm::Attribute<0x0008,0x9206>;
    using NumFramesAttr = gdcm::Attribute<0x0028,0x0008>;

    using ProtoNameAttr = gdcm::Attribute<0x0018,0x1030>;
    
    using ConvKernelAttr = gdcm::Attribute<0x0018,0x1210>;

    std::set<gdcm::Tag> tags_to_read;
    tags_to_read.insert(PatientIDAttr::GetTag());
    tags_to_read.insert(StudyUIDAttr::GetTag());
    tags_to_read.insert(SeriesUIDAttr::GetTag());
    tags_to_read.insert(ModalityAttr::GetTag());

    tags_to_read.insert(ImgPosPatAttr::GetTag());
    tags_to_read.insert(ImgOrientPatAttr::GetTag());
    tags_to_read.insert(RowsAttr::GetTag());
    tags_to_read.insert(ColsAttr::GetTag());
    tags_to_read.insert(PixelSpacingAttr::GetTag());

    tags_to_read.insert(PatPosAttr::GetTag());
    tags_to_read.insert(PatOrientAttr::GetTag());

    tags_to_read.insert(StudyDescAttr::GetTag());
    tags_to_read.insert(SeriesDescAttr::GetTag());

    tags_to_read.insert(ImageTypeAttr::GetTag());

    tags_to_read.insert(SecCapDevManAttr::GetTag());
    tags_to_read.insert(SecCapDevSWVersAttr::GetTag());

    tags_to_read.insert(SWVersionsAttr::GetTag());

    tags_to_read.insert(VolPropsAttr::GetTag());
    tags_to_read.insert(NumFramesAttr::GetTag());

    tags_to_read.insert(ProtoNameAttr::GetTag());
    
    tags_to_read.insert(ConvKernelAttr::GetTag());

    if (dcm_reader.ReadSelectedTags(tags_to_read))
    {
      gdcm::DataSet& ds = dcm_reader.GetFile().GetDataSet();

      // using excessive scoping here just to avoid using the wrong/previous
      // attribute object

      // using StringStripExtraNulls, since:
      // Sometimes extra null characters are added, which causes problems later.

      {
        PatientIDAttr pat_id_attr;
        pat_id_attr.SetFromDataSet(ds);
        dcm_info->patient_id = StringStripExtraNulls(pat_id_attr.GetValue());
      }

      {
        StudyUIDAttr study_uid_attr;
        study_uid_attr.SetFromDataSet(ds);
        dcm_info->study_uid = StringStripExtraNulls(study_uid_attr.GetValue());
        dcm_info->study_uid = dcm_info->study_uid.substr(0, dcm_info->study_uid.size() - 1);
      }

      {
        SeriesUIDAttr series_uid_attr;
        series_uid_attr.SetFromDataSet(ds);
        dcm_info->series_uid = StringStripExtraNulls(series_uid_attr.GetValue());
      }

      {
        ModalityAttr modality_attr;
        modality_attr.SetFromDataSet(ds);
        dcm_info->modality = StringStripExtraNulls(modality_attr.GetValue());
      }

      // get patient position, directions, and image spacing.
      {
        ImgPosPatAttr img_pos_pat_attr;
        img_pos_pat_attr.SetFromDataSet(ds);
        xregASSERT(img_pos_pat_attr.GetNumberOfValues() == 3);
        dcm_info->img_pos_wrt_pat[0] = img_pos_pat_attr.GetValue(0);
        dcm_info->img_pos_wrt_pat[1] = img_pos_pat_attr.GetValue(1);
        dcm_info->img_pos_wrt_pat[2] = img_pos_pat_attr.GetValue(2);
      }

      {
        ImgOrientPatAttr img_orient_pat_attr;
        img_orient_pat_attr.SetFromDataSet(ds);
        xregASSERT(img_orient_pat_attr.GetNumberOfValues() == 6);
        dcm_info->col_dir[0] = img_orient_pat_attr.GetValue(0);
        dcm_info->col_dir[1] = img_orient_pat_attr.GetValue(1);
        dcm_info->col_dir[2] = img_orient_pat_attr.GetValue(2);
        dcm_info->row_dir[0] = img_orient_pat_attr.GetValue(3);
        dcm_info->row_dir[1] = img_orient_pat_attr.GetValue(4);
        dcm_info->row_dir[2] = img_orient_pat_attr.GetValue(5);
      }

      {
        RowsAttr rows_attr;
        rows_attr.SetFromDataSet(ds);
        xregASSERT(rows_attr.GetNumberOfValues() == 1);
        dcm_info->num_rows = rows_attr.GetValue();
      }

      {
        ColsAttr cols_attr;
        cols_attr.SetFromDataSet(ds);
        xregASSERT(cols_attr.GetNumberOfValues() == 1);
        dcm_info->num_cols = cols_attr.GetValue();
      }

      {
        PixelSpacingAttr ps_attr;
        ps_attr.SetFromDataSet(ds);
        xregASSERT(ps_attr.GetNumberOfValues() == 2);
        dcm_info->col_spacing = ps_attr.GetValue(0);
        dcm_info->row_spacing = ps_attr.GetValue(1);
      }

      {
        dcm_info->pat_pos_valid = true;
        PatPosAttr pat_pos_attr;
        pat_pos_attr.SetFromDataSet(ds);
        xregASSERT(pat_pos_attr.GetNumberOfValues() == 1);
        dcm_info->pat_pos = StringStripExtraNulls(pat_pos_attr.GetValue());
      }

      {
        dcm_info->pat_orient_valid = true;
        PatOrientAttr pat_orient_attr;
        pat_orient_attr.SetFromDataSet(ds);
        xregASSERT(pat_orient_attr.GetNumberOfValues() == 2);
        dcm_info->pat_orient[0] = StringStripExtraNulls(pat_orient_attr.GetValue(0));
        dcm_info->pat_orient[1] = StringStripExtraNulls(pat_orient_attr.GetValue(1));
      }

      {
        StudyDescAttr study_desc_attr;
        study_desc_attr.SetFromDataSet(ds);

        if (study_desc_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(study_desc_attr.GetNumberOfValues() == 1);
          
          dcm_info->study_desc_valid = true;
          dcm_info->study_desc = StringStripExtraNulls(study_desc_attr.GetValue());
        }
        else
        {
          dcm_info->study_desc_valid = false;
        }
      }

      {
        SeriesDescAttr series_desc_attr;
        series_desc_attr.SetFromDataSet(ds);

        if (series_desc_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(series_desc_attr.GetNumberOfValues() == 1);

          dcm_info->series_desc_valid = true;
          dcm_info->series_desc = StringStripExtraNulls(series_desc_attr.GetValue());
        }
        else
        {
          dcm_info->series_desc_valid = false;
        }
      }

      {
        ImageTypeAttr img_type_attr;
        img_type_attr.SetFromDataSet(ds);

        const int len_img_type_attr = img_type_attr.GetNumberOfValues();

        if (len_img_type_attr > 0)
        {
          dcm_info->image_type_valid = true;
          
          dcm_info->image_type.clear();
          dcm_info->image_type.reserve(len_img_type_attr);
         
          for (int img_type_idx = 0; img_type_idx < len_img_type_attr; ++img_type_idx)
          {
            dcm_info->image_type.push_back(StringStrip(StringStripExtraNulls(
                                              img_type_attr.GetValue(img_type_idx))));
          }
        }
        else
        {
          dcm_info->image_type_valid = false;
        }
      }

      {
        SecCapDevManAttr sec_cap_dev_man_attr;
        sec_cap_dev_man_attr.SetFromDataSet(ds);

        if (sec_cap_dev_man_attr.GetNumberOfValues() > 0)
        {
          dcm_info->sec_cap_dev_manufacturer_valid = true;
          xregASSERT(sec_cap_dev_man_attr.GetNumberOfValues() == 1);
          dcm_info->sec_cap_dev_manufacturer = StringStripExtraNulls(sec_cap_dev_man_attr.GetValue());
        }
        else
        {
          dcm_info->sec_cap_dev_manufacturer_valid = false;
        }
      }

      {
        SecCapDevSWVersAttr sec_cap_dev_sw_vers_attr;
        sec_cap_dev_sw_vers_attr.SetFromDataSet(ds);

        if (sec_cap_dev_sw_vers_attr.GetNumberOfValues() > 0)
        {
          dcm_info->sec_cap_dev_software_versions_valid = true;

          xregASSERT(sec_cap_dev_sw_vers_attr.GetNumberOfValues() == 1);
          dcm_info->sec_cap_dev_software_versions = StringStripExtraNulls(sec_cap_dev_sw_vers_attr.GetValue());
        }
        else
        {
          dcm_info->sec_cap_dev_software_versions_valid = false;
        }
      }

      {
        SWVersionsAttr sw_vers_attr;
        sw_vers_attr.SetFromDataSet(ds);

        if (sw_vers_attr.GetNumberOfValues() > 0)
        {
          dcm_info->software_versions_valid = true;

          const int num_sw_ver_toks = sw_vers_attr.GetNumberOfValues();

          dcm_info->software_versions.clear();
          dcm_info->software_versions.reserve(num_sw_ver_toks);

          for (int sw_ver_idx = 0; sw_ver_idx < num_sw_ver_toks; ++sw_ver_idx)
          {
            dcm_info->software_versions.push_back(StringStripExtraNulls(sw_vers_attr.GetValue(sw_ver_idx)));
          }
        }
        else
        {
          dcm_info->software_versions_valid = false;
        }
      }

      // See note below about GDCM always populating fields even when they are not present
      // For this case it is always set to empty, but it is more appropriate to mark as invalid
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x9206)))
      {
        VolPropsAttr vol_props_attr;
        vol_props_attr.SetFromDataSet(ds);

        if (vol_props_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(vol_props_attr.GetNumberOfValues() == 1);
          
          dcm_info->vol_props_valid = true;
          dcm_info->vol_props = StringStrip(StringStripExtraNulls(vol_props_attr.GetValue()));
        }
        else
        {
          dcm_info->vol_props_valid = false;
        }
      }
      else
      {
        dcm_info->vol_props_valid = false;
      }
      
      // Doing things a little differently here as the call to SetFromDataSet() will always 
      // result in a value even when the number of frames attribute is not present in the
      // file. In the case when it is not present, there appears to be some memory corruption
      // as the value is randomly set to 0 or some other large value.
      if (ds.FindDataElement(gdcm::Tag(0x0028,0x0008)))
      {
        NumFramesAttr num_frames_attr;
        num_frames_attr.SetFromDataSet(ds);

        if (num_frames_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(num_frames_attr.GetNumberOfValues() == 1);

          dcm_info->num_frames_valid = true;
          dcm_info->num_frames = num_frames_attr.GetValue();
        }
        else
        {
          dcm_info->num_frames_valid = false;
        }
      }
      else
      {
        dcm_info->num_frames_valid = false;
      }

      {
        ProtoNameAttr proto_name_attr;
        proto_name_attr.SetFromDataSet(ds);

        if (proto_name_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(proto_name_attr.GetNumberOfValues() == 1);
          
          dcm_info->proto_name_valid = true;
          dcm_info->proto_name = StringStripExtraNulls(proto_name_attr.GetValue());
        }
        else
        {
          dcm_info->proto_name_valid = false;
        }
      }

      {
        ConvKernelAttr conv_kernel_attr;
        conv_kernel_attr.SetFromDataSet(ds);

        if (conv_kernel_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(conv_kernel_attr.GetNumberOfValues() == 1);

          dcm_info->conv_kernel_valid = true;
          dcm_info->conv_kernel = StringStrip(StringStripExtraNulls(conv_kernel_attr.GetValue()));
        }
        else
        {
          dcm_info->conv_kernel_valid = false;
        }
      }

      dcm_info->file_path = dcm_path;
    }
    else
    {
      xregThrow("Unable to read DICOM File!");
    }
  }
  else
  {
    xregThrow("Invalid DICOM File!");
  }
}

void xreg::PrintDICOMFileBasicFields(const DICOMFIleBasicFields& dcm_info, std::ostream& out,
                                     const std::string& indent)
{
  const std::string kNOT_PROVIDED_STR("<Not Provided>");

  out << indent << "               File Path: " << dcm_info.file_path << '\n'
      << indent << "              Patient ID: " << dcm_info.patient_id << '\n'
      << indent << "               Study UID: " << dcm_info.study_uid << '\n'
      << indent << "              Series UID: " << dcm_info.series_uid << '\n'
      << indent << "                Modality: " << dcm_info.modality << '\n'
      << indent << "          Image Position: " << fmt::sprintf("[%+10.4f,%+10.4f,%+10.4f]", dcm_info.img_pos_wrt_pat[0], dcm_info.img_pos_wrt_pat[1], dcm_info.img_pos_wrt_pat[2]) << '\n'
      << indent << "          Image Col Dir.: " << fmt::sprintf("[%+0.4f,%+0.4f,%+0.4f]", dcm_info.col_dir[0], dcm_info.col_dir[1], dcm_info.col_dir[2]) << '\n'
      << indent << "          Image Row Dir.: " << fmt::sprintf("[%+0.4f,%+0.4f,%+0.4f]", dcm_info.row_dir[0], dcm_info.row_dir[1], dcm_info.row_dir[2]) << '\n'
      << indent << "       Image Col Spacing: " << fmt::sprintf("%0.4f", dcm_info.col_spacing) << '\n'
      << indent << "       Image Row Spacing: " << fmt::sprintf("%0.4f", dcm_info.row_spacing) << '\n'
      << indent << "          Image Num Rows: " << dcm_info.num_rows << '\n'
      << indent << "          Image Num Cols: " << dcm_info.num_cols << '\n'
      << indent << "        Patient Position: " << (dcm_info.pat_pos_valid ? dcm_info.pat_pos : kNOT_PROVIDED_STR) << '\n'
      << indent << "         Patient Orient.: " << (dcm_info.pat_orient_valid ? fmt::sprintf("%s , %s", dcm_info.pat_orient[0], dcm_info.pat_orient[1]) : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Study Desc.: " << (dcm_info.study_desc_valid ? dcm_info.study_desc : kNOT_PROVIDED_STR) << '\n'
      << indent << "            Series Desc.: " << (dcm_info.series_desc_valid ? dcm_info.series_desc : kNOT_PROVIDED_STR) << '\n'
      << indent << "              Image Type: " << (dcm_info.image_type_valid ? JoinTokens(dcm_info.image_type, " , ") : kNOT_PROVIDED_STR) << '\n'
      << indent << "     Sec. Cap. Dev. Man.: " << (dcm_info.sec_cap_dev_manufacturer_valid ? dcm_info.sec_cap_dev_manufacturer : kNOT_PROVIDED_STR) << '\n'
      << indent << "  Sec. Cap. Dev. SW Ver.: " << (dcm_info.sec_cap_dev_software_versions_valid ? dcm_info.sec_cap_dev_software_versions : kNOT_PROVIDED_STR) << '\n'
      << indent << "       Software Versions: " << (dcm_info.software_versions_valid ? JoinTokens(dcm_info.software_versions, " , ") : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Vol. Props.: " << (dcm_info.vol_props_valid ? dcm_info.vol_props : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Num. Frames: " << (dcm_info.num_frames_valid ? fmt::format("{}", dcm_info.num_frames) : kNOT_PROVIDED_STR) << '\n'
      << indent << "           Protocol Name: " << (dcm_info.proto_name_valid ? dcm_info.proto_name : kNOT_PROVIDED_STR) << '\n'
      << indent << "            Conv. Kernel: " << (dcm_info.conv_kernel_valid ? dcm_info.conv_kernel : kNOT_PROVIDED_STR) << '\n';

  out.flush();
}

bool xreg::IsLocalizer(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.image_type_valid &&
          (std::find(dcm_info.image_type.begin(), dcm_info.image_type.end(), "LOCALIZER")
                  != dcm_info.image_type.end());
}

bool xreg::IsMRLocalizer(const DICOMFIleBasicFields& dcm_info)
{
  return (dcm_info.modality == "MR") &&
         (dcm_info.image_type_valid &&
          (std::find(dcm_info.image_type.begin(), dcm_info.image_type.end(), "LOCALIZER")
                  != dcm_info.image_type.end()));
}

bool xreg::IsVolDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.vol_props_valid && (dcm_info.vol_props == "VOLUME");
}

bool xreg::IsMultiFrameDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.num_frames_valid && (dcm_info.num_frames > 1);
}

void xreg::GetDICOMDirs(const std::string& root_dir_path, PathStringList* dir_paths)
{
  PathList paths_to_check;

  Path root_path(root_dir_path);
  if (root_path.is_dir())
  {
    paths_to_check.push_back(root_path);
  }

  dir_paths->clear();

  PathList cur_dir_elems;

  while (!paths_to_check.empty())
  {
    Path cur_path = paths_to_check.back();
    paths_to_check.pop_back();

    cur_dir_elems.clear();
    cur_path.get_dir_contents(&cur_dir_elems);

    bool cur_dir_has_dcm = false;

    for (const auto& path : cur_dir_elems)
    {
      if (path.is_dir())
      {
        if ((path.filename().string() != ".") && (path.filename().string() != ".."))
        {
          // sub directory, we should eventually check it for DICOM files
          paths_to_check.push_back(path);
        }
      }
      else if (/*path.is_reg_file() &&*/ !cur_dir_has_dcm)  // this allows sym links, etc.
      {
        // we have not found a DICOM file in this directory yet, and the
        // current element is a file, check to see if it's a DICOM

        gdcm::Reader dcm_reader;
        dcm_reader.SetFileName(path.string().c_str());
        cur_dir_has_dcm = dcm_reader.CanRead();
      }
    }

    if (cur_dir_has_dcm)
    {
      dir_paths->push_back(cur_path.string());
    }
  }
}

void xreg::GetDICOMFileNamesInDir(const std::string& dir, PathStringList* dcm_file_names)
{
  using size_type = PathList::size_type;

  PathList dcm_paths;
  GetDICOMFilePathObjsInDir(dir, &dcm_paths);

  const size_type num_dcms = dcm_paths.size();

  dcm_file_names->resize(num_dcms);

  for (size_type i = 0; i < num_dcms; ++i)
  {
    dcm_file_names->operator[](i) = dcm_paths[i].filename().string();
  }
}

void xreg::GetDICOMFilePathObjsInDir(const std::string& dir, PathList* dcm_paths)
{
  dcm_paths->clear();

  Path dir_path(dir);

  if (dir_path.is_dir())
  {
    PathList dir_elems;
    dir_path.get_dir_contents(&dir_elems);

    for (const auto& path : dir_elems)
    {
      if (!path.is_dir())  // only want to check files
      {
        gdcm::Reader dcm_reader;
        dcm_reader.SetFileName(path.string().c_str());
        if (dcm_reader.CanRead())
        {
          dcm_paths->push_back(path);
        }
      }
    }
  }
}

void xreg::GetOrgainizedDICOMInfos(const std::string& root_dir_path,
                                   OrganizedDICOMFiles* org_dcm,
                                   const bool inc_localizer,
                                   const bool inc_multi_frame_files)
{
  org_dcm->patient_infos.clear();
  org_dcm->root_dir = root_dir_path;

  PathStringList dcm_dirs;
  GetDICOMDirs(root_dir_path, &dcm_dirs);

  // temporary variables
  PathList cur_dir_dcm_file_paths;
  DICOMFIleBasicFields tmp_basic_fields;

  for (const auto& dcm_dir : dcm_dirs)
  {
    cur_dir_dcm_file_paths.clear();
    GetDICOMFilePathObjsInDir(dcm_dir, &cur_dir_dcm_file_paths);

    for (const auto& dcm_file_path : cur_dir_dcm_file_paths)
    {
      const std::string cur_file_path = dcm_file_path.string();

      ReadDICOMFileBasicFields(cur_file_path, &tmp_basic_fields);

      if ((inc_localizer  || !IsLocalizer(tmp_basic_fields)) &&
          (inc_multi_frame_files || !IsMultiFrameDICOMFile(tmp_basic_fields)))
      {
        org_dcm->patient_infos[tmp_basic_fields.patient_id]
                                [tmp_basic_fields.study_uid]
                                  [tmp_basic_fields.series_uid].push_back(cur_file_path);
      }
    }
  }
}

void xreg::PrintOrganizedDICOMFiles(const OrganizedDICOMFiles& org_dcm, std::ostream& out,
                                    const bool print_meta, const std::string& indent)
{
  out << indent << org_dcm.root_dir << '\n';

  const std::string indent2 = indent + "  ";
  const std::string indent3 = indent2 + "  ";
  const std::string indent4 = indent3 + "  ";
  const std::string indent5 = indent4 + "  ";

  out << indent << "Number of Patients: " << org_dcm.patient_infos.size() << '\n';

  for (const auto& patid2study : org_dcm.patient_infos)
  {
    const OrganizedDICOMFiles::StudyUIDToSeriesUIDInfosMap& study2series_map = patid2study.second;

    out << indent2 << "Patient ID: " << patid2study.first << '\n'
        << indent2 << "Number of Studies: " << study2series_map.size() << '\n';

    for (const auto& study2series : study2series_map)
    {
      const OrganizedDICOMFiles::SeriesUIDToFilePathsMap& series2paths_map = study2series.second;

      out << indent3 << "Study UID: " << study2series.first << '\n'
          << indent3 << "Number of Series: " << series2paths_map.size() << '\n';

      bool first_series = true;

      for (const auto& series2paths : series2paths_map)
      {
        const PathStringList& paths = series2paths.second;

        const bool print_dcm_info = print_meta && !paths.empty();

        DICOMFIleBasicFields dcm_info;

        if (print_dcm_info)
        {
          ReadDICOMFileBasicFields(paths[0], &dcm_info);

          if (first_series)
          {
            out << indent3 << "Study Description: "
                << (dcm_info.study_desc_valid ? dcm_info.study_desc.c_str()
                                              : "(Not Provided)")
                << std::endl;
          }
        }

        out << indent4 << "Series UID: " << series2paths.first << '\n'
            << indent4 << "Number of Files: " << paths.size() << '\n';
        
        if (print_dcm_info)
        {
          out << indent4 << "Series Description: "
              << (dcm_info.series_desc_valid ? dcm_info.series_desc.c_str()
                                            : "(Not Provided)")
              << std::endl;
        }

        for (const auto& path : paths)
        {
          out << indent5 << path << '\n';
        }

        first_series = false;
      }
    }
  }

  out.flush();
}

void xreg::ReadDICOMInfosFromDir(const std::string& dir_path, DICOMFIleBasicFieldsList* dcm_infos)
{
  using size_type = PathStringList::size_type;

  PathStringList dcm_paths;
  GetDICOMFileNamesInDir(dir_path, &dcm_paths);

  const size_type num_dcm = dcm_paths.size();

  DICOMFIleBasicFieldsList& dcms = *dcm_infos;

  dcms.resize(num_dcm);

  for (size_type dcm_idx = 0; dcm_idx < num_dcm; ++dcm_idx)
  {
    ReadDICOMFileBasicFields(dcm_paths[dcm_idx], &dcms[dcm_idx]);
  }
}

bool xreg::ReorderAndCheckDICOMInfos::operator()(const DICOMFIleBasicFieldsList& src_infos,
                                               DICOMFIleBasicFieldsList* dst_infos)
{
  const CoordScalar kTOL = 1.0e-6;

  const_in_plane_spacings  = true;
  const_in_plane_dims      = true;
  single_out_of_plane_axis = true;

  const_out_of_plane_spacing = true;
  out_of_plane_spacing_const_val = 0;

  const size_type num_dcm = src_infos.size();

  // 3 -> not yet set, should be 0, 1, or 2.
  out_of_plane_axis_dim = 3;

  if (num_dcm > 1)
  {
    Pt3 tmp_pat_pos_diff;

    // I could break out of this loop early when any of the validity checks fail,
    // but it will probably not incurr too much of a performance penalty and
    // may find any other inconsistencies in the entire collection
    for (size_type dcm_idx = 1; dcm_idx < num_dcm; ++dcm_idx)
    {
      tmp_pat_pos_diff = src_infos[dcm_idx].img_pos_wrt_pat - src_infos[dcm_idx - 1].img_pos_wrt_pat;

      // get the axis in which patient position changes the most and
      // treat this as the out of plane direction
      
      size_type axis_of_max_diff = 0;
      double    cur_max_diff     = std::abs(tmp_pat_pos_diff[0]);

      for (size_type axis_idx = 1; axis_idx < 3; ++axis_idx)
      {
        const double cur_diff = std::abs(tmp_pat_pos_diff[axis_idx]);

        if (cur_diff > cur_max_diff)
        {
          axis_of_max_diff = axis_idx;
          cur_max_diff     = cur_diff;
        }
      }
      
      if (out_of_plane_axis_dim > 2)
      {
        // out of plane axis has not been set yet, set it
        out_of_plane_axis_dim = axis_of_max_diff;
      }
      else if (out_of_plane_axis_dim != axis_of_max_diff)
      {
        this->dout() << "Out of plane axis has changed from " << out_of_plane_axis_dim
                     << ", to " << axis_of_max_diff << " between DICOMs " << dcm_idx -1
                     << ", and " << dcm_idx << std::endl;
        single_out_of_plane_axis = false;
      }

      for (size_type axis_idx = 0; axis_idx < 3; ++axis_idx)
      {

        // in-plane spacing
        if (std::abs(src_infos[dcm_idx - 1].col_spacing - src_infos[dcm_idx].col_spacing) > kTOL)
        {
          this->dout() << "In-plane column spacings changed from " << src_infos[dcm_idx - 1].col_spacing
                       << ", to " << src_infos[dcm_idx].col_spacing << ", between DICOMs "
                       << dcm_idx - 1 << " and " << dcm_idx << std::endl;
          const_in_plane_spacings = false;
        }

        if (std::abs(src_infos[dcm_idx - 1].row_spacing - src_infos[dcm_idx].row_spacing) > kTOL)
        {
          this->dout() << "In-plane row spacings changed from " << src_infos[dcm_idx - 1].row_spacing
                       << ", to " << src_infos[dcm_idx].row_spacing << ", between DICOMs "
                       << dcm_idx - 1 << " and " << dcm_idx << std::endl;
          const_in_plane_spacings = false;
        }

        // in-plane dims
        if (src_infos[dcm_idx - 1].num_rows != src_infos[dcm_idx].num_rows)
        {
          this->dout() << "In-plane number of rows changed from " << src_infos[dcm_idx - 1].num_rows
                       << ", to " << src_infos[dcm_idx].num_rows << ", between DICOMs "
                       << dcm_idx - 1 << " and " << dcm_idx << std::endl;
          const_in_plane_dims = false;
        }

        if (src_infos[dcm_idx - 1].num_cols != src_infos[dcm_idx].num_cols)
        {
          this->dout() << "In-plane number of columns changed from " << src_infos[dcm_idx - 1].num_cols
                       << ", to " << src_infos[dcm_idx].num_cols << ", between DICOMs "
                       << dcm_idx - 1 << " and " << dcm_idx << std::endl;
          const_in_plane_dims = false;
        }
      }
    }

    if (single_out_of_plane_axis && (out_of_plane_axis_dim >= 3))
    {
      this->dout() << "Out of plane axis not found!" << std::endl;
      single_out_of_plane_axis = false;  // no out of plane axis
    }
    else
    {
      this->dout() << "Out of plane axis dim = " << out_of_plane_axis_dim << std::endl;
    }
  }

  if (single_out_of_plane_axis && const_in_plane_spacings && const_in_plane_dims)
  {
    this->dout() << "copying input DICOM infos..." << std::endl;

    *dst_infos = src_infos;

    if (num_dcm > 1)
    {
      {
        this->dout() << "sorting output DICOM infos..." << std::endl;
        
        const size_type sort_dim = out_of_plane_axis_dim;

        std::sort(dst_infos->begin(), dst_infos->end(),
                  [sort_dim] (const DICOMFIleBasicFields& x, const DICOMFIleBasicFields& y)
                  {
                    return x.img_pos_wrt_pat[sort_dim] < y.img_pos_wrt_pat[sort_dim];
                  });
      }

      DICOMFIleBasicFieldsList& dst_infos_ref = *dst_infos;

      // Look for duplicate slice locations
      std::vector<bool> dcm_to_rm_mask(num_dcm, false);
      size_type num_to_rm = 0;

      for (size_type dcm_idx = 1; dcm_idx < num_dcm; ++dcm_idx)
      {
        if ((dst_infos_ref[dcm_idx].img_pos_wrt_pat[out_of_plane_axis_dim] - dst_infos_ref[dcm_idx - 1].img_pos_wrt_pat[out_of_plane_axis_dim]) < kTOL)
        {
          this->dout() << "No out of plane position change from " << dcm_idx - 1 << " to " << dcm_idx << std::endl;
          dcm_to_rm_mask[dcm_idx] = true;
          ++num_to_rm;
        }
      }

      if (num_to_rm)
      {
        this->dout() << "removing out of plane slices " << num_to_rm << "..." << std::endl;

        DICOMFIleBasicFieldsList tmp_dcm;
        tmp_dcm.reserve(num_dcm - num_to_rm);

        for (size_type dcm_idx = 0; dcm_idx < num_dcm; ++dcm_idx)
        {
          if (!dcm_to_rm_mask[dcm_idx])
          {
            tmp_dcm.push_back(dst_infos_ref[dcm_idx]);
          }
        }

        tmp_dcm.swap(dst_infos_ref);
      }

      // check for a constant out of plane spacing
      out_of_plane_spacing_const_val = dst_infos_ref[1].img_pos_wrt_pat[out_of_plane_axis_dim] - dst_infos_ref[0].img_pos_wrt_pat[out_of_plane_axis_dim];

      for (size_type dcm_idx = 2; dcm_idx < num_dcm; ++dcm_idx)
      {
        const CoordScalar next_spacing = dst_infos_ref[dcm_idx].img_pos_wrt_pat[out_of_plane_axis_dim] - dst_infos_ref[dcm_idx - 1].img_pos_wrt_pat[out_of_plane_axis_dim];
        const CoordScalar spacing_diff = std::abs(next_spacing - out_of_plane_spacing_const_val);

        if (spacing_diff > out_of_plane_spacing_tol)
        {
          this->dout() << "non-constant out of plane spacing: " << out_of_plane_spacing_const_val
                       << " -> " << next_spacing << ", diff: " << spacing_diff << std::endl;
          const_out_of_plane_spacing = false;
          out_of_plane_spacing_const_val = 0;
          break;
        }
      }

      if (const_out_of_plane_spacing)
      {
        this->dout() << "constant out of plane spacing: " << out_of_plane_spacing_const_val << std::endl;
      }
    }
  }
  else
  {
    this->dout() << "Invalid/Inconsistent parameters, not loading output DICOM infos." << std::endl;
    dst_infos->clear();
  }

  return single_out_of_plane_axis && const_in_plane_spacings && const_in_plane_dims;
}
