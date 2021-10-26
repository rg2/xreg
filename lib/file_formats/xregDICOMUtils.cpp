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

#include "xregDICOMUtils.h"

#include <algorithm>
#include <set>

#include <gdcmReader.h>
#include <gdcmAttribute.h>
//#include <gdcmVersion.h>

#include <fmt/printf.h>

#include "xregAssert.h"
#include "xregHDF5.h"
#include "xregHDF5Internal.h"
#include "xregStringUtils.h"

xreg::DICOMFIleBasicFields xreg::ReadDICOMFileBasicFields(const std::string& dcm_path)
{
  gdcm::Reader dcm_reader;
  dcm_reader.SetFileName(dcm_path.c_str());
  if (dcm_reader.CanRead())
  {
    using PatientIDAttr = gdcm::Attribute<0x0010,0x0020>;
    using StudyUIDAttr  = gdcm::Attribute<0x0020,0x000D>;
    using SeriesUIDAttr = gdcm::Attribute<0x0020,0x000E>;
    
    using PatientNameAttr = gdcm::Attribute<0x0010,0x0010>;
    
    using StudyTimeAttr       = gdcm::Attribute<0x0008,0x0030>;
    using SeriesTimeAttr      = gdcm::Attribute<0x0008,0x0031>;
    using AcquisitionTimeAttr = gdcm::Attribute<0x0008,0x0032>;
    using ContentTimeAttr     = gdcm::Attribute<0x0008,0x0033>;

    using ModalityAttr = gdcm::Attribute<0x0008,0x0060>;

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
    
    using ManufacturerAttr          = gdcm::Attribute<0x0008,0x0070>;
    using InstitutionNameAttr       = gdcm::Attribute<0x0008,0x0080>;
    using InstitutionDeptAttr       = gdcm::Attribute<0x0008,0x1040>;
    using ManufacturerModelNameAttr = gdcm::Attribute<0x0008,0x1090>;

    using SecCapDevManAttr    = gdcm::Attribute<0x0018,0x1016>;
    using SecCapDevSWVersAttr = gdcm::Attribute<0x0018,0x1019>;

    using SWVersionsAttr = gdcm::Attribute<0x0018,0x1020>;

    using VolPropsAttr  = gdcm::Attribute<0x0008,0x9206>;
    using NumFramesAttr = gdcm::Attribute<0x0028,0x0008>;

    using ProtoNameAttr = gdcm::Attribute<0x0018,0x1030>;
    
    using ConvKernelAttr = gdcm::Attribute<0x0018,0x1210>;

    using BodyPartAttr = gdcm::Attribute<0x0018,0x0015>;
    using ViewPosAttr  = gdcm::Attribute<0x0018,0x5101>;
    
    using DistSrcToDetAttr = gdcm::Attribute<0x0018,0x1110>;
    using DistSrcToPatAttr = gdcm::Attribute<0x0018,0x1111>;
    
    using KVPAttr          = gdcm::Attribute<0x0018,0x0060>;
    using TubeCurrentAttr  = gdcm::Attribute<0x0018,0x1151>;
    using ExposuremAsAttr  = gdcm::Attribute<0x0018,0x1152>;
    using ExposuremuAsAttr = gdcm::Attribute<0x0018,0x1153>;
    using ExposureTimeAttr = gdcm::Attribute<0x0018,0x1150>;
    using DoseAreaProdAttr = gdcm::Attribute<0x0018,0x115E>;
    
    using FOVShapeAttr     = gdcm::Attribute<0x0018,0x1147>;
    // We never use the FOV dims attribute, because it does not actually compile!
    //using FOVDimsAttr      = gdcm::Attribute<0x0018,0x1149>;
    using FOVOriginOffAttr = gdcm::Attribute<0x0018,0x7030>;
    using FOVRotAttr       = gdcm::Attribute<0x0018,0x7032>;
    using FOVHorizFlipAttr = gdcm::Attribute<0x0018,0x7034>;

    using IntensifierDiameterAttr = gdcm::Attribute<0x0018,0x1162>;
    using ImagerPixelSpacingAttr  = gdcm::Attribute<0x0018,0x1164>;
    
    using GridFocalDistAttr = gdcm::Attribute<0x0018,0x704c>;
    
    using WinCenterAttr = gdcm::Attribute<0x0028,0x1050>;
    using WinWidthAttr  = gdcm::Attribute<0x0028,0x1051>;

    std::set<gdcm::Tag> tags_to_read;
    tags_to_read.insert(PatientIDAttr::GetTag());
    tags_to_read.insert(StudyUIDAttr::GetTag());
    tags_to_read.insert(SeriesUIDAttr::GetTag());
    tags_to_read.insert(PatientNameAttr::GetTag());
   
    tags_to_read.insert(StudyTimeAttr::GetTag());
    tags_to_read.insert(SeriesTimeAttr::GetTag());
    tags_to_read.insert(AcquisitionTimeAttr::GetTag());
    tags_to_read.insert(ContentTimeAttr::GetTag());

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

    tags_to_read.insert(ManufacturerAttr::GetTag());
    tags_to_read.insert(InstitutionNameAttr::GetTag());
    tags_to_read.insert(InstitutionDeptAttr::GetTag());
    tags_to_read.insert(ManufacturerModelNameAttr::GetTag());

    tags_to_read.insert(SecCapDevManAttr::GetTag());
    tags_to_read.insert(SecCapDevSWVersAttr::GetTag());

    tags_to_read.insert(SWVersionsAttr::GetTag());

    tags_to_read.insert(VolPropsAttr::GetTag());
    tags_to_read.insert(NumFramesAttr::GetTag());

    tags_to_read.insert(ProtoNameAttr::GetTag());
    
    tags_to_read.insert(ConvKernelAttr::GetTag());
    
    tags_to_read.insert(BodyPartAttr::GetTag());
    tags_to_read.insert(ViewPosAttr::GetTag());
    
    tags_to_read.insert(DistSrcToDetAttr::GetTag());
    tags_to_read.insert(DistSrcToPatAttr::GetTag());

    tags_to_read.insert(KVPAttr::GetTag());
    tags_to_read.insert(TubeCurrentAttr::GetTag());
    tags_to_read.insert(ExposuremAsAttr::GetTag());
    tags_to_read.insert(ExposuremuAsAttr::GetTag());
    tags_to_read.insert(ExposureTimeAttr::GetTag());
    tags_to_read.insert(DoseAreaProdAttr::GetTag());
    
    tags_to_read.insert(FOVShapeAttr::GetTag());
    tags_to_read.insert(gdcm::Tag(0x0018,0x1149));  // FOV Dims
    tags_to_read.insert(FOVOriginOffAttr::GetTag());
    tags_to_read.insert(FOVRotAttr::GetTag());
    tags_to_read.insert(FOVHorizFlipAttr::GetTag());

    tags_to_read.insert(IntensifierDiameterAttr::GetTag());
    tags_to_read.insert(ImagerPixelSpacingAttr::GetTag());
    
    tags_to_read.insert(GridFocalDistAttr::GetTag());
    
    tags_to_read.insert(WinCenterAttr::GetTag());
    tags_to_read.insert(WinWidthAttr::GetTag());

    if (dcm_reader.ReadSelectedTags(tags_to_read))
    {
      DICOMFIleBasicFields dcm_info;

      gdcm::DataSet& ds = dcm_reader.GetFile().GetDataSet();

      // using excessive scoping here just to avoid using the wrong/previous
      // attribute object

      // using StringStripExtraNulls, since:
      // Sometimes extra null characters are added, which causes problems later.

      {
        PatientIDAttr pat_id_attr;
        pat_id_attr.SetFromDataSet(ds);
        dcm_info.patient_id = StringStripExtraNulls(pat_id_attr.GetValue());
      }

      {
        StudyUIDAttr study_uid_attr;
        study_uid_attr.SetFromDataSet(ds);
        dcm_info.study_uid = StringStripExtraNulls(study_uid_attr.GetValue());
        dcm_info.study_uid = dcm_info.study_uid.substr(0, dcm_info.study_uid.size() - 1);
      }

      {
        SeriesUIDAttr series_uid_attr;
        series_uid_attr.SetFromDataSet(ds);
        dcm_info.series_uid = StringStripExtraNulls(series_uid_attr.GetValue());
      }

      {
        PatientNameAttr pat_name_attr;
        pat_name_attr.SetFromDataSet(ds);
        dcm_info.patient_name = StringStripExtraNulls(pat_name_attr.GetValue());
      }

      {
        StudyTimeAttr study_time_attr;
        study_time_attr.SetFromDataSet(ds);
        dcm_info.study_time = StringCast<double>(StringStripExtraNulls(study_time_attr.GetValue()));
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x0031)))
      {
        SeriesTimeAttr series_time_attr;
        series_time_attr.SetFromDataSet(ds);

        if (series_time_attr.GetNumberOfValues() > 0)
        {
          dcm_info.series_time = StringCast<double>(StringStripExtraNulls(series_time_attr.GetValue()));
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x0032)))
      {
        AcquisitionTimeAttr aquis_time_attr;
        aquis_time_attr.SetFromDataSet(ds);

        if (aquis_time_attr.GetNumberOfValues() > 0)
        {
          dcm_info.acquisition_time = StringCast<double>(StringStripExtraNulls(aquis_time_attr.GetValue()));
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x0033)))
      {
        ContentTimeAttr content_time_attr;
        content_time_attr.SetFromDataSet(ds);

        if (content_time_attr.GetNumberOfValues() > 0)
        {
          dcm_info.content_time = StringCast<double>(StringStripExtraNulls(content_time_attr.GetValue()));
        }
      }

      {
        ModalityAttr modality_attr;
        modality_attr.SetFromDataSet(ds);
        dcm_info.modality = StringStripExtraNulls(modality_attr.GetValue());
      }

      // get patient position, directions, and image spacing.
      {
        ImgPosPatAttr img_pos_pat_attr;
        img_pos_pat_attr.SetFromDataSet(ds);
        xregASSERT(img_pos_pat_attr.GetNumberOfValues() == 3);
        dcm_info.img_pos_wrt_pat[0] = img_pos_pat_attr.GetValue(0);
        dcm_info.img_pos_wrt_pat[1] = img_pos_pat_attr.GetValue(1);
        dcm_info.img_pos_wrt_pat[2] = img_pos_pat_attr.GetValue(2);
      }

      {
        ImgOrientPatAttr img_orient_pat_attr;
        img_orient_pat_attr.SetFromDataSet(ds);
        xregASSERT(img_orient_pat_attr.GetNumberOfValues() == 6);
        dcm_info.col_dir[0] = img_orient_pat_attr.GetValue(0);
        dcm_info.col_dir[1] = img_orient_pat_attr.GetValue(1);
        dcm_info.col_dir[2] = img_orient_pat_attr.GetValue(2);
        dcm_info.row_dir[0] = img_orient_pat_attr.GetValue(3);
        dcm_info.row_dir[1] = img_orient_pat_attr.GetValue(4);
        dcm_info.row_dir[2] = img_orient_pat_attr.GetValue(5);
      }

      {
        RowsAttr rows_attr;
        rows_attr.SetFromDataSet(ds);
        xregASSERT(rows_attr.GetNumberOfValues() == 1);
        dcm_info.num_rows = rows_attr.GetValue();
      }

      {
        ColsAttr cols_attr;
        cols_attr.SetFromDataSet(ds);
        xregASSERT(cols_attr.GetNumberOfValues() == 1);
        dcm_info.num_cols = cols_attr.GetValue();
      }

      {
        PixelSpacingAttr ps_attr;
        ps_attr.SetFromDataSet(ds);
        xregASSERT(ps_attr.GetNumberOfValues() == 2);
        dcm_info.col_spacing = ps_attr.GetValue(0);
        dcm_info.row_spacing = ps_attr.GetValue(1);
      }

      {
        PatPosAttr pat_pos_attr;
        pat_pos_attr.SetFromDataSet(ds);
        
        if (pat_pos_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(pat_pos_attr.GetNumberOfValues() == 1);
          dcm_info.pat_pos = StringStripExtraNulls(pat_pos_attr.GetValue());
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0020,0x0020)))
      {
        PatOrientAttr pat_orient_attr;
        pat_orient_attr.SetFromDataSet(ds);
        
        if (pat_orient_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(pat_orient_attr.GetNumberOfValues() == 2);
          
          dcm_info.pat_orient = std::array<std::string,2> {
                                  StringStripExtraNulls(pat_orient_attr.GetValue(0)),
                                  StringStripExtraNulls(pat_orient_attr.GetValue(1)) };
        }
      }

      {
        StudyDescAttr study_desc_attr;
        study_desc_attr.SetFromDataSet(ds);

        if (study_desc_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(study_desc_attr.GetNumberOfValues() == 1);
          
          dcm_info.study_desc = StringStripExtraNulls(study_desc_attr.GetValue());
        }
      }

      {
        SeriesDescAttr series_desc_attr;
        series_desc_attr.SetFromDataSet(ds);

        if (series_desc_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(series_desc_attr.GetNumberOfValues() == 1);

          dcm_info.series_desc = StringStripExtraNulls(series_desc_attr.GetValue());
        }
      }

      {
        ImageTypeAttr img_type_attr;
        img_type_attr.SetFromDataSet(ds);

        const int len_img_type_attr = img_type_attr.GetNumberOfValues();

        if (len_img_type_attr > 0)
        {
          std::vector<std::string> image_type;
          
          image_type.reserve(len_img_type_attr);
         
          for (int img_type_idx = 0; img_type_idx < len_img_type_attr; ++img_type_idx)
          {
            image_type.push_back(StringStrip(StringStripExtraNulls(
                                              img_type_attr.GetValue(img_type_idx))));
          }
          
          dcm_info.image_type = image_type;
        }
      }

      {
        ManufacturerAttr manufacturer_attr;
        manufacturer_attr.SetFromDataSet(ds);

        if (manufacturer_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(manufacturer_attr.GetNumberOfValues() == 1);

          dcm_info.manufacturer = StringStrip(StringStripExtraNulls(manufacturer_attr.GetValue()));
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x0080)))
      {
        InstitutionNameAttr inst_name_attr;
        inst_name_attr.SetFromDataSet(ds);

        if (inst_name_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(inst_name_attr.GetNumberOfValues() == 1);

          dcm_info.institution_name = StringStrip(StringStripExtraNulls(inst_name_attr.GetValue()));
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0008,0x1040)))
      {
        InstitutionDeptAttr inst_dept_attr;
        inst_dept_attr.SetFromDataSet(ds);

        if (inst_dept_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(inst_dept_attr.GetNumberOfValues() == 1);

          dcm_info.department_name = StringStrip(StringStripExtraNulls(inst_dept_attr.GetValue()));
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0008,0x1090)))
      {
        ManufacturerModelNameAttr manufacturer_model_attr;
        manufacturer_model_attr.SetFromDataSet(ds);

        if (manufacturer_model_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(manufacturer_model_attr.GetNumberOfValues() == 1);

          dcm_info.manufacturers_model_name = StringStrip(StringStripExtraNulls(
                                                    manufacturer_model_attr.GetValue()));
        }
      }

      {
        SecCapDevManAttr sec_cap_dev_man_attr;
        sec_cap_dev_man_attr.SetFromDataSet(ds);

        if (sec_cap_dev_man_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(sec_cap_dev_man_attr.GetNumberOfValues() == 1);
          dcm_info.sec_cap_dev_manufacturer = StringStripExtraNulls(sec_cap_dev_man_attr.GetValue());
        }
      }

      {
        SecCapDevSWVersAttr sec_cap_dev_sw_vers_attr;
        sec_cap_dev_sw_vers_attr.SetFromDataSet(ds);

        if (sec_cap_dev_sw_vers_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(sec_cap_dev_sw_vers_attr.GetNumberOfValues() == 1);
          dcm_info.sec_cap_dev_software_versions = StringStripExtraNulls(sec_cap_dev_sw_vers_attr.GetValue());
        }
      }

      {
        SWVersionsAttr sw_vers_attr;
        sw_vers_attr.SetFromDataSet(ds);

        if (sw_vers_attr.GetNumberOfValues() > 0)
        {
          const int num_sw_ver_toks = sw_vers_attr.GetNumberOfValues();

          std::vector<std::string> software_versions;
          software_versions.reserve(num_sw_ver_toks);

          for (int sw_ver_idx = 0; sw_ver_idx < num_sw_ver_toks; ++sw_ver_idx)
          {
            software_versions.push_back(StringStripExtraNulls(sw_vers_attr.GetValue(sw_ver_idx)));
          }
          
          dcm_info.software_versions = software_versions;
        }
      }

      // See *** note *** below about GDCM always populating fields even when they are not present
      // For this case it is always set to empty, but it is more appropriate to mark as invalid
      if (ds.FindDataElement(gdcm::Tag(0x0008,0x9206)))
      {
        VolPropsAttr vol_props_attr;
        vol_props_attr.SetFromDataSet(ds);

        if (vol_props_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(vol_props_attr.GetNumberOfValues() == 1);
          
          dcm_info.vol_props = StringStrip(StringStripExtraNulls(vol_props_attr.GetValue()));
        }
      }

      // *** NOTE ***      
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

          dcm_info.num_frames = num_frames_attr.GetValue();
        }
      }

      {
        ProtoNameAttr proto_name_attr;
        proto_name_attr.SetFromDataSet(ds);

        if (proto_name_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(proto_name_attr.GetNumberOfValues() == 1);
          
          dcm_info.proto_name = StringStripExtraNulls(proto_name_attr.GetValue());
        }
      }

      {
        ConvKernelAttr conv_kernel_attr;
        conv_kernel_attr.SetFromDataSet(ds);

        if (conv_kernel_attr.GetNumberOfValues() > 0)
        {
          if (conv_kernel_attr.GetNumberOfValues() > 1)
          {
            std::cerr << "WARNING: conv. kernel attr. (0018,1210) has more than one value! "
                         "Only the first value will be used!" << std::endl;
          }

          dcm_info.conv_kernel = StringStrip(StringStripExtraNulls(conv_kernel_attr.GetValue(0)));
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x0015)))
      {
        BodyPartAttr body_part_attr;
        body_part_attr.SetFromDataSet(ds);

        if (body_part_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(body_part_attr.GetNumberOfValues() == 1);

          dcm_info.body_part_examined = StringStripExtraNulls(body_part_attr.GetValue());
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x5101)))
      {
        ViewPosAttr view_pos_attr;
        view_pos_attr.SetFromDataSet(ds);

        if (view_pos_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(view_pos_attr.GetNumberOfValues() == 1);
          
          dcm_info.view_position = StringStripExtraNulls(view_pos_attr.GetValue());
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1110)))
      {
        DistSrcToDetAttr dist_src_to_det_attr;
        dist_src_to_det_attr.SetFromDataSet(ds);

        if (dist_src_to_det_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(dist_src_to_det_attr.GetNumberOfValues() == 1);
          
          const auto src_to_det_mm = dist_src_to_det_attr.GetValue();

          // Some datasets have this value, but it is set to zero and is not useful.
          // Only populate this field when the value is non-zero.
          if (src_to_det_mm > 1.0e-8)
          {
            dcm_info.dist_src_to_det_mm = src_to_det_mm;
          }
          else
          {
            std::cerr << "WARNING: source-to-detector distance field present, "
              "but not a positive value. Discarding." << std::endl;
          }
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1111)))
      {
        DistSrcToPatAttr dist_src_to_pat_attr;
        dist_src_to_pat_attr.SetFromDataSet(ds);

        if (dist_src_to_pat_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(dist_src_to_pat_attr.GetNumberOfValues() == 1);
          
          const auto src_to_pat_mm = dist_src_to_pat_attr.GetValue();

          // Some datasets have this value, but it is set to zero and is not useful.
          // Only populate this field when the value is non-zero.
          if (src_to_pat_mm > 1.0e-8)
          {
            dcm_info.dist_src_to_pat_mm = src_to_pat_mm;
          }
          else
          {
            std::cerr << "WARNING: source-to-patient distance field present, "
              "but not a positive value. Discarding." << std::endl;
          }
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x0060)))
      {
        KVPAttr kvp_attr;
        kvp_attr.SetFromDataSet(ds);

        if (kvp_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(kvp_attr.GetNumberOfValues() == 1);
          
          dcm_info.kvp = kvp_attr.GetValue();
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1151)))
      {
        TubeCurrentAttr tube_current_attr;
        tube_current_attr.SetFromDataSet(ds);

        if (tube_current_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(tube_current_attr.GetNumberOfValues() == 1);
          
          dcm_info.tube_current_mA = tube_current_attr.GetValue();
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1152)))
      {
        ExposuremAsAttr exposure_mAs_attr;
        exposure_mAs_attr.SetFromDataSet(ds);

        if (exposure_mAs_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(exposure_mAs_attr.GetNumberOfValues() == 1);
          
          dcm_info.exposure_mAs = exposure_mAs_attr.GetValue();
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1153)))
      {
        ExposuremuAsAttr exposure_muAs_attr;
        exposure_muAs_attr.SetFromDataSet(ds);

        if (exposure_muAs_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(exposure_muAs_attr.GetNumberOfValues() == 1);
          
          dcm_info.exposure_muAs = exposure_muAs_attr.GetValue();
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1150)))
      {
        ExposureTimeAttr exposure_time_attr;
        exposure_time_attr.SetFromDataSet(ds);

        if (exposure_time_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(exposure_time_attr.GetNumberOfValues() == 1);
          
          dcm_info.exposure_time_ms = exposure_time_attr.GetValue();
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x115E)))
      {
        DoseAreaProdAttr dose_area_prod_attr;
        dose_area_prod_attr.SetFromDataSet(ds);

        if (dose_area_prod_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(dose_area_prod_attr.GetNumberOfValues() == 1);
          
          dcm_info.dose_area_product_dGy_cm_sq = dose_area_prod_attr.GetValue();
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1147)))
      {
        FOVShapeAttr fov_shape_attr;
        fov_shape_attr.SetFromDataSet(ds);

        if (fov_shape_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(fov_shape_attr.GetNumberOfValues() == 1);
          
          dcm_info.fov_shape = StringStrip(StringStripExtraNulls(fov_shape_attr.GetValue()));
        }
      }
     
      {
        // See note above why we do things differently for FOV dims
        gdcm::Tag fov_dims_tag(0x0018,0x1149);

        if (ds.FindDataElement(fov_dims_tag))
        {
          // The FOV dims value is stored as an integer string or two integer strings,
          // separated by \, when the FOV shape is RECTANGLE

          try
          {
            std::vector<char> val_str = dynamic_cast<const gdcm::ByteValue&>(
                                            ds.GetDataElement(fov_dims_tag).GetValue());
            val_str.push_back(0);

            dcm_info.fov_dims = StringCast<unsigned long>(StringSplit(val_str.data(), "\\"));
          }
          catch (const std::bad_cast&) { }
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x7030)))
      {
        FOVOriginOffAttr fov_origin_off_attr;
        fov_origin_off_attr.SetFromDataSet(ds);

        if (fov_origin_off_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(fov_origin_off_attr.GetNumberOfValues() == 2);
          
          std::array<unsigned long,2> tmp_fov_origin_off;
          
          tmp_fov_origin_off[0] = fov_origin_off_attr.GetValue(0);
          tmp_fov_origin_off[1] = fov_origin_off_attr.GetValue(1);

          dcm_info.fov_origin_off = tmp_fov_origin_off;
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x7032)))
      {
        FOVRotAttr fov_rot_attr;
        fov_rot_attr.SetFromDataSet(ds);

        if (fov_rot_attr.GetNumberOfValues() > 0)
        {
          const int tmp_fov_rot = fov_rot_attr.GetValue();

          xregASSERT((tmp_fov_rot == 0) || (tmp_fov_rot == 90) ||
                     (tmp_fov_rot == 180) || (tmp_fov_rot == 270));

          dcm_info.fov_rot = static_cast<DICOMFIleBasicFields::FOVRot>(tmp_fov_rot);
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x7034)))
      {
        FOVHorizFlipAttr fov_horiz_flip_attr;
        fov_horiz_flip_attr.SetFromDataSet(ds);

        if (fov_horiz_flip_attr.GetNumberOfValues() > 0)
        {
          dcm_info.fov_horizontal_flip = fov_horiz_flip_attr.GetValue() == "YES";
        }
      }
    
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1162)))
      {
        IntensifierDiameterAttr intensifier_diam_attr;
        intensifier_diam_attr.SetFromDataSet(ds);

        if (intensifier_diam_attr.GetNumberOfValues() > 0)
        {
          xregASSERT(intensifier_diam_attr.GetNumberOfValues() == 1);
          
          dcm_info.intensifier_diameter_mm = intensifier_diam_attr.GetValue();
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0018,0x1164)))
      {
        ImagerPixelSpacingAttr imager_pixel_spacing_attr;
        imager_pixel_spacing_attr.SetFromDataSet(ds);

        xregASSERT(imager_pixel_spacing_attr.GetNumberOfValues() == 2);
        
        dcm_info.imager_pixel_spacing =
          std::array<CoordScalar,2> { static_cast<CoordScalar>(imager_pixel_spacing_attr.GetValue(0)),
                                      static_cast<CoordScalar>(imager_pixel_spacing_attr.GetValue(1)) };
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0018,0x704c)))
      {
        GridFocalDistAttr grid_focal_dist_attr;
        grid_focal_dist_attr.SetFromDataSet(ds);

        if (grid_focal_dist_attr.GetNumberOfValues() > 0)
        {
          dcm_info.grid_focal_dist_mm = grid_focal_dist_attr.GetValue();
        }
      }
      
      if (ds.FindDataElement(gdcm::Tag(0x0028,0x1050)))
      {
        WinCenterAttr win_center_attr;
        win_center_attr.SetFromDataSet(ds);

        if (win_center_attr.GetNumberOfValues() > 0)
        {
          dcm_info.window_center = win_center_attr.GetValue(0);
        }
      }

      if (ds.FindDataElement(gdcm::Tag(0x0028,0x1051)))
      {
        WinWidthAttr win_width_attr;
        win_width_attr.SetFromDataSet(ds);

        if (win_width_attr.GetNumberOfValues() > 0)
        {
          dcm_info.window_width = win_width_attr.GetValue(0);
        }
      }
    
      dcm_info.file_path = dcm_path;
      
      return dcm_info;
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

  out << indent << "                   File Path: " << dcm_info.file_path << '\n'
      << indent << "                  Patient ID: " << dcm_info.patient_id << '\n'
      << indent << "                Patient Name: " << dcm_info.patient_name << '\n'
      << indent << "                   Study UID: " << dcm_info.study_uid << '\n'
      << indent << "                  Series UID: " << dcm_info.series_uid << '\n'
      << indent << "                  Study Time: " << dcm_info.study_time << '\n'
      << indent << "                 Series Time: " << (dcm_info.series_time ? fmt::format("{}", *dcm_info.series_time) : kNOT_PROVIDED_STR) << '\n'
      << indent << "            Acquisition Time: " << (dcm_info.acquisition_time ? fmt::format("{}", *dcm_info.acquisition_time) : kNOT_PROVIDED_STR) << '\n'
      << indent << "                Content Time: " << (dcm_info.content_time ? fmt::format("{}", *dcm_info.content_time) : kNOT_PROVIDED_STR) << '\n'
      << indent << "                    Modality: " << dcm_info.modality << '\n'
      << indent << "              Image Position: " << fmt::sprintf("[%+10.4f,%+10.4f,%+10.4f]", dcm_info.img_pos_wrt_pat[0], dcm_info.img_pos_wrt_pat[1], dcm_info.img_pos_wrt_pat[2]) << '\n'
      << indent << "              Image Col Dir.: " << fmt::sprintf("[%+0.4f,%+0.4f,%+0.4f]", dcm_info.col_dir[0], dcm_info.col_dir[1], dcm_info.col_dir[2]) << '\n'
      << indent << "              Image Row Dir.: " << fmt::sprintf("[%+0.4f,%+0.4f,%+0.4f]", dcm_info.row_dir[0], dcm_info.row_dir[1], dcm_info.row_dir[2]) << '\n'
      << indent << "           Image Col Spacing: " << fmt::sprintf("%0.4f", dcm_info.col_spacing) << '\n'
      << indent << "           Image Row Spacing: " << fmt::sprintf("%0.4f", dcm_info.row_spacing) << '\n'
      << indent << "              Image Num Rows: " << dcm_info.num_rows << '\n'
      << indent << "              Image Num Cols: " << dcm_info.num_cols << '\n'
      << indent << "            Patient Position: " << (dcm_info.pat_pos ? *dcm_info.pat_pos : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Patient Orient.: " << (dcm_info.pat_orient ?
                                                          fmt::sprintf("%s , %s",
                                                                       (*dcm_info.pat_orient)[0],
                                                                       (*dcm_info.pat_orient)[1]) :
                                                          kNOT_PROVIDED_STR) << '\n'
      << indent << "                 Study Desc.: " << (dcm_info.study_desc ? *dcm_info.study_desc : kNOT_PROVIDED_STR) << '\n'
      << indent << "                Series Desc.: " << (dcm_info.series_desc ? *dcm_info.series_desc : kNOT_PROVIDED_STR) << '\n'
      << indent << "                  Image Type: " << (dcm_info.image_type ? JoinTokens(*dcm_info.image_type, " , ") : kNOT_PROVIDED_STR) << '\n'
      << indent << "                Manufacturer: " << dcm_info.manufacturer << '\n'
      << indent << "            Institution Name: " << (dcm_info.institution_name ? *dcm_info.institution_name : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Department Name: " << (dcm_info.department_name ? *dcm_info.department_name : kNOT_PROVIDED_STR) << '\n'
      << indent << "           Manuf. Model Name: " << (dcm_info.manufacturers_model_name ? *dcm_info.manufacturers_model_name : kNOT_PROVIDED_STR) << '\n'
      << indent << "         Sec. Cap. Dev. Man.: " << (dcm_info.sec_cap_dev_manufacturer ? *dcm_info.sec_cap_dev_manufacturer : kNOT_PROVIDED_STR) << '\n'
      << indent << "      Sec. Cap. Dev. SW Ver.: " << (dcm_info.sec_cap_dev_software_versions ? *dcm_info.sec_cap_dev_software_versions : kNOT_PROVIDED_STR) << '\n'
      << indent << "           Software Versions: " << (dcm_info.software_versions ? JoinTokens(*dcm_info.software_versions, " , ") : kNOT_PROVIDED_STR) << '\n'
      << indent << "                 Vol. Props.: " << (dcm_info.vol_props ? *dcm_info.vol_props : kNOT_PROVIDED_STR) << '\n'
      << indent << "                 Num. Frames: " << (dcm_info.num_frames ? fmt::format("{}", *dcm_info.num_frames) : kNOT_PROVIDED_STR) << '\n'
      << indent << "               Protocol Name: " << (dcm_info.proto_name ? *dcm_info.proto_name : kNOT_PROVIDED_STR) << '\n'
      << indent << "                Conv. Kernel: " << (dcm_info.conv_kernel ? *dcm_info.conv_kernel : kNOT_PROVIDED_STR) << '\n'
      << indent << "                   Body Part: " << (dcm_info.body_part_examined ? *dcm_info.body_part_examined : kNOT_PROVIDED_STR) << '\n'
      << indent << "               View Position: " << (dcm_info.view_position ? *dcm_info.view_position : kNOT_PROVIDED_STR) << '\n'
      << indent << "      Dist. Src-to-Det. (mm): " << (dcm_info.dist_src_to_det_mm ? fmt::format("{:.1f}", *dcm_info.dist_src_to_det_mm) : kNOT_PROVIDED_STR) << '\n'
      << indent << "      Dist. Src-to-Pat. (mm): " << (dcm_info.dist_src_to_pat_mm ? fmt::format("{:.1f}", *dcm_info.dist_src_to_pat_mm) : kNOT_PROVIDED_STR) << '\n'
      << indent << "          Peak Engergy (kVp): " << (dcm_info.kvp ? fmt::format("{:.1f}", *dcm_info.kvp) : kNOT_PROVIDED_STR) << '\n'
      << indent << "           Tube Current (mA): " << (dcm_info.tube_current_mA ? fmt::format("{:.1f}", *dcm_info.tube_current_mA) : kNOT_PROVIDED_STR) << '\n'
      << indent << "              Exposure (mAs): " << (dcm_info.exposure_mAs ? fmt::format("{:.3f}", *dcm_info.exposure_mAs) : kNOT_PROVIDED_STR) << '\n'
      << indent << "             Exposure (muAs): " << (dcm_info.exposure_muAs ? fmt::format("{:.3f}", *dcm_info.exposure_muAs) : kNOT_PROVIDED_STR) << '\n'
      << indent << "          Exposure Time (ms): " << (dcm_info.exposure_time_ms ? fmt::format("{:.3f}", *dcm_info.exposure_time_ms) : kNOT_PROVIDED_STR) << '\n'
      << indent << "  Dose Area Prod. (dGy*cm^2): " << (dcm_info.dose_area_product_dGy_cm_sq ? fmt::format("{:.3f}", *dcm_info.dose_area_product_dGy_cm_sq) : kNOT_PROVIDED_STR) << '\n'
      << indent << "                   FOV Shape: " << (dcm_info.fov_shape ? *dcm_info.fov_shape : kNOT_PROVIDED_STR) << '\n'
      << indent << "         FOV Dimensions (mm): " << (dcm_info.fov_dims ? JoinTokens(ToStrings(*dcm_info.fov_dims), " , ") : kNOT_PROVIDED_STR) << '\n'
      << indent << "           FOV Origin Offset: " << (dcm_info.fov_origin_off ? fmt::format("[{} , {}]", (*dcm_info.fov_origin_off)[0], (*dcm_info.fov_origin_off)[1]) : kNOT_PROVIDED_STR) << '\n'
      << indent << "                FOV Rotation: " << (dcm_info.fov_rot ? fmt::format("{}", static_cast<int>(*dcm_info.fov_rot)) : kNOT_PROVIDED_STR) << '\n'
      << indent << "         FOV Horizontal Flip: " << (dcm_info.fov_horizontal_flip ? std::string(*dcm_info.fov_horizontal_flip ? "YES" : "NO") : kNOT_PROVIDED_STR) << '\n'
      << indent << "      Intensifier Diam. (mm): " << (dcm_info.intensifier_diameter_mm ? fmt::format("{:.1f}", *dcm_info.intensifier_diameter_mm) : kNOT_PROVIDED_STR) << '\n'
      << indent << " Imager Pix. Spacing (mm/px): "
                << (dcm_info.imager_pixel_spacing ?
                      fmt::format("[ {:.3f} , {:.3f}]",
                                  (*dcm_info.imager_pixel_spacing)[0],
                                  (*dcm_info.imager_pixel_spacing)[1]) :
                      kNOT_PROVIDED_STR)
                << '\n'
      << indent << "       Grid Focal Dist. (mm): " << (dcm_info.grid_focal_dist_mm ? fmt::format("{}", *dcm_info.grid_focal_dist_mm) : kNOT_PROVIDED_STR) << '\n'
      << indent << "               Window Center: " << (dcm_info.window_center ? fmt::format("{}", *dcm_info.window_center) : kNOT_PROVIDED_STR) << '\n'
      << indent << "                Window Width: " << (dcm_info.window_width ? fmt::format("{}", *dcm_info.window_width) : kNOT_PROVIDED_STR) << '\n'
                << '\n';

  out.flush();
}

bool xreg::IsLocalizer(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.image_type &&
          (std::find(dcm_info.image_type->begin(), dcm_info.image_type->end(), "LOCALIZER")
                  != dcm_info.image_type->end());
}

bool xreg::IsMRLocalizer(const DICOMFIleBasicFields& dcm_info)
{
  return (dcm_info.modality == "MR") &&
         (dcm_info.image_type &&
          (std::find(dcm_info.image_type->begin(), dcm_info.image_type->end(), "LOCALIZER")
                  != dcm_info.image_type->end()));
}

bool xreg::IsVolDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.vol_props && (*dcm_info.vol_props == "VOLUME");
}

bool xreg::IsMultiFrameDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.num_frames && (*dcm_info.num_frames > 1);
}

bool xreg::IsSecondaryDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.image_type &&
          (std::find(dcm_info.image_type->begin(), dcm_info.image_type->end(), "SECONDARY")
                  != dcm_info.image_type->end());
}

bool xreg::IsDerivedDICOMFile(const DICOMFIleBasicFields& dcm_info)
{
  return dcm_info.image_type &&
          (std::find(dcm_info.image_type->begin(), dcm_info.image_type->end(), "DERIVED")
                  != dcm_info.image_type->end());
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
                                   const bool inc_multi_frame_files,
                                   const bool inc_secondary,
                                   const bool inc_derived,
                                   const std::vector<std::string>& modalities)
{
  const bool check_modality = !modalities.empty();

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

      tmp_basic_fields = ReadDICOMFileBasicFields(cur_file_path);

      if ((inc_localizer  || !IsLocalizer(tmp_basic_fields)) &&
          (inc_multi_frame_files || !IsMultiFrameDICOMFile(tmp_basic_fields)) &&
          (inc_secondary || !IsSecondaryDICOMFile(tmp_basic_fields)) &&
          (inc_derived || !IsDerivedDICOMFile(tmp_basic_fields)) &&
          (!check_modality ||
           (std::find(modalities.begin(), modalities.end(), tmp_basic_fields.modality)
                                                                     != modalities.end())))
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
          dcm_info = ReadDICOMFileBasicFields(paths[0]);

          if (first_series)
          {
            out << indent3 << "Study Description: "
                << (dcm_info.study_desc ? dcm_info.study_desc->c_str()
                                        : "(Not Provided)")
                << std::endl;
          }
        }

        out << indent4 << "Series UID: " << series2paths.first << '\n'
            << indent4 << "Number of Files: " << paths.size() << '\n';
        
        if (print_dcm_info)
        {
          out << indent4 << "Series Description: "
              << (dcm_info.series_desc ? dcm_info.series_desc->c_str()
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
    dcms[dcm_idx] = ReadDICOMFileBasicFields(dcm_paths[dcm_idx]);
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

namespace
{

template <class tScalar>
void WriteOptionalScalarH5(const std::string& field_name,
                           const boost::optional<tScalar>& opt_val,
                           H5::Group* h5)
{
  if (opt_val)
  {
    xreg::WriteSingleScalarH5(field_name, *opt_val, h5);
  }
}

void WriteOptionalStringH5(const std::string& field_name,
                           const boost::optional<std::string>& opt_str,
                           H5::Group* h5)
{
  if (opt_str)
  {
    xreg::WriteStringH5(field_name, *opt_str, h5);
  }
}

void WriteOptionalListOfStrings(const std::string& field_name,
                                const boost::optional<std::vector<std::string>>& opt_strs,
                                H5::Group* h5)
{
  using namespace xreg;

  if (opt_strs)
  {
    H5::Group strs_g = h5->createGroup(field_name);
    
    const size_type len = opt_strs->size();

    SetScalarAttr("len", static_cast<long>(len), &strs_g);

    for (size_type i = 0; i < len; ++i)
    {
      WriteStringH5(fmt::format("{:02d}", i), opt_strs->operator[](i), &strs_g);
    }
  }
}

template <class tScalar>
boost::optional<tScalar> ReadOptionalScalarH5(const std::string& field_name,
                                              const H5::Group& h5)
{
  using Scalar = tScalar;

  boost::optional<Scalar> opt_val;

  if (xreg::ObjectInGroupH5(field_name, h5))
  {
    opt_val = xreg::detail::ReadSingleScalarH5Helper<Scalar>(field_name, h5);
  }

  return opt_val;
}

boost::optional<std::string> ReadOptionalStringH5(const std::string& field_name,
                                                  const H5::Group& h5)
{
  boost::optional<std::string> opt_str;

  if (xreg::ObjectInGroupH5(field_name, h5))
  {
    opt_str = xreg::ReadStringH5(field_name, h5);
  }

  return opt_str;
}

boost::optional<std::vector<std::string>>
ReadOptionalListOfStrings(const std::string& field_name,
                          const H5::Group& h5)
{
  using namespace xreg;
  
  boost::optional<std::vector<std::string>> opt_strs;
  
  if (ObjectInGroupH5(field_name, h5))
  {
    H5::Group strs_g = h5.openGroup(field_name);
    
    const size_type len = GetScalarLongAttr("len", strs_g);
    
    std::vector<std::string> strs;
    strs.reserve(len);

    for (size_type i = 0; i < len; ++i)
    {
      strs.push_back(ReadStringH5(fmt::format("{:02d}", i), strs_g));
    }

    opt_strs = strs;
  }

  return opt_strs;
}

}  // un-named

void xreg::WriteDICOMFieldsH5(const DICOMFIleBasicFields& dcm_info, H5::Group* h5)
{
  WriteStringH5("file-path", dcm_info.file_path, h5);

  WriteStringH5("patient-id", dcm_info.patient_id, h5);
  WriteStringH5("series-uid", dcm_info.series_uid, h5);
  WriteStringH5("study-uid", dcm_info.study_uid, h5);

  WriteStringH5("patient-name", dcm_info.patient_name, h5);
  
  WriteSingleScalarH5("study-time", dcm_info.study_time, h5);
  WriteOptionalScalarH5("series-time", dcm_info.series_time, h5);
  WriteOptionalScalarH5("acquisition-time", dcm_info.acquisition_time, h5);
  WriteOptionalScalarH5("content-time", dcm_info.content_time, h5);
  
  WriteStringH5("modality", dcm_info.modality, h5);
  
  WriteMatrixH5("img-pos-wrt-pat", dcm_info.img_pos_wrt_pat, h5);

  WriteMatrixH5("row-dir", dcm_info.row_dir, h5);
  WriteMatrixH5("col-dir", dcm_info.col_dir, h5);

  WriteSingleScalarH5("row-spacing", dcm_info.row_spacing, h5);
  WriteSingleScalarH5("col-spacing", dcm_info.col_spacing, h5);

  WriteSingleScalarH5("num-rows", dcm_info.num_rows, h5);
  WriteSingleScalarH5("num-cols", dcm_info.num_cols, h5);

  WriteOptionalStringH5("pat-pos", dcm_info.pat_pos, h5);

  if (dcm_info.pat_orient)
  {
    const auto& pat_orient = *dcm_info.pat_orient;

    WriteStringH5("pat-orient-x", pat_orient[0], h5);
    WriteStringH5("pat-orient-y", pat_orient[1], h5);
  }
  
  WriteOptionalStringH5("study-desc", dcm_info.study_desc, h5);
  
  WriteOptionalStringH5("series-desc", dcm_info.series_desc, h5);

  WriteOptionalListOfStrings("image-type", dcm_info.image_type, h5);

  WriteStringH5("manufacturer", dcm_info.manufacturer, h5);

  WriteOptionalStringH5("institution-name", dcm_info.institution_name, h5);
  
  WriteOptionalStringH5("department-name", dcm_info.department_name, h5);

  WriteOptionalStringH5("manufacturers-model-name", dcm_info.manufacturers_model_name, h5);
  
  WriteOptionalStringH5("sec-cap-dev-manufacturer", dcm_info.sec_cap_dev_manufacturer, h5);

  WriteOptionalStringH5("sec-cap-dev-software-versions", dcm_info.sec_cap_dev_software_versions, h5);

  WriteOptionalListOfStrings("software-versions", dcm_info.software_versions, h5);

  WriteOptionalStringH5("vol-props", dcm_info.vol_props, h5);

  WriteOptionalScalarH5("num-frames", dcm_info.num_frames, h5);

  WriteOptionalStringH5("proto-name", dcm_info.proto_name, h5);

  WriteOptionalStringH5("conv-kernel", dcm_info.conv_kernel, h5);

  WriteOptionalStringH5("body-part-examined", dcm_info.body_part_examined, h5);

  WriteOptionalStringH5("view-position", dcm_info.view_position, h5);

  WriteOptionalScalarH5("dist-src-to-det-mm", dcm_info.dist_src_to_det_mm, h5);

  WriteOptionalScalarH5("dist-src-to-pat-mm", dcm_info.dist_src_to_pat_mm, h5);

  WriteOptionalScalarH5("kvp", dcm_info.kvp, h5);

  WriteOptionalScalarH5("tube-current-mA", dcm_info.tube_current_mA, h5);

  WriteOptionalScalarH5("exposure-mAs", dcm_info.exposure_mAs, h5);

  WriteOptionalScalarH5("exposure-muAs", dcm_info.exposure_muAs, h5);

  WriteOptionalScalarH5("exposure-time-ms", dcm_info.exposure_time_ms, h5);

  WriteOptionalScalarH5("dose-area-product-dGy-cm-sq", dcm_info.dose_area_product_dGy_cm_sq, h5);

  WriteOptionalStringH5("fov-shape", dcm_info.fov_shape, h5);

  if (dcm_info.fov_dims)
  {
    WriteVectorH5("fov-dims", *dcm_info.fov_dims, h5);
  }
  
  if (dcm_info.fov_origin_off)
  {
    const auto origin_off = *dcm_info.fov_origin_off;

    WriteSingleScalarH5("fov-origin-off-rows", origin_off[0], h5);
    WriteSingleScalarH5("fov-origin-off-cols", origin_off[1], h5);
  }

  if (dcm_info.fov_rot)
  {
    WriteSingleScalarH5("fov-rot", static_cast<int>(*dcm_info.fov_rot), h5);
  }

  WriteOptionalScalarH5("fov-horizontal-flip", dcm_info.fov_horizontal_flip, h5);

  WriteOptionalScalarH5("intensifier-diameter-mm", dcm_info.intensifier_diameter_mm, h5);

  if (dcm_info.imager_pixel_spacing)
  {
    const auto& ps = *dcm_info.imager_pixel_spacing;

    WriteSingleScalarH5("imager-pixel-row-spacing", ps[0], h5);
    WriteSingleScalarH5("imager-pixel-col-spacing", ps[1], h5);
  }

  WriteOptionalScalarH5("grid-focal-dist-mm", dcm_info.grid_focal_dist_mm, h5);
  
  WriteOptionalScalarH5("window-center", dcm_info.window_center, h5);
  WriteOptionalScalarH5("window-width", dcm_info.window_width, h5);
}

xreg::DICOMFIleBasicFields xreg::ReadDICOMFieldsH5(const H5::Group& h5)
{
  DICOMFIleBasicFields dcm_info;

  dcm_info.file_path = ReadStringH5("file-path", h5);

  dcm_info.patient_id = ReadStringH5("patient-id", h5);
  dcm_info.series_uid = ReadStringH5("series-uid", h5);
  dcm_info.study_uid = ReadStringH5("study-uid", h5);

  dcm_info.patient_name = ReadStringH5("patient-name", h5);
  
  dcm_info.study_time = ReadSingleScalarH5Double("study-time", h5);
  dcm_info.series_time = ReadOptionalScalarH5<double>("series-time", h5);
  dcm_info.acquisition_time = ReadOptionalScalarH5<double>("acquisition-time", h5);
  dcm_info.content_time = ReadOptionalScalarH5<double>("content-time", h5);
  
  dcm_info.modality = ReadStringH5("modality", h5);
  
  dcm_info.img_pos_wrt_pat = detail::ReadMatrixH5Helper<CoordScalar>("img-pos-wrt-pat", h5);

  dcm_info.row_dir = detail::ReadMatrixH5Helper<CoordScalar>("row-dir", h5);
  dcm_info.col_dir = detail::ReadMatrixH5Helper<CoordScalar>("col-dir", h5);

  dcm_info.row_spacing = ReadSingleScalarH5CoordScalar("row-spacing", h5);
  dcm_info.col_spacing = ReadSingleScalarH5CoordScalar("col-spacing", h5);

  dcm_info.num_rows = ReadSingleScalarH5ULong("num-rows", h5);
  dcm_info.num_cols = ReadSingleScalarH5ULong("num-cols", h5);

  dcm_info.pat_pos = ReadOptionalStringH5("pat-pos", h5);

  if (ObjectInGroupH5("pat-orient-x", h5) && ObjectInGroupH5("pat-orient-y", h5))
  {
    dcm_info.pat_orient = std::array<std::string,2>{ ReadStringH5("pat-orient-x", h5),
                                                     ReadStringH5("pat-orient-y", h5) };
  }

  dcm_info.study_desc = ReadOptionalStringH5("study-desc", h5);
  
  dcm_info.series_desc = ReadOptionalStringH5("series-desc", h5);

  dcm_info.image_type = ReadOptionalListOfStrings("image-type", h5);

  dcm_info.manufacturer = ReadStringH5("manufacturer", h5);

  dcm_info.institution_name = ReadOptionalStringH5("institution-name", h5);
  
  dcm_info.department_name = ReadOptionalStringH5("department-name", h5);

  dcm_info.manufacturers_model_name = ReadOptionalStringH5("manufacturers-model-name", h5);
  
  dcm_info.sec_cap_dev_manufacturer = ReadOptionalStringH5("sec-cap-dev-manufacturer", h5);

  dcm_info.sec_cap_dev_software_versions = ReadOptionalStringH5("sec-cap-dev-software-versions", h5);

  dcm_info.software_versions = ReadOptionalListOfStrings("software-versions", h5);

  dcm_info.vol_props = ReadOptionalStringH5("vol-props", h5);

  dcm_info.num_frames = ReadOptionalScalarH5<unsigned long>("num-frames", h5);

  dcm_info.proto_name = ReadOptionalStringH5("proto-name", h5);

  dcm_info.conv_kernel = ReadOptionalStringH5("conv-kernel", h5);

  dcm_info.body_part_examined = ReadOptionalStringH5("body-part-examined", h5);

  dcm_info.view_position = ReadOptionalStringH5("view-position", h5);

  dcm_info.dist_src_to_det_mm = ReadOptionalScalarH5<double>("dist-src-to-det-mm", h5);

  dcm_info.dist_src_to_pat_mm = ReadOptionalScalarH5<double>("dist-src-to-pat-mm", h5);

  dcm_info.kvp = ReadOptionalScalarH5<double>("kvp", h5);

  dcm_info.tube_current_mA = ReadOptionalScalarH5<double>("tube-current-mA", h5);

  dcm_info.exposure_mAs = ReadOptionalScalarH5<double>("exposure-mAs", h5);

  dcm_info.exposure_muAs = ReadOptionalScalarH5<double>("exposure-muAs", h5);

  dcm_info.exposure_time_ms = ReadOptionalScalarH5<double>("exposure-time-ms", h5);

  dcm_info.dose_area_product_dGy_cm_sq = ReadOptionalScalarH5<double>("dose-area-product-dGy-cm-sq", h5);

  dcm_info.fov_shape = ReadOptionalStringH5("fov-shape", h5);

  if (ObjectInGroupH5("fov-dims", h5))
  {
    dcm_info.fov_dims = ReadVectorH5ULong("fov-dims", h5);
  }
  
  if (ObjectInGroupH5("fov-origin-off-rows", h5) && ObjectInGroupH5("fov-origin-off-cols", h5))
  {
    dcm_info.fov_origin_off = std::array<unsigned long,2>{
                                 ReadSingleScalarH5ULong("fov-origin-off-rows", h5),
                                 ReadSingleScalarH5ULong("fov-origin-off-cols", h5) };
  }

  if (ObjectInGroupH5("fov-rot", h5))
  {
    dcm_info.fov_rot = static_cast<DICOMFIleBasicFields::FOVRot>(
                                            ReadSingleScalarH5Int("fov-rot", h5));
  }

  dcm_info.fov_horizontal_flip = ReadOptionalScalarH5<unsigned char>("fov-horizontal-flip", h5);

  dcm_info.intensifier_diameter_mm = ReadOptionalScalarH5<double>("intensifier-diameter-mm", h5);

  if (ObjectInGroupH5("imager-pixel-row-spacing", h5) &&
      ObjectInGroupH5("imager-pixel-col-spacing", h5))
  {
    dcm_info.imager_pixel_spacing = std::array<CoordScalar,2>{ 
              ReadSingleScalarH5CoordScalar("imager-pixel-row-spacing", h5),
              ReadSingleScalarH5CoordScalar("imager-pixel-col-spacing", h5) };
  }

  dcm_info.grid_focal_dist_mm = ReadOptionalScalarH5<double>("grid-focal-dist-mm", h5);
  
  dcm_info.window_center = ReadOptionalScalarH5<double>("window-center", h5);
  dcm_info.window_width = ReadOptionalScalarH5<double>("window-width", h5);

  return dcm_info;
}

