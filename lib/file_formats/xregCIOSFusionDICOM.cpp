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

#include "xregCIOSFusionDICOM.h"

#include <set>

#include <gdcmReader.h>
#include <gdcmAttribute.h>

#include "xregStringUtils.h"
#include "xregExceptionUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregOpenCVUtils.h"
#include "xregHDF5.h"

xreg::CIOSFusionDICOMInfo xreg::ReadCIOSFusionDICOMMetadata(const std::string& path)
{
  CIOSFusionDICOMInfo meta;

  gdcm::Reader dcm_reader;
  dcm_reader.SetFileName(path.c_str());
  
  if (dcm_reader.CanRead())
  {
    using StudyTimeAttr  = gdcm::Attribute<0x0008,0x0030>;
    using SeriesTimeAttr = gdcm::Attribute<0x0008,0x0031>;
    using AcquisitionTimeAttr = gdcm::Attribute<0x0008,0x0032>;

    using PatNameAttr = gdcm::Attribute<0x0010,0x0010>;
    using PatIdAttr   = gdcm::Attribute<0x0010,0x0020>;
  
    using KVPAttr = gdcm::Attribute<0x0018,0x0060>;

    using DistSrcToDetAttr = gdcm::Attribute<0x0018,0x1110>;

    using TubeCurrentAttr  = gdcm::Attribute<0x0018,0x1151>;
    using ExposureAttr     = gdcm::Attribute<0x0018,0x1152>;
    using ExposureMuAsAttr = gdcm::Attribute<0x0018,0x1153>;

    using PixelSpacingAttr = gdcm::Attribute<0x0018,0x1164>;

    using FOVOriginOffAttr = gdcm::Attribute<0x0018,0x7030>;
    using FOVRotAttr       = gdcm::Attribute<0x0018,0x7032>;
    using FOVHorizFlipAttr = gdcm::Attribute<0x0018,0x7034>;

    using GridFocalDistAttr = gdcm::Attribute<0x0018,0x704c>;
    
    using TubeCurrentInMuAAttr = gdcm::Attribute<0x0018,0x8151>;

    using RowsAttr = gdcm::Attribute<0x0028,0x0010>;
    using ColsAttr = gdcm::Attribute<0x0028,0x0011>;

    using WinCenterAttr = gdcm::Attribute<0x0028,0x1050>;
    using WinWidthAttr  = gdcm::Attribute<0x0028,0x1051>;

    std::set<gdcm::Tag> tags_to_read;

    tags_to_read.insert(StudyTimeAttr::GetTag());
    tags_to_read.insert(SeriesTimeAttr::GetTag());
    tags_to_read.insert(AcquisitionTimeAttr::GetTag());
    
    tags_to_read.insert(PatNameAttr::GetTag());
    tags_to_read.insert(PatIdAttr::GetTag());

    tags_to_read.insert(KVPAttr::GetTag());

    tags_to_read.insert(DistSrcToDetAttr::GetTag());

    tags_to_read.insert(TubeCurrentAttr::GetTag());
    tags_to_read.insert(ExposureAttr::GetTag());
    tags_to_read.insert(ExposureMuAsAttr::GetTag());

    tags_to_read.insert(PixelSpacingAttr::GetTag());

    tags_to_read.insert(FOVOriginOffAttr::GetTag());
    tags_to_read.insert(FOVRotAttr::GetTag());
    tags_to_read.insert(FOVHorizFlipAttr::GetTag());

    tags_to_read.insert(GridFocalDistAttr::GetTag());

    tags_to_read.insert(TubeCurrentInMuAAttr::GetTag());
    
    tags_to_read.insert(RowsAttr::GetTag());
    tags_to_read.insert(ColsAttr::GetTag());

    tags_to_read.insert(WinCenterAttr::GetTag());
    tags_to_read.insert(WinWidthAttr::GetTag());
        
    if (dcm_reader.ReadSelectedTags(tags_to_read))
    {
      gdcm::DataSet& ds = dcm_reader.GetFile().GetDataSet();

      {
        StudyTimeAttr study_time_attr;
        study_time_attr.SetFromDataSet(ds);
        meta.study_time = StringCast<double>(study_time_attr.GetValue());
      }

      {
        SeriesTimeAttr series_time_attr;
        series_time_attr.SetFromDataSet(ds);
        meta.series_time = StringCast<double>(series_time_attr.GetValue());
      }

      {
        AcquisitionTimeAttr acq_time_attr;
        acq_time_attr.SetFromDataSet(ds);
        meta.acquisition_time = StringCast<double>(acq_time_attr.GetValue());
      }

      {
        PatNameAttr pat_name_attr;
        pat_name_attr.SetFromDataSet(ds);
        meta.pat_name = StringStripExtraNulls(pat_name_attr.GetValue());
      }

      {
        PatIdAttr pat_id_attr;
        pat_id_attr.SetFromDataSet(ds);
        meta.pat_id = StringStripExtraNulls(pat_id_attr.GetValue());
      }

      {
        auto de = ds.GetDataElement(KVPAttr::GetTag());
        
        if (!de.IsEmpty())
        {
          KVPAttr kvp_attr;
          kvp_attr.SetFromDataElement(de);
        
          meta.kvp = kvp_attr.GetValue();
        }
      }

      {
        DistSrcToDetAttr dist_src_to_det_attr;
        dist_src_to_det_attr.SetFromDataSet(ds);
        meta.dist_src_to_det = dist_src_to_det_attr.GetValue();
      }

      {
        auto de = ds.GetDataElement(TubeCurrentAttr::GetTag());

        if (!de.IsEmpty())
        {
          TubeCurrentAttr tube_current_attr;
          tube_current_attr.SetFromDataElement(de);

          meta.tube_current = tube_current_attr.GetValue();
        }
      }

      {
        auto de = ds.GetDataElement(ExposureAttr::GetTag());

        if (!de.IsEmpty())
        {
          ExposureAttr exposure_attr;
          exposure_attr.SetFromDataElement(de);

          meta.exposure = exposure_attr.GetValue();
        }
      }

      {
        auto de = ds.GetDataElement(ExposureMuAsAttr::GetTag());

        if (!de.IsEmpty())
        {
          ExposureMuAsAttr exposure_mu_As_attr;
          exposure_mu_As_attr.SetFromDataElement(de);

          meta.exposure_mu_As = exposure_mu_As_attr.GetValue();
        }
      }

      {
        PixelSpacingAttr pixel_spacing_attr;
        pixel_spacing_attr.SetFromDataSet(ds);

        meta.pixel_row_spacing = pixel_spacing_attr.GetValue(0);
        meta.pixel_col_spacing = pixel_spacing_attr.GetValue(1);
      }

      {
        FOVOriginOffAttr fov_origin_off_attr;
        fov_origin_off_attr.SetFromDataSet(ds);

        meta.fov_origin_row_off = fov_origin_off_attr.GetValue(0);
        meta.fov_origin_col_off = fov_origin_off_attr.GetValue(1);
      }

      {
        FOVRotAttr fov_rot_attr;
        fov_rot_attr.SetFromDataSet(ds);
        
        const double r = fov_rot_attr.GetValue();

        if (r == 0)
        {
          meta.fov_rot = CIOSFusionDICOMInfo::kZERO;
        }
        else if (r == 90)
        {
          meta.fov_rot = CIOSFusionDICOMInfo::kNINETY;
        }
        else if (r == 180)
        {
          meta.fov_rot = CIOSFusionDICOMInfo::kONE_EIGHTY;
        }
        else if (r == 270)
        {
          meta.fov_rot = CIOSFusionDICOMInfo::kTWO_SEVENTY;
        }
        else
        {
          xregThrow("Unsupported FOV Rotation Value: %f", r);
        }
      }

      {
        FOVHorizFlipAttr fov_horiz_flip_attr;
        fov_horiz_flip_attr.SetFromDataSet(ds);

        meta.fov_horizontal_flip = fov_horiz_flip_attr.GetValue() == "YES";
      }

      {
        GridFocalDistAttr grid_focal_dist_attr;
        grid_focal_dist_attr.SetFromDataSet(ds);

        meta.grid_focal_dist = grid_focal_dist_attr.GetValue();
      }

      {
        auto de = ds.GetDataElement(TubeCurrentInMuAAttr::GetTag());

        if (!de.IsEmpty())
        {
          TubeCurrentInMuAAttr tube_current_in_muA_attr;
          tube_current_in_muA_attr.SetFromDataElement(de);

          meta.tube_current_muA = tube_current_in_muA_attr.GetValue();
        }
      }

      {
        RowsAttr rows_attr;
        rows_attr.SetFromDataSet(ds);

        meta.rows = rows_attr.GetValue();
      }

      {
        ColsAttr cols_attr;
        cols_attr.SetFromDataSet(ds);

        meta.cols = cols_attr.GetValue();
      }

      {
        WinCenterAttr win_center_attr;
        win_center_attr.SetFromDataSet(ds);

        meta.window_center = win_center_attr.GetValue(0);
      }

      {
        WinWidthAttr win_width_attr;
        win_width_attr.SetFromDataSet(ds);

        meta.window_width = win_width_attr.GetValue(0);
      }
    }
    else
    {
      xregThrow("ReadCIOSFusionDICOMMetadata: Failed to read tags!");
    }
  }
  else
  {
    xregThrow("ReadCIOSFusionDICOMMetadata: Could not read DICOM!");
  }

  return meta;
}

xreg::Mat4x4 xreg::CIOSFusionCBCTExtrins()
{
  Mat4x4 extrins = Mat4x4::Identity();
    
  extrins(0,0) =    0.0014173902529073;
  extrins(0,1) =    0.0000057733016248;
  extrins(0,2) =    0.9999990957527923;
  extrins(0,3) = -157.5822163123799555;
  extrins(1,0) =   -0.9999907608500266;
  extrins(1,1) =   -0.0040758357952800;
  extrins(1,2) =    0.0014174006256202;
  extrins(1,3) =   -2.6757496083693013;
  extrins(2,0) =    0.0040758413782892;
  extrins(2,1) =   -0.9999918324063819;
  extrins(2,2) =   -0.0000000000000000;
  extrins(2,3) = -631.7764689585426368;

  return extrins;
}

xreg::CIOSFusionDICOMInfo xreg::MakeNaiveCIOSFusionMetaDR()
{
  CIOSFusionDICOMInfo meta;
  
  meta.dist_src_to_det = 1020;

  meta.pixel_row_spacing = 0.194;
  meta.pixel_col_spacing = 0.194;

  meta.rows = 1536;
  meta.cols = 1536;

  return meta;
}

xreg::CameraModel
xreg::NaiveCamModelFromCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                  const bool iso_center_at_origin)
{
  CameraModel cam;
  cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  const Mat3x3 intrins = MakeNaiveIntrins(meta.dist_src_to_det,
                                          meta.rows, meta.cols,
                                          meta.pixel_row_spacing,
                                          meta.pixel_col_spacing,
                                          true); 

  Mat4x4 extrins = Mat4x4::Identity();

  if (iso_center_at_origin)
  {
    extrins = CIOSFusionCBCTExtrins();
  }

  cam.setup(intrins, extrins, meta.rows, meta.cols,
            meta.pixel_row_spacing, meta.pixel_col_spacing);

  return cam;
}

xreg::CameraModel
xreg::NaiveCamModelFromCIOSFusionExtrins(const CIOSFusionDICOMInfo& meta,
                                         const Mat4x4 extrins)
{
  CameraModel cam;
  cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  const Mat3x3 intrins = MakeNaiveIntrins(meta.dist_src_to_det,
                                          meta.rows, meta.cols,
                                          meta.pixel_row_spacing,
                                          meta.pixel_col_spacing,
                                          true); 

  cam.setup(intrins, extrins, meta.rows, meta.cols,
            meta.pixel_row_spacing, meta.pixel_col_spacing);

  return cam;
}

namespace
{

using namespace xreg;

template <class tPixelType>
void ModifyImageWithCIOSFusionRotFlipFlagsHelper(const CIOSFusionDICOMInfo& meta,
                                                 itk::Image<tPixelType,2>* img)
{
  using PixelType = tPixelType;

  cv::Mat img_ocv = ShallowCopyItkToOpenCV(img);
  
  // check for flipping, etc.
  if (meta.fov_rot == CIOSFusionDICOMInfo::kNINETY)
  {
    xregASSERT(meta.rows == meta.cols);

    cv::Mat tmp = img_ocv.clone();
    cv::transpose(tmp, img_ocv);
    FlipImageColumns(&img_ocv);
  }
  else if (meta.fov_rot == CIOSFusionDICOMInfo::kONE_EIGHTY)
  {
    FlipImageRows(&img_ocv);
    FlipImageColumns(&img_ocv);
  }
  else if (meta.fov_rot == CIOSFusionDICOMInfo::kTWO_SEVENTY)
  {
    xregASSERT(meta.rows == meta.cols);

    cv::Mat tmp = img_ocv.clone();
    cv::transpose(tmp, img_ocv);
    FlipImageRows(&img_ocv);
  }

  // not clear if rows or columns should be flipped here
  // hopefully, we do not encounter this case
  if (meta.fov_horizontal_flip)
  {
    FlipImageColumns(&img_ocv);
  }
}

template <class tPixelType>
std::tuple<typename itk::Image<tPixelType,2>::Pointer,CIOSFusionDICOMInfo>
ReadCIOSFusionDICOMHelper(const std::string& path, const bool no_rot_or_flip_based_on_meta)
{
  using PixelType = tPixelType;
  using Img = itk::Image<PixelType,2>;
  using ImgPtr = typename Img::Pointer;

  const CIOSFusionDICOMInfo meta = ReadCIOSFusionDICOMMetadata(path);

  ImgPtr img = ReadDICOM2DFromDisk<tPixelType>(path);

  if (!no_rot_or_flip_based_on_meta)
  {
    ModifyImageWithCIOSFusionRotFlipFlagsHelper(meta, img.GetPointer());
  }

  return std::make_tuple(img, meta);
}

template <class tPointMapItr>
void UpdateLandmarkMapForCIOSFusionHelper(const CIOSFusionDICOMInfo& meta,
                                          tPointMapItr begin_it, tPointMapItr end_it,
                                          const bool no_rot_or_flip)
{
  using PointMapIt = tPointMapItr;

  for (PointMapIt it = begin_it; it != end_it; ++it)
  {
    auto& p = it->second;

    // convert from physical to index
    p(0) /= meta.pixel_col_spacing;
    p(1) /= meta.pixel_row_spacing;

    if (!no_rot_or_flip &&
        (meta.fov_horizontal_flip || (meta.fov_rot != CIOSFusionDICOMInfo::kZERO)))
    {
      if (meta.fov_rot == CIOSFusionDICOMInfo::kNINETY)
      {
        std::swap(p(0), p(1));
      }
      else if (meta.fov_rot == CIOSFusionDICOMInfo::kONE_EIGHTY)
      {
        p(0) = meta.cols - 1 - p(0);
        p(1) = meta.rows - 1 - p(1);
      }
      else if (meta.fov_rot == CIOSFusionDICOMInfo::kTWO_SEVENTY)
      {
        std::swap(p(0), p(1));
        p(1) = meta.rows - 1 - p(1);
      }

      // not clear if rows or columns should be flipped here
      // hopefully, we do not encounter this case
      if (meta.fov_horizontal_flip)
      {
        p(0) = meta.cols - 1 - p(0);
      }
    }
  }
}

}  // un-named

void xreg::ModifyImageWithCIOSFusionRotFlipFlags(const CIOSFusionDICOMInfo& meta,
                                                 itk::Image<unsigned short,2>* img)
{
  ModifyImageWithCIOSFusionRotFlipFlagsHelper(meta, img);
}

void xreg::ModifyImageWithCIOSFusionRotFlipFlags(const CIOSFusionDICOMInfo& meta,
                                                 itk::Image<float,2>* img)
{
  ModifyImageWithCIOSFusionRotFlipFlagsHelper(meta, img);
}

std::tuple<itk::Image<unsigned short,2>::Pointer,xreg::CIOSFusionDICOMInfo>
xreg::ReadCIOSFusionDICOMUShort(const std::string& path, const bool no_rot_or_flip_based_on_meta)
{
  return ReadCIOSFusionDICOMHelper<unsigned short>(path, no_rot_or_flip_based_on_meta);
}

std::tuple<itk::Image<float,2>::Pointer,xreg::CIOSFusionDICOMInfo>
xreg::ReadCIOSFusionDICOMFloat(const std::string& path, const bool no_rot_or_flip_based_on_meta)
{
  return ReadCIOSFusionDICOMHelper<float>(path, no_rot_or_flip_based_on_meta);
}

void xreg::UpdateLandmarkMapForCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                          LandMap2* pts,
                                          const bool no_rot_or_flip)
{
  UpdateLandmarkMapForCIOSFusionHelper(meta, pts->begin(), pts->end(), no_rot_or_flip);
}

void xreg::UpdateLandmarkMapForCIOSFusion(const CIOSFusionDICOMInfo& meta,
                                          LandMap3* pts,
                                          const bool no_rot_or_flip)
{
  UpdateLandmarkMapForCIOSFusionHelper(meta, pts->begin(), pts->end(), no_rot_or_flip);
}


void xreg::WriteCIOSMetaH5(const CIOSFusionDICOMInfo& meta, H5::Group* h5)
{
  WriteSingleScalarH5("study-time",       meta.study_time,       h5);
  WriteSingleScalarH5("series-time",      meta.series_time,      h5);
  WriteSingleScalarH5("acquisition-time", meta.acquisition_time, h5);

  WriteStringH5("pat-name", meta.pat_name, h5, false);
  WriteStringH5("pat-id",   meta.pat_id,   h5, false);

  if (meta.kvp)
  {
    WriteSingleScalarH5("kvp", *meta.kvp, h5);
  }

  WriteSingleScalarH5("dist-src-to-det", meta.dist_src_to_det, h5);

  if (meta.tube_current)
  {
    WriteSingleScalarH5("tube-current", *meta.tube_current, h5);
  }

  if (meta.exposure)
  {
    WriteSingleScalarH5("exposure", *meta.exposure, h5);
  }

  if (meta.exposure_mu_As)
  {
    WriteSingleScalarH5("exposure-mu-As", *meta.exposure_mu_As, h5);
  }

  WriteSingleScalarH5("pixel-row-spacing", meta.pixel_row_spacing, h5);
  WriteSingleScalarH5("pixel-col-spacing", meta.pixel_col_spacing, h5);

  WriteSingleScalarH5("fov-origin-row-off", meta.fov_origin_row_off, h5);
  WriteSingleScalarH5("fov-origin-col-off", meta.fov_origin_col_off, h5);

  WriteSingleScalarH5("fov-rot", static_cast<int>(meta.fov_rot), h5);
  
  WriteSingleScalarH5("fov-horizontal-flip", meta.fov_horizontal_flip, h5);

  WriteSingleScalarH5("grid-focal-dist", meta.grid_focal_dist, h5);
  
  if (meta.tube_current_muA)
  {
    WriteSingleScalarH5("tube-current-muA", *meta.tube_current_muA, h5);
  }

  WriteSingleScalarH5("rows", meta.rows, h5);
  WriteSingleScalarH5("cols", meta.cols, h5);

  WriteSingleScalarH5("window-center", meta.window_center, h5);
  WriteSingleScalarH5("window-width",  meta.window_width,  h5);
}

xreg::CIOSFusionDICOMInfo xreg::ReadCIOSMetaH5(const H5::Group& h5)
{
  auto read_optional_double = [&h5] (const std::string& n)
  {
    boost::optional<double> to_ret;

    if (ObjectInGroupH5(n, h5))
    {
      to_ret = ReadSingleScalarH5Double(n, h5);
    }

    return to_ret;
  };
  
  CIOSFusionDICOMInfo meta;

  meta.study_time       = ReadSingleScalarH5Double("study-time",       h5);
  meta.series_time      = ReadSingleScalarH5Double("series-time",      h5);
  meta.acquisition_time = ReadSingleScalarH5Double("acquisition-time", h5);

  meta.pat_name = ReadStringH5("pat-name", h5);
  meta.pat_id   = ReadStringH5("pat-id",   h5);

  meta.kvp = read_optional_double("kvp");
  
  meta.dist_src_to_det = ReadSingleScalarH5Double("dist-src-to-det", h5);

  meta.tube_current   = read_optional_double("tube-current");
  meta.exposure       = read_optional_double("exposure");
  meta.exposure_mu_As = read_optional_double("exposure-mu-As");

  meta.pixel_row_spacing = ReadSingleScalarH5Double("pixel-row-spacing", h5);
  meta.pixel_col_spacing = ReadSingleScalarH5Double("pixel-col-spacing", h5);

  meta.fov_origin_row_off = ReadSingleScalarH5ULong("fov-origin-row-off", h5);
  meta.fov_origin_col_off = ReadSingleScalarH5ULong("fov-origin-col-off", h5);

  meta.fov_rot = static_cast<CIOSFusionDICOMInfo::FOVRot>(ReadSingleScalarH5Int("fov-rot", h5));
  
  meta.fov_horizontal_flip = ReadSingleScalarH5Bool("fov-horizontal-flip", h5);

  meta.grid_focal_dist = ReadSingleScalarH5Double("grid-focal-dist", h5);

  meta.tube_current_muA = read_optional_double("tube-current-muA");

  meta.rows = ReadSingleScalarH5ULong("rows", h5);
  meta.cols = ReadSingleScalarH5ULong("cols", h5);

  meta.window_center = ReadSingleScalarH5Double("window-center", h5);
  meta.window_width  = ReadSingleScalarH5Double("window-width",  h5);

  return meta;
}

