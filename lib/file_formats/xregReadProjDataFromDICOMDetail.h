/*
 * MIT License
 *
 * Copyright (c) 2021-2022 Robert Grupp
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

#ifndef XREGREADPROJDATAFROMDICOMDETAIL_H_
#define XREGREADPROJDATAFROMDICOMDETAIL_H_

#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>

#include <fmt/format.h>

#include "xregAnatCoordFrames.h"
#include "xregCIOSFusionDICOM.h"
#include "xregDICOMUtils.h"
#include "xregLandmarkFiles.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregOpenCVUtils.h"
#include "xregReadProjDataFromDICOM.h"

namespace xreg
{
namespace detail
{

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
ReadProjDataFromDICOMHelper(const std::string& dcm_path, const ReadProjDataFromDICOMParams& params)
{
  boost::iostreams::stream<boost::iostreams::null_sink> null_ostream((boost::iostreams::null_sink()));

  std::ostream& vout    = params.vout    ? *params.vout    : null_ostream;
  std::ostream& err_out = params.err_out ? *params.err_out : std::cerr;
  
  vout << "reading DICOM metadata..." << std::endl; 
  const auto dcm_info = ReadDICOMFileBasicFields(dcm_path);

  vout << "  input modality: " << dcm_info.modality << std::endl;

  const bool modality_is_xa = dcm_info.modality == "XA";
  const bool modality_is_dx = dcm_info.modality == "DX";
  const bool modality_is_cr = dcm_info.modality == "CR";
  const bool modality_is_rf = dcm_info.modality == "RF";

  if (!(modality_is_xa || modality_is_dx || modality_is_cr || modality_is_rf))
  {
    err_out << "WARNING: unexpected modality: " << dcm_info.modality << std::endl;
  }

  CameraModel::CameraCoordFrame proj_frame;

  if (!params.proj_frame)
  {
    vout << "automatically selecting proj. frame Z direction using modality..." << std::endl;
    
    proj_frame = (modality_is_xa || modality_is_rf) ?
                    CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z :
                    CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;
  }
  else
  {
    vout << "using specified value for camera coord frame: "
         << static_cast<int>(*params.proj_frame) << std::endl;
    
    proj_frame = *params.proj_frame;
  }

  vout << "setting up camera model..." << std::endl;

  float src_to_det_to_use = static_cast<float>(params.src_to_det_default);
  
  float row_spacing_to_use = static_cast<float>(params.spacing_default);
  float col_spacing_to_use = row_spacing_to_use;

  if (dcm_info.dist_src_to_det_mm)
  {
    src_to_det_to_use = *dcm_info.dist_src_to_det_mm;
  }
  else
  {
    err_out << "WARNING: source to detector field not present in DICOM, will use default value of "
            << params.src_to_det_default << std::endl;
  }

  bool spacing_in_meta = true;

  // prefer to use the imager spacing field when available  
  if (dcm_info.imager_pixel_spacing)
  {
    vout << "using imager pixel spacing field" << std::endl;
    
    const auto& s = *dcm_info.imager_pixel_spacing;
    
    row_spacing_to_use = s[0];
    col_spacing_to_use = s[1];
  }
  else if ((dcm_info.row_spacing > 1.0e-6) && (dcm_info.col_spacing > 1.0e-6))
  {
    // next, use the image pixel spacing field - this is less preferred than the
    // imager spacing as this field is supposed to be defined with respect to a
    // patient coordinate frame, which does not make sense for a 2D radiograph 

    vout << "using image pixel spacing..." << std::endl;

    row_spacing_to_use = dcm_info.row_spacing;
    col_spacing_to_use = dcm_info.col_spacing; 
  }
  else
  {
    // No other tags explicitly specify the spacing, try and guess using some other metadata fields.
    
    spacing_in_meta = false;

    bool guess_made = false;

    if (params.guess_spacing)
    {
      vout << "pixel spacing metadata not available - attempting to guess..." << std::endl;
      
      if (dcm_info.fov_shape)
      {
        const auto& fov_shape_str = *dcm_info.fov_shape;

        vout << "  FOV shape available: " << fov_shape_str << std::endl;

        if (dcm_info.fov_dims)
        {
          const auto& fov_dims = *dcm_info.fov_dims;

          const size_type num_fov_dims = fov_dims.size();

          unsigned long num_rows_for_guess = dcm_info.num_rows;
          unsigned long num_cols_for_guess = dcm_info.num_cols;

          if (dcm_info.fov_origin_off)
          {
            const auto& fov_origin_off = *dcm_info.fov_origin_off;

            vout << "FOV origin offset available: [ " << fov_origin_off[0] << " , "
                 << fov_origin_off[1] << " ]" << std::endl;

            num_rows_for_guess -= 2 * fov_origin_off[0];
            num_cols_for_guess -= 2 * fov_origin_off[1];

            vout << "  number of rows used for spacing guess: " << num_rows_for_guess << std::endl;
            vout << "  number of cols used for spacing guess: " << num_cols_for_guess << std::endl;
          }

          if (fov_shape_str == "ROUND")
          {
            if (num_fov_dims == 1)
            {
              if (dcm_info.num_cols != dcm_info.num_rows)
              {
                err_out << "WARNING: number of image rows and columns are not equal!"
                           "Guessed pixels spacings may have substantial errors!" << std::endl;
              }
              
              row_spacing_to_use = static_cast<float>(fov_dims[0]) /
                                      std::max(num_cols_for_guess, num_rows_for_guess);
              col_spacing_to_use = row_spacing_to_use;

              vout << "  using round FOV diameter of " << fov_dims[0]
                   << " mm to guess isotropic spacing of "
                   << fmt::format("{:.3f}", row_spacing_to_use) << " mm/pixel" << std::endl;

              guess_made = true;
            }
            else
            {
              err_out << "expected ROUND FOV dims to have length 1, got: " << num_fov_dims << std::endl;
            }
          }
          else if (fov_shape_str == "RECTANGLE")
          {
            if (num_fov_dims == 2)
            {
              col_spacing_to_use = static_cast<float>(fov_dims[0]) / num_cols_for_guess;
              row_spacing_to_use = static_cast<float>(fov_dims[1]) / num_rows_for_guess;

              vout << "  using rect FOV row length of " << fov_dims[0]
                   << " mm to guess column spacing of "
                   << fmt::format("{:.3f}", col_spacing_to_use) << " mm/pixel" << std::endl;
              
              vout << "  using rect FOV column length of " << fov_dims[0]
                   << " mm to guess row spacing of "
                   << fmt::format("{:.3f}", row_spacing_to_use) << " mm/pixel" << std::endl;

              guess_made = true;
            }
            else
            {
              err_out << "expected RECTANGLE FOV dims to have length 2, got: "
                      << num_fov_dims << std::endl;
            }
          }
          else
          {
            err_out << "unsupported/unknown FOV shape string: " << fov_shape_str << std::endl;
          }
        }
        else
        {
          vout << "  FOV dims not available" << std::endl;
        }
      }

      if (!guess_made)
      {
        if (dcm_info.intensifier_diameter_mm)
        {
          if (dcm_info.num_cols != dcm_info.num_rows)
          {
            err_out << "WARNING: number of image rows and columns are not equal!"
                       "Guessed pixels spacings may have substantial errors!" << std::endl;
          }
          
          const float d = static_cast<float>(*dcm_info.intensifier_diameter_mm);
          
          row_spacing_to_use = d / std::max(dcm_info.num_cols, dcm_info.num_rows);
          col_spacing_to_use = row_spacing_to_use;

          vout << "using intensifier diameter of " << d
               << " mm to guess isotropic pixel spacing of " << row_spacing_to_use
               << " mm / pixel" << std::endl;

          guess_made = true;
        }
      }
    }
   
    if (!guess_made)
    { 
      vout << "spacing not found in metadata, using default spacing: "
           << params.spacing_default << std::endl;
    }
  }

  CameraModel cam;

  cam.coord_frame_type = proj_frame;

  const bool is_bayview_cios_dcm = (dcm_info.manufacturer == "SIEMENS") &&
                                   (dcm_info.institution_name &&
                                      (*dcm_info.institution_name == "Johns Hopkins Univ")) &&
                                   (dcm_info.department_name &&
                                      (*dcm_info.department_name == "Applied Physics Lab")) &&
                                   (dcm_info.manufacturers_model_name &&
                                      (*dcm_info.manufacturers_model_name == "Fluorospot Compact S1")) &&
                                   dcm_info.dist_src_to_det_mm;

  if (params.no_bayview_check || !is_bayview_cios_dcm)
  {
    vout << "setting camera model with naive intrinsics and identity extrinsics..." << std::endl;

    cam.setup(src_to_det_to_use,
              dcm_info.num_rows, dcm_info.num_cols,
              row_spacing_to_use, col_spacing_to_use);
  }
  else
  {
    vout << "bayview file detected, setting up camera model with calculated extrinsics..." << std::endl;

    if (cam.coord_frame_type != CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
    {
      err_out << "WARNING: C-arm projective frame type does not match the expected value!\n "
                 "  Expected: " << static_cast<int>(CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
              << ", have: " << static_cast<int>(cam.coord_frame_type)
              << std::endl;
    }
    
    const Mat3x3 intrins = MakeNaiveIntrins(*dcm_info.dist_src_to_det_mm,
                                            dcm_info.num_rows, dcm_info.num_cols,
                                            row_spacing_to_use,
                                            col_spacing_to_use,
                                            true);

    cam.setup(intrins, CIOSFusionCBCTExtrins(),
              dcm_info.num_rows, dcm_info.num_cols,
              row_spacing_to_use, col_spacing_to_use);
  }

  const size_type num_frames = dcm_info.num_frames ? * dcm_info.num_frames : 1;
  
  std::vector<ProjData<tPixelScalar>> pd(num_frames);
  
  if (num_frames == 1)
  {
    vout << "1 frame - reading 2D image pixel data from DICOM..." << std::endl;
    pd[0].img = ReadDICOM2DFromDisk<tPixelScalar>(dcm_path);
  }
  else
  {
    vout << num_frames << " frames - reading 3D image pixel data from DICOM..." << std::endl;
    auto frames = ReadDICOM3DFromDisk<tPixelScalar>(dcm_path);
  
    const auto spacing_vol = frames->GetSpacing();
  
    const std::array<double,2> spacing_slice = { spacing_vol[0], spacing_vol[1] };

    const auto origin_vol = frames->GetOrigin();

    const std::array<double,2> origin_slice = { origin_vol[0], origin_vol[1] };

    vout << "  converting in-plane slices to individual projection frames..." << std::endl; 
    
    const auto* cur_frame_buf = frames->GetBufferPointer();

    const size_type num_pix_per_frame = cam.num_det_cols * cam.num_det_rows;
    
    for (size_type i = 0; i < num_frames; ++i, cur_frame_buf += num_pix_per_frame)
    {
      auto dst_frame = MakeITK2DVol<tPixelScalar>(cam.num_det_cols, cam.num_det_rows);

      dst_frame->SetSpacing(spacing_slice.data());
      dst_frame->SetOrigin(origin_slice.data());

      std::copy(cur_frame_buf, cur_frame_buf + num_pix_per_frame, dst_frame->GetBufferPointer());

      pd[i].img = dst_frame;
    }
  }

  {
    auto img_spacing = pd[0].img->GetSpacing();

    if (std::abs(img_spacing[0] - cam.det_col_spacing) > 1.0e-3)
    {
      err_out << "WARNING: Image column spacing (" << img_spacing[0]
              <<") differs from camera model column spacings ("
              << cam.det_col_spacing
              << "). Image values will be updated to match camera model."
              << std::endl;
    }

    if (std::abs(img_spacing[1] - cam.det_row_spacing) > 1.0e-3)
    {
      err_out << "WARNING: Image row spacing (" << img_spacing[1]
              <<") differs from camera model row spacings ("
              << cam.det_row_spacing 
              << "). Image values will be updated to match camera model."
              << std::endl;
    }
  }

  // Always prefer the spacing obtained by interpreting DICOM fields
  const std::array<double,2> spacing_to_use = { cam.det_col_spacing, cam.det_row_spacing };

  auto orig_dcm_meta = std::make_shared<DICOMFIleBasicFields>();
  *orig_dcm_meta = dcm_info;
        
  const auto dcm_rot = dcm_info.fov_rot ? *dcm_info.fov_rot : DICOMFIleBasicFields::kZERO;
        
  const bool do_horiz_flip = dcm_info.fov_horizontal_flip && (*dcm_info.fov_horizontal_flip);

  for (size_type i = 0; i < num_frames; ++i)
  {
    pd[i].cam = cam;
    
    // Always prefer the spacing obtained by interpreting DICOM fields
    pd[i].img->SetSpacing(spacing_to_use.data());
    
    pd[i].det_spacings_from_orig_meta = spacing_in_meta;

    pd[i].orig_dcm_meta = orig_dcm_meta;

    if (!params.no_proc)
    {
      cv::Mat img_ocv = ShallowCopyItkToOpenCV(pd[i].img.GetPointer());

      if (dcm_rot != DICOMFIleBasicFields::kZERO)
      {
        if (dcm_rot == DICOMFIleBasicFields::kNINETY)
        {
          xregASSERT(pd[i].cam.num_det_rows == pd[i].cam.num_det_cols);

          cv::Mat tmp = img_ocv.clone();
          cv::transpose(tmp, img_ocv);
          FlipImageColumns(&img_ocv);
        }
        else if (dcm_rot == DICOMFIleBasicFields::kONE_EIGHTY)
        {
          FlipImageRows(&img_ocv);
          FlipImageColumns(&img_ocv);
        }
        else if (dcm_rot == DICOMFIleBasicFields::kTWO_SEVENTY)
        {
          xregASSERT(pd[i].cam.num_det_rows == pd[i].cam.num_det_cols);
          
          cv::Mat tmp = img_ocv.clone();
          cv::transpose(tmp, img_ocv);
          FlipImageRows(&img_ocv);
        }
      }

      if (do_horiz_flip)
      {
        FlipImageColumns(&img_ocv);
      }
    }
  }

  return pd;
}

template <class tPixelScalar>
std::vector<ProjData<tPixelScalar>>
ReadProjDataFromDICOMHelper(const std::string& dcm_path, const std::string& fcsv_path,
                            const ReadProjDataFromDICOMParams& params)
{
  boost::iostreams::stream<boost::iostreams::null_sink> null_ostream((boost::iostreams::null_sink()));

  std::ostream& vout = params.vout ? *params.vout : null_ostream;

  auto pd = ReadProjDataFromDICOMHelper<tPixelScalar>(dcm_path, params);
  
  if (!fcsv_path.empty())
  {
    vout << "reading landmarks from file (to LPS coords) and converting to pixels..." << std::endl;
    const auto lands_3d = ReadLandmarksFileNamePtMap(fcsv_path, true);
        
    xregASSERT(!pd.empty());
    
    // We need to use the pixel spacing that was used by 3D Slicer to create the FCSV file
    // when converting landmarks to pixel indices. The FCSV pixel spacing will be equal to
    // the spacing used in our projection data camera model when the spacing was explicitly
    // available in the DICOM metadata. Otherwise, we need to have the FCSV spacing
    // specified. The FCSV spacing is typically 1.0 when no metadata is provided in the
    // DICOM.

    const bool det_spacings_from_orig_meta = *pd[0].det_spacings_from_orig_meta;

    const auto lands = PhysPtsToInds(DropPtDim(lands_3d, 2),
                                     det_spacings_from_orig_meta ?
                                        pd[0].cam.det_col_spacing :
                                        static_cast<CoordScalar>(params.fcsv_spacing_default),
                                     det_spacings_from_orig_meta ?
                                        pd[0].cam.det_row_spacing :
                                        static_cast<CoordScalar>(params.fcsv_spacing_default));
  
    for (auto& p : pd)
    {
      p.landmarks = lands;
      
      if (!params.no_proc && p.orig_dcm_meta)
      {
        const auto& dcm_info = *p.orig_dcm_meta;

        const auto dcm_rot = dcm_info.fov_rot ? *dcm_info.fov_rot : DICOMFIleBasicFields::kZERO;
        
        const bool do_horiz_flip = dcm_info.fov_horizontal_flip && (*dcm_info.fov_horizontal_flip);

        for (auto& lkv: p.landmarks)
        {
          auto& l = lkv.second;

          if (dcm_rot != DICOMFIleBasicFields::kZERO)
          {
            if (dcm_rot == DICOMFIleBasicFields::kNINETY)
            {
              std::swap(l(0), l(1));
            }
            else if (dcm_rot == DICOMFIleBasicFields::kONE_EIGHTY)
            {
              l(0) = p.cam.num_det_cols - 1 - l(0);
              l(1) = p.cam.num_det_rows - 1 - l(1);
            }
            else if (dcm_rot == DICOMFIleBasicFields::kTWO_SEVENTY)
            {
              std::swap(l(0), l(1));
              l(1) = p.cam.num_det_rows - 1 - l(1);
            }
          }

          if (do_horiz_flip)
          {
            l(0) = p.cam.num_det_cols - 1 - l(0);
          }
        }
      }
    }
  }
  else
  {
    vout << "empty path provided for FCSV file - no landmarks will be populated" << std::endl;
  }

  return pd;
}

}  // detail
}  // xreg

#endif

