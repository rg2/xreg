/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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

#include <fmt/format.h>

#include "xregProgOptUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregDICOMUtils.h"
#include "xregITKIOUtils.h"
#include "xregFCSVUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregStringUtils.h"
#include "xregCIOSFusionDICOM.h"

using namespace xreg;
  
constexpr int kEXIT_VAL_SUCCESS  = 0;
constexpr int kEXIT_VAL_BAD_USE  = 1;
constexpr int kEXIT_VAL_BAD_DATA = 2;

template <class tPixelScalar>
int ReadPixelsAndWriteToH5(const CameraModel& cam,
                           const size_type num_frames,
                           const std::string& src_dcm_path,
                           const std::string& dst_pd_path,
                           std::ostream& vout,
                           const LandMap2& lands)
{
  std::vector<ProjData<tPixelScalar>> pd(num_frames);
  
  if (num_frames == 1)
  {
    vout << "1 frame - reading 2D image pixel data from DICOM..." << std::endl;
    pd[0].img = ReadDICOM2DFromDisk<tPixelScalar>(src_dcm_path);
  }
  else
  {
    vout << num_frames << " frames - reading 3D image pixel data from DICOM..." << std::endl;
    auto frames = ReadDICOM3DFromDisk<tPixelScalar>(src_dcm_path);
  
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
      std::cerr << "WARNING: Image column spacing (" << img_spacing[0]
                <<") differs from camera model column spacings ("
                << cam.det_col_spacing
                << "). Image values will be updated to match camera model."
                << std::endl;
    }

    if (std::abs(img_spacing[1] - cam.det_row_spacing) > 1.0e-3)
    {
      std::cerr << "WARNING: Image row spacing (" << img_spacing[1]
                <<") differs from camera model row spacings ("
                << cam.det_row_spacing 
                << "). Image values will be updated to match camera model."
                << std::endl;
    }
  }

  // Always prefer the spacing obtained by interpreting DICOM fields
  const std::array<double,2> spacing_to_use = { cam.det_col_spacing, cam.det_row_spacing };

  for (size_type i = 0; i < num_frames; ++i)
  {
    pd[i].cam       = cam;
    pd[i].landmarks = lands;
    
    // Always prefer the spacing obtained by interpreting DICOM fields
    pd[i].img->SetSpacing(spacing_to_use.data());
  }

  vout << "saving to proj data HDF5..." << std::endl;
  WriteProjDataH5ToDisk(pd, dst_pd_path);

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}

int main(int argc, char* argv[])
{
  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("TODO");
  po.set_arg_usage("<Input DICOM File> <Output Proj. Data File> [<landmarks FCSV file>]");
  po.set_min_num_pos_args(2);

  po.add("src-to-det", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "src-to-det",
         "Source to detector (mm) value to use ONLY when the corresponding DICOM field is not populated.")
    << 1000.0;
  
  po.add("spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "spacing",
         "Image row and column spacing (mm / pixel) value to use ONLY when a suitable value may "
         "not be obtained from the DICOM metadata.")
    << 1.0;

  po.add("guess-spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "guess-spacing",
         "Guess pixel spacings based on other metadata values, such FOV shape and size. "
         "This overrides any value set by \"spacing\" unless the metadata needed to make a guess is "
         "not available.")
    << false;

  po.add("proj-frame", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "proj-frame",
         "Orientation of the projective coordinate frame to be used unless overriden by another "
         "superceding flag. "
         "Origin is always at the X-ray source. "
         "The X axis runs parallel with the direction along an image row and is oriented in the "
         "increasing column direction. "
         "The Y axis runs parallel with the direction along an image column and is oriented "
         "in the increasing row direction. "
         "The Z axis is orthogonal to the detector plane and this flag determines the direction "
         "(either towards the X-ray source or away). "
         "Values: "
         "\"det-neg-z\" --> Moving from the X-ray source to the detector is a movement in the "
         "negative Z direction. "
         "\"det-pos-z\" --> Moving from the X-ray source to the detector is a movement in the "
         "positive Z direction. "
         "\"auto\" --> Automatically select the orientation based on the modality field. "
         "\"XA\" and \"RF\" yield \"det-neg-z\" while \"CR\" and \"DX\" yield \"det-pos-z\"")
    << "auto";
  
  po.add("no-bayview-check", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-bayview-check",
         "Do NOT inspect metadata fields to determine that the input file was created using the "
         "Siemens CIOS Fusion C-arm in the JHMI Bayview lab. When this check IS performed and a "
         "file is determined to have been created using the Bayview C-arm, then the extrinsic "
         "transformation of the projection is populated from a previously computed transformation "
         "which has an X-axis and center of rotation corresponding to the oribital rotation of "
         "the C-arm.")
    << false;

  po.add("pixel-type", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "pixel-type",
         "Pixel type used when saving the output projection data. Valid values are: "
         "\"float\" for 32-bit floats and \"uint16\" for unsigned 16-bit integers.")
    << "float";

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_BAD_USE;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  std::ostream& vout = po.vout();

  const double src_to_det_default = po.get("src-to-det");
  const double spacing_default = po.get("spacing");

  const bool guess_spacing = po.get("guess-spacing");

  const std::string proj_frame_str = ToLowerCase(po.get("proj-frame").as_string());

  const bool no_bayview_check = po.get("no-bayview-check");

  const std::string pixel_type_str = po.get("pixel-type");

  const std::string& src_dcm_path = po.pos_args()[0];
  const std::string& dst_pd_path  = po.pos_args()[1];

  const std::string fcsv_path = (po.pos_args().size() > 2) ? po.pos_args()[2] : std::string();

  vout << "reading DICOM metadata..." << std::endl; 
  const auto dcm_info = ReadDICOMFileBasicFields(src_dcm_path);

  vout << "  input modality: " << dcm_info.modality << std::endl;

  const bool modality_is_xa = dcm_info.modality == "XA";
  const bool modality_is_dx = dcm_info.modality == "DX";
  const bool modality_is_cr = dcm_info.modality == "CR";
  const bool modality_is_rf = dcm_info.modality == "RF";

  if (!(modality_is_xa || modality_is_dx || modality_is_cr || modality_is_rf))
  {
    std::cerr << "WARNING: unexpected modality: " << dcm_info.modality << std::endl;
  }

  bool proj_frame_use_det_neg_z = false;

  if (proj_frame_str == "auto")
  {
    vout << "automatically selecting proj. frame Z direction using modality..." << std::endl;
    proj_frame_use_det_neg_z = modality_is_xa || modality_is_rf;
  }
  else if (proj_frame_str == "det-neg-z")
  {
    vout << "forcing proj. frame Z direction to det-neg-z" << std::endl;
    proj_frame_use_det_neg_z = true;
  }
  else if (proj_frame_str == "det-pos-z")
  {
    vout << "forcing proj. frame Z direction to det-pos-z" << std::endl;
    proj_frame_use_det_neg_z = false;
  }
  else
  {
    std::cerr << "ERROR: Unsupported \"proj-frame\" value: " << proj_frame_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  vout << "  det-neg-z: " << BoolToYesNo(proj_frame_use_det_neg_z) << std::endl;

  vout << "setting up camera model..." << std::endl;

  float src_to_det_to_use = static_cast<float>(src_to_det_default);
  
  float row_spacing_to_use = static_cast<float>(spacing_default);
  float col_spacing_to_use = row_spacing_to_use;

  if (dcm_info.dist_src_to_det_mm)
  {
    src_to_det_to_use = *dcm_info.dist_src_to_det_mm;
  }
  else
  {
    std::cerr << "WARNING: source to detector field not present in DICOM, will use default value of "
              << src_to_det_default << std::endl;
  }
  
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

    bool guess_made = false;

    if (guess_spacing)
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
                std::cerr << "WARNING: number of image rows and columns are not equal!"
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
              std::cerr << "expected ROUND FOV dims to have length 1, got: " << num_fov_dims << std::endl;
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
              std::cerr << "expected RECTANGLE FOV dims to have length 2, got: "
                        << num_fov_dims << std::endl;
            }
          }
          else
          {
            std::cerr << "unsupported/unknown FOV shape string: " << fov_shape_str << std::endl;
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
            std::cerr << "WARNING: number of image rows and columns are not equal!"
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
      vout << "spacing not found in metadata, using default spacing: " << spacing_default << std::endl;
    }
  }

  CameraModel cam;

  cam.coord_frame_type = proj_frame_use_det_neg_z ? CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z :
                                                    CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;

  const bool is_bayview_cios_dcm = (dcm_info.manufacturer == "SIEMENS") &&
                                   (dcm_info.institution_name &&
                                      (*dcm_info.institution_name == "Johns Hopkins Univ")) &&
                                   (dcm_info.department_name &&
                                      (*dcm_info.department_name == "Applied Physics Lab")) &&
                                   (dcm_info.manufacturers_model_name &&
                                      (*dcm_info.manufacturers_model_name == "Fluorospot Compact S1")) &&
                                   dcm_info.dist_src_to_det_mm;

  if (no_bayview_check || !is_bayview_cios_dcm)
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
      std::cerr << "WARNING: C-arm projective frame type does not match the expected value!\n "
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

  LandMap2 lands;

  if (!fcsv_path.empty())
  {
    vout << "reading landmarks from FCSV and converting to pixels..." << std::endl;
    auto lands_3d = ReadFCSVFileNamePtMap(fcsv_path);
    
    ConvertRASToLPS(&lands_3d);
    
    lands = PhysPtsToInds(DropPtDim(lands_3d, 2), col_spacing_to_use, row_spacing_to_use);
  }

  const size_type num_frames = dcm_info.num_frames ? * dcm_info.num_frames : 1;

  int ret_val = kEXIT_VAL_SUCCESS;

  if (pixel_type_str == "float")
  {
    ret_val = ReadPixelsAndWriteToH5<float>(cam, num_frames, src_dcm_path, dst_pd_path, vout, lands);
  }
  else if (pixel_type_str == "uint16")
  {
    ret_val = ReadPixelsAndWriteToH5<unsigned short>(cam, num_frames, src_dcm_path, dst_pd_path, vout, lands);
  }
  else
  {
    std::cerr << "ERROR: unsupported output pixel type: " << pixel_type_str << std::endl;

    ret_val = kEXIT_VAL_BAD_USE;
  }

  return ret_val;
}

