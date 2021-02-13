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

#include "xregProgOptUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregDICOMUtils.h"
#include "xregITKIOUtils.h"
#include "xregFCSVUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"

using namespace xreg;
  
constexpr int kEXIT_VAL_SUCCESS  = 0;
constexpr int kEXIT_VAL_BAD_USE  = 1;
constexpr int kEXIT_VAL_BAD_DATA = 2;

template <class tPixelScalar>
int ReadPixelsAndWriteToH5(const CameraModel& cam,
                           const std::string& src_dcm_path,
                           const std::string& dst_pd_path,
                           std::ostream& vout,
                           const LandMap2& lands)
{
  ProjData<tPixelScalar> pd;
  
  pd.cam = cam;
  
  pd.landmarks = lands;

  vout << "reading image pixel data from DICOM..." << std::endl;
  pd.img = ReadDICOM2DFromDisk<tPixelScalar>(src_dcm_path);

  {
    auto img_spacing = pd.img->GetSpacing();

    if (std::abs(img_spacing[0] - pd.cam.det_col_spacing) > 1.0e-3)
    {
      std::cerr << "WARNING: Image column spacing (" << img_spacing[0]
                <<") differs from camera model column spacings ("
                << pd.cam.det_col_spacing << ")" << std::endl;
    }

    if (std::abs(img_spacing[1] - pd.cam.det_row_spacing) > 1.0e-3)
    {
      std::cerr << "WARNING: Image row spacing (" << img_spacing[1]
                <<") differs from camera model row spacings ("
                << pd.cam.det_row_spacing << ")" << std::endl;
    }
    
    // Always prefer the spacing obtained by interpreting DICOM fields
    img_spacing[0] = pd.cam.det_col_spacing;
    img_spacing[1] = pd.cam.det_row_spacing;
    
    pd.img->SetSpacing(img_spacing);
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

  po.set_help("");
  po.set_arg_usage("<Input DICOM File> <Output Proj. Data File> [<landmarks FCSV file>]");
  po.set_min_num_pos_args(2);

  po.add("src-to-det", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "src-to-det",
         "Source to detector (mm) value to use ONLY when the corresponding DICOM field is not populated.")
    << 1000.0;
  
  po.add("spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "spacing",
         "Image row and column spacing (mm / pixel) value to use ONLY when a suitable value may "
         "not be obtained from the DICOM metadata.")
    << 1.0;

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
  
  const std::string pixel_type_str = po.get("pixel-type");

  const std::string& src_dcm_path = po.pos_args()[0];
  const std::string& dst_pd_path  = po.pos_args()[1];

  const std::string fcsv_path = (po.pos_args().size() > 2) ? po.pos_args()[2] : std::string();

  vout << "reading DICOM metadata..." << std::endl; 
  const auto dcm_info = ReadDICOMFileBasicFields(src_dcm_path);
  
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
  // TODO: could add some other checks based on detector tags before looking
  //       at the pixel spacing below:
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
    vout << "spacing not found in metadata, using default spacing: " << spacing_default << std::endl;
  }

  CameraModel cam;

  //cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;
  cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;
  cam.setup(src_to_det_to_use,
            dcm_info.num_rows, dcm_info.num_cols,
            row_spacing_to_use, col_spacing_to_use);

  LandMap2 lands;

  if (!fcsv_path.empty())
  {
    vout << "reading landmarks from FCSV and converting to pixels..." << std::endl;
    auto lands_3d = ReadFCSVFileNamePtMap(fcsv_path);
    
    ConvertRASToLPS(&lands_3d);
    
    lands = PhysPtsToInds(DropPtDim(lands_3d, 2), col_spacing_to_use, row_spacing_to_use);
  }

  int ret_val = kEXIT_VAL_SUCCESS;

  if (pixel_type_str == "float")
  {
    ret_val = ReadPixelsAndWriteToH5<float>(cam, src_dcm_path, dst_pd_path, vout, lands);
  }
  else if (pixel_type_str == "uint16")
  {
    ret_val = ReadPixelsAndWriteToH5<unsigned short>(cam, src_dcm_path, dst_pd_path, vout, lands);
  }
  else
  {
    std::cerr << "ERROR: unsupported output pixel type: " << pixel_type_str << std::endl;

    ret_val = kEXIT_VAL_BAD_USE;
  }

  return ret_val;
}

