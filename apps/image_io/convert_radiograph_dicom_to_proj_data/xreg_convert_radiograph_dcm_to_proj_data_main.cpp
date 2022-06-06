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

#include "xregProgOptUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregReadProjDataFromDICOM.h"
#include "xregStringUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
    
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Convert a DICOM radiograph file into a xReg HDF5 proj. data file. "
              "Support for video fluoroscopy is provided using the number of frames DICOM tag.");
  po.set_arg_usage("<Input DICOM File> <Output Proj. Data File> [<landmarks file>]");
  po.set_min_num_pos_args(2);

  po.add("src-to-det", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "src-to-det",
         "Source to detector (mm) value to use ONLY when the corresponding DICOM field is not populated.")
    << 1000.0;
  
  po.add("spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "spacing",
         "Image row and column spacing (mm / pixel) value to use ONLY when a suitable value may "
         "not be obtained from the DICOM metadata.")
    << 1.0;

  po.add("no-guess-spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-guess-spacing",
         "Do NOT guess pixel spacings based on other metadata values, such FOV shape and size. "
         "A guess will override any value set by \"spacing\" unless the metadata needed to make "
         "a guess is not available.")
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
  
  po.add("no-proc", 'n', ProgOpts::kSTORE_TRUE, "no-proc",
         "Do not perform any pre-processing to the image pixels - e.g. do NOT flip "
         "or rotate the image using the DICOM FOV Rotation or FOV Horizontal Flip fields.")
    << false;
  
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

  po.add("lands-spacing", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "lands-spacing",
         "Default (isotopic) pixel spacing to assume when parsing landmark files when no 2D pixel "
         "spacing is provided by the 2D image metadata.")
    << 1.0;

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

  ReadProjDataFromDICOMParams read_dcm_params;
  
  read_dcm_params.vout = &vout;

  read_dcm_params.src_to_det_default = po.get("src-to-det");
  read_dcm_params.spacing_default = po.get("spacing");

  read_dcm_params.guess_spacing = !po.get("no-guess-spacing");

  read_dcm_params.fcsv_spacing_default = po.get("lands-spacing");

  const std::string proj_frame_str = ToLowerCase(po.get("proj-frame").as_string());
  
  if (proj_frame_str == "auto")
  {
    vout << "automatically selecting proj. frame Z direction using modality..." << std::endl;
  }
  else if (proj_frame_str == "det-neg-z")
  {
    vout << "forcing proj. frame Z direction to det-neg-z" << std::endl;
    read_dcm_params.proj_frame = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;
  }
  else if (proj_frame_str == "det-pos-z")
  {
    vout << "forcing proj. frame Z direction to det-pos-z" << std::endl;
    read_dcm_params.proj_frame = CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;
  }
  else
  {
    std::cerr << "ERROR: Unsupported \"proj-frame\" value: " << proj_frame_str << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  read_dcm_params.no_proc = po.get("no-proc");
  
  read_dcm_params.no_bayview_check = po.get("no-bayview-check");

  const std::string pixel_type_str = po.get("pixel-type");

  const std::string& src_dcm_path = po.pos_args()[0];
  const std::string& dst_pd_path  = po.pos_args()[1];

  const std::string fcsv_path = (po.pos_args().size() > 2) ? po.pos_args()[2] : std::string();

  if (pixel_type_str == "float")
  {
    vout << "populating float32 proj. data from DICOM..." << std::endl;
    const auto pd = ReadProjDataFromDICOMF32(src_dcm_path, fcsv_path, read_dcm_params);
    
    vout << "  saving to proj data HDF5..." << std::endl;
    WriteProjDataH5ToDisk(pd, dst_pd_path);
  }
  else if (pixel_type_str == "uint16")
  {
    vout << "populating uint16 proj. data from DICOM..." << std::endl;
    const auto pd = ReadProjDataFromDICOMU16(src_dcm_path, fcsv_path, read_dcm_params);
    
    vout << "  saving to proj data HDF5..." << std::endl;
    WriteProjDataH5ToDisk(pd, dst_pd_path);
  }
  else
  {
    std::cerr << "ERROR: unsupported output pixel type: " << pixel_type_str << std::endl;

    return kEXIT_VAL_BAD_USE;
  }

  return kEXIT_VAL_SUCCESS;
}

