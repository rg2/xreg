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

#ifndef XREGREADPROJDATAFROMDICOM_H_
#define XREGREADPROJDATAFROMDICOM_H_

#include <boost/optional.hpp>

#include "xregProjData.h"

namespace xreg
{

struct ReadProjDataFromDICOMParams
{
  double src_to_det_default = 1000.0;
  
  double spacing_default = 1.0;
  
  // attempt to guess the image spacing when the DICOM metadata does not provide an explicit value.
  bool guess_spacing = true;

  // will auto-set the projective frame based on modality when no value is provided
  boost::optional<CameraModel::CameraCoordFrame> proj_frame;

  bool no_bayview_check = false;

  // Do not perform any pre-processing to the image pixels - e.g. do NOT flip or rotate the image
  // using the DICOM FOV Rotation or FOV Horizontal Flip fields.
  bool no_proc = false;

  // This is used for converting landmarks in physical FCSV coordinates to pixel locations.
  // When no metadata for row/column spacing is explicitly specified in the DICOM, 3D Slicer
  // will typically use a default spacing of 1.0. This field exist in the event that another
  // default FCSV spacing needs to be specified.
  double fcsv_spacing_default = 1.0;

  // Output stream to print verbose information helpful in debugging, etc.
  // A null (e.g. like /dev/null) output stream will be used when nullptr is provided.
  std::ostream* vout = nullptr;

  // Output stream to print warnings and error messages.
  // std::cerr will be used when given a nullptr.
  std::ostream* err_out = nullptr;
};

ProjDataF32List ReadProjDataFromDICOMF32(const std::string& dcm_path,
                                         const ReadProjDataFromDICOMParams& params =
                                           ReadProjDataFromDICOMParams());

ProjDataF32List ReadProjDataFromDICOMF32(const std::string& dcm_path,
                                         const std::string& fcsv_path,
                                         const ReadProjDataFromDICOMParams& params =
                                           ReadProjDataFromDICOMParams());

ProjDataU16List ReadProjDataFromDICOMU16(const std::string& dcm_path,
                                         const ReadProjDataFromDICOMParams& params =
                                           ReadProjDataFromDICOMParams());

ProjDataU16List ReadProjDataFromDICOMU16(const std::string& dcm_path,
                                         const std::string& fcsv_path,
                                         const ReadProjDataFromDICOMParams& params =
                                           ReadProjDataFromDICOMParams());

}  // xreg

#endif

