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

#include "xregH5CamModelIO.h"

#include "xregHDF5.h"

void xreg::WriteCamModelH5(const CameraModel& cam, H5::Group* h5)
{
  SetStringAttr("xreg-type", "cam-model", h5);
  
  WriteSingleScalarH5("num-rows", cam.num_det_rows, h5);
  WriteSingleScalarH5("num-cols", cam.num_det_cols, h5);
  WriteSingleScalarH5("row-spacing", cam.det_row_spacing, h5);
  WriteSingleScalarH5("col-spacing", cam.det_col_spacing, h5);

  if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
  {
    WriteStringH5("cam-coord-frame-type", "origin-at-focal-pt-det-pos-z", h5, false);
  }
  else if (cam.coord_frame_type == CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
  {
    WriteStringH5("cam-coord-frame-type", "origin-at-focal-pt-det-neg-z", h5, false);
  }
  else
  {
    WriteStringH5("cam-coord-frame-type", "origin-on-det", h5, false);
  }
  
  WriteMatrixH5("intrinsic", cam.intrins, h5, false);
  WriteAffineTransform4x4("extrinsic", cam.extrins, h5, false);
}

void xreg::WriteCamModelH5ToDisk(const CameraModel& cam, const std::string& dst_path)
{
  H5::H5File h5(dst_path, H5F_ACC_TRUNC);
  
  WriteCamModelH5(cam, &h5);
  
  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();
}

xreg::CameraModel xreg::ReadCamModelH5(const H5::Group& h5)
{
  CameraModel cam;

  const std::string coord_str = ReadStringH5("cam-coord-frame-type", h5);
  if (coord_str == "origin-at-focal-pt-det-pos-z")
  {
    cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z;
  }
  else if (coord_str == "origin-at-focal-pt-det-neg-z")
  {
    cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;
  }
  else
  {
    cam.coord_frame_type = CameraModel::kORIGIN_ON_DETECTOR;
  }

  cam.setup(ReadMatrixH5CoordScalar("intrinsic", h5),
            ReadMatrixH5CoordScalar("extrinsic", h5),
            ReadSingleScalarH5ULong("num-rows", h5),
            ReadSingleScalarH5ULong("num-cols", h5),
            ReadSingleScalarH5CoordScalar("row-spacing", h5),
            ReadSingleScalarH5CoordScalar("col-spacing", h5)); 

  return cam;
}

xreg::CameraModel xreg::ReadCamModelH5FromDisk(const std::string& src_path)
{
  return ReadCamModelH5(H5::H5File(src_path, H5F_ACC_RDONLY));
}
