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

#include "xregCIOSFusionDICOM.h"

#include "xregAssert.h"

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

xreg::DICOMFIleBasicFields xreg::MakeNaiveCIOSFusionMetaDR()
{
  DICOMFIleBasicFields meta;
  
  meta.dist_src_to_det_mm = 1020;

  meta.imager_pixel_spacing = std::array<CoordScalar,2>{0.194, 0.194};
  
  meta.row_spacing = 0.194;
  meta.col_spacing = 0.194;

  meta.num_rows = 1536;
  meta.num_cols = 1536;

  return meta;
}

xreg::CameraModel
xreg::NaiveCamModelFromCIOSFusion(const DICOMFIleBasicFields& meta,
                                  const bool iso_center_at_origin)
{
  xregASSERT(bool(meta.dist_src_to_det_mm));
  xregASSERT(bool(meta.imager_pixel_spacing));
  
  const auto& ps = *meta.imager_pixel_spacing;

  CameraModel cam;
  cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  const Mat3x3 intrins = MakeNaiveIntrins(*meta.dist_src_to_det_mm,
                                          meta.num_rows, meta.num_cols,
                                          ps[1], ps[0], true); 

  Mat4x4 extrins = Mat4x4::Identity();

  if (iso_center_at_origin)
  {
    extrins = CIOSFusionCBCTExtrins();
  }

  cam.setup(intrins, extrins, meta.num_rows, meta.num_cols, ps[1], ps[0]);

  return cam;
}

xreg::CameraModel
xreg::NaiveCamModelFromCIOSFusionExtrins(const DICOMFIleBasicFields& meta,
                                         const Mat4x4 extrins)
{
  xregASSERT(bool(meta.dist_src_to_det_mm));
  xregASSERT(bool(meta.imager_pixel_spacing));
  
  const auto& ps = *meta.imager_pixel_spacing;
  
  CameraModel cam;
  cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  const Mat3x3 intrins = MakeNaiveIntrins(*meta.dist_src_to_det_mm,
                                          meta.num_rows, meta.num_cols,
                                          ps[1], ps[0], true); 

  cam.setup(intrins, extrins, meta.num_rows, meta.num_cols, ps[1], ps[0]);

  return cam;
}

