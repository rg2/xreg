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

#ifndef XREGRADRAWPROJ_H_
#define XREGRADRAWPROJ_H_

#include "xregProjData.h"

namespace xreg
{

// Store metadata found in .rad/.raw projection files from the Ljubljana 2D/3D datasets.
// Data is available here: http://lit.fe.uni-lj.si/tools.php?lang=eng
struct RadRawProjInfo
{
  size_type num_cols;
  size_type num_rows;

  float col_spacing_mm_per_pixel;
  float row_spacing_mm_per_pixel;

  std::string data_type;

  FrameTransform proj_to_world;

  Pt3 src_wrt_world;
};

RadRawProjInfo ReadRadRawProjInfo(const std::string& file_path);

std::tuple<RadRawProjInfo,itk::Image<float,2>::Pointer> ReadRadRawProj(const std::string& rad_file_path);

ProjDataF32 ReadRawProjAsProjData(const std::string& rad_file_path);

}  // xreg

#endif

