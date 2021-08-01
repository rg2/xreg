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

#include "xregRadRawProj.h"

#include <fstream>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregFilesystemUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregRigidUtils.h"
#include "xregStringUtils.h"

xreg::RadRawProjInfo xreg::ReadRadRawProjInfo(const std::string& file_path)
{
  std::ifstream in(file_path);
  
  const auto lines = GetNonEmptyLinesFromStream(in);

  const size_type num_lines = lines.size();
  
  RadRawProjInfo info;

  bool found_size      = false;
  bool found_step      = false;
  bool found_data_type = false;
  bool found_tpos      = false;
  bool found_src_pos   = false;

  const char* size_param_str      = "Size[pixels]";
  const char* step_param_str      = "Step[mm]";
  const char* data_type_param_str = "DataType";
  const char* src_pos_param_str   = "SourcePosition[mm]";
  const char* pos_param_str       = "TPosition";

  for (size_type line_idx = 0; line_idx < num_lines; ++line_idx)
  {
    const auto colon_sep_toks = StringSplit(lines[line_idx], ":");
      
    const size_type num_colon_sep_toks = colon_sep_toks.size();

    if (num_colon_sep_toks > 0)
    {
      const auto& param_name = colon_sep_toks[0];

      if (num_colon_sep_toks > 1)
      {
        const auto& param_val = colon_sep_toks[1];

        if (param_name == size_param_str)
        {
          const auto size_toks = StringCast<size_type>(StringSplit(param_val));
          
          xregASSERT(size_toks.size() == 2);

          info.num_cols = size_toks[0];
          info.num_rows = size_toks[1];

          found_size = true;
        }
        else if (param_name == step_param_str)
        {
          const auto step_toks = StringCast<float>(StringSplit(param_val));
          
          xregASSERT(step_toks.size() == 2);

          info.col_spacing_mm_per_pixel = step_toks[0];
          info.row_spacing_mm_per_pixel = step_toks[1];

          found_step = true;
        }
        else if (param_name == data_type_param_str)
        {
          info.data_type = StringStrip(param_val);

          found_data_type = true;
        }
        else if (param_name == src_pos_param_str)
        {
          const auto pos_toks = StringCast<float>(StringSplit(param_val));

          xregASSERT(pos_toks.size() == 3);

          info.src_wrt_world(0) = pos_toks[0];
          info.src_wrt_world(1) = pos_toks[1];
          info.src_wrt_world(2) = pos_toks[2];

          found_src_pos = true;
        }
      }
      else if (param_name == pos_param_str)
      {
        if ((line_idx + 4) < num_lines)
        {
          for (size_type r = 0; r < 4; ++r)
          {
            const auto mat_row_toks = StringCast<float>(StringSplit(lines[line_idx + r + 1]));
            xregASSERT(mat_row_toks.size() == 4);

            for (size_type c = 0; c < 4; ++c)
            {
              info.proj_to_world.matrix()(r,c) = mat_row_toks[c];
            }
          }

          line_idx += 4;

          found_tpos = true;
        }
      }
    }
  }

  if (!(found_size && found_step && found_data_type && found_tpos && found_src_pos))
  {
    std::vector<std::string> params_not_found;

    if (!found_size)
    {
      params_not_found.push_back(size_param_str);
    }

    if (!found_step)
    {
      params_not_found.push_back(step_param_str);
    }

    if (!found_data_type)
    {
      params_not_found.push_back(data_type_param_str);
    }

    if (!found_tpos)
    {
      params_not_found.push_back(pos_param_str);
    }

    if (!found_src_pos)
    {
      params_not_found.push_back(src_pos_param_str);
    }

    xregThrow("Failed to find required .rad metadata fields: %s",
              JoinTokens(params_not_found, " , ").c_str());
  }

  return info;
}

std::tuple<xreg::RadRawProjInfo,itk::Image<float,2>::Pointer>
xreg::ReadRadRawProj(const std::string& rad_file_path)
{
  const auto info = ReadRadRawProjInfo(rad_file_path);
 
  xregASSERT(info.data_type == "float");

  auto img = MakeITK2DVol<float>(info.num_cols, info.num_rows);

  {
    const std::array<float,2> tmp_spacing = { info.col_spacing_mm_per_pixel,
                                              info.row_spacing_mm_per_pixel };

    img->SetSpacing(tmp_spacing.data());
  }

  FileInputStream fin(fmt::format("{}.raw", std::get<0>(Path(rad_file_path).split_ext())));

  xregASSERT(fin.num_bytes_left() == (sizeof(float) * info.num_cols * info.num_rows));

  fin.read_remaining_bytes(img->GetBufferPointer());

  return std::make_tuple(info, img);
}

xreg::ProjDataF32 xreg::ReadRawProjAsProjData(const std::string& rad_file_path)
{
  ProjDataF32 pd;

  auto rad_data = ReadRadRawProj(rad_file_path);

  const auto& info = std::get<0>(rad_data);
  
  pd.img = std::get<1>(rad_data);

  // The .rad projective frame has an origin at the (0,0) pixel in the 2D image
  // with x axis aligned with increasing columns, y axis with increasing rows,
  // and z axis pointing towards the source.
  //
  // This code moves the origin to be at the source, keeps the axes orientation unchanged,
  // and creates an appropriate intrinsic matrix assuming zero shear.
  // The extrinsic matrix will be updated to account for translating the origin.

  const FrameTransform rad_world_to_proj = info.proj_to_world.inverse(); 

  const Pt3 src_wrt_proj_rad = rad_world_to_proj * info.src_wrt_world;

  const CoordScalar src_to_det_dist = src_wrt_proj_rad(2);

  xregASSERT(src_to_det_dist > 1.0e-6);

  Mat3x3 K = Mat3x3::Identity();

  K(0,0) = -src_to_det_dist / info.col_spacing_mm_per_pixel;
  K(1,1) = -src_to_det_dist / info.row_spacing_mm_per_pixel;

  K(0,2) = src_wrt_proj_rad(0) / info.col_spacing_mm_per_pixel;
  K(1,2) = src_wrt_proj_rad(1) / info.row_spacing_mm_per_pixel;

  pd.cam.coord_frame_type = CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z;

  pd.cam.setup(K, TransXYZ4x4(-src_wrt_proj_rad) * rad_world_to_proj.matrix(),
               info.num_rows, info.num_cols,
               info.col_spacing_mm_per_pixel, info.row_spacing_mm_per_pixel);

  pd.det_spacings_from_orig_meta = true;

  return pd;
}

