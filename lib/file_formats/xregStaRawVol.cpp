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

#include "xregStaRawVol.h"

#include <fstream>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregFilesystemUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregStringUtils.h"

xreg::StaVolInfo xreg::ReadStaVolInfo(const std::string& sta_file_path)
{
  std::ifstream in(sta_file_path);
  
  const auto lines = GetNonEmptyLinesFromStream(in);

  const size_type num_lines = lines.size();
  
  StaVolInfo info;

  bool found_size      = false;
  bool found_step      = false;
  bool found_data_type = false;
  bool found_tpos      = false;

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

        if (param_name == "Size[pixels]")
        {
          const auto size_toks = StringCast<size_type>(StringSplit(param_val));
          
          xregASSERT(size_toks.size() == 3);

          info.dims[0] = size_toks[0];
          info.dims[1] = size_toks[1];
          info.dims[2] = size_toks[2];

          found_size = true;
        }
        else if (param_name == "Step[mm]")
        {
          const auto step_toks = StringCast<float>(StringSplit(param_val));
          
          xregASSERT(step_toks.size() == 3);

          info.spacing[0] = step_toks[0];
          info.spacing[1] = step_toks[1];
          info.spacing[2] = step_toks[2];

          found_step = true;
        }
        else if (param_name == "DataType")
        {
          info.data_type_str = StringStrip(param_val);

          found_data_type = true;
        }
      }
      else if (param_name == "TPosition")
      {
        if ((line_idx + 4) < num_lines)
        {
          for (size_type r = 0; r < 4; ++r)
          {
            const auto mat_row_toks = StringCast<float>(StringSplit(lines[line_idx + r + 1]));
            xregASSERT(mat_row_toks.size() == 4);

            for (size_type c = 0; c < 4; ++c)
            {
              info.vol_to_world.matrix()(r,c) = mat_row_toks[c];
            }
          }

          line_idx += 4;

          found_tpos = true;
        }
      }
    }
  }

  if (!(found_size && found_step && found_data_type && found_tpos))
  {
    xregThrow("Failed to find required .sta metadata fields!");
  }

  return info;
}

itk::Image<float,3>::Pointer xreg::ReadStaRawVol(const std::string& sta_file_path)
{
  // read in the metadata from the STA text file
  const auto sta_info = ReadStaVolInfo(sta_file_path);

  // only supporting uint8 for now as all of the data is of this type
  xregASSERT(sta_info.data_type_str == "uint8");

  // allocate the ITK image object that will be returned, this allocates the pixel buffer
  auto vol = MakeITK3DVol<float>(sta_info.dims[0], sta_info.dims[1], sta_info.dims[2]);
 
  // set the important metadata in the ITK image

  vol->SetSpacing(sta_info.spacing.data());

  SetITKOriginPoint(vol.GetPointer(), Pt3(sta_info.vol_to_world.matrix().block(0,3,3,1)));
  
  SetITKDirectionMatrix(vol.GetPointer(), Mat3x3(sta_info.vol_to_world.matrix().block(0,0,3,3)));
 
  // read in the pixels from the .raw file, casting them from uint8 to float
  {
    FileInputStream fin(fmt::format("{}.raw", std::get<0>(Path(sta_file_path).split_ext())));

    const size_type tot_num_voxels = sta_info.dims[0] * sta_info.dims[1] * sta_info.dims[2];

    xregASSERT(fin.num_bytes_left() == tot_num_voxels);

    std::vector<FileInputStream::uint8> vol_buf_as_uint8(tot_num_voxels);

    fin.read_remaining_bytes(vol_buf_as_uint8.data());

    std::transform(vol_buf_as_uint8.begin(), vol_buf_as_uint8.end(), vol->GetBufferPointer(),
                   [] (const FileInputStream::uint8& x) { return static_cast<float>(x); });
  }

  return vol;
}

