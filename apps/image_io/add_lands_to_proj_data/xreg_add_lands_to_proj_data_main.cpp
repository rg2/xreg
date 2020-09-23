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

#include "xregProgOptUtils.h"
#include "xregStringUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregHDF5.h"
#include "xregFCSVUtils.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;
  constexpr int kEXIT_VAL_NO_PROJS = 2;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Add 2D landmarks to projection data file. "
              "Existing landmarks are overwritten. "
              "The index of the projection may be optionally specified by "
              "prepending the index and a \":\" to the file path. An index of zero "
              "is used when no index is provided for the first landmark file. "
              "When no index is provided for subsequent files, the last used "
              "index + 1 is used.");
  po.set_arg_usage("<Proj. Data File> <[proj. index:]FCSV path #1> [<[proj. index:]FCSV path #2> [... <[proj. index:]FCSV path #N>]]");
  po.set_min_num_pos_args(2);

  po.add("no-pat-rot-up", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-pat-rot-up",
         "Ignore any flags for rotating the image to achieve patient \"up\" orientation.")
    << false;

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

  const bool ignore_pat_rot_up = po.get("no-pat-rot-up");

  vout << "opening proj data for reading/writing..." << std::endl;
  H5::H5File h5(po.pos_args()[0], H5F_ACC_RDWR);

  vout << "reading proj data metadata models..." << std::endl;
  const auto proj_metas = ReadProjDataH5F32(h5,false);

  const size_type num_projs = proj_metas.size();
  vout << "number of cameras/projections in file: " << num_projs << std::endl;

  if (num_projs)
  {
    size_type dst_proj_idx = 0;

    for (auto lands_file_path_it = po.pos_args().begin() + 1;
         lands_file_path_it != po.pos_args().end();
         ++lands_file_path_it, ++dst_proj_idx)
    {
      const auto tmp_toks = StringSplit(*lands_file_path_it, ":");
     
      const std::string* cur_lands_path = nullptr;

      if (tmp_toks.size() > 1)
      {
        // user passed a destination index, e.g. "1:/some/path/my_lands.fcsv"
        dst_proj_idx   = StringCast<size_type>(tmp_toks[0]);
        cur_lands_path = &tmp_toks[1];
      }
      else
      {
        // user did not pass a destination index, e.g. "/some/path/my_lands.fcsv"
        cur_lands_path = &*lands_file_path_it;
      }
      
      vout << "  cur. lands: " << *cur_lands_path << '\n'
           << "  dst. index: " << dst_proj_idx << std::endl;

      xregASSERT(dst_proj_idx < num_projs);
      
      const auto& cur_proj_meta = proj_metas[dst_proj_idx];
     
      const auto& cur_cam = cur_proj_meta.cam;
      
      CoordScalar spacing_for_phys_to_ind_x = cur_cam.det_col_spacing;
      CoordScalar spacing_for_phys_to_ind_y = cur_cam.det_row_spacing;

      bool need_to_rot = false;

      const bool check_rot_field = !ignore_pat_rot_up && cur_proj_meta.rot_to_pat_up;

      if (check_rot_field)
      {
        switch (*cur_proj_meta.rot_to_pat_up)
        {
        case ProjDataRotToPatUp::kZERO:
        case ProjDataRotToPatUp::kONE_EIGHTY:
          // nothing to change
          break;
        case ProjDataRotToPatUp::kNINETY:
        case ProjDataRotToPatUp::kTWO_SEVENTY:
          // rows and cols were swapped when annotating lands
          std::swap(spacing_for_phys_to_ind_x,spacing_for_phys_to_ind_y);
          break;
        default:
          xregThrow("unsupported rotation field: %d", static_cast<int>(*cur_proj_meta.rot_to_pat_up));
          break;
        }
      }
      
      vout << "    reading landmarks from file and converting RAS --> LPS..." << std::endl;
      auto lands_fcsv = ReadFCSVFileNamePtMap(*cur_lands_path);
      ConvertRASToLPS(&lands_fcsv);
      
      vout << "\n    FCSV contents:\n";
      PrintLandmarkMap(lands_fcsv, vout);
      vout << "-------------------------------------\n" << std::endl;

      vout << "    converting to 2D indices..." << std::endl;
      auto lands = PhysPtsToInds(DropPtDim(lands_fcsv, 2),
                                 spacing_for_phys_to_ind_x, spacing_for_phys_to_ind_y);

      if (check_rot_field)
      {
        switch (*cur_proj_meta.rot_to_pat_up)
        {
        case ProjDataRotToPatUp::kZERO:
          vout << "    no rotation necessary for patient up." << std::endl;
          break;
        case ProjDataRotToPatUp::kONE_EIGHTY:
        {
          vout << "    rotating 180 degrees for patient up..." << std::endl;

          // Rotate by 180 deg --> flip rows and flip cols
          for (auto& kv : lands)
          {
            kv.second[0] = static_cast<CoordScalar>(cur_cam.num_det_cols - 1) - kv.second[0];
            kv.second[1] = static_cast<CoordScalar>(cur_cam.num_det_rows - 1) - kv.second[1];
          }

          break;
        }
        case ProjDataRotToPatUp::kNINETY:
        case ProjDataRotToPatUp::kTWO_SEVENTY:
          // sorry we don't support these for now
        default:
          xregThrow("unsupported rotation field: %d", static_cast<int>(*cur_proj_meta.rot_to_pat_up));
          break;
        }
      }

      vout << "\n    Converted Indices:\n";
      PrintLandmarkMap(lands, vout);
      vout << "-------------------------------------\n" << std::endl;

      vout << "    updating HDF5 file with lands..." << std::endl;
      AddLandsToProjDataH5(lands, dst_proj_idx, &h5, true);
    }
  }
  else
  {
    std::cerr << "ERROR: NO PROJECTIONS IN FILE!" << std::endl;
    return kEXIT_VAL_NO_PROJS;
  }

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}
