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
#include "xregHDF5.h"
#include "xregH5ProjDataIO.h"

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  using namespace xreg;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Concatenates more than one proj. data HDF5 files into a single output "
      "HDF5 file. The projections of the first file are the first projections inserted "
      "into the output file, followed by those of the second, and so forth. This utility "
      "is helpful when setting up a multiple-view geometry from a collection of "
      "single-views.");
  po.set_arg_usage("<Input Proj. Data File #1> <Input Proj. Data File #2> "
      "[... <Input Proj. Data File #N>] <Output Concatenated Proj. Data File>");
  po.set_min_num_pos_args(3);

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

  const auto pos_args = po.pos_args();

  const size_type num_input_proj_files = pos_args.size() - 1;

  const std::string dst_cat_proj_path = pos_args.back();

  vout << "opening H5 file for writing: " << dst_cat_proj_path << std::endl;
  H5::H5File dst_h5(dst_cat_proj_path, H5F_ACC_TRUNC);
  
  SetStringAttr("xreg-type", kXREG_PROJ_DATA_ATTR_STR, &dst_h5);

  const auto dst_h5_id = dst_h5.getId();

  size_type num_dst_projs = 0;

  for (size_type src_file_idx = 0; src_file_idx < num_input_proj_files; ++src_file_idx)
  {
    const auto& cur_src_h5_path = pos_args[src_file_idx];

    vout << "  opening up proj. H5 file " << src_file_idx << " for reading: "
         << cur_src_h5_path << std::endl;

    H5::H5File cur_src_h5(cur_src_h5_path, H5F_ACC_RDONLY);

    const auto cur_src_h5_id = cur_src_h5.getId();

    const size_type cur_file_num_projs = ReadSingleScalarH5ULong("num-projs", cur_src_h5);

    vout << "    num-projs: " << cur_file_num_projs << std::endl;

    for (size_type cur_file_proj_idx = 0; cur_file_proj_idx < cur_file_num_projs;
        ++cur_file_proj_idx, ++num_dst_projs)
    {
      vout << "      copying proj: " << cur_file_proj_idx << std::endl;

      const std::string src_group_name = fmt::format("proj-{:03d}", cur_file_proj_idx);

      const std::string dst_group_name = fmt::format("proj-{:03d}", num_dst_projs);

      H5Ocopy(cur_src_h5_id, src_group_name.c_str(),
              dst_h5_id, dst_group_name.c_str(),
              H5P_DEFAULT, H5P_DEFAULT);
    }
  }

  WriteSingleScalarH5("num-projs", num_dst_projs, &dst_h5);
  
  dst_h5.flush(H5F_SCOPE_GLOBAL);
  dst_h5.close();

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}
