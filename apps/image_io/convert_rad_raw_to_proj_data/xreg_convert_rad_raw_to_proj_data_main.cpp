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
#include "xregRadRawProj.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Converts a collection of 2D projection images from .rad/.raw format "
              "into an HDF5 projection data file for use by xReg. "
              "This may be used to convert projections from the Ljubljana 2D/3D datasets "
              "located at: http://lit.fe.uni-lj.si/tools.php?lang=eng.");
  po.set_arg_usage("<Output Proj. Data File> <.rad path #1> [<.rad path #2> [... <.rad path #N>]]");
  po.set_min_num_pos_args(2);

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

  const std::string& dst_pd_path = po.pos_args()[0];

  const size_type num_rad_files = po.pos_args().size() - 1;

  vout << "number of .rad files to read: " << num_rad_files << std::endl;

  ProjDataF32List pd;
  pd.reserve(num_rad_files);

  for (size_type i = 0; i < num_rad_files; ++i)
  {
    const std::string& cur_rad_path = po.pos_args()[i+1]; 

    vout << "reading .rad file #" << (i + 1) << ": " << cur_rad_path << std::endl;

    pd.push_back(ReadRawProjAsProjData(cur_rad_path));
  }
  
  vout << "saving to proj data HDF5..." << std::endl;
  WriteProjDataH5ToDisk(pd, dst_pd_path);

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}

