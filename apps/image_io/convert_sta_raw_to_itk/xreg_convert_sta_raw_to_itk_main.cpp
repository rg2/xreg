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
#include "xregITKIOUtils.h"
#include "xregStaRawVol.h"
  
int main(int argc, char* argv[])
{
  using namespace xreg;

  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;
  
  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Convert a .sta/.raw volume to an ITK compatible volume file (e.g. .nii.gz). "
              "The .sta/.raw format is used in the Ljubljana 2D/3D datasets "
              "(http://lit.fe.uni-lj.si/tools.php?lang=eng).");
  po.set_arg_usage("<Input .sta file> <Output Volume>");

  po.set_min_num_pos_args(2);

  try
  {
    po.parse(argc, argv);
  }
  catch (const xreg::ProgOpts::Exception& e)
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

  const std::string input_sta_path  = po.pos_args()[0];
  const std::string output_vol_path = po.pos_args()[1];

  std::ostream& vout = po.vout();

  vout << "reading .sta/.raw data..." << std::endl;
  auto vol = ReadStaRawVol(input_sta_path);

  vout << "writing to output ITK volume file..." << std::endl;
  WriteITKImageToDisk(vol.GetPointer(), output_vol_path);

  vout << "exiting..." << std::endl;

  return kEXIT_VAL_SUCCESS;
}
