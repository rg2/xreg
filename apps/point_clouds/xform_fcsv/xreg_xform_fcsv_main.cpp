/*
 * MIT License
 *
 * Copyright (c) 2020-2022 Robert Grupp
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

// xreg
#include "xregProgOptUtils.h"
#include "xregFCSVUtils.h"  // Still need this to write FCSV out
#include "xregLandmarkFiles.h"
#include "xregAnatCoordFrames.h"
#include "xregITKIOUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  const int kEXIT_VAL_SUCCESS = 0;
  const int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Applies a transformation to points in an FCSV file. "
              "Currently, only rigid is supported.");
  po.set_arg_usage("<Input Landmarks File> <Transform File> <Output FCSV File>");
  po.set_min_num_pos_args(3);

  po.add("ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "ras",
         "Treat the transformation and landmarks as being with respect to the RAS coordinate frame, rather than LPS.")
    << false;
  
  po.add("invert", 'i', ProgOpts::kSTORE_TRUE, "invert",
         "Apply the inverse of the provided transform")
    << false;  // defaults to non-invert

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

  const bool use_ras = po.get("ras");

  const bool invert_xform = po.get("invert");

  auto fcsv_map = ReadLandmarksFileNamePtMultiMap(po.pos_args()[0], !use_ras);

  FrameTransform xform = ReadITKAffineTransformFromFile(po.pos_args()[1]);

  if (invert_xform)
  {
    xform = xform.inverse();
  }

  if (use_ras)
  {
    FrameTransform ras2lps = FrameTransform::Identity();
    ras2lps.matrix()(0,0) = -1;
    ras2lps.matrix()(1,1) = -1;

    xform = ras2lps * xform * ras2lps;
  }

  for (auto& name_pt : fcsv_map)
  {
    name_pt.second = xform * name_pt.second;
  }

  WriteFCSVFileFromNamePtMap(po.pos_args()[2], fcsv_map, !use_ras);

  return kEXIT_VAL_SUCCESS;
}
