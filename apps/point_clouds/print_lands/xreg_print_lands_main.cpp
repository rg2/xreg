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

// STD
#include <iostream>

#include <fmt/format.h>

// xreg
#include "xregProgOptUtils.h"
#include "xregLandmarkFiles.h"
#include "xregLandmarkMapUtils.h"
#include "xregAnatCoordFrames.h"

using namespace xreg;

int main(int argc, char* argv[])
{
  const int kEXIT_VAL_SUCCESS = 0;
  const int kEXIT_VAL_BAD_USE = 1;

  // First, set up the program options

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Prints the contents of a landmarks file to stdout in a prettier format than the original FCSV/JSON.");
  po.set_arg_usage("<Landmarks file path (e.g. .fcsv/.json/.mrk.json)>");
  po.set_min_num_pos_args(1);

  po.add("no-dups", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-dups",
         "Do not allow duplicate landmark names.")
    << false;

  po.add("ras", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "ras",
         "Print landmarks in RAS coordinates instead of LPS coordinates")
    << false;

  po.add("no-sort", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-sort",
         "Do NOT sort the mapping so that the landmarks are printed out lexographically sorted by name.")
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

  const bool no_dups   = po.get("no-dups");
  const bool lands_ras = po.get("ras");
  const bool no_sort   = po.get("no-sort");

  const std::string lands_file_path = po.pos_args()[0];

  if (!no_dups)
  {
    auto lands_map = ReadLandmarksFileNamePtMultiMap(lands_file_path, !lands_ras);

    if (no_sort)
    {
      PrintLandmarkMap(lands_map, std::cout);
    }
    else
    {
      std::multimap<std::string,Pt3> ordered_map(lands_map.begin(), lands_map.end());
      PrintLandmarkMap(ordered_map, std::cout);
    }
  }
  else
  {
    auto lands_map = ReadLandmarksFileNamePtMap(lands_file_path, !lands_ras);
    
    if (no_sort)
    {
      PrintLandmarkMap(lands_map, std::cout);
    }
    else
    {
      std::map<std::string,Pt3> ordered_map(lands_map.begin(), lands_map.end());
      PrintLandmarkMap(ordered_map, std::cout);
    }
  }

  std::cout.flush();

  return kEXIT_VAL_SUCCESS;
}
