/*
 * MIT License
 *
 * Copyright (c) 2022 Robert Grupp
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

#include "xregSlicerMarkupsJSON.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "xregAssert.h"
#include "xregFCSVUtils.h"  // For duplicate point exception
#include "xregStringUtils.h"
#include "xregAnatCoordFrames.h"

namespace
{

std::tuple<xreg::Pt3List,std::vector<std::string>>
ReadSlicerMarkupsJSONHelper(
  const std::string& json_path,
  const bool pop_names_list,
  const bool output_in_lps)
{
  using namespace xreg;

  using boost::property_tree::ptree;

  ptree pt;
  read_json(json_path, pt);

  Pt3List pts;

  std::vector<std::string> names;

  Pt3 tmp_pt;

  auto& markups_children = pt.get_child("markups");

  for (auto markups_it = markups_children.begin(); markups_it != markups_children.end(); ++markups_it)
  {
    const auto coord_sys_str = ToUpperCase(markups_it->second.get<std::string>("coordinateSystem"));

    const bool input_is_lps = coord_sys_str == "LPS";
    if (!input_is_lps && (coord_sys_str != "RAS"))
    {
      xregThrow("unsupported coordinate frame: %s", coord_sys_str.c_str());
    }

    auto& ctrl_pts_children = markups_it->second.get_child("controlPoints");
    
    for (auto ctrl_pts_it = ctrl_pts_children.begin(); ctrl_pts_it != ctrl_pts_children.end(); ++ctrl_pts_it)
    {
      auto& cur_pt = ctrl_pts_it->second;

      const auto name_str = cur_pt.get<std::string>("label");

      if (pop_names_list)
      {
        names.push_back(name_str);
      }

      auto& pos_pt = cur_pt.get_child("position");

      int i = 0;
      for (auto pos_it = pos_pt.begin(); pos_it != pos_pt.end(); ++pos_it, ++i)
      {
        if (i >= 3)
        {
          xregThrow("Invalid point data for %s, too many scalar values", name_str.c_str());
        }

        tmp_pt[i] = StringCast<CoordScalar>(pos_it->second.data());
      }
      xregASSERT(i == 3);

      if (input_is_lps != output_in_lps)
      {
        // input coordinate frame differs from desired output, need to rotate by 180 degrees
        ConvertRASToLPS(&tmp_pt);
      }

      pts.push_back(tmp_pt);
    }
  }

  return std::make_tuple(pts, names);
}

}  // un-named

xreg::Pt3List xreg::ReadSlicerMarkupsJSONFilePts(const std::string& json_path, const bool output_in_lps)
{
  return std::get<0>(ReadSlicerMarkupsJSONHelper(json_path, false, output_in_lps));
}

xreg::LandMap3 xreg::ReadSlicerMarkupsJSONFileNamePtMap(const std::string& json_path, const bool output_in_lps)
{
  const auto pts_and_names = ReadSlicerMarkupsJSONHelper(json_path, true, output_in_lps);

  const auto& pts   = std::get<0>(pts_and_names);
  const auto& names = std::get<1>(pts_and_names);

  const size_type num_pts = pts.size();

  xregASSERT(num_pts == names.size());

  LandMap3 m;

  for (size_type i = 0; i < num_pts; ++i)
  {
    if (!m.emplace(names[i], pts[i]).second)
    {
      throw FCSVDuplicatePointNameException(names[i].c_str());
    }
  }

  return m;
}

xreg::LandMultiMap3 xreg::ReadSlicerMarkupsJSONFileNamePtMultiMap(const std::string& json_path, const bool output_in_lps)
{
  const auto pts_and_names = ReadSlicerMarkupsJSONHelper(json_path, true, output_in_lps);

  const auto& pts   = std::get<0>(pts_and_names);
  const auto& names = std::get<1>(pts_and_names);

  const size_type num_pts = pts.size();

  xregASSERT(num_pts == names.size());

  LandMultiMap3 m;

  for (size_type i = 0; i < num_pts; ++i)
  {
    m.emplace(names[i], pts[i]);
  }

  return m;
}
