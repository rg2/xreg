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

#include "xregFCSVUtils.h"

#include <fstream>

#include "xregAssert.h"
#include "xregAnatCoordFrames.h"
#include "xregCSVUtils.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"

namespace
{

using namespace xreg;

/// \brief Reads a FCSV file into a CSV string structure - all fields are read.
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param pop_names_list True indicates the names of each fiducial should be extracted and returned
/// \param output_in_lps Output points in LPS coordinate frame when true, otherwise points are output in RAS
/// \return A tuple consisting of the list of 3D points stored in the FCSV and potentially a list of the points' corresponding names
std::tuple<Pt3List, std::vector<std::string>>
ReadFCSVFileAllTxt(const std::string& fcsv_path, const bool pop_names_list, const bool output_in_lps)
{
  std::ifstream fin(fcsv_path.c_str());

  const auto lines = GetNonEmptyLinesFromStream(fin);

  // Check for sufficient number of lines to contain the header information
  if (lines.size() < 3)
  {
    xregThrow("Expected at least 3 lines in FCSV, got %lu", lines.size());
  }

  // Determine the version of the FCSV file, this is needed to determine the
  // coordinate frame of the data

  const auto& version_line = lines[0];

  xregASSERT(StringStartsWith(version_line, "# Markups fiducial file version"));

  const auto version_line_toks = StringSplit(version_line, "=");
  xregASSERT(version_line_toks.size() == 2);

  const auto version_str = StringStrip(version_line_toks[1]);

  // Now determine whether the coordinate frame is either RAS or LPS

  bool input_is_lps = true;

  const bool is_version_5 = version_str == "5.0";

  if ((version_str == "4.10") || (version_str == "4.8") || (version_str == "4.6"))
  {
    xregASSERT(lines[1] == "# CoordinateSystem = 0");
    input_is_lps = false;
  }
  else if ((version_str == "4.11") || is_version_5)
  {
    const auto& coord_frame_line = lines[1];

    xregASSERT(StringStartsWith(coord_frame_line, "# CoordinateSystem"));

    const auto coord_frame_line_toks = StringSplit(coord_frame_line, "=");
    xregASSERT(coord_frame_line_toks.size() == 2);

    const auto coord_frame_str = StringStrip(coord_frame_line_toks[1]);

    if (coord_frame_str == "RAS")
    {
      input_is_lps = false;
    }
    else
    {
      xregASSERT(coord_frame_str == "LPS");
    }
  }
  else
  {
    xregThrow("Unsupported FCSV version: %s", version_str.c_str());
  }

  // Tokenize the remaining CSV-formated data
  const auto csv_data = ReadCSVFileStr(std::vector<std::string>(lines.begin() + 3, lines.end()), false);

  const size_type num_pts = csv_data.size();

  Pt3List pts;
  pts.reserve(num_pts);

  std::vector<std::string> pt_names;
  if (pop_names_list)
  {
    pt_names.reserve(num_pts);
  }

  // Parse the point data to numeric types and optionally save the fiducial point names

  for (const auto& csv_line : csv_data)
  {
    xregASSERT(csv_line.size() == (is_version_5 ? 16 : 14));

    pts.push_back(Pt3(
      StringCast<CoordScalar>(csv_line[1]),
      StringCast<CoordScalar>(csv_line[2]),
      StringCast<CoordScalar>(csv_line[3])));

    if (pop_names_list)
    {
      pt_names.push_back(csv_line[11]);
    }
  }

  if (input_is_lps != output_in_lps)
  {
    // input coordinate frame differs from desired output, need to rotate by 180 degrees
    ConvertRASToLPS(&pts);
  }

  return std::make_tuple(pts, pt_names);
}

}  // un-named

xreg::FCSVDuplicatePointNameException::FCSVDuplicatePointNameException(const char* dup_landmark_name)
  : StringMessageException(dup_landmark_name)
{ }

xreg::Pt3List xreg::ReadFCSVFilePts(const std::string& fcsv_path, const bool output_in_lps)
{
  Pt3List pts;
  std::tie(pts, std::ignore) = ReadFCSVFileAllTxt(fcsv_path, false, output_in_lps);

  return pts;
}

xreg::LandMap3 xreg::ReadFCSVFileNamePtMap(const std::string& fcsv_path, const bool output_in_lps)
{
  const auto pts_and_names = ReadFCSVFileAllTxt(fcsv_path, true, output_in_lps);

  const auto& pts = std::get<0>(pts_and_names);
  const auto& names = std::get<1>(pts_and_names);

  const size_type num_pts = pts.size();

  xregASSERT(num_pts == names.size());

  LandMap3 m;

  for (size_type i = 0; i < num_pts; ++i)
  {
    auto insert_val = m.emplace(names[i], pts[i]);

    if (!insert_val.second)
    {
      throw FCSVDuplicatePointNameException(insert_val.first->first.c_str());
    }
  }

  return m;
}

xreg::LandMultiMap3
xreg::ReadFCSVFileNamePtMultiMap(const std::string& fcsv_path, const bool output_in_lps)
{
  const auto pts_and_names = ReadFCSVFileAllTxt(fcsv_path, true, output_in_lps);

  const auto& pts = std::get<0>(pts_and_names);
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

namespace
{

template <class tIt>
void WriteFCSVFileFromNamePtMapHelper(const std::string& fcsv_path,
                                      tIt pts_begin, tIt pts_end,
                                      const bool input_is_lps)
{
  std::ofstream out(fcsv_path);

  out << "# Markups fiducial file version = 4.6\n"
      << "# CoordinateSystem = 0\n"
      << "# columns = id,x,y,z,ow,ox,oy,oz,vis,sel,lock,label,desc,associatedNodeID";

  CoordScalar tmp_x = 0;
  CoordScalar tmp_y = 0;

  for (tIt pts_it = pts_begin; pts_it != pts_end; ++pts_it)
  {
    tmp_x = pts_it->second[0];
    tmp_y = pts_it->second[1];

    if (input_is_lps)
    {
      // inputs are LPS, but we are writing out to an older FCSV version which stores data in RAS
      // Therefore, convert to RAS
      tmp_x *= -1;
      tmp_y *= -1;
    }

    out << fmt::sprintf("\n,%.8f,%.8f,%.8f,0,0,0,1,1,1,0,%s,,",
                        tmp_x, tmp_y, pts_it->second[2], pts_it->first);
  }

  out << std::endl;
}

}  // un-named

void xreg::WriteFCSVFileFromNamePtMap(const std::string& fcsv_path,
                                      const LandMap3& pts,
                                      const bool input_is_lps)
{
  WriteFCSVFileFromNamePtMapHelper(fcsv_path, pts.begin(), pts.end(), input_is_lps);
}

void xreg::WriteFCSVFileFromNamePtMap(const std::string& fcsv_path,
                                      const LandMultiMap3& pts,
                                      const bool input_is_lps)
{
  WriteFCSVFileFromNamePtMapHelper(fcsv_path, pts.begin(), pts.end(), input_is_lps);
}

void xreg::WriteFCSVFilePts(const std::string& fcsv_path,
                            const Pt3List& pts,
                            const bool input_is_lps,
                            const bool use_empty_names)
{
  auto create_name_str_from_idx = [](const size_type idx)
                                  {
                                    return ToString(idx);
                                  };
  auto create_empty_name_str = [](const size_type) { return std::string(); };

  std::function<std::string(size_type)> create_name_str;
  if (use_empty_names)
  {
    create_name_str = create_empty_name_str;
  }
  else
  {
    create_name_str = create_name_str_from_idx;
  }
  // MSVC 2015 does not like this (which is why I use the above std::function):
  //auto create_name_str = use_empty_names ? create_empty_name_str : create_name_str_from_idx;

  const size_type num_pts = pts.size();

  LandMultiMap3 tmp_pt_map;
  tmp_pt_map.reserve(num_pts);

  for (size_type i = 0; i < num_pts; ++i)
  {
    tmp_pt_map.emplace(create_name_str(i), pts[i]);
  }

  WriteFCSVFileFromNamePtMap(fcsv_path, tmp_pt_map, input_is_lps);
}

void MergeFCSVFiles(const std::vector<std::string>& fcsv_paths, std::ostream& out)
{
  const size_type num_fcsv = fcsv_paths.size();

  for (size_type fcsv_idx = 0; fcsv_idx < num_fcsv; ++fcsv_idx)
  {
    std::ifstream in(fcsv_paths[fcsv_idx]);

    const auto lines = GetNonEmptyLinesFromStream(in);

    // write out the first three "comment" lines - this assumes that the
    // coordinate frames, etc. are consistent between the FCSV files
    if (fcsv_idx == 0)
    {
      out << lines[0] << '\n' << lines[1] << '\n' << lines[2] << '\n';
    }

    const size_type num_lines = lines.size();
    for (size_type i = 3; i < num_lines; ++i)
    {
      out << lines[i] << '\n';
    }
  }

  out.flush();
}

