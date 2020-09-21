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

#include "xregFCSVUtils.h"

#include <fstream>

#include "xregCSVUtils.h"
#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"

namespace
{

using namespace xreg;

/// \brief Reads a FCSV file into a CSV string structure - all fields are read.
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param csv_vals The CSV structure to populate
CSVFileStringValued ReadFCSVFileAllTxt(const std::string& fcsv_path)
{
  std::ifstream fin(fcsv_path.c_str());

  auto lines = GetNonEmptyLinesFromStream(fin);

  // first three lines are comments - they do have some information, but let's
  // just assume it's constant for now

  return ReadCSVFileStr(std::vector<std::string>(lines.begin() + 3, lines.end()), false);
}

}  // un-named

xreg::FCSVDuplicatePointNameException::FCSVDuplicatePointNameException(const char* dup_landmark_name)
  : StringMessageException(dup_landmark_name)
{ }

xreg::Pt3List xreg::ReadFCSVFilePts(const std::string& fcsv_path)
{
  const CSVFileStringValued csv_vals = ReadFCSVFileAllTxt(fcsv_path);

  const size_type num_pts = csv_vals.size();

  Pt3List pts(num_pts);

  for (size_type i = 0; i < num_pts; ++i)
  {
    auto& cur_pt = pts[i];
    cur_pt[0] = StringCast<CoordScalar>(csv_vals[i][1]);
    cur_pt[1] = StringCast<CoordScalar>(csv_vals[i][2]);
    cur_pt[2] = StringCast<CoordScalar>(csv_vals[i][3]);
  }

  return pts;
}

xreg::LandMap3 xreg::ReadFCSVFileNamePtMap(const std::string& fcsv_path)
{
  const auto csv_vals = ReadFCSVFileAllTxt(fcsv_path);

  const size_type num_pts = csv_vals.size();

  LandMap3 m;

  Pt3 tmp_pt;

  for (size_type i = 0; i < num_pts; ++i)
  {
    tmp_pt[0] = StringCast<CoordScalar>(csv_vals[i][1]);
    tmp_pt[1] = StringCast<CoordScalar>(csv_vals[i][2]);
    tmp_pt[2] = StringCast<CoordScalar>(csv_vals[i][3]);

    auto insert_val = m.emplace(csv_vals[i][11], tmp_pt);

    if (!insert_val.second)
    {
      throw FCSVDuplicatePointNameException(insert_val.first->first.c_str());
    }
  }

  return m;
}

xreg::LandMultiMap3
xreg::ReadFCSVFileNamePtMultiMap(const std::string& fcsv_path)
{
  const CSVFileStringValued csv_vals = ReadFCSVFileAllTxt(fcsv_path);

  const size_type num_pts = csv_vals.size();

  LandMultiMap3 m;

  Pt3 tmp_pt;

  for (size_type i = 0; i < num_pts; ++i)
  {
    tmp_pt[0] = StringCast<CoordScalar>(csv_vals[i][1]);
    tmp_pt[1] = StringCast<CoordScalar>(csv_vals[i][2]);
    tmp_pt[2] = StringCast<CoordScalar>(csv_vals[i][3]);

    m.emplace(csv_vals[i][11], tmp_pt);
  }

  return m;
}

namespace
{

template <class tIt>
void WriteFCSVFileFromNamePtMapHelper(const std::string& fcsv_path,
                                      tIt pts_begin, tIt pts_end)
{
  std::ofstream out(fcsv_path);

  out << "# Markups fiducial file version = 4.6\n"
      << "# CoordinateSystem = 0\n"
      << "# columns = id,x,y,z,ow,ox,oy,oz,vis,sel,lock,label,desc,associatedNodeID";

  for (tIt pts_it = pts_begin; pts_it != pts_end; ++pts_it)
  {
    out << fmt::sprintf("\n,%.8f,%.8f,%.8f,0,0,0,1,1,1,0,%s,,",
                        pts_it->second[0], pts_it->second[1], pts_it->second[2], pts_it->first);
  }

  out << std::endl;
}

}  // un-named

void xreg::WriteFCSVFileFromNamePtMap(const std::string& fcsv_path, const LandMap3& pts)
{
  WriteFCSVFileFromNamePtMapHelper(fcsv_path, pts.begin(), pts.end());
}

void xreg::WriteFCSVFileFromNamePtMap(const std::string& fcsv_path,
                                      const LandMultiMap3& pts)
{
  WriteFCSVFileFromNamePtMapHelper(fcsv_path, pts.begin(), pts.end());
}

void xreg::WriteFCSVFilePts(const std::string& fcsv_path, const Pt3List& pts,
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

  WriteFCSVFileFromNamePtMap(fcsv_path, tmp_pt_map);
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

