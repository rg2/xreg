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

#include "xregACSVUtils.h"

#include <fstream>

#include "xregStringUtils.h"
#include "xregFilesystemUtils.h"

std::tuple<xreg::Pt3,xreg::Pt3>
xreg::ReadROIFromACSV(const std::string& acsv_path, const bool ras2lps)
{
  Pt3 center;
  Pt3 half_len;

  std::ifstream in(acsv_path);

  const auto lines = GetNonEmptyLinesFromStream(in);

  const size_type num_lines = lines.size();

  const size_type kNOT_FOUND = size_type(-1);

  bool found_data = false;

  bool looking_for_center = true;

  for (size_type i = 0; i < num_lines; ++i)
  {
    if (lines[i][0] != '#')
    {
      const auto toks = StringSplit(lines[i], "|");
     
      if (toks[0] == "point")
      {
        Pt3& cur_pt = looking_for_center ? center : half_len;

        if (toks.size() > 3)
        {
          cur_pt[0] = StringCast<CoordScalar>(toks[1]);
          cur_pt[1] = StringCast<CoordScalar>(toks[2]);
          cur_pt[2] = StringCast<CoordScalar>(toks[3]);

          if (looking_for_center)
          {
            looking_for_center = false;
          }
          else
          {
            found_data = true;
            break;  // we're done
          }
        }
        else
        {
          const char* pt_str = looking_for_center ? "Center point" : "Half length";

          xregThrow("%s line does not have enough tokens: %s", pt_str, lines[i].c_str());
        }
      } 
    }
  }

  if (!found_data)
  {
    const char* pt_str = looking_for_center ? "Center point" : "Half length";
    xregThrow("%s not found in ACSV!", pt_str);
  }

  if (ras2lps)
  {
    center[0] *= -1;
    center[1] *= -1;
  }

  return std::make_tuple(center, half_len);
}

