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

#include "xregLandmarkFiles.h"

#include "xregFilesystemUtils.h"
#include "xregStringUtils.h"

#include "xregFCSVUtils.h"
#include "xregSlicerMarkupsJSON.h"
#include "xregCSVUtils.h"

bool xreg::IsSupportedLandmarksFilePts(const std::string& path)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());
  return (file_ext == ".fcsv") || (file_ext == ".json") || (file_ext == ".csv");
}

bool xreg::IsSupportedLandmarksFileNamePtMap(const std::string& path)
{
  const std::string file_ext = ToLowerCase(Path(path).file_extension());
  return (file_ext == ".fcsv") || (file_ext == ".json");
}

xreg::Pt3List xreg::ReadLandmarksFilePts(const std::string& path, const bool output_in_lps)
{
  Pt3List pts;

  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".fcsv")
  {
    pts = ReadFCSVFilePts(path, output_in_lps);
  }
  else if (file_ext == ".json")
  {
    pts = ReadSlicerMarkupsJSONFilePts(path, output_in_lps);
  }
  else if (file_ext == ".csv")
  {
    std::cerr << "WARNING: reading a .csv 3D point cloud file, ignoring \"output_in_lps\" parameter!" << std::endl;

    pts = Read3DPtCloudCSV(path, false);
  }
  else
  {
    xregThrow("unknown file extension for landmark files: %s", file_ext.c_str());
  }

  return pts;
}

xreg::LandMap3 xreg::ReadLandmarksFileNamePtMap(const std::string& path, const bool output_in_lps)
{
  LandMap3 pts;

  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".fcsv")
  {
    pts = ReadFCSVFileNamePtMap(path, output_in_lps);
  }
  else if (file_ext == ".json")
  {
    pts = ReadSlicerMarkupsJSONFileNamePtMap(path, output_in_lps);
  }
  else
  {
    xregThrow("unknown file extension for landmark files: %s", file_ext.c_str());
  }

  return pts;
}

xreg::LandMultiMap3 xreg::ReadLandmarksFileNamePtMultiMap(const std::string& path, const bool output_in_lps)
{
  LandMultiMap3 pts;

  const std::string file_ext = ToLowerCase(Path(path).file_extension());

  if (file_ext == ".fcsv")
  {
    pts = ReadFCSVFileNamePtMultiMap(path, output_in_lps);
  }
  else if (file_ext == ".json")
  {
    pts = ReadSlicerMarkupsJSONFileNamePtMultiMap(path, output_in_lps);
  }
  else
  {
    xregThrow("unknown file extension for landmark files: %s", file_ext.c_str());
  }

  return pts;
}
