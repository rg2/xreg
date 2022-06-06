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

#ifndef XREGLANDMARKFILES_H_
#define XREGLANDMARKFILES_H_

#include "xregCommon.h"

namespace xreg
{

/// \brief Determines if a file format is compatible with ReadLandmarksFilePts().
///        e.g. can a list of 3D points be loaded from the file.
///
/// This is determined by interpreting the file extension.
bool IsSupportedLandmarksFilePts(const std::string& path);

/// \brief Determines if a file format is compatible with
//         ReadLandmarksFileNamePtMap() or ReadLandmarksFileNamePtMultiMap().
///        e.g. can a name/3D point mapping be loaded from the file.
///
/// This is determined by interpreting the file extension.
bool IsSupportedLandmarksFileNamePtMap(const std::string& path);

/// \brief Reads a landmarks file into a list of 3D points - all other information
///        is discarded. Supported file formats are FCSV and 3D Slicer Markups JSON.
///
/// \param path The path on disk to the landmarks file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
/// \return pts The list of points to populate
Pt3List ReadLandmarksFilePts(const std::string& path, const bool output_in_lps = true);

/// \brief Reads a landmarks file into a mapping from point names/labels to
///        the point 3D data. Supported file formats are FCSV and 3D Slicer Markups JSON.
///
/// \param path The path on disk to the landmarks file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMap3 ReadLandmarksFileNamePtMap(const std::string& path, const bool output_in_lps = true);

/// \brief Read a landmarks file into a multi-mapping from point names/labels to the point 3D data.
///        Supported file formats are FCSV and 3D Slicer Markups JSON.
///
/// This allows for duplicate names, but different point mappings.
///
/// \param path The path on disk to the landmarks file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMultiMap3 ReadLandmarksFileNamePtMultiMap(const std::string& path, const bool output_in_lps = true);

}  // xreg

#endif
