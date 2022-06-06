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

#ifndef XREGSLICERMARKUPSJSON_H_
#define XREGSLICERMARKUPSJSON_H_

#include "xregCommon.h"

namespace xreg
{

/// \brief Reads a 3D Slicer Markups JSON file into a list of 3D points - all other information
///        is discarded
///
/// \param fcsv_path The path on disk to the marksup JSON file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
/// \return pts The list of points to populate
Pt3List ReadSlicerMarkupsJSONFilePts(const std::string& json_path, const bool output_in_lps = true);

/// \brief Reads a 3D Slicer Markups JSON file into a mapping from point names/labels to
///        the point 3D data.
///
/// \param fcsv_path The path on disk to the markups JSON file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMap3 ReadSlicerMarkupsJSONFileNamePtMap(const std::string& json_path, const bool output_in_lps = true);

/// \brief Read a 3D Slicer Markups JSON file into a multi-mapping from point names/labels to the point 3D data.
///
/// This allows for duplicate names, but different point mappings.
///
/// \param fcsv_path The path on disk to the markups JSON file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMultiMap3 ReadSlicerMarkupsJSONFileNamePtMultiMap(const std::string& json_path, const bool output_in_lps = true);

}  // xreg

#endif
