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

#ifndef XREGFCSVUTILS_H_
#define XREGFCSVUTILS_H_

/// \file
/// Utilities for working with .fcsv files, usually output from 3D-Slicer.

#include "xregCommon.h"
#include "xregExceptionUtils.h"

namespace xreg
{

/// \brief Exception that is thrown when building a mapping from landmark
///        names/descriptions in a FCSV file and two entires have the same name.
class FCSVDuplicatePointNameException : public StringMessageException
{
public:
  explicit FCSVDuplicatePointNameException(const char* dup_landmark_name);
};

/// \brief Reads a FCSV file into a list of 3D points - all other information
///        is discarded
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
/// \return pts The list of points to populate
Pt3List ReadFCSVFilePts(const std::string& fcsv_path, const bool output_in_lps = true);

/// \brief Writes a name -> point (3D) mapping to disk in FCSV format
void WriteFCSVFileFromNamePtMap(const std::string& fcsv_path,
                                const LandMap3& pts,
                                const bool input_is_lps = true);

void WriteFCSVFileFromNamePtMap(const std::string& fcsv_path,
                                const LandMultiMap3& pts,
                                const bool input_is_lps = true);

/// \brief Writes list of 3D points to a FCSV file, name name for each
///        point is simply the index converted to a string.
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param pts The list of points
void WriteFCSVFilePts(const std::string& fcsv_path, const Pt3List& pts,
                      const bool input_is_lps = true,
                      const bool use_empty_names = false);

/// \brief Reads a FCSV file into a mapping from point names/labels to
///        the point 3D data.
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMap3 ReadFCSVFileNamePtMap(const std::string& fcsv_path, const bool output_in_lps = true);

/// \brief Read a FCSV file into a multi-mapping from point names/labels to the point 3D data.
///
/// This allows for duplicate names, but different point mappings.
///
/// \param fcsv_path The path on disk to the FCSV file
/// \param output_in_lps Output points are in LPS coordinates when true, RAS when false
LandMultiMap3 ReadFCSVFileNamePtMultiMap(const std::string& fcsv_path, const bool output_in_lps = true);

/// \brief Concatenates the contents of several FCSV files.
///
/// Cannot simply use the cat command as there are three leading "header lines"
/// that must be stripped after the first file.
/// NOTE: that this routine assumes that all FCSV files use the same coordinate frame. All data will be
/// copied and pasted and the first file's coordinate frame will be used.
/// \param fcsv_paths Collection of FCSV file paths
/// \param out The output stream to write the merged FCSV contents
void MergeFCSVFiles(const std::vector<std::string>& fcsv_paths, std::ostream& out);

}  // xreg

#endif
