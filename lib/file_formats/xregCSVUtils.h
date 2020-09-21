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

#ifndef XREGCSVUTILS_H_
#define XREGCSVUTILS_H_

#include <vector>
#include <string>
#include <fstream>

#include <fmt/printf.h>

#include "xregFilesystemUtils.h"

namespace xreg
{

typedef std::vector<std::string> CSVLineStringValued;
typedef std::vector<CSVLineStringValued> CSVFileStringValued;

typedef std::vector<float> CSVLineFloatValued;
typedef std::vector<CSVLineFloatValued> CSVFileFloatValued;

typedef std::vector<double> CSVLineDoubleValued;
typedef std::vector<CSVLineDoubleValued> CSVFileDoubleValued;

typedef std::vector<int> CSVLineIntValued;
typedef std::vector<CSVLineIntValued> CSVFileIntValued;

typedef std::vector<long> CSVLineLongValued;
typedef std::vector<CSVLineLongValued> CSVFileLongValued;

typedef std::vector<unsigned long> CSVLineULongValued;
typedef std::vector<CSVLineULongValued> CSVFileULongValued;

/// \brief Reads a CSV file of string values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileStringValued ReadCSVFileStr(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of float values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileFloatValued ReadCSVFileFloat(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of double values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileDoubleValued ReadCSVFileDouble(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of int values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileIntValued ReadCSVFileInt(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of long values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileLongValued ReadCSVFileLong(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of unsigned long values
///
/// Empty lines are skipped
/// \param path The CSV file path
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileULongValued ReadCSVFileULong(const std::string& path, const bool has_header = true);

/// \brief Reads a CSV file of strings, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileStringValued ReadCSVFileStr(const std::vector<std::string>& lines, const bool has_header = true);

/// \brief Reads a CSV file of floats, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileFloatValued ReadCSVFileFloat(const std::vector<std::string>& lines, const bool has_header = true);

/// \brief Reads a CSV file of doubles, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileDoubleValued ReadCSVFileDouble(const std::vector<std::string>& lines, const bool has_header = true);

/// \brief Reads a CSV file of ints, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileIntValued ReadCSVFileInt(const std::vector<std::string>& lines, const bool has_header = true);

/// \brief Reads a CSV file of longs, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileLongValued ReadCSVFileLong(const std::vector<std::string>& lines, const bool has_header = true);

/// \brief Reads a CSV file of unsigned longs, but from a list of lines stored in memory.
///
/// \param lines The list of line strings - every line is assumed to be valid
/// \param csv_file The CSV file structure to be populates; a list of string tokenizations
/// \param has_header Boolean flag indicating if the first line is a header and may be skipped
CSVFileULongValued ReadCSVFileULong(const std::vector<std::string>& lines, const bool has_header = true);

void WriteCSVFile(const std::string& path, const std::vector<std::vector<std::string>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(const std::string& path, const std::vector<std::vector<float>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(const std::string& path, const std::vector<std::vector<double>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(const std::string& path, const std::vector<std::vector<int>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(const std::string& path, const std::vector<std::vector<long>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(const std::string& path, const std::vector<std::vector<unsigned long>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<std::string>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<float>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<double>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<int>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<long>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());

void WriteCSVFile(std::ostream& out, const std::vector<std::vector<unsigned long>>& vals,
                  const std::vector<std::string>& hdr_vals = std::vector<std::string>());


/// \brief Writes a matrix to a CSV file in row-major format.
///
/// The matrix type must provide a () operator, so that mat(r,c) is the value
/// at the rth row and cth column.
void WriteMatrixToCSV(const std::string& path,
                      const Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>& mat);

/// \brief Writes a matrix to a CSV file in row-major format.
///
/// The matrix type must provide a () operator, so that mat(r,c) is the value
/// at the rth row and cth column.
void WriteMatrixToCSV(const std::string& path,
                      const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& mat);

/// \brief Writes a matrix to a CSV file in row-major format.
///
/// The matrix type must provide a () operator, so that mat(r,c) is the value
/// at the rth row and cth column.
void WriteMatrixToCSV(const std::string& path,
                      const Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic>& mat);

/// \brief Writes a matrix to a CSV file in row-major format.
///
/// The matrix type must provide a () operator, so that mat(r,c) is the value
/// at the rth row and cth column.
void WriteMatrixToCSV(const std::string& path,
                      const Eigen::Matrix<long,Eigen::Dynamic,Eigen::Dynamic>& mat);

Pt3List Read3DPtCloudCSV(const std::string& path, const bool has_header);

}  // xreg

#endif
