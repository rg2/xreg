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

#include "xregCSVUtils.h"

#include "xregStringUtils.h"
#include "xregAssert.h"

namespace
{

using namespace xreg;

template <class T>
std::vector<std::vector<T>> ReadCSVFile(const std::vector<std::string>& lines, const bool has_header)
{
  typedef std::vector<std::vector<T>> CSVFileType;
  typedef typename CSVFileType::value_type CSVLineType;
  typedef typename CSVFileType::size_type size_type;

  std::vector<std::vector<T>> csv_file;

  const size_type num_src_lines = lines.size();

  const size_type num_dst_lines = has_header ? (num_src_lines - 1) : num_src_lines;
  const size_type start_line    = has_header ? 1 : 0;

  csv_file.resize(num_dst_lines);

  std::vector<std::string> toks;

  for (size_type i = 0; i < num_dst_lines; ++i)
  {
    CSVLineType& cur_csv_line = csv_file[i];

    toks = StringSplit(lines[start_line + i], ",", false);

    // TODO: optimize this for the case when we're not casting from strings
    const size_type num_toks = toks.size();
    cur_csv_line.resize(num_toks);
    for (size_type tok_index = 0; tok_index < num_toks; ++tok_index)
    {
      cur_csv_line[tok_index] = StringCast<T>(toks[tok_index]);
    }
  }

  return csv_file;
}

template <class T>
std::vector<std::vector<T>> ReadCSVFile(const std::string& path, const bool has_header)
{
  std::ifstream fin(path.c_str());

  return ReadCSVFile<T>(GetNonEmptyLinesFromStream(fin), has_header);
}

// TODO: parameterize on precision
template <class T>
void WriteFloatVal(std::ostream& out, const T& val)
{
  out << fmt::sprintf("%.6f", val);
}

void WriteVal(std::ostream& out, const double& val)
{
  WriteFloatVal(out, val);
}

void WriteVal(std::ostream& out, const float& val)
{
  WriteFloatVal(out, val);
}

template <class T>
void WriteVal(std::ostream& out, const T& val)
{
  out << val;
}

template <class T>
void WriteValsLine(std::ostream& out, const std::vector<T>& vals)
{
  typedef typename std::vector<T>::size_type size_type;

  if (!vals.empty())
  {
    WriteVal(out, vals[0]);

    const size_type num_vals = vals.size();
    for (size_type i = 1; i < num_vals; ++i)
    {
      out << ',';
      WriteVal(out, vals[i]);
    }
  }

  out << std::endl;
}

template <class T>
void WriteCSVFileHelper(std::ostream& out, const std::vector<std::vector<T>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  if (!hdr_vals.empty())
  {
    WriteValsLine(out, hdr_vals);
  }

  for (typename std::vector<std::vector<T> >::const_iterator line_it = vals.begin();
       line_it != vals.end(); ++line_it)
  {
    WriteValsLine(out, *line_it);
  }
}

template <class T>
void WriteCSVFileHelper(const std::string& path, const std::vector<std::vector<T>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  std::ofstream out(path.c_str());
  WriteCSVFileHelper(out, vals, hdr_vals);
  out.flush();
}

template <class tMatrix>
void WriteMatrixToCSVHelper(const std::string& path, const tMatrix& mat)
{
  std::ofstream out(path.c_str());

  const long nr = static_cast<long>(mat.rows());
  const long nc = static_cast<long>(mat.cols());

  for (long r = 0; r < nr; ++r)
  {
    for (long c = 0; c < (nc-1); ++c)
    {
      WriteVal(out, mat(r,c));
      out << ',';
    }

    WriteVal(out, mat(r,nc-1));
    out << std::endl;
  }
}

}  // un-named

xreg::CSVFileStringValued xreg::ReadCSVFileStr(const std::string& path, const bool has_header)
{
  return ReadCSVFile<std::string>(path, has_header);
}

xreg::CSVFileFloatValued xreg::ReadCSVFileFloat(const std::string& path, const bool has_header)
{
  return ReadCSVFile<float>(path, has_header);
}

xreg::CSVFileDoubleValued xreg::ReadCSVFileDouble(const std::string& path, const bool has_header)
{
  return ReadCSVFile<double>(path, has_header);
}

xreg::CSVFileIntValued xreg::ReadCSVFileInt(const std::string& path, const bool has_header)
{
  return ReadCSVFile<int>(path, has_header);
}

xreg::CSVFileLongValued xreg::ReadCSVFileLong(const std::string& path, const bool has_header)
{
  return ReadCSVFile<long>(path, has_header);
}

xreg::CSVFileULongValued xreg::ReadCSVFileULong(const std::string& path, const bool has_header)
{
  return ReadCSVFile<unsigned long>(path, has_header);
}

xreg::CSVFileStringValued xreg::ReadCSVFileStr(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<std::string>(lines, has_header);
}

xreg::CSVFileFloatValued xreg::ReadCSVFileFloat(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<float>(lines, has_header);
}

xreg::CSVFileDoubleValued xreg::ReadCSVFileDouble(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<double>(lines, has_header);
}

xreg::CSVFileIntValued xreg::ReadCSVFileInt(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<int>(lines, has_header);
}

xreg::CSVFileLongValued xreg::ReadCSVFileLong(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<long>(lines, has_header);
}

xreg::CSVFileULongValued xreg::ReadCSVFileULong(const std::vector<std::string>& lines, const bool has_header)
{
  return ReadCSVFile<unsigned long>(lines, has_header);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<std::string>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<float>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<double>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<int>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<long>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(const std::string& path, const std::vector<std::vector<unsigned long>>& vals,
                        const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(path, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<std::string>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<float>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<double>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<int>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<long>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteCSVFile(std::ostream& out, const std::vector<std::vector<unsigned long>>& vals,
                  const std::vector<std::string>& hdr_vals)
{
  WriteCSVFileHelper(out, vals, hdr_vals);
}

void xreg::WriteMatrixToCSV(const std::string& path,
                            const Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>& mat)
{
  WriteMatrixToCSVHelper(path, mat);
}

void xreg::WriteMatrixToCSV(const std::string& path,
                            const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& mat)
{
  WriteMatrixToCSVHelper(path, mat);
}

void xreg::WriteMatrixToCSV(const std::string& path,
                            const Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic>& mat)
{
  WriteMatrixToCSVHelper(path, mat);
}

void xreg::WriteMatrixToCSV(const std::string& path,
                            const Eigen::Matrix<long,Eigen::Dynamic,Eigen::Dynamic>& mat)
{
  WriteMatrixToCSVHelper(path, mat);
}

xreg::Pt3List xreg::Read3DPtCloudCSV(const std::string& path, const bool has_header)
{
  const auto csv_contents = ReadCSVFileFloat(path, has_header);
  
  Pt3List pts;
  pts.reserve(csv_contents.size());

  for (const auto& csv_line : csv_contents)
  {
    xregASSERT(csv_line.size() == 3);
    
    pts.push_back(Pt3{csv_line[0], csv_line[1], csv_line[2]});
  }

  return pts;
}

