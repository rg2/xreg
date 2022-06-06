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

#ifndef XREGSTRINGUTILS_H_
#define XREGSTRINGUTILS_H_

#include <string>
#include <sstream>
#include <vector>
#include <array>

#include <Eigen/Core>

namespace xreg
{

/// \brief Casts a string to another type. e.g. int i = StringCast<int>("5");
template <class T>
T StringCast(const std::string& s)
{
  T t;
  std::stringstream ss;
  ss << s;
  ss >> t;
  return t;
}

/// \brief Casts a string to another type.
///
/// Specialization for casting to string, essentially performs a copy.
template <>
inline std::string StringCast<std::string>(const std::string& s)
{
  return s;
}

/// \brief Casts a collection of strings to a collection of other types
template <class T>
std::vector<T> StringCast(const std::vector<std::string>& toks)
{
  using size_type = std::vector<std::string>::size_type;
  const size_type num_toks = toks.size();

  std::vector<T> cast_toks(num_toks);

  for (size_type i = 0; i < num_toks; ++i)
  {
    cast_toks[i] = StringCast<T>(toks[i]);
  }

  return cast_toks;
}

/// \brief Casts a range of strings defined by iterators to a vector of
///        other types.
template <class T, class Iter>
std::vector<T> StringCast(Iter begin_it, Iter end_it)
{
  std::vector<T> cast_toks;

  for (Iter it = begin_it; it != end_it; ++it)
  {
    cast_toks.push_back(StringCast<T>(*it));
  }

  return cast_toks;
}

/// \brief Casts a collection of strings to a collection of other types
///
/// Specialization for casting to strings, which performs a copy
template <>
inline std::vector<std::string> StringCast<std::string>(const std::vector<std::string>& toks)
{
  return toks;
}

/// \brief Convert all characters in a string to their upper case representations.
std::string ToUpperCase(const std::string& s);

/// \brief Convert all characters in a string to their lower case representations.
std::string ToLowerCase(const std::string& s);

/// \brief Join a collection of strings into a single string with a string separator
///
/// This should behave as the join method on Python strings:
/// Python: ','.join(['Value 1', 'Value 2'])  --> 'Value 1,Value 2'
/// C++ example 1: const char* strs[2] = { "Value 1", "Value 2" };
///                JoinTokens(strs, strs + 2, ",");  --> "Value 1,Value 2"
/// C++ example 2: std::vector<std::string> strs;
///                <populate strs in some way>
///                JoinTokens(strs.begin(), strs.end(), "; ");  --> "strs[0]; strs[1]; ... strs[N-1]; strs[N]"
template <class StrItr>
std::string JoinTokens(StrItr toks_begin, StrItr toks_end, const std::string& join_str)
{
  std::stringstream ss;

  const bool is_empty = toks_begin == toks_end;

  if (!is_empty)
  {
    StrItr tok_it = toks_begin;

    ss << *toks_begin;

    ++tok_it;

    for (; tok_it != toks_end; ++tok_it)
    {
      ss << join_str << *tok_it;
    }
  }

  return ss.str();
}

template <class A>
std::string JoinTokens(const std::vector<std::string,A>& string_list, const std::string& join_str)
{
  return JoinTokens(string_list.begin(), string_list.end(), join_str);
}

/// \brief Split a string into tokens.
///
/// The dimlimiting characters are specified by a string, where each character is
/// a delimiting character. The default is to separate by whitespace characters.
/// Also, the default behavior ignores empty string tokens.
std::vector<std::string> StringSplit(const std::string& s,
                                     const std::string& sep_toks_str = " \t",
                                     const bool ignore_empty_toks = true);

/// \brief Strip leading and trailing characters from a string.
///
/// By default, the characters to strip are whitespace characters.
std::string StringStrip(const std::string& s, const std::string& strip_chars = " \t\r\n");

/// \brief Remove all instances of specific characters from a string.
///
/// By default, the characters to remove are whitespace characters.
std::string StringRemoveAll(const std::string& s, const std::string& strip_chars = " \t\r\n");

/// \brief Strip extra trailing null characters from a string.
///
/// A null character is a valid non-terminating character of std::string, however
/// the presence of these null characters causes difficult to debug problems,
/// such as when using std::string::find and passing to interfaces using C-style
/// strings.
std::string StringStripExtraNulls(const std::string& s);

/// \brief Thrown when parsing an invalid Matlab range string
class InvalidMatlabRangeStringException { };

/// \brief Parses a Matlab-style range string, retrieving the starting value,
///        increment, and ending value.
void ParseMatlabStyleRange(const std::string& range_str, double* start_val, double* end_val, double* inc_val);

/// \brief Parses a Matlab-style range string, retrieving all values included in the range.
std::vector<double> ParseMatlabStyleRange(const std::string& range_str);

/// \brief Determines if a string ends with another string
bool StringEndsWith(const std::string& s, const std::string& ending);

/// \brief Determines if a string starts with another string
bool StringStartsWith(const std::string& s, const std::string& beginning);

/// \brief Generically converts an instance to a string using std::ostringstream.
template <class T>
std::string ToString(const T& x)
{
  std::ostringstream oss;
  oss << x;
  return oss.str();
}

/// \brief Converts a double-precision floating point number to a string with 16 fraction digits
std::string ToString(const double& x);

/// \brief Converts a single-precision floating point number to a string with 8 fraction digits
std::string ToString(const float& x);

/// \brief Converts a collection of instances to the corresponding collection of strings.
template <class tIter>
std::vector<std::string> ToStrings(tIter begin_it, tIter end_it)
{
  const std::vector<std::string>::size_type num_items = std::distance(begin_it, end_it);

  std::vector<std::string> s;
  s.reserve(num_items);

  for (tIter it = begin_it; it != end_it; ++it)
  {
    s.push_back(ToString(*it));
  }

  return s;
}

template <class T, class A>
std::vector<std::string> ToStrings(const std::vector<T,A>& v)
{
  return ToStrings(v.begin(), v.end());
}

/// \brief Converts a fixed-size Eigen column vector into a string
template <class T, int tNumRows, int tOpts, int tMaxRows, int tMaxCols>
std::string ToString(const Eigen::Matrix<T,tNumRows,1,tOpts,tMaxRows,tMaxCols>& v,
                     const std::string delim = " ")
{
  return JoinTokens(ToStrings(&v[0], &v[0] + tNumRows), delim);
}

// Examples:
// 0             --> 0
// 0,1,2         --> 0,1,2
// 0-3,7,10      --> 0,1,2,3,7,10
// 4,1-2,8,11-13 --> 4,1,2,8,11,12,13
std::vector<long> ParseCSVRangeOfInts(const std::string& s);

std::string SubstituteChars(const std::string& s, const char src_c, const char dst_c);

std::array<char,256> IdentityCharMap();

std::string MapChars(const std::string& s, const std::array<char,256>& m);

std::string BoolToYesNo(const bool b);

std::string BoolToTrueFalse(const bool b);

}  // xreg

#endif
