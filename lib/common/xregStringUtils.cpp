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

#include "xregStringUtils.h"

#include <memory>
#include <cmath>
#include <numeric> // std::iota

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <fmt/printf.h>

#include "xregExceptionUtils.h"

std::string xreg::ToUpperCase(const std::string& s)
{
  std::string u = s;
  boost::algorithm::to_upper(u);
  return u;
}

std::string xreg::ToLowerCase(const std::string& s)
{
  std::string l = s;
  boost::algorithm::to_lower(l);
  return l;
}

std::vector<std::string> xreg::StringSplit(const std::string& s,
                                           const std::string& sep_toks_str,
                                           const bool ignore_empty_toks)
{
  using BoostCharSep = boost::char_separator<char>;

  std::unique_ptr<BoostCharSep> sep;
  if (ignore_empty_toks)
  {
    sep.reset(new BoostCharSep(sep_toks_str.c_str()));
  }
  else
  {
    sep.reset(new BoostCharSep(sep_toks_str.c_str(), "", boost::keep_empty_tokens));
  }

  boost::tokenizer<boost::char_separator<char> > tmp_toks(s, *sep.get());

  return std::vector<std::string>(tmp_toks.begin(), tmp_toks.end());
}

std::string xreg::StringStrip(const std::string& s, const std::string& strip_chars)
{
  using size_type = std::string::size_type;

  std::string s_stripped;

  const size_type len = s.size();

  if (len > 0)
  {
    size_type start_idx = 0;
    for (; start_idx < len; ++start_idx)
    {
      // Note that we're searching the list of chars to strip - in the most
      // common case, a string will not be stripped at all, which means the
      // first char of the string is not one of the stripping chars, so this
      // loop will break on the first iteration. Otherwise, this loop breaks
      // on the first index of the input string which corresponds to a
      // stripping char
      // Let N be the length of the input string and M be the number of splitting
      // characters. This loop has NM complexity in the worst case, M complexity
      // in the best case, where M << N.
      if (strip_chars.find(s[start_idx]) == std::string::npos)
      {
        break;
      }
    }

    // we need to use a signed type here, to deal with the case of the input
    // string consisting entirely of stripping chars (would result in --0, which
    // underflows to the maximum index)
    long end_idx = static_cast<long>(len - 1);
    for (; end_idx >= static_cast<long>(start_idx); --end_idx)
    {
      // same thing as before, but now working backwards

      if (strip_chars.find(s[end_idx]) == std::string::npos)
      {
        break;
      }
    }

    if (static_cast<long>(start_idx) <= end_idx)
    {
      s_stripped = s.substr(start_idx, end_idx - start_idx + 1);
    }
  }

  return s_stripped;
}

std::string xreg::StringRemoveAll(const std::string& s, const std::string& strip_chars)
{
  using size_type = std::string::size_type;

  const size_type src_str_len = s.size();

  std::vector<char> char_buf;
  char_buf.reserve(src_str_len + 1);

  for (size_type i = 0; i < src_str_len; ++i)
  {
    if (strip_chars.find_first_of(s[i]) == std::string::npos)
    {
      char_buf.push_back(s[i]);
    }
  }

  char_buf.push_back(0);

  return std::string(&char_buf[0]);
}

std::string xreg::StringStripExtraNulls(const std::string& s)
{
  using size_type = std::string::size_type;

  std::string s_stripped;

  const size_type len = s.size();

  if (len > 0)
  {
    size_type num_extra_nulls = 0;

    for (; num_extra_nulls < len; ++num_extra_nulls)
    {
      if (s[len - num_extra_nulls - 1] != '\0')
      {
        break;
      }
    }

    if (len != num_extra_nulls)
    {
      s_stripped = s.substr(0, len - num_extra_nulls);
    }
  }

  return s_stripped;
}

void xreg::ParseMatlabStyleRange(const std::string& range_str, double* start_val, double* end_val, double* inc_val)
{
  using size_type = std::vector<std::string>::size_type;

  std::vector<std::string> toks = StringSplit(range_str, ":");

  const size_type num_toks = toks.size();

  double sv = 0;
  double ev = 0;
  double iv = 0;

  double tok1 = 0;
  double tok2 = 0;
  double tok3 = 0;

  if (num_toks >= 1)
  {
    tok1 = StringCast<double>(toks[0]);

    if (num_toks >= 2)
    {
      tok2 = StringCast<double>(toks[1]);

      if (num_toks >= 3)
      {
        tok3 = StringCast<double>(toks[2]);
      }
    }
  }

  if (num_toks == 1)
  {
    sv = tok1;
    ev = tok1;
    iv = 1;  // non-zero increment indicates range is not empty
  }
  else if (num_toks == 2)
  {
    sv = tok1;
    ev = tok2;
    iv = 1;
  }
  else if (num_toks == 3)
  {
    sv = tok1;
    ev = tok3;
    iv = tok2;
  }
  else
  {
    throw InvalidMatlabRangeStringException();
  }

  if (start_val)
  {
    *start_val = sv;
  }

  if (end_val)
  {
    *end_val = ev;
  }

  if (inc_val)
  {
    *inc_val = iv;
  }
}

std::vector<double> xreg::ParseMatlabStyleRange(const std::string& range_str)
{
  using size_type = std::vector<double>::size_type;

  std::vector<double> vals;

  double sv = 0;
  double ev = 0;
  double iv = 0;

  ParseMatlabStyleRange(range_str, &sv, &ev, &iv);

  if (std::abs(iv) > 1.0e-12)  // check for non-zero increment
  {
    const double signed_dist = ev - sv;
    const double num_steps = signed_dist / iv;

    // num_steps can be zero, eg. 1:1 --> [1]

    if (num_steps > -1.0e-12)  // check for non-negative
    {
      const size_type num_vals = static_cast<size_type>(num_steps) + 1;
      vals.reserve(num_vals);

      double cur_val = sv;
      for (size_type i = 0; i < num_vals; ++i, cur_val += iv)
      {
        vals.push_back(cur_val);
      }
    }
    // else if num_steps is negative, then range is empty
  }
  // else if zero increment, range is empty

  return vals;
}

bool xreg::StringStartsWith(const std::string& s, const std::string& beginning)
{
  bool to_ret = true;

  if (beginning.size() <= s.size())
  {
    auto beg_it = beginning.begin();
    auto s_it = s.begin();

    for (; beg_it != beginning.end(); ++beg_it, ++s_it)
    {
      if (*beg_it != *s_it)
      {
        to_ret = false;
        break;
      }
    }
  }
  else
  {
    to_ret = false;
  }

  return to_ret;
}

bool xreg::StringEndsWith(const std::string& s, const std::string& ending)
{
  bool to_ret = true;

  if (ending.size() <= s.size())
  {
    std::string::const_reverse_iterator ending_it = ending.rbegin();
    std::string::const_reverse_iterator s_it      = s.rbegin();

    for (; ending_it != ending.rend(); ++ending_it, ++s_it)
    {
      if (*ending_it != *s_it)
      {
        to_ret = false;
        break;
      }
    }
  }
  else
  {
    // ending could not fit into s
    to_ret = false;
  }

  return to_ret;
}

std::string xreg::ToString(const double& x)
{
  return fmt::sprintf("%.16f", x);
}

std::string xreg::ToString(const float& x)
{
  return fmt::sprintf("%.8f", x);
}

std::vector<long> xreg::ParseCSVRangeOfInts(const std::string& s)
{
  std::vector<long> ints;

  const auto csv_toks = StringSplit(s, ",");
  
  for (const auto& csv_tok : csv_toks)
  {
    const auto dash_toks = StringSplit(csv_tok, "-");
    
    const auto num_dash_toks = dash_toks.size();

    if (num_dash_toks == 1)
    {
      ints.push_back(StringCast<long>(dash_toks[0]));
    }
    else if (num_dash_toks == 2)
    {
      const long start = StringCast<long>(dash_toks[0]);
      const long stop  = StringCast<long>(dash_toks[1]);
      
      const long num_new = stop - start + 1;
      if (num_new > 0)
      {
        std::vector<long> tmp(num_new);
        std::iota(tmp.begin(), tmp.end(), start);
        ints.insert(ints.end(), tmp.begin(), tmp.end());
      }
    }
    else
    {
      xregThrow("Invalid input! only one dash allowed to specify a range!");
    }
  }

  return ints;
}

std::string xreg::SubstituteChars(const std::string& s, const char src_c, const char dst_c)
{
  std::string dst_s;
  dst_s.reserve(s.size());

  for (auto& c : s)
  {
    dst_s.push_back((c != src_c) ? c : dst_c);
  }

  return dst_s;
}

std::array<char,256> xreg::IdentityCharMap()
{
  std::array<char,256> m;

  for (int c = 0; c < 256; ++c)
  {
    m[c] = static_cast<char>(c);
  }

  return m;
}

std::string xreg::MapChars(const std::string& s, const std::array<char,256>& m)
{
  std::string sm;
  sm.reserve(s.size());

  for (auto& c : s)
  {
    sm.push_back(m[c]);
  }

  return sm;
}

std::string xreg::BoolToYesNo(const bool b)
{
  return b ? "Yes" : "No";
}

std::string xreg::BoolToTrueFalse(const bool b)
{
  return b ? "True" : "False";
}

