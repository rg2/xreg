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

#include "xregProgOptUtils.h"

#include <sstream>
#include <iterator>
#include <memory>
#include <cctype>

// This is for determining the size of the terminal window
#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/ioctl.h>
#endif

// For when verbose output is not passed - use a null stream
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>

#include <itkVersion.h>
#include <vtkVersion.h>

#include <tbb/tbb_stddef.h>
#include <tbb/task_scheduler_init.h>

#include <opencv2/core/version.hpp>

#include <nlopt.hpp>

// just for getting version info
#include <boost/version.hpp>
#include <Eigen/Eigen>

#include <boost/compute/system.hpp>

#include "xregAssert.h"

#include "xregStringUtils.h"
#include "xregFilesystemUtils.h"

#include "xregVersionInfo.h"

namespace
{

const char* kLONG_FLAG_PREFIX  = "--";
const char kSHORT_FLAG_PREFIX = '-';

using StorageActionStringMap = std::map<xreg::ProgOpts::StoreAction,std::string>;

StorageActionStringMap& StorageActionToStringMapFn()
{
  static bool need_to_init = true;
  static StorageActionStringMap m;

  if (need_to_init)
  {
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_TRUE, "STORE TRUE"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_FALSE, "STORE FALSE"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_STRING, "STORE STRING"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_DOUBLE, "STORE DOUBLE"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_CHAR, "STORE CHAR"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_INT32, "STORE INT32"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_UINT32, "STORE UINT32"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_INT64, "STORE INT64"));
    m.insert(StorageActionStringMap::value_type(xreg::ProgOpts::kSTORE_UINT64, "STORE UINT64"));

    need_to_init = false;
  }

  return m;
}

#define StorageActionToString StorageActionToStringMapFn()

bool IsBooleanAction(const xreg::ProgOpts::StoreAction sa)
{
  return (sa == xreg::ProgOpts::kSTORE_TRUE) || (sa == xreg::ProgOpts::kSTORE_FALSE);
}

/// \brief Get the terminal dimensions in units of characters
std::tuple<xreg::ProgOpts::size_type,xreg::ProgOpts::size_type>
GetTerminalSize()
{
  // If either of the below calls fails to obtain terminal sizes, then dimensions
  // of zero (e.g. no terminal) will be used.
  xreg::ProgOpts::size_type width = 0;
  xreg::ProgOpts::size_type height = 0;

#ifdef _WIN32
  CONSOLE_SCREEN_BUFFER_INFO csbi;
  if (GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi))
  {
    width  = static_cast<xreg::ProgOpts::size_type>(csbi.dwSize.X);
    height = static_cast<xreg::ProgOpts::size_type>(csbi.dwSize.Y);
  }
#else
  struct winsize w;
  if (-1 != ioctl(STDOUT_FILENO, TIOCGWINSZ, &w))
  {
    width  = static_cast<xreg::ProgOpts::size_type>(w.ws_col);
    height = static_cast<xreg::ProgOpts::size_type>(w.ws_row);
  }
#endif

  return std::make_tuple(width, height);
}

std::string FormatLines(const std::string& in, const xreg::ProgOpts::size_type indent,
                        const xreg::ProgOpts::size_type max_width)
{
  xregASSERT(indent < max_width);  // in order to print anything, we need to at least print 1 character per line

  using size_type = xreg::ProgOpts::size_type;

  std::ostringstream oss;

  const size_type in_len = in.size();

  size_type num_chars_remaining = in_len;

  bool needs_indent = false;

  while (num_chars_remaining)
  {
    size_type num_chars_cur_line = std::min(num_chars_remaining, max_width);

    if (needs_indent)
    {
      if ((num_chars_cur_line + indent) > max_width)
      {
        num_chars_cur_line -= num_chars_cur_line + indent - max_width;
      }

      for (size_type indent_index = 0; indent_index < indent; ++indent_index)
      {
        oss << ' ';
      }
    }
    else
    {
      needs_indent = true;
    }

    // this adds a hyphen when words span a line; makes it easier to read
    const bool needs_dash = (num_chars_cur_line > 1) && (num_chars_cur_line == max_width) &&
                            std::isalnum(in[in_len - num_chars_remaining + num_chars_cur_line - 1]);
    if (needs_dash)
    {
      --num_chars_cur_line;
    }

    oss << in.substr(in_len - num_chars_remaining, num_chars_cur_line) << (needs_dash ? "-" : "") << '\n';

    num_chars_remaining -= num_chars_cur_line;
  }

  return oss.str();
}

void WritePrettyHelp(const xreg::ProgOpts::StringList& flag_strs,
                     const xreg::ProgOpts::StringList& desc_strs,
                     std::ostream& out)
{
  using size_type = xreg::ProgOpts::size_type;

  const size_type num_flags = flag_strs.size();
  xregASSERT(num_flags == desc_strs.size());

  if (num_flags > 0)
  {
    size_type term_width  = 0;
    size_type term_height = 0;
    std::tie(term_width, term_height) = GetTerminalSize();

    size_type max_flag_str_len = flag_strs[0].size();
    for (size_type i = 1; i < num_flags; ++i)
    {
      const size_type cur_len = flag_strs[i].size();
      if (cur_len > max_flag_str_len)
      {
        max_flag_str_len = cur_len;
      }
    }

    for (size_type i = 0; i < num_flags; ++i)
    {
      std::ostringstream oss;
      oss << flag_strs[i];

      const size_type num_spaces_needed = max_flag_str_len - flag_strs[i].size();
      for (size_type space_index = 0; space_index < num_spaces_needed; ++space_index)
      {
        oss << ' ';
      }

      oss << desc_strs[i];

      // Only apply formatting when the terminal width is larger than the flag length
      // that we would indent by. We see terminal widths of zero on Google Colab, so it
      // is important to not attempt formatting when it is not reasonable to do so.
      if (max_flag_str_len < term_width)
      {
        out << FormatLines(oss.str(), max_flag_str_len, term_width);
      }
      else
      {
        out << oss.str() << std::endl;
      }
    }

    out.flush();
  }
}

const std::string kTBB_MAX_NUM_THREADS_ARG_STR = "tbb-max-threads";

/// \brief Global for storing TBB thread information.
///
/// By default, this is a null instance, so TBB defaults are used.
/// When a user specifies a number of threads, a new instance is 
/// created here. This is outside of the program options class, so that
/// a user option will still have precedence even when the program
/// options are destructed.
std::unique_ptr<tbb::task_scheduler_init>& ProgOptsTBBTaskSchedInit()
{
  static std::unique_ptr<tbb::task_scheduler_init> tbb_sched_init;

  return tbb_sched_init;
}

const std::vector<std::tuple<std::string,std::string>>&
ValidBackendNameAndDescs()
{
  static bool need_to_init = true;

  static std::vector<std::tuple<std::string,std::string>> backend_names_and_descs;
  
  if (need_to_init)
  {
    if (!boost::compute::system::platforms().empty())
    {
      backend_names_and_descs.push_back(
                std::make_tuple(std::string("ocl"),
                                std::string("OpenCL processing, usually GPU, but could be CPU.")));
    }

    backend_names_and_descs.push_back(
                std::make_tuple(std::string("cpu"),
                                std::string("Standard CPU processing, potentially using TBB.")));
    
    need_to_init = false;
  }

  return backend_names_and_descs;
}

}  // un-named

xreg::ProgOpts::StoreLogicException::StoreLogicException(const StoreAction existing_action,
                                                       const StoreAction cur_action)
{
  set_msg("Inconsistent storage action (existing: %s, cur: %s)",
          StorageActionToString[existing_action].c_str(),
          StorageActionToString[cur_action].c_str());
}

xreg::ProgOpts::ArgVal::ArgVal(const bool x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const double x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const char x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const int32 x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const uint32 x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const int64 x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const uint64 x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const std::string& x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal::ArgVal(const char* x)
{
  set_default(x);
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const bool x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const double x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const char x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const int32 x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const uint32 x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const int64 x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const uint64 x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const std::string& x)
{
  set_default(x);
  return *this;
}

xreg::ProgOpts::ArgVal& xreg::ProgOpts::ArgVal::operator=(const char* x)
{
  set_default(x);
  return *this;
}

void xreg::ProgOpts::ArgVal::set_default(const bool x)
{
  is_str = false;
  prim_vals.b = x;
}

void xreg::ProgOpts::ArgVal::set_default(const double x)
{
  is_str = false;
  prim_vals.d = x;
}

void xreg::ProgOpts::ArgVal::set_default(const char x)
{
  is_str = false;
  prim_vals.c = x;
}

void xreg::ProgOpts::ArgVal::set_default(const int32 x)
{
  is_str = false;
  prim_vals.i32 = x;
}

void xreg::ProgOpts::ArgVal::set_default(const uint32 x)
{
  is_str = false;
  prim_vals.ui32 = x;
}

void xreg::ProgOpts::ArgVal::set_default(const int64 x)
{
  is_str = false;
  prim_vals.i64 = x;
}

void xreg::ProgOpts::ArgVal::set_default(const uint64 x)
{
  is_str = false;
  prim_vals.ui64 = x;
}

void xreg::ProgOpts::ArgVal::set_default(const std::string& x)
{
  is_str = true;
  str_val = x;
}

void xreg::ProgOpts::ArgVal::set_default(const char* x)
{
  is_str = true;
  str_val = x;
}

xreg::ProgOpts::ArgVal::operator bool() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to bool");
  }

  return prim_vals.b;
}

xreg::ProgOpts::ArgVal::operator double() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to double");
  }

  return prim_vals.d;
}

xreg::ProgOpts::ArgVal::operator char() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to char");
  }

  return prim_vals.c;
}

xreg::ProgOpts::ArgVal::operator xreg::ProgOpts::int32() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to int32");
  }

  return prim_vals.i32;
}

xreg::ProgOpts::ArgVal::operator xreg::ProgOpts::uint32() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to uint32");
  }

  return prim_vals.ui32;
}

xreg::ProgOpts::ArgVal::operator xreg::ProgOpts::int64() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to int64");
  }

  return prim_vals.i64;
}

xreg::ProgOpts::ArgVal::operator xreg::ProgOpts::uint64() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to uint64");
  }

  return prim_vals.ui64;
}

xreg::ProgOpts::ArgVal::operator std::string() const
{
  if (!is_str)
  {
    throw InvalidConversionException("Converting non-string to string (std::string)");
  }

  return str_val;
}

xreg::ProgOpts::ArgVal::operator const char*() const
{
  if (!is_str)
  {
    throw InvalidConversionException("Converting non-string to string (char*)");
  }

  return str_val.c_str();
}

bool xreg::ProgOpts::ArgVal::as_bool() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to bool");
  }

  return prim_vals.b;
}

double xreg::ProgOpts::ArgVal::as_double() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to double");
  }

  return prim_vals.d;
}

char xreg::ProgOpts::ArgVal::as_char() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to char");
  }

  return prim_vals.c;
}

xreg::ProgOpts::int32 xreg::ProgOpts::ArgVal::as_int32() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to int32");
  }

  return prim_vals.i32;
}

xreg::ProgOpts::uint32 xreg::ProgOpts::ArgVal::as_uint32() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to uint32");
  }

  return prim_vals.ui32;
}

xreg::ProgOpts::int64 xreg::ProgOpts::ArgVal::as_int64() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to int64");
  }

  return prim_vals.i64;
}

xreg::ProgOpts::uint64 xreg::ProgOpts::ArgVal::as_uint64() const
{
  if (is_str)
  {
    throw InvalidConversionException("Converting string to uint64");
  }

  return prim_vals.ui64;
}

std::string xreg::ProgOpts::ArgVal::as_string() const
{
  if (!is_str)
  {
    throw InvalidConversionException("Converting non-string to string (std::string)");
  }

  return str_val;
}

xreg::ProgOpts::Arg::Arg()
{
  //long_flag = "";
  short_flag = '\0';

  required = false;

  num_vals = 1;

  set_default = false;
  default_vals.resize(num_vals);
  cur_default_val_index = 0;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::nvals(const size_type nv)
{
  num_vals = nv;
  default_vals.resize(num_vals);

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const bool x)
{
  if ((action != kSTORE_TRUE) && (action != kSTORE_FALSE))
  {
    throw StoreLogicException(action, x ? kSTORE_TRUE : kSTORE_FALSE);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const double x)
{
  if (action != kSTORE_DOUBLE)
  {
    throw StoreLogicException(action, kSTORE_DOUBLE);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const char x)
{
  if (action != kSTORE_CHAR)
  {
    throw StoreLogicException(action, kSTORE_CHAR);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;


  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const int32 x)
{
  if (action != kSTORE_INT32)
  {
    throw StoreLogicException(action, kSTORE_INT32);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const uint32 x)
{
  if (action != kSTORE_UINT32)
  {
    throw StoreLogicException(action, kSTORE_UINT32);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const int64 x)
{
  if (action != kSTORE_INT64)
  {
    throw StoreLogicException(action, kSTORE_INT64);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const uint64 x)
{
  if (action != kSTORE_UINT64)
  {
    throw StoreLogicException(action, kSTORE_UINT64);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const std::string& x)
{
  if (action != kSTORE_STRING)
  {
    throw StoreLogicException(action, kSTORE_STRING);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

xreg::ProgOpts::Arg& xreg::ProgOpts::Arg::operator<<(const char* x)
{
  if (action != kSTORE_STRING)
  {
    throw StoreLogicException(action, kSTORE_STRING);
  }

  set_default = true;
  if (cur_default_val_index == default_vals.size())
  {
    throw TooManyDefaultValsException(long_flag.c_str(), short_flag, dest.c_str(), default_vals.size());
  }
  default_vals[cur_default_val_index] = x;
  ++cur_default_val_index;

  return *this;
}

const std::string xreg::ProgOpts::kNO_LONG_FLAG = "";

std::string xreg::ProgOpts::Readable(const StoreAction a, const ArgVal& val)
{
  std::string to_ret;

  if ((a == kSTORE_TRUE) || (a == kSTORE_FALSE))
  {
    xregASSERT(!val.is_str);
    to_ret = val.prim_vals.b ? "true" : "false";
  }
  else if (a == kSTORE_STRING)
  {
    xregASSERT(val.is_str);
    to_ret.push_back('\"');
    to_ret.append(val.str_val);
    to_ret.push_back('\"');
  }
  else if (a == kSTORE_CHAR)
  {
    xregASSERT(!val.is_str);
    to_ret.push_back('\'');
    to_ret.push_back(val.prim_vals.c);
    to_ret.push_back('\'');
  }
  else
  {
    xregASSERT(!val.is_str);

    std::stringstream ss;

    if (a == kSTORE_DOUBLE)
    {
      ss << val.prim_vals.d;
    }
    else if (a == kSTORE_INT32)
    {
      ss << val.prim_vals.i32;
    }
    else if (a == kSTORE_UINT32)
    {
      ss << val.prim_vals.ui32;
    }
    else if (a == kSTORE_INT64)
    {
      ss << val.prim_vals.i64;
    }
    else if (a == kSTORE_UINT64)
    {
      ss << val.prim_vals.ui64;
    }
    else
    {
      xregThrow("Unsupported storage action!");
    }

    to_ret = ss.str();
  }

  return to_ret;
}

std::string xreg::ProgOpts::Readable(const StoreAction a, const ArgValList& vals)
{
  std::stringstream ss;

  const size_type num_vals = vals.size();

  if (num_vals > 1)
  {
    for (size_type val_index = 0; val_index < (num_vals - 1); ++val_index)
    {
      ss << Readable(a, vals[val_index]) << " , ";
    }
  }

  ss << Readable(a, vals[num_vals - 1]);

  return ss.str();
}

xreg::ProgOpts::ProgOpts()
{
  version_num_str_ = ProjVerStrAndGitSHA1IfAvail();

  opencl_id_str_to_dev_map_ = BuildDevIDStrsToDevMap();
}

void xreg::ProgOpts::print_help(std::ostream& out) const
{
  // print out the leading help string if it is available
  if (!help_str_.empty())
  {
    out << help_str_ << std::endl;
  }

  // when printing the argument flags, keep track of their lengths so we can
  // offset the help messages to all line up.
  // e.g. we want something like this:
  // --help,-h             Prints help message
  // --alg,-a              Algorithm to use
  // --ultimate-param,-u   The ultimate parameter
  //                       ^
  //                    consistent alignment

  const char* kFLAG_HELP_SPACER = "   ";

  StringList flag_strs;
  StringList msg_strs;

  // add the user-specified help "only" flag if specified by the user
  const bool has_long_help_flag  = !help_long_flag_.empty();
  const bool has_short_help_flag = help_short_flag_ != '\0';
  std::string help_flags_str;

  if (has_long_help_flag || has_short_help_flag)
  {
    std::ostringstream oss;
    if (has_long_help_flag)
    {
      oss << kLONG_FLAG_PREFIX << help_long_flag_;
    }
    if (has_short_help_flag)
    {
      if (has_long_help_flag)
      {
        oss << ',';
      }
      oss << kSHORT_FLAG_PREFIX << help_short_flag_;
    }
    oss << kFLAG_HELP_SPACER;

    help_flags_str = oss.str();
    flag_strs.push_back(help_flags_str);
  }

  // only do this if there are arguments available/specified
  if (!help_flags_str.empty() || !args_.empty())
  {
    out << "Arguments:" << std::endl;

    // First compute the "--long, -s" strings, saving them off along with their
    // lengths
    for (ArgList::const_iterator it = args_.begin(); it != args_.end(); ++it)
    {
      std::ostringstream oss;

      const bool has_long_flag = !it->long_flag.empty();

      if (has_long_flag)
      {
        oss << kLONG_FLAG_PREFIX << it->long_flag;
      }

      if (it->short_flag != '\0')
      {
        if (has_long_flag)
        {
          oss << ',';
        }
        oss << kSHORT_FLAG_PREFIX << it->short_flag;
      }

      // Including the indentation string between the flags and help message
      oss << kFLAG_HELP_SPACER;

      flag_strs.push_back(oss.str());
    }

    msg_strs.reserve(flag_strs.size());

    // help message for the built in help flags if they are available
    if (!help_flags_str.empty())
    {
      msg_strs.push_back("Prints this help message");
    }

    // the help message plus an explanation of the default value
    for (ArgList::const_iterator it = args_.begin(); it != args_.end(); ++it)
    {
      std::ostringstream oss;

      oss << it->help_str;
      if (it->set_default)
      {
        oss << " (default: " << Readable(it->action, it->default_vals) << " )";
      }

      msg_strs.push_back(oss.str());
    }

    // write everything out with nice indenting and respecting the terminal size
    WritePrettyHelp(flag_strs, msg_strs, out);
  }

  // print out a help epilogue if specified by the user
  if (!help_epi_str_.empty())
  {
    out << help_epi_str_ << std::endl;
  }

  if (print_help_ocl_str_)
  {
    const size_type num_dev = opencl_id_str_to_dev_map_.size();

    out << '\n' << num_dev << " Available OpenCL Devices (#: ID, Vender, Name):\n";

    size_type dev_idx = 0;
    for (const auto& id_dev_map : opencl_id_str_to_dev_map_)
    {
      const boost::compute::device& dev = id_dev_map.second;

      out << "  " << (dev_idx + 1) << ". " << id_dev_map.first << ", " << dev.vendor()
          << ", " << dev.name() << std::endl;
      
      ++dev_idx;
    }
  }

  if (print_help_backend_str_)
  {
    const auto& valid_backends = ValidBackendNameAndDescs();
   
    const size_type num_backends = valid_backends.size();

    out << '\n' << num_backends << " compute backends are available:\n";

    for (size_type backend_idx = 0; backend_idx < num_backends; ++backend_idx)
    {
      const auto& cur_backend_info = valid_backends[backend_idx];

      out << "  " << (backend_idx+1) << ". " << std::get<0>(cur_backend_info)
          << ": " << std::get<1>(cur_backend_info) << std::endl;
    }
  }

  out << "\nVERSION INFORMATION:\n";

  if (!version_num_str_.empty())
  {
    out << "   xReg Version: " << version_num_str_ << std::endl;
  }

  out << "  Boost Version: " << (BOOST_VERSION / 100000) << '.' << ((BOOST_VERSION / 100) % 1000) << '.' << (BOOST_VERSION % 100) << std::endl;

  out << "  Eigen Version: " << EIGEN_WORLD_VERSION << '.' << EIGEN_MAJOR_VERSION
      << '.' << EIGEN_MINOR_VERSION << std::endl;

  out << "    ITK Version: " << itk::Version::GetITKVersion() << std::endl;

  out << "    VTK Version: " << vtkVersion::GetVTKVersion() << std::endl;

  out << "    TBB Version: " << TBB_VERSION_MAJOR << '.' << TBB_VERSION_MINOR
      << " (interface: "<< TBB_INTERFACE_VERSION_MAJOR << '.' << (TBB_INTERFACE_VERSION - (TBB_INTERFACE_VERSION_MAJOR * 1000)) << ')' << std::endl;

  out << " OpenCV Version: " << CV_MAJOR_VERSION << '.' << CV_MINOR_VERSION << std::endl;

  {
    int major  = 0;
    int minor  = 0;
    int bugfix = 0;
    nlopt_version(&major, &minor, &bugfix);

    out << "  NLopt Version: " << major << '.' << minor << '.' << bugfix << std::endl;
  }

  {
    const auto plats = boost::compute::system::platforms();

    for (const auto& plat : plats)
    {
      out << "OpenCL Platform: " << plat.vendor() << " " << plat.version() << std::endl;
    }
  }

  if (!compile_date_.empty())
  {
    out << "Compiled " << compile_date_ << std::endl;
  }
}

xreg::ProgOpts::Arg& xreg::ProgOpts::add(const Arg& arg)
{
  args_.push_back(arg);
  return args_.back();
}

xreg::ProgOpts::Arg& xreg::ProgOpts::add(const std::string& long_flag,
                                         const char short_flag,
                                         const StoreAction action,
                                         const std::string& dest,
                                         const std::string& help_str)
{
  Arg a;
  a.long_flag   = long_flag;
  a.short_flag  = short_flag;
  a.required    = false;
  if ((action == kSTORE_TRUE) || (action == kSTORE_FALSE))
  {
    // boolean flags should not have any additional arguments
    a.num_vals = 0;
    a.default_vals.resize(1);
  }
  else
  {
    //a.max_vals = ~size_type(0);
    // for now default to one argument
    a.num_vals = 1;
    a.default_vals.resize(a.num_vals);
  }
  a.dest        = dest;
  a.set_default = false;
  a.help_str    = help_str;
  a.action      = action;

  args_.push_back(a);

  return args_.back();
}

void xreg::ProgOpts::parse(int argc, char* argv[])
{
  prog_name_ = xreg::GetFileName(argv[0]);
  
  if (add_verbose_flag_)
  {
    add(verbose_long_flag_, verbose_short_flag_, ProgOpts::kSTORE_TRUE, verbose_long_flag_,
        "Print verbose information to stdout.")
      << false;
  }

  // reset any previously parsed values to the defaults, will also set help_was_set_ to false
  set_defaults();

  const size_type num_args = static_cast<size_type>(argc);

  // look for just the help flags, before we possibly throw an exception; this may
  // set help_was_set_ to true
  find_help(num_args - 1, argv + 1);

  // if the help flag has been passed by the user skip any other parsing.
  if (!help_was_set_)
  {
    // mappings from possible flag strings to argument structs
    FlagArgMap long_flag_arg_map;
    FlagArgMap short_flag_arg_map;

    build_flag_arg_map(&long_flag_arg_map, &short_flag_arg_map);

    // for each argument on the command line
    for (size_type arg_index = 1; arg_index < num_args; ++arg_index)
    {
      bool is_pos_arg = false;
      bool is_unrec_flag = true;

      FlagArgMap::iterator flag_it;

      // first determine if this entry could represent a flag or a positional
      // argument.
      std::string long_flag;
      std::string short_flag;
      get_flag_str(argv[arg_index], &long_flag, &short_flag);

      if (!long_flag.empty())
      {
        // this is a long flag, e.g. --something
        flag_it = long_flag_arg_map.find(long_flag);
        is_unrec_flag = flag_it == long_flag_arg_map.end();
      }
      else if (!short_flag.empty())
      {
        // this is a short flag, e.g. -s
        flag_it = short_flag_arg_map.find(short_flag);
        is_unrec_flag = flag_it == short_flag_arg_map.end();
      }
      else
      {
        // must be a positional argument
        is_pos_arg = true;
      }

      // if it's still possible for this to be a flag
      if (!is_pos_arg)
      {
        if (!is_unrec_flag)
        {
          // We recognize this flag string

          const size_type num_vals = flag_it->second->num_vals;
          const StoreAction store_action = flag_it->second->action;

          const std::string& dest_str = flag_it->second->dest;

          DestValueMap::iterator val_it = arg_value_map_.find(dest_str);

          if (val_it != arg_value_map_.end())
          {
            if (!val_it->second.defaulted)
            {
              // The destination already has a value assigned to it and it was
              // NOT the default value, therefore another argument has assigned a
              // value here; this is not supported behavior
              StoreLogicException e;
              e.set_msg("Value has already been assigned; dest: %s, flag: %s; Did you pass a flag more than once?",
                        dest_str.c_str(),
                        !long_flag.empty() ? long_flag.c_str() : short_flag.c_str());
              throw e;
            }
            else if (!IsBooleanAction(val_it->second.action) &&
                     !IsBooleanAction(store_action) &&
                     (val_it->second.action != store_action))
            {
              // The storage action that is associated with this default value is
              // not consistent with the storage action of the current flag
              StoreLogicException e;
              e.set_msg("Inconsistent storage value; dest: %s, flag: %s, dest action: %s, cur action: %s",
                        dest_str.c_str(),
                        !long_flag.empty() ? long_flag.c_str() : short_flag.c_str(),
                        StorageActionToString[val_it->second.action].c_str(),
                        StorageActionToString[store_action].c_str());
              throw e;
            }
            else
            {
              // A default value had previously been set, but is the user provided value now overrides the default
              val_it->second.defaulted = false;
            }
          }
          else
          {
            // The destination does not have a value, let's make an entry
            ParsedValue x;
            x.defaulted = false;
            x.action = store_action;

            if ((store_action == kSTORE_TRUE) || (store_action == kSTORE_FALSE))
            {
              // ensure that we always have 1 field - otherwise it MAY be the case that num_vals is set to 0
              // and the following line will cause a segfault:
              //    dst_val.vals[0] = true|false;
              x.vals.resize(1);
            }
            else
            {
              x.vals.resize(num_vals);
            }

            val_it = arg_value_map_.insert(DestValueMap::value_type(dest_str, x)).first;
          }

          ParsedValue& dst_val = val_it->second;

          if (store_action == kSTORE_TRUE)
          {
            dst_val.vals[0] = true;
          }
          else if (store_action == kSTORE_FALSE)
          {
            dst_val.vals[0] = false;
          }
          else
          {
            // Lookup the arguments in subsequent entries in the provided argv

            size_type tmp_arg_index = arg_index + 1;
            size_type val_index = 0;
            for (; (val_index < num_vals) && (tmp_arg_index < num_args);
                 ++tmp_arg_index, ++val_index)
            {
              switch (store_action)
              {
              case kSTORE_TRUE:
              case kSTORE_FALSE:
                // should not actually get here (at least right now, since bool
                // switches have no additional arguments)
                dst_val.vals[val_index] = StringCast<bool>(argv[tmp_arg_index]);
                break;
              case kSTORE_DOUBLE:
                dst_val.vals[val_index] = StringCast<double>(argv[tmp_arg_index]);
                break;
              case kSTORE_CHAR:
                dst_val.vals[val_index] = StringCast<char>(argv[tmp_arg_index]);
                break;
              case kSTORE_INT32:
                dst_val.vals[val_index] = StringCast<int32>(argv[tmp_arg_index]);
                break;
              case kSTORE_UINT32:
                dst_val.vals[val_index] = StringCast<uint32>(argv[tmp_arg_index]);
                break;
              case kSTORE_INT64:
                dst_val.vals[val_index] = StringCast<int64>(argv[tmp_arg_index]);
                break;
              case kSTORE_UINT64:
                dst_val.vals[val_index] = StringCast<uint64>(argv[tmp_arg_index]);
                break;
              case kSTORE_STRING:
              default:
                dst_val.vals[val_index] = argv[tmp_arg_index];
                break;
              }
            }

            if (val_index != num_vals)
            {
              // we ran out of arguments on the command line
              NotEnoughArgsException e;
              e.set_msg("Not enough values for %s (dest: %s); had %lu, need %lu",
                        flag_it->first.c_str(), dest_str.c_str(), val_index, num_vals);

              throw e;
            }

            arg_index = tmp_arg_index - 1;  // minus 1 to account for the increment in the outer for loop
          }
        }
        else if (!allow_unrec_flags_as_args_)
        {
          // we do not recognize this flag string, and we do not support
          // unrecognized flags
          throw UnrecognizedFlagException(!long_flag.empty() ? long_flag.c_str() : short_flag.c_str());
        }
        else
        {
          // we do not recognize this flag string, but we will support
          // unrecognized flags as positional arguments; the major use for this
          // is to allow negative numbers as a positional argument, e.g.
          // "add -5 6 --print" where -5 is the first positional argument,
          // 6 is the second, and print is a flag
          is_pos_arg = true;
        }
      }

      if (is_pos_arg)
      {
        // we know for certain that this entry is a non-flag (positional) argument
        pos_args_.push_back(argv[arg_index]);
      }
    }

    if (pos_args_.size() < min_num_pos_args_)
    {
      NotEnoughArgsException e;
      e.set_msg("Not enough positional arguments; %lu provided, need %lu",
                pos_args_.size(), min_num_pos_args_);

      throw e;
    }
  }

  if (tbb_max_num_threads_opt_added_ && has(kTBB_MAX_NUM_THREADS_ARG_STR))
  {
    ProgOptsTBBTaskSchedInit().reset(
        new tbb::task_scheduler_init(get(kTBB_MAX_NUM_THREADS_ARG_STR).as_uint32()));
  }

  if (print_help_backend_str_ && has("backend"))
  {
    // Backend specification is enabled, check that the passed string is valid.

    const std::string backend_str = get("backend");
    
    bool backend_is_valid = false;

    const auto& backend_info = ValidBackendNameAndDescs();

    for (const auto& cur_info : backend_info)
    {
      if (std::get<0>(cur_info) == backend_str)
      {
        backend_is_valid = true;
        break;
      }
    }
    
    if (!backend_is_valid)
    {
      xregThrow("Invalid backend identifier string: %s", backend_str.c_str());
    } 
  }
}

bool xreg::ProgOpts::has(const std::string& opt_name) const
{
  return arg_value_map_.find(opt_name) != arg_value_map_.end();
}

void xreg::ProgOpts::build_flag_arg_map(FlagArgMap* long_flag_arg_map,
                                      FlagArgMap* short_flag_arg_map)
{
  // return type for inserts into a FlagArgMap, the bool second component
  // indicates if the value was actually inserted
  using FlagArgInsertVal = std::pair<FlagArgMap::iterator,bool>;

  // temporary string for a single character flag, need to represent as a string
  // for representation in the map
  std::string tmp_short_flag(1, ' ');

  // for each argument specified by the user
  for (auto it = args_.begin(); it != args_.end(); ++it)
  {
    // if a long flag is specified, try to insert a mapping entry
    if (it->long_flag != kNO_LONG_FLAG)
    {
      FlagArgInsertVal flag_ins = long_flag_arg_map->insert(FlagArgMap::value_type(it->long_flag, &*it));
      if (!flag_ins.second)
      {
        // This flag string already has a mapping, indicates that this flag
        // string is not unique since it must be from another argument; this
        // behavior is user error
        DuplicateArgumentException e;
        e.set_msg("Flag already registered; flag: %s, cur dest: %s, existing dest: %s",
                  it->long_flag.c_str(), it->dest.c_str(), flag_ins.first->second->dest.c_str());
        throw e;
      }
    }

    // if a short flag is specified, try to insert a mapping entry
    if (it->short_flag != kNO_SHORT_FLAG)
    {
      // copy char to string
      tmp_short_flag[0] = it->short_flag;
      FlagArgInsertVal flag_ins = short_flag_arg_map->insert(FlagArgMap::value_type(tmp_short_flag, &*it));
      if (!flag_ins.second)
      {
        // see above comment about throwing for the long flag case
        DuplicateArgumentException e;
        e.set_msg("Flag already registered; flag: %c, cur dest: %s, existing dest: %s",
                  it->short_flag, it->dest.c_str(), flag_ins.first->second->dest.c_str());
        throw e;
      }
    }
  }
}

void xreg::ProgOpts::set_defaults()
{
  help_was_set_ = false;

  arg_value_map_.clear();

  for (ArgList::iterator it = args_.begin(); it != args_.end(); ++it)
  {
    if (it->set_default)
    {
      ParsedValue x;
      x.defaulted = true;
      x.vals = it->default_vals;
      x.action = it->action;

      if (!arg_value_map_.insert(DestValueMap::value_type(it->dest,x)).second)
      {
        // another argument has already specified a default value for this
        // destination, this is not allowed.
        DuplicateArgumentException e;
        e.set_msg("Default value already set for dest (%s); cur flags %s,%c",
                  it->dest.c_str(),
                  (it->long_flag != kNO_LONG_FLAG) ? it->long_flag.c_str() : "",
                  (it->short_flag != kNO_SHORT_FLAG) ? it->short_flag : ' ');
        throw e;
      }
    }
  }
}

void xreg::ProgOpts::set_arg_usage(const std::string& arg_usage_str)
{
  arg_usage_str_ = arg_usage_str;
}

void xreg::ProgOpts::print_usage(std::ostream& out)
{
  out << "Usage " << prog_name_;
  if (!args_.empty())
  {
    out << " [options]";
  }
  if (!arg_usage_str_.empty())
  {
    out << ' ' << arg_usage_str_;
  }
  out << std::endl;
}

bool xreg::ProgOpts::val_defaulted(const std::string& opt_name) const
{
  DestValueMap::const_iterator it = arg_value_map_.find(opt_name);
  return (it != arg_value_map_.end()) && it->second.defaulted;
}

const xreg::ProgOpts::ArgVal& xreg::ProgOpts::get_val(const std::string& opt_name) const
{
  DestValueMap::const_iterator it = arg_value_map_.find(opt_name);
  if (it == arg_value_map_.end())
  {
    throw ValueNotSetException(opt_name.c_str());
  }

  return it->second.vals[0];
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, BoolList* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, CharList* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}


void xreg::ProgOpts::get_vals(const std::string& opt_name, DoubleList* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, Int32List* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, UInt32List* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, Int64List* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, UInt64List* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::get_vals(const std::string& opt_name, StringList* vals)
{
  vals->clear();
  get_vals(opt_name, std::back_inserter(*vals));
}

void xreg::ProgOpts::find_help(const size_type num_args, char* argv[])
{
  if (!help_long_flag_.empty() || (help_short_flag_ != '\0'))
  {
    const char tmp_help_short_str[2] = { help_short_flag_, '\0' };

    std::string long_flag;
    std::string short_flag;

    for (size_type i = 0; i < num_args; ++i)
    {
      get_flag_str(argv[i], &long_flag, &short_flag);
      if ((long_flag == help_long_flag_) || (short_flag == tmp_help_short_str))
      {
        help_was_set_ = true;
        break;
      }
    }
  }
}

void xreg::ProgOpts::get_flag_str(const std::string& arg_str,
                                std::string* long_flag, std::string* short_flag) const
{
  if ((arg_str.size() > 2) && (arg_str.substr(0,2) == kLONG_FLAG_PREFIX))
  {
    // this is a long flag, e.g. --something
    *long_flag  = arg_str.substr(2);
    *short_flag = "";
  }
  else if ((arg_str.size() > 1) && (arg_str[0] == kSHORT_FLAG_PREFIX))
  {
    // this is a short flag, e.g. -s
    *long_flag = "";
    *short_flag = arg_str.substr(1);
  }
  else
  {
    // must be a positional argument
    *long_flag = "";
    *short_flag = "";
  }
}

void xreg::ProgOpts::set_help_flags(const std::string& help_long, const char help_short)
{
  help_long_flag_  = help_long;
  help_short_flag_ = help_short;
}

void xreg::ProgOpts::set_add_verbose_flag(const bool add_verbose_flag)
{
  add_verbose_flag_ = add_verbose_flag;
}

void xreg::ProgOpts::set_verbose_flags(const std::string& verbose_long, const char verbose_short)
{
  verbose_long_flag_  = verbose_long;
  verbose_short_flag_ = verbose_short;
}
 
std::ostream& xreg::ProgOpts::vout()
{
  if (!add_verbose_flag_)
  {
    xregThrow("VERBOSE FLAG NOT AUTOMATICALLY ADDED TO PROG OPTS, CANNOT GET VERBOSE OSTREAM!");
  }
 
  static boost::iostreams::stream<boost::iostreams::null_sink> null_ostream((boost::iostreams::null_sink()));

  return get(verbose_long_flag_).as_bool() ? std::cout : null_ostream;
}

void xreg::ProgOpts::add_tbb_max_num_threads_flag()
{
  add(kTBB_MAX_NUM_THREADS_ARG_STR, ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_UINT32,
      kTBB_MAX_NUM_THREADS_ARG_STR,
      "Specify the maximum number of threads to use with the Intel TBB library. "
      "When not specified, this uses TBB default, which is typically the maximum "
      "number of processing cores available. "
      "Not sure what will happen if other code also attempts to control this number.");
  
  tbb_max_num_threads_opt_added_ = true; 
}

void xreg::ProgOpts::add_backend_flags()
{
  add_ocl_select_flag();

  const auto& valid_backends = ValidBackendNameAndDescs();

  std::stringstream ss;
  
  ss << "Specify the compute backend to use. Valid backends are: ";

  const size_type num_backends = valid_backends.size();

  for (size_type backend_idx = 0; backend_idx < num_backends; ++backend_idx)
  {
    const auto& cur_name = std::get<0>(valid_backends[backend_idx]);

    ss << '\"' << cur_name << '\"' << ((backend_idx == (num_backends - 1)) ? ". " : ", ");
  }

  ss << "See the epilogue of this help message for descriptions of each backend.";

  add("backend", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "backend", ss.str())
    << std::get<0>(valid_backends[0]);

  print_help_backend_str_ = true;
}

void xreg::ProgOpts::add_ocl_select_flag()
{
  set_print_help_ocl_str(true);

  add("ocl-id", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "ocl-id",
      "Specify the OpenCL device to use with a unique identifier string - the available device ID strings "
      "may be obtained with the help print-out. The default behavior is to use the default "
      "device specified by the boost::compute library, which may not be constant (e.g. it "
      "may vary depending on system resources, etc.).")
    << "";
}

boost::compute::device xreg::ProgOpts::selected_ocl()
{
  if (!selected_ocl_dev_.id())
  {
    const std::string ocl_id = get("ocl-id").as_string();
    
    if (!ocl_id.empty())
    {
      auto id_dev_it = opencl_id_str_to_dev_map_.find(ocl_id);
      if (id_dev_it != opencl_id_str_to_dev_map_.end())
      {
        selected_ocl_dev_ = id_dev_it->second;
      }
      else
      {
        xregThrow("Invalid OpenCL Device ID String: %s", ocl_id.c_str());
      }
    }
    else
    {
      try
      {
        selected_ocl_dev_ = boost::compute::system::default_device();
      }
      catch (std::exception& e)
      {
        std::cerr << "Failed to create default device; "
                     "exception message: " << e.what()
                  << "\n\nIf you do not have any OpenCL devices, "
                     "try using a CPU only flag."
                  << std::endl;
        throw;
      }
    }
  }

  return selected_ocl_dev_;
}
  
std::tuple<boost::compute::context,boost::compute::command_queue>
xreg::ProgOpts::selected_ocl_ctx_queue()
{
  if (!selected_ocl_ctx_queue_set_)
  {
    boost::compute::device dev = selected_ocl();

    selected_ocl_ctx_   = boost::compute::context(dev);
    selected_ocl_queue_ = boost::compute::command_queue(selected_ocl_ctx_, dev);
    
    selected_ocl_ctx_queue_set_ = true;
  }

  return std::make_tuple(selected_ocl_ctx_, selected_ocl_queue_);
}

bool xreg::ProgOpts::dest_exists(const std::string& dest_str) const
{
  bool dst_exists = false;

  for (ArgList::const_iterator arg_it = args_.begin(); (arg_it != args_.end()) && !dst_exists; ++arg_it)
  {
    dst_exists = arg_it->dest == dest_str;
  }

  return dst_exists;
}
  
void xreg::ProgOpts::set_help(const std::string& help_str)
{
  help_str_ = help_str;
}

void xreg::ProgOpts::set_help_epilogue(const std::string& help_str)
{
  help_epi_str_ = help_str;
}
  
void xreg::ProgOpts::set_min_num_pos_args(const size_type n)
{
  min_num_pos_args_ = n;
}

const xreg::ProgOpts::StringList& xreg::ProgOpts::pos_args() const
{
  return pos_args_;
}

  
const xreg::ProgOpts::ArgVal& xreg::ProgOpts::get(const std::string& opt_name) const
{
  return get_val(opt_name);
}

bool xreg::ProgOpts::help_set() const
{
  return help_was_set_;
}

void xreg::ProgOpts::set_allow_unrecognized_flags(const bool allow)
{
  allow_unrec_flags_as_args_ = allow;
}

const std::string& xreg::ProgOpts::compile_date() const
{
  return compile_date_;
}

void xreg::ProgOpts::set_compile_date(const std::string& s)
{
  compile_date_ = s;
}

const std::string& xreg::ProgOpts::version_num_str() const
{
  return version_num_str_;
}

void xreg::ProgOpts::set_version_num_str(const std::string& s)
{
  version_num_str_ = s;
}

void xreg::ProgOpts::set_print_help_ocl_str(const bool print_ocl)
{
  print_help_ocl_str_ = print_ocl;
}
