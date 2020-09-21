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

/**
 * @file
 * @brief Utilities for parsing command line arguments.
 **/

#ifndef XREGPROGOPTUTILS_H_
#define XREGPROGOPTUTILS_H_

#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

#include <boost/compute/command_queue.hpp>

#include "xregOpenCLSys.h"
#include "xregExceptionUtils.h"
#include "xregSizedTypeUtils.h"

namespace xreg
{

class ProgOpts
{
public:
  using size_type = std::size_t;

  static const std::string kNO_LONG_FLAG;  // set to empty string, defined in .cpp
  static const char        kNO_SHORT_FLAG = '\0';

  using int32  = sized_types::int32;
  using uint32 = sized_types::uint32;
  using int64  = sized_types::int64;
  using uint64 = sized_types::uint64;

  using BoolList   = std::vector<bool>;
  using CharList   = std::vector<char>;
  using DoubleList = std::vector<double>;
  using Int32List  = std::vector<int32>;
  using UInt32List = std::vector<uint32>;
  using Int64List  = std::vector<int64>;
  using UInt64List = std::vector<uint64>;
  using StringList = std::vector<std::string>;

  enum StoreAction
  {
    kSTORE_TRUE = 0,
    kSTORE_FALSE,
    kSTORE_STRING,
    kSTORE_DOUBLE,
    kSTORE_CHAR,
    kSTORE_INT32,
    kSTORE_UINT32,
    kSTORE_INT64,
    kSTORE_UINT64
  };

  class Exception : public StringMessageException { };

  class StoreLogicException : public Exception
  {
  public:
    StoreLogicException(const StoreAction existing_action, const StoreAction cur_action);

    StoreLogicException() { }

    virtual ~StoreLogicException() throw() { }
  };

  class DuplicateArgumentException : public Exception { };

  class UnrecognizedFlagException : public Exception
  {
  public:
    UnrecognizedFlagException(const char* flag)
    {
      set_msg("Unrecognized flag: %s", flag);
    }

    virtual ~UnrecognizedFlagException() throw() { }
  };

  class TooManyDefaultValsException : public Exception
  {
  public:
    TooManyDefaultValsException(const char* long_flag, const char short_flag,
                                const char* dest, const size_type max_vals)
    {
      set_msg("Too many values input as default for flags (%s,%c), dest (%s), max vals (%lu)",
              (long_flag != kNO_LONG_FLAG) ? long_flag : "",
              (short_flag != kNO_SHORT_FLAG) ? short_flag : ' ',
              dest, max_vals);
    }

    virtual ~TooManyDefaultValsException() throw() { }
  };

  class NotEnoughArgsException : public Exception { };

  class ValueNotSetException : public Exception
  {
  public:
    ValueNotSetException(const char* opt_name)
    {
      set_msg("Destination value not set: %s", opt_name);
    }

    virtual ~ValueNotSetException() throw() { }
  };

  class InvalidConversionException : public Exception
  {
  public:
    InvalidConversionException(const char* msg)
    {
      set_msg(msg);
    }

    virtual ~InvalidConversionException() throw() { }
  };

  // For now do not change this to use boost/std::variant.
  // This change would result in too much refactoring and may increase compile times.
  struct ArgVal
  {
    union ArgPrimValue
    {
      bool b;
      double d;
      char c;
      int32 i32;
      uint32 ui32;
      int64 i64;
      uint64 ui64;
    } prim_vals;

    std::string str_val;

    bool is_str;

    ArgVal() { }

    ArgVal(const bool x);

    ArgVal(const double x);

    ArgVal(const char x);

    ArgVal(const int32 x);

    ArgVal(const uint32 x);

    ArgVal(const int64 x);

    ArgVal(const uint64 x);

    ArgVal(const std::string& x);

    ArgVal(const char* x);

    ArgVal& operator=(const bool x);

    ArgVal& operator=(const double x);

    ArgVal& operator=(const char x);

    ArgVal& operator=(const int32 x);

    ArgVal& operator=(const uint32 x);

    ArgVal& operator=(const int64 x);

    ArgVal& operator=(const uint64 x);

    ArgVal& operator=(const std::string& x);

    ArgVal& operator=(const char* x);

    void set_default(const bool x);

    void set_default(const double x);

    void set_default(const char x);

    void set_default(const int32 x);

    void set_default(const uint32 x);

    void set_default(const int64 x);

    void set_default(const uint64 x);

    void set_default(const std::string& x);

    void set_default(const char* x);

    operator bool() const;

    operator double() const;

    operator char() const;

    operator int32() const;

    operator uint32() const;

    operator int64() const;

    operator uint64() const;

    operator std::string() const;

    operator const char*() const;

    bool as_bool() const;

    double as_double() const;

    char as_char() const;

    int32 as_int32() const;

    uint32 as_uint32() const;

    int64 as_int64() const;

    uint64 as_uint64() const;

    std::string as_string() const;
  };

  using ArgValList = std::vector<ArgVal>;

  struct Arg
  {
    std::string long_flag;  ///< empty string indicates not used.
    char short_flag;        ///< null character (0 or '\0') indicates not used

    bool required;  ///< Required to be specified by the user

    size_type num_vals;  ///< The number of values following the argument

    std::string dest;

    bool set_default;
    ArgValList default_vals;
    size_type cur_default_val_index;

    std::string help_str;

    StoreAction action;

    Arg();

    Arg& nvals(const size_type nv);

    Arg& operator<<(const bool x);

    Arg& operator<<(const double x);

    Arg& operator<<(const char x);

    Arg& operator<<(const int32 x);

    Arg& operator<<(const uint32 x);

    Arg& operator<<(const int64 x);

    Arg& operator<<(const uint64 x);

    Arg& operator<<(const std::string& x);

    Arg& operator<<(const char* x);

  };

  using ArgList = std::vector<Arg>;

  using DestStoreActionMap = std::map<std::string,StoreAction>;

  using FlagArgMap = std::map<std::string,Arg*>;

  ProgOpts();
  
  ProgOpts(const ProgOpts&) = delete;
  ProgOpts& operator=(const ProgOpts&) = delete;

  void set_help(const std::string& help_str);

  void set_help_epilogue(const std::string& help_str);

  void print_help(std::ostream& out) const;

  Arg& add(const Arg& arg);

  Arg& add(const std::string& long_flag, const char short_flag,
           const StoreAction action, const std::string& dest,
           const std::string& help_str = "");

  void parse(int argc, char* argv[]);

  void set_min_num_pos_args(const size_type n);
  
  const StringList& pos_args() const;

  bool has(const std::string& opt_name) const;

  /**
   * @brief Retrieves a command line option value.
   *
   * Retrieves the first value stored, or in most cases the only value stored.
   * @param opt_name The name of the option variable to retrieve.
   * @return An ArgVal representation of the variable; typically this is
   *         implicitly cast to a type specified by the user.
   **/
  const ArgVal& get_val(const std::string& opt_name) const;

  /**
   * @brief Retrieves a set of command line option boolean values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, BoolList* vals);

  /**
   * @brief Retrieves a set of command line option character values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, CharList* vals);

  /**
   * @brief Retrieves a set of command line option double values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, DoubleList* vals);

  /**
   * @brief Retrieves a set of command line option int32 values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, Int32List* vals);

  /**
   * @brief Retrieves a set of command line option uint32 values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, UInt32List* vals);

  /**
   * @brief Retrieves a set of command line option int64 values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, Int64List* vals);

  /**
   * @brief Retrieves a set of command line option uint64 values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, UInt64List* vals);

  /**
   * @brief Retrieves a set of command line option string values.
   * @param opt_name The name of the option variable to retrieve.
   * @param vals The list of values to set.
   **/
  void get_vals(const std::string& opt_name, StringList* vals);

  /**
   * @brief Retrieves a set of command line option values.
   *
   * @param opt_name The name of the option variable to retrieve
   * @param out_it An output iterator used to store the variable values
   *               as types of ArgVal.
   **/
  template <class OutItr>
  void get_vals(const std::string& opt_name, OutItr out_it) const
  {
    auto it = arg_value_map_.find(opt_name);
    if (it == arg_value_map_.end())
    {
      throw ValueNotSetException(opt_name.c_str());
    }

    std::copy(it->second.vals.begin(), it->second.vals.end(), out_it);
  }

  /**
   * @brief Retrieves a command line option value.
   *
   * Retrieves the first value stored, or in most cases the only value stored.
   * @param opt_name The name of the option variable to retrieve.
   * @return An ArgVal representation of the variable; typically this is
   *         implicitly cast to a type specified by the user.
   * @see get_val
   **/
  const ArgVal& get(const std::string& opt_name) const;

  /**
   * @brief Indicates if a flag's value was set via the default logic.
   *
   * @param opt_name The name of the option/flag to query
   * @return true when this flag's value was set via the default logic; false otherwise
   **/
  bool val_defaulted(const std::string& opt_name) const;

  /**
   * @brief Sets the arguments usage string to print.
   *
   * The usage string is the portion of the usage string printed after:
   * "Usage: <program name> [options]" and is usally used to describe the
   * use of positional arguments.
   * @param The portion of the usage string to set
   **/
  void set_arg_usage(const std::string& arg_usage_str);

  /**
   * @brief Prints the full usage string to an output stream.
   *
   * This prints out a full usage string of the following format:
   * "Usage: <program name> [options] [<argument usage>]"
   * @param out The stream to write to
   **/
  void print_usage(std::ostream& out);

  bool help_set() const;

  void set_help_flags(const std::string& help_long, const char help_short);
 
  void set_add_verbose_flag(const bool add_verbose_flag);

  void set_verbose_flags(const std::string& verbose_long, const char verbose_short);

  std::ostream& vout();

  void set_allow_unrecognized_flags(const bool allow);

  const std::string& compile_date() const;

  void set_compile_date(const std::string& s);

  const std::string& version_num_str() const;

  void set_version_num_str(const std::string& s);

  void set_print_help_ocl_str(const bool print_ocl);

  void add_ocl_select_flag();

  void add_backend_flags();

  void add_tbb_max_num_threads_flag();

  boost::compute::device selected_ocl();

  std::tuple<boost::compute::context,boost::compute::command_queue> selected_ocl_ctx_queue();

  /// \brief Checks to see if a variable/argument with a specific destination
  ///        string has been added.
  bool dest_exists(const std::string& dest_str) const;

private:

  struct ParsedValue
  {
    StoreAction action;
    bool defaulted;
    ArgValList vals;
  };

  using DestValueMap = std::map<std::string, ParsedValue>;

  static std::string Readable(const StoreAction a, const ArgVal& val);

  static std::string Readable(const StoreAction a, const ArgValList& vals);

  void build_flag_arg_map(FlagArgMap* long_flag_arg_map,
                          FlagArgMap* short_flag_arg_map);

  void set_defaults();

  void find_help(const size_type num_args, char* argv[]);

  void get_flag_str(const std::string& arg_str, std::string* long_flag,
                    std::string* short_flag) const;

  bool allow_unrec_flags_as_args_ = false;

  ArgList args_;

  DestValueMap arg_value_map_;

  std::string help_str_;

  std::string help_epi_str_;

  bool print_help_backend_str_ = false;

  bool print_help_ocl_str_ = false;

  size_type min_num_pos_args_ = 0;

  StringList pos_args_;

  std::string arg_usage_str_;

  std::string prog_name_;

  bool help_was_set_ = false;  // indicates that the user passed the help flag
  std::string help_long_flag_ = "help";
  char help_short_flag_ = 'h';

  bool add_verbose_flag_ = true;
  std::string verbose_long_flag_ = "verbose";
  char verbose_short_flag_ = 'v';

  std::string compile_date_;
  std::string version_num_str_;

  OpenCLNameDevMap opencl_id_str_to_dev_map_;

  // creating a default instance of boost::compute::device
  // should not throw an exception, it will initialize the
  // device ID to null.
  boost::compute::device selected_ocl_dev_;

  bool selected_ocl_ctx_queue_set_ = false;

  boost::compute::context selected_ocl_ctx_;
  boost::compute::command_queue selected_ocl_queue_;

  bool tbb_max_num_threads_opt_added_ = false;
};

}  // xreg

#define xregPROG_OPTS_SET_COMPILE_DATE(po) po.set_compile_date(__DATE__ " " __TIME__)

#endif
