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
 * @brief Utility routines for filesystem usage (e.g. list files in the
 * current directory).
 *
 * These are designed to be platform independent. The implementations are
 * not designed to be the most efficient with regards to speed or memory
 * efficiency, since any filesystem operations will be a minimal portion
 * of program execution. These routines exist, so that another library
 * does not need to be linked in. For example, the Boost Filesystem module
 * provides all of these functionality, but is not a header only
 * implementation, and forces a user to have a compiled version of Boost
 * (which can grow to be quite large in file size).
 **/

#ifndef XREGFILESYSTEMUTILS_H_
#define XREGFILESYSTEMUTILS_H_

#include <vector>
#include <string>
#include <cstdio>

#include <boost/container/flat_set.hpp>

#include "xregCommon.h"
#include "xregStreams.h"
#include "xregExceptionUtils.h"

namespace xreg
{

class Path;

///< List type for storing filesystem path strings
using PathStringList = std::vector<std::string>;

///< List type for storing filesystem Path objects
using PathList = std::vector<Path>;

///< File/Path separator token on Windows
const char kWIN_FILE_SEP   = '\\';

///< File/Path separator token on POSIX systems
const char kPOSIX_FILE_SEP = '/';

#ifdef _WIN32
// I am going to allow \ and / on Windows
const unsigned long kNUM_FILE_SEPS = 2;
#else
const unsigned long kNUM_FILE_SEPS = 1;
#endif

/**
 * @brief Valid file separator tokens that are accepted.
 *
 * kNUM_FILE_SEPS characters long.
 **/
extern const char* kFILE_SEPS;  // defined in the .cpp

/**
 * @brief Base class for all exceptions dealing with filesystem operations.
 */
class FileSystemException : public StringMessageException
{
public:
  FileSystemException() throw();
  FileSystemException(const char* path, const char* msg) throw();

  virtual ~FileSystemException() throw() { }
};

/**
 * @brief An abstract representation of a filesystem path.
 *
 * This also contains various utility methods for extracting
 * commonly used information, such as the contents of a directory
 * or the filename component.
 **/
class Path
{
public:
  /**
   * @brief Default Constructor; defaults to an empty path.
   **/
  Path();

  /**
   * @brief Copy Constructor for a std::string path value.
   *
   * @param path_str The string that represents this Path object
   **/
  Path(const std::string& path_str);

  /**
   * @brief Copy Constructor for a C string (const char*) path value.
   *
   * @param path_str The string that represents this Path object
   **/
  Path(const char* path_str);

  /**
   * @brief Compute the parent path of this path item.
   *
   * The containing directory of this item.
   * WARNING: This returns relative parents when given relative paths.
   * For example, if the current path stored is "test" and the absolute path is
   * "/blah/test" this routine will return the ".." and not "/blah." However,
   * if the current path stored is "/blah/test" this routine will return
   * "/blah"
   * @return A path object representing the parent path
   **/
  Path parent() const;

  /**
   * @brief Extracts the path component corresponding with the filename.
   * @return A Path object representing only the filename
   **/
  Path filename() const;

  /**
   * @brief Extracts the file extension associated with this file path.
   * @return The file extension string (including the period/dot/.)
   **/
  std::string file_extension() const;

  /**
   * @brief Splits a path into the prefix string prior to the file extension and
   *        the file extension.
   *
   * @param prefix The extracted prefix (component prior to the file extension)
   * @param ext The extracted file extension (including the period/dot)
   * @see SplitPathPrefixExtension
   **/
  void split_ext(std::string* prefix, std::string* ext) const;

  std::tuple<std::string,std::string> split_ext() const
  {
    std::string prefix;
    std::string ext;

    split_ext(&prefix, &ext);

    return std::make_tuple(prefix, ext);
  }

  /// \brief Splits a path into a prefix and extension, but allows an extension to have
  ///        an internal dot to account for compression (e.g. ".nii.gz")
  ///
  /// \see split_ext
  void split_ext_w_comp(std::string* prefix, std::string* ext) const;

  std::tuple<std::string,std::string> split_ext_w_comp() const
  {
    std::string prefix;
    std::string ext;

    split_ext_w_comp(&prefix, &ext);

    return std::make_tuple(prefix, ext);
  }

  /**
   * @brief Extracts the prefix portion of this path.
   *
   * @returns A path object representing the prefix (e.g. without file extension)
   **/
   Path prefix() const;

  /**
   * @brief Retrieves a string representation of this path
   * @return A std::string representation of this path
   **/
  const std::string& string() const
  {
    return path_str_;
  }

  /**
   * @brief Determines if the filesystem element at this path actually exists.
   * @return true when the filesystem element exists, false otherwise
   **/
  bool exists() const;

  /**
   * @brief Determines if the filesystem element at this path is a directory.
   * @return true when the filesystem element is a directory, false otherwise
   **/
  bool is_dir() const;

  /**
   * @brief Determines if the filesystem element at this path is a regular file.
   * @return true when the filesystem element is a regular file, false otherwise
   **/
  bool is_reg_file() const;

  /**
   * @ brief Determines if the file at this path is executable.
   **/
  bool is_exec() const;

  /**
   * @brief Appends a filesystem path to the back of this path.
   *
   * This is analogous to joining two path components.
   * @param p The path to be appended onto the end of this path
   **/
  void append(const Path& p);

  /**
   * @brief Shorthand for appending filesystem paths.
   *
   * @param rhs The path to be appended onto the end of this path
   * @return *this
   * @see append
   **/
  Path& operator+=(const Path& rhs);

  /**
   * @brief Retrieves the directory contents of this path.
   *
   * @param paths The list of Path objects to populate
   * @param filenames_only (optional) Store filename only Path
   *        objects; defaults to false
   **/
  void get_dir_contents(PathList* paths, const bool filenames_only = false) const;

  /// \brief Sets this path to the POSIX root path: /
  void set_posix_root();

  /// \brief Converts current path, possibly a relative path, into an absolute path.
  Path absolute_path() const;

private:

  using size_type = std::string::size_type;

  /**
   * @brief Determines if the string representation of this path has a trailing separator.
   * @return true when the path string has a trailing separator (e.g. ends in / on POSIX),
   *         false otherwise
   **/
  bool has_trailing_sep() const;

  std::string path_str_;
};

/// \brief Path concatenation operator with +.
Path operator+(const Path& p1, const Path& p2);

/**
 * @brief Converts a path to use only Windows separators.
 *
 * Replaces each forward slash (/) with a backslash (\\).
 * @param path A path string to convert
 * @return A path string with only backslash separators
 **/
std::string ConvertToWinPath(const std::string& path);

/**
 * @brief Converts a path to use only POSIX separators.
 *
 * Replaces each backslash (\\) with a forward slash (/).
 * @param path A path string to convert
 * @return A path string with only forward slash separators
 **/
std::string ConvertToPosixPath(const std::string& path);

/**
 * @brief Retrieves the file name string from a full path string.
 *
 * @param path The full path string
 * @return The filename string
 **/
std::string GetFileName(const std::string& path);

/**
 * @brief Extracts a file extension string from a full path string.
 *
 * The file extension will include the period/dot (.).
 * @param path The full path string
 * @return The extension string
 **/
std::string GetFileExtension(const std::string& path);

/**
 * @brief Splits a path into the prefix string prior to the file extension and
 *        the file extension.
 *
 * @param path The path to parse
 * @param prefix The extracted prefix (component prior to the file extension)
 * @param ext The extracted file extension (including the period/dot)
 **/
void SplitPathPrefixExtension(const std::string& path, std::string* prefix,
                              std::string* ext);

/**
 * @brief Represents a set of file extensions.
 *
 * File extensions are represented by strings with the first character set to
 * a period ('.'). All operations and comparisons on the extension strings are
 * done in a case insensitive manner.
 **/
class FileExtensions
{
public:

  /**
   * @brief Default constructor, initializes the set of extensions to empty.
   **/
  FileExtensions() { }

  /**
   * @brief Destructor.
   **/
  virtual ~FileExtensions() { }

  /**
   * @brief Adds an extension
   *
   * @param e The file extension to add
   **/
  void add(const std::string& e);

  /**
   * @brief Checks if a file has an extension represented by this object.
   *
   * @param p The path to check; this may be a full filesystem path, only
   *          the filename portion is checked
   * @return true when the file has an extension represented by this object;
   *         false otherwise
   **/
  virtual bool check_file(const Path& p) const;

private:
  using StringSet = boost::container::flat_set<std::string>;
  StringSet exts_;
};

/**
 * @brief Represents a set of every possible file extension.
 **/
class AllowAllFileExtensions : public FileExtensions
{
public:
  /**
   * @brief Destructor.
   **/
  virtual ~AllowAllFileExtensions() { }

  /**
   * @brief Always returns that the file matches extensions
   * @param p The path to check, since this routine always returns true
   *          it does not necessarily need to be valid
   * @return true
   **/
  virtual bool check_file(const Path&) const
  {
    return true;
  }
};

/**
 * @brief Gets the list of files in a directory that match a set of file extensions.
 *
 * @param p The directory to check
 * @param files The list of paths to populate
 * @param exts The set of file extensions to filter by
 **/
void GetFilePathsFromDir(const Path& p, PathStringList* files, const FileExtensions& exts);

/**
 * @brief Gets the list of files in a directory that matches a specific file extension.
 *
 * @param p The directory to check
 * @param files The list of paths to populate
 * @param ext The file extension to filter by
 **/
void GetFilePathsFromDir(const Path& p, PathStringList* files, const std::string& ext);

/**
 * @brief Gets the list of all files in a directory.
 *
 * @param p The directory to check
 * @param files The list of paths to populate
 **/
void GetFilePathsFromDir(const Path& p, PathStringList* files);

/// \brief Gets the list of all files in a directory with a specific suffix.
///
/// This is useful for getting a list of files that end with a pseudo-extension,
/// such as .nii.gz.
void GetFilePathsFromDirWithSuffix(const Path& p, PathStringList* files,
                                   const std::string& suffix,
                                   const bool case_sensitive = true);

/// \brief Get a list of non-empty lines in a text stream.
///
/// \param in The text stream
/// \return The list of lines
PathStringList GetNonEmptyLinesFromStream(std::istream& in);

/// \brief Removes all non-comment lines from a list string lines.
///
/// A comment may be any string pattern. Any line that starts with any amount
/// of whitespace followed by a comment string pattern is considered to be
/// a comment line.
/// \param src_paths The input list of string lines that may contain comments
/// \param dst_paths The output list of string lines that will not contain any comments
/// \param comment_strs The list of comment string patterns
void ExtractNonCommentLines(const PathStringList& src_paths, PathStringList* dst_paths,
                            const PathStringList& comment_strs);

/// \brief Removes all non-comment lines from a list string lines.
///
/// A comment may be any string pattern. Any line that starts with any amount
/// of whitespace followed by a comment string pattern is considered to be
/// a comment line.
/// \param src_paths The input list of string lines that may contain comments
/// \param dst_paths The output list of string lines that will not contain any comments
/// \param comment_strs The comment string pattern
void ExtractNonCommentList(const PathStringList& src_paths, PathStringList* dst_paths,
                           const std::string& comment_str);

/**
* @brief Get a list of non-empty lines, that also do not begin with a comment character, in a text stream.
*
* @param in The text stream
* @param paths The list of lines to populate
* @param comment_str The comment string pattern - defaults to #
**/
void GetNonEmptyNonCommentLinesFromStream(std::istream& in, PathStringList* paths,
                                         const std::string& comment_str = "#");

/**
 * @brief Helper class to be used when a FILE* is needed.
 *
 * This class is exception
 * friendly, that is the destructor will call fclose() if necessary. An
 * instance of this class may be used where ever a FILE* would be used, for
 * example:
 * @code
 *   ScopedCFile fp("myfile.txt", "rb");
 *   fread(fp, 1, 256, some_buf);
 *   // if this is the end of scope, then fp is closed equivalent to fclose(fp)
 * @endcode
 */
class ScopedCFile
{
public:
  /**
   * Constructor - called exactly as fopen().
   * @param path The path to the file to open
   * @param mode C String describing the mode to open the file in
   */
  ScopedCFile(const char* path, const char* mode);

  /**
   * Closes the file for reading/writing. Any subsequent reads/writes to any
   * FILE*'s this object was assigned to are undefined.
   */
  ~ScopedCFile();

  /**
   * Closes the file for reading/writing. Any subsequent reads/writes with
   * this object or and FILE*'s it was assigned to are undefined.
   */
  void close();

  /**
   * Cast to FILE*, returns the appropriate FILE*.
   */
  operator FILE*();

private:
  // no copying
  ScopedCFile(const ScopedCFile&);
  ScopedCFile& operator=(const ScopedCFile&);

  FILE* fp_;
};

class FileInputStream : public InputStream
{
public:
  FileInputStream(const std::string& path);

  virtual ~FileInputStream() { }

  InputStream::size_type num_bytes_left();

  void read_remaining_bytes(void* buf);

private:
  virtual void raw_read(void* ptr, const InputStream::size_type num_bytes);

  // no copying
  FileInputStream(const FileInputStream&);
  FileInputStream& operator=(const FileInputStream&);

  ScopedCFile fp_;
  std::string file_path_; // for exception throwing...
};

class FileOutputStream : public OutputStream
{
public:
  FileOutputStream(const std::string& path);

  ~FileOutputStream();

private:
  void raw_write(const void* ptr, const OutputStream::size_type num_bytes);

  // no copying
  FileOutputStream(const FileOutputStream&);
  FileOutputStream& operator=(const FileOutputStream&);

  ScopedCFile fp_;
  std::string file_path_; // for exception throwing...
};

/**
 * @brief Copies a file to another file path or directory.
 *
 * @param src_path The path of the file to copy
 * @param dst_path The destination path of the file, if a directory is passed,
 *                 then a file with the same basename as src_path will be
 *                 created in the directory
 **/
void CopyFileSystemItem(const std::string& src_path, const std::string& dst_path);

/**
 * @brief Generates a unique path name with a incremental numeric identifier.
 *
 * As an example, suppose directory D contains the files screen_00.png and screen_01.png,
 * then calling this routine with the pattern "D/screen_%02lu.png" will return
 * "screen_02.png"
 * @param path_pattern The path pattern string to use, the numeric specifier
 *                     should be specified via %lu
 * @return The
 **/
std::string GetUniqueNumericIDPath(const std::string& path_pattern);

/// \brief Creates a single directory; the parent directory must exist.
void MakeDir(const std::string& path);

/// \brief Creates a directory, recursively creating any directories in the path
///        that do not already exist.
void MakeDirRecursive(const std::string& path);

/// \brief Moves a filesystem item to a new location. Analogous to mv.
///
/// When dst_path is an existing directory, then the source is moved
/// into the destination directory.
void MoveFileSystemItem(const std::string& src_path, const std::string& dst_path);

/// \brief Recursively removes a directory and all of its contents
void RemoveDirRecursive(const std::string& path);

/// \brief Creates a temporary directory on construction and deletes it (recursively)
///        on destruction.
class CreateTempDir
{
public:
  explicit CreateTempDir(const std::string& prefix = "");

  ~CreateTempDir();

  CreateTempDir(const CreateTempDir&) = delete;

  CreateTempDir& operator=(const CreateTempDir&) = delete;

  std::string path() const
  {
    return p_.string();
  }

  void set_should_delete(const bool sd)
  {
    should_delete_ = sd;
  }

private:
  
  Path p_;

  bool should_delete_ = true;
};

std::string FindExeOnSystemPath(const std::string& exe_name);

/// \brief Returns true if a file path has an extension corresponding to
///        and HDF5 file.
bool IsHDF5FileExt(const std::string& path);

/// \brief Reads the contents of a file into a string
std::string FileToString(const std::string& path);

std::vector<float> FileToFloats(const std::string& path);

std::vector<double> FileToDoubles(const std::string& path);

CoordScalarList FileToCoordScalars(const std::string& path);

}  // xreg

#endif
