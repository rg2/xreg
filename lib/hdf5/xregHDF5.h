/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#ifndef XREGHDF5_H_
#define XREGHDF5_H_

#include <array>

#include <fmt/format.h>

#include <H5Cpp.h>

#include "xregCommon.h"
#include "xregAssert.h"
#include "xregExceptionUtils.h"
#include "xregITKBasicImageUtils.h"

namespace xreg
{

template <class T>
H5::DataType LookupH5DataType();

// This is the same as unsigned int
//template <>
//inline H5::DataType LookupH5DataType<hbool_t>()
//{
//  return H5::PredType::NATIVE_HBOOL;
//}

template <>
inline H5::DataType LookupH5DataType<char>()
{
  return H5::PredType::NATIVE_CHAR;
}

template <>
inline H5::DataType LookupH5DataType<short>()
{
  return H5::PredType::NATIVE_SHORT;
}

template <>
inline H5::DataType LookupH5DataType<int>()
{
  return H5::PredType::NATIVE_INT;
}

template <>
inline H5::DataType LookupH5DataType<long>()
{
  return H5::PredType::NATIVE_LONG;
}

template <>
inline H5::DataType LookupH5DataType<unsigned char>()
{
  return H5::PredType::NATIVE_UCHAR;
}

template <>
inline H5::DataType LookupH5DataType<unsigned short>()
{
  return H5::PredType::NATIVE_USHORT;
}

template <>
inline H5::DataType LookupH5DataType<unsigned int>()
{
  return H5::PredType::NATIVE_UINT;
}

template <>
inline H5::DataType LookupH5DataType<unsigned long>()
{
  return H5::PredType::NATIVE_ULONG;
}

template <>
inline H5::DataType LookupH5DataType<float>()
{
  return H5::PredType::NATIVE_FLOAT;
}

template <>
inline H5::DataType LookupH5DataType<double>()
{
  return H5::PredType::NATIVE_DOUBLE;
}

template <>
inline H5::DataType LookupH5DataType<std::string>()
{
  H5::StrType str_type(H5::PredType::C_S1);
  str_type.setSize(H5T_VARIABLE);

  return str_type;
}

#ifdef _WIN32

template <>
inline H5::DataType LookupH5DataType<unsigned long long>()
{
  return H5::PredType::NATIVE_ULLONG;
}

#endif

void SetScalarAttr(const std::string& key, const long  val, H5::Group* h5);

long GetScalarLongAttr(const std::string& key, const H5::Group& h5);

H5::DataType GetH5StringDataType();

H5::DataType GetH5StringDataType(const std::string& s);

bool SetStringAttr(const std::string& key, const std::string& val, H5::Group* h5);

std::string GetStringAttr(const std::string& key, const H5::Group& h5);

H5::DataSet WriteStringH5(const std::string& field_name,
                          const std::string& field_val,
                          H5::Group* h5,
                          const bool compress = false);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const unsigned char& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const char& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const unsigned short& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const short& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const unsigned int& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const int& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const unsigned long& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const long& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const float& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const double& field_val,
                                H5::Group* h5);

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const bool& field_val,
                                H5::Group* h5);

#ifdef _WIN32

H5::DataSet WriteSingleScalarH5(const std::string& field_name,
                                const size_type& field_val,
                                H5::Group* h5);

#endif

H5::DataSet CreateVectorH5UChar(const std::string& field_name,
                                const unsigned long len,
                                H5::Group* h5,
                                const bool compress = true);

H5::DataSet CreateVectorH5Char(const std::string& field_name,
                               const unsigned long len,
                               H5::Group* h5,
                               const bool compress = true);

H5::DataSet CreateVectorH5UShort(const std::string& field_name,
                                 const unsigned long len,
                                 H5::Group* h5,
                                 const bool compress = true);

H5::DataSet CreateVectorH5Short(const std::string& field_name,
                                const unsigned long len,
                                H5::Group* h5,
                                const bool compress = true);

H5::DataSet CreateVectorH5UInt(const std::string& field_name,
                               const unsigned long len,
                               H5::Group* h5,
                               const bool compress = true);

H5::DataSet CreateVectorH5Int(const std::string& field_name,
                              const unsigned long len,
                              H5::Group* h5,
                              const bool compress = true);

H5::DataSet CreateVectorH5ULong(const std::string& field_name,
                                const unsigned long len,
                                H5::Group* h5,
                                const bool compress = true);

H5::DataSet CreateVectorH5Long(const std::string& field_name,
                               const unsigned long len,
                               H5::Group* h5,
                               const bool compress = true);

H5::DataSet CreateVectorH5Float(const std::string& field_name,
                                const unsigned long len,
                                H5::Group* h5,
                                const bool compress = true);

H5::DataSet CreateVectorH5Double(const std::string& field_name,
                                 const unsigned long len,
                                 H5::Group* h5,
                                 const bool compress = true);

H5::DataSet CreateVectorH5Bool(const std::string& field_name,
                               const unsigned long len,
                               H5::Group* h5,
                               const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<unsigned char>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<char>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<unsigned short>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<short>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<unsigned int>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<int>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<unsigned long>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<long>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<float>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<double>& v,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<bool>& v,
                          H5::Group* h5,
                          const bool compress = true);

#ifdef _WIN32

H5::DataSet WriteVectorH5(const std::string& field_name,
                          const std::vector<size_type>& v,
                          H5::Group* h5,
                          const bool compress = true);

#endif

void WriteVectorElemH5(const unsigned char& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const char& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const unsigned short& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const short& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const unsigned int& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const int& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const unsigned long& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const long& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const float& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const double& x, const unsigned long i, H5::DataSet* h5);

void WriteVectorElemH5(const bool& x, const unsigned long i, H5::DataSet* h5);

H5::DataSet CreateMatrixH5Float(const std::string& field_name,
                                const unsigned long num_rows,
                                const unsigned long num_cols,
                                H5::Group* h5,
                                const bool compress = true);

H5::DataSet CreateMatrixH5Double(const std::string& field_name,
                                 const unsigned long num_rows,
                                 const unsigned long num_cols,
                                 H5::Group* h5,
                                 const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Pt2& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Pt3& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Pt4& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Pt5& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Pt6& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const PtN& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Mat2x2& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Mat3x3& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Mat4x4& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const Mat3x4& mat,
                          H5::Group* h5,
                          const bool compress = true);

H5::DataSet WriteMatrixH5(const std::string& field_name,
                          const MatMxN& mat,
                          H5::Group* h5,
                          const bool compress = true);

void WriteMatrixRowH5(const float* row_buf, const unsigned long row_idx, H5::DataSet* h5);

void WriteMatrixRowH5(const double* row_buf, const unsigned long row_idx, H5::DataSet* h5);

H5::DataSet WriteAffineTransform4x4(const std::string& field_name,
                                    const FrameTransform& xform,
                                    H5::Group* h5,
                                    const bool compress = true);

void WriteImageH5(const itk::Image<unsigned char,2>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<unsigned short,2>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<short,2>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<float,2>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<double,2>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<unsigned char,3>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<unsigned short,3>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<short,3>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<float,3>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteImageH5(const itk::Image<double,3>* img, 
                  H5::Group* h5,
                  const bool compress = true);

void WriteSegImageH5(const itk::Image<unsigned char,2>* img, 
                     H5::Group* h5,
                     const std::unordered_map<unsigned char,std::string>& seg_labels_def = std::unordered_map<unsigned char,std::string>(),
                     const bool compress = true);

void WriteSegImageH5(const itk::Image<unsigned short,2>* img, 
                     H5::Group* h5,
                     const std::unordered_map<unsigned short,std::string>& seg_labels_def = std::unordered_map<unsigned short,std::string>(),
                     const bool compress = true);

void WriteSegImageH5(const itk::Image<unsigned char,3>* img, 
                     H5::Group* h5,
                     const std::unordered_map<unsigned char,std::string>& seg_labels_def = std::unordered_map<unsigned char,std::string>(),
                     const bool compress = true);

void WriteSegImageH5(const itk::Image<unsigned short,3>* img, 
                     H5::Group* h5,
                     const std::unordered_map<unsigned short,std::string>& seg_labels_def = std::unordered_map<unsigned short,std::string>(),
                     const bool compress = true);

void WriteLandmarksMapH5(const LandMap2& m, H5::Group* h5);

void WriteLandmarksMapH5(const LandMap3& m, H5::Group* h5);

void WriteLandmarksMapH5(const LandMap4& m, H5::Group* h5);

H5::DataSet WriteListOfPointsAsMatrixH5(const std::string& field_name,
                                        const Pt2List& pts,
                                        H5::Group* h5,
                                        const bool compress = true);

H5::DataSet WriteListOfPointsAsMatrixH5(const std::string& field_name,
                                        const Pt3List& pts,
                                        H5::Group* h5,
                                        const bool compress = true);

H5::DataSet WriteListOfArraysToMatrixH5(const std::string& field_name,
                                        const std::vector<std::array<size_type,3>>& arrays,
                                        H5::Group* h5,
                                        const bool compress = true);

std::string ReadStringH5(const std::string& field_name,
                         const H5::Group& h5);

unsigned char ReadSingleScalarH5UChar(const std::string& field_name,
                                      const H5::Group& h5);

char ReadSingleScalarH5Char(const std::string& field_name,
                            const H5::Group& h5);

unsigned short ReadSingleScalarH5UShort(const std::string& field_name,
                                        const H5::Group& h5);

short ReadSingleScalarH5Short(const std::string& field_name,
                              const H5::Group& h5);

unsigned int ReadSingleScalarH5UInt(const std::string& field_name,
                                    const H5::Group& h5);

int ReadSingleScalarH5Int(const std::string& field_name,
                          const H5::Group& h5);

unsigned long ReadSingleScalarH5ULong(const std::string& field_name,
                                      const H5::Group& h5);

long ReadSingleScalarH5Long(const std::string& field_name,
                            const H5::Group& h5);

float ReadSingleScalarH5Float(const std::string& field_name,
                              const H5::Group& h5);

double ReadSingleScalarH5Double(const std::string& field_name,
                                const H5::Group& h5);

bool ReadSingleScalarH5Bool(const std::string& field_name,
                            const H5::Group& h5);

CoordScalar ReadSingleScalarH5CoordScalar(const std::string& field_name,
                                          const H5::Group& h5);

std::vector<unsigned char>
ReadVectorH5UChar(const std::string& field_name, const H5::Group& h5);

std::vector<char>
ReadVectorH5Char(const std::string& field_name, const H5::Group& h5);

std::vector<unsigned short>
ReadVectorH5UShort(const std::string& field_name, const H5::Group& h5);

std::vector<short>
ReadVectorH5Short(const std::string& field_name, const H5::Group& h5);

std::vector<unsigned int>
ReadVectorH5UInt(const std::string& field_name, const H5::Group& h5);

std::vector<int>
ReadVectorH5Int(const std::string& field_name, const H5::Group& h5);

std::vector<unsigned long>
ReadVectorH5ULong(const std::string& field_name, const H5::Group& h5);

std::vector<long>
ReadVectorH5Long(const std::string& field_name, const H5::Group& h5);

std::vector<float>
ReadVectorH5Float(const std::string& field_name, const H5::Group& h5);

std::vector<double>
ReadVectorH5Double(const std::string& field_name, const H5::Group& h5);

std::vector<bool>
ReadVectorH5Bool(const std::string& field_name, const H5::Group& h5);

std::vector<CoordScalar>
ReadVectorH5CoordScalar(const std::string& field_name, const H5::Group& h5);

std::vector<size_type>
ReadVectorH5SizeType(const std::string& field_name, const H5::Group& h5);

MatMxN ReadMatrixH5CoordScalar(const std::string& field_name, const H5::Group& h5);

MatMxN ReadMatrixH5Float(const std::string& field_name, const H5::Group& h5);

MatMxN_d ReadMatrixH5Double(const std::string& field_name, const H5::Group& h5);

FrameTransform
ReadAffineTransform4x4H5(const std::string& field_name,
                         const H5::Group& h5);

itk::Image<unsigned char,2>::Pointer
ReadITKImageH5UChar2D(const H5::Group& h5);

itk::Image<char,2>::Pointer
ReadITKImageH5Char2D(const H5::Group& h5);

itk::Image<unsigned short,2>::Pointer
ReadITKImageH5UShort2D(const H5::Group& h5);

itk::Image<short,2>::Pointer
ReadITKImageH5Short2D(const H5::Group& h5);

itk::Image<float,2>::Pointer
ReadITKImageH5Float2D(const H5::Group& h5);

itk::Image<double,2>::Pointer
ReadITKImageH5Double2D(const H5::Group& h5);

itk::Image<unsigned char,3>::Pointer
ReadITKImageH5UChar3D(const H5::Group& h5);

itk::Image<char,3>::Pointer
ReadITKImageH5Char3D(const H5::Group& h5);

itk::Image<unsigned short,3>::Pointer
ReadITKImageH5UShort3D(const H5::Group& h5);

itk::Image<short,3>::Pointer
ReadITKImageH5Short3D(const H5::Group& h5);

itk::Image<float,3>::Pointer
ReadITKImageH5Float3D(const H5::Group& h5);

itk::Image<double,3>::Pointer
ReadITKImageH5Double3D(const H5::Group& h5);

LandMap2 ReadLandmarksMapH5Pt2(const H5::Group& h5);

LandMap3 ReadLandmarksMapH5Pt3(const H5::Group& h5);

LandMap4 ReadLandmarksMapH5Pt4(const H5::Group& h5);

Pt2List ReadLandsAsPtCloudH5Pt2(const H5::Group& h5);

Pt3List ReadLandsAsPtCloudH5Pt3(const H5::Group& h5);

Pt4List ReadLandsAsPtCloudH5Pt4(const H5::Group& h5);

Pt2List
ReadListOfPointsFromMatrixH5Pt2(const std::string& field_name, const H5::Group& h5);

Pt3List
ReadListOfPointsFromMatrixH5Pt3(const std::string& field_name, const H5::Group& h5);

Pt4List
ReadListOfPointsFromMatrixH5Pt4(const std::string& field_name, const H5::Group& h5);

PtNList
ReadListOfPointsFromMatrixH5PtN(const std::string& field_name, const H5::Group& h5);

std::vector<std::array<size_type,3>>
ReadListOfArraysFromMatrixH5Sizes3(const std::string& field_name, const H5::Group& h5);

/// \brief Return a list of item names in a file/group
std::vector<std::string> GetH5ObjNames(const H5::Group& h5);

/// \brief Check to see if an object exists in a group
bool ObjectInGroupH5(const std::string& obj_name, const H5::Group& h5);

/// \brief Open a group if it already exists or create one
H5::Group OpenOrCreateGroupH5(const std::string& group_name, H5::Group* h5);

/// \brief Get lists of names groups and datasets which are direct children of this group/file.
///
/// The first tuple element is the list of group names, the second is the list of dataset names.
std::tuple<std::vector<std::string>,std::vector<std::string>>
GetH5GroupAndDatasetNames(const H5::Group& h5);

/// \brief Instance of this object will disable printing of HDF5 exceptions
///        while in scope.
class HideH5ExceptionPrints
{
public:
  
  HideH5ExceptionPrints();

  ~HideH5ExceptionPrints();

  HideH5ExceptionPrints(const HideH5ExceptionPrints&) = delete;

  HideH5ExceptionPrints& operator=(const HideH5ExceptionPrints&) = delete;

private:
  H5E_auto2_t auto_print_fn;
  void* auto_print_data = nullptr;
};

}  // xreg

#endif

