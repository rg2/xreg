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

#include "xregHDF5.h"

#include "xregHDF5Internal.h"
#include "xregITKBasicImageUtils.h"

void xreg::SetScalarAttr(const std::string& key, const long val, H5::Group* h5)
{
  const auto dt = LookupH5DataType<long>();

  H5::Attribute attr = h5->createAttribute(key, dt, H5S_SCALAR);

  attr.write(dt, &val);
}

long xreg::GetScalarLongAttr(const std::string& key, const H5::Group& h5)
{
  const H5::Attribute attr = h5.openAttribute(key);
  
  long val;
  attr.read(attr.getDataType(), &val);

  return val;
}

H5::DataType xreg::GetH5StringDataType()
{
  return LookupH5DataType<std::string>();
}

H5::DataType xreg::GetH5StringDataType(const std::string& s)
{
  return H5::StrType(H5::PredType::C_S1, std::max(std::string::size_type(1), s.size()));
}

bool xreg::SetStringAttr(const std::string& key, const std::string& val, H5::Group* h5)
{
  bool attr_set = false;

  H5::Group* h5_g = dynamic_cast<H5::Group*>(h5);

  if (h5_g)
  {
    const auto dt = GetH5StringDataType(val);

    H5::Attribute attr = h5_g->createAttribute(key, dt, H5S_SCALAR);

    attr.write(dt, val);

    attr_set = true;
  }
  
  return attr_set;
}

std::string xreg::GetStringAttr(const std::string& key, const H5::Group& h5)
{
  const auto* h5_g = dynamic_cast<const H5::Group*>(&h5);

  if (h5_g)
  {
    const H5::Attribute attr = h5_g->openAttribute(key);
    
    std::string val;
    attr.read(attr.getDataType(), val);

    return val;
  }
  else
  {
    xregThrow("Invalid hdf5 type - cannot cast to group and openAttribute()");
  }
}

H5::DataSet xreg::WriteStringH5(const std::string& field_name,
                                const std::string& field_val,
                                H5::Group* h5,
                                const bool compress)
{
  H5::DSetCreatPropList props;
  props.copy(H5::DSetCreatPropList::DEFAULT);
  
  if (compress)
  {
    // NOTE: THIS DOES NOT SEEM TO BE CORRECT - CRASHES ABOUT MISMATCH
    //       OF DIMENSION
    const hsize_t chunk_dims = std::min(field_val.size(),
                                        std::string::size_type(1024));
    props.setChunk(1, &chunk_dims);
    props.setDeflate(9);
  }

  const auto data_type = GetH5StringDataType(field_val);

  H5::DataSet data_set = h5->createDataSet(field_name, data_type,
                                           H5S_SCALAR, props);

  data_set.write(field_val, data_type);

  return data_set;
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const unsigned char& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<unsigned char>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const char& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<char>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const unsigned short& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<unsigned short>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const short& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<short>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const unsigned int& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<unsigned int>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const int& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<int>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const unsigned long& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<unsigned long>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const long& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<long>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const float& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<float>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const double& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<double>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const bool& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<unsigned char>(field_name, static_cast<unsigned char>(field_val), h5);
}

H5::DataSet xreg::CreateVectorH5UChar(const std::string& field_name,
                                      const unsigned long len,
                                      H5::Group* h5,
                                      const bool compress)
{
  return detail::CreateVectorH5Helper<unsigned char>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Char(const std::string& field_name,
                                     const unsigned long len,
                                     H5::Group* h5,
                                     const bool compress)
{
  return detail::CreateVectorH5Helper<char>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5UShort(const std::string& field_name,
                                       const unsigned long len,
                                       H5::Group* h5,
                                       const bool compress)
{
  return detail::CreateVectorH5Helper<unsigned short>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Short(const std::string& field_name,
                                      const unsigned long len,
                                      H5::Group* h5,
                                      const bool compress)
{
  return detail::CreateVectorH5Helper<short>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5UInt(const std::string& field_name,
                                     const unsigned long len,
                                     H5::Group* h5,
                                     const bool compress)
{
  return detail::CreateVectorH5Helper<unsigned int>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Int(const std::string& field_name,
                                    const unsigned long len,
                                    H5::Group* h5,
                                    const bool compress)
{
  return detail::CreateVectorH5Helper<int>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5ULong(const std::string& field_name,
                                      const unsigned long len,
                                      H5::Group* h5,
                                      const bool compress)
{
  return detail::CreateVectorH5Helper<unsigned long>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Long(const std::string& field_name,
                                     const unsigned long len,
                                     H5::Group* h5,
                                     const bool compress)
{
  return detail::CreateVectorH5Helper<long>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Float(const std::string& field_name,
                                      const unsigned long len,
                                      H5::Group* h5,
                                      const bool compress)
{
  return detail::CreateVectorH5Helper<float>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Double(const std::string& field_name,
                                       const unsigned long len,
                                       H5::Group* h5,
                                       const bool compress)
{
  return detail::CreateVectorH5Helper<double>(field_name, len, h5, compress);
}

H5::DataSet xreg::CreateVectorH5Bool(const std::string& field_name,
                                     const unsigned long len,
                                     H5::Group* h5,
                                     const bool compress)
{
  return detail::CreateVectorH5Helper<unsigned char>(field_name, len, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<unsigned char>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<unsigned char>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<char>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<char>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<unsigned short>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<unsigned short>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<short>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<short>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<unsigned int>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<unsigned int>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<int>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<int>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<unsigned long>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<unsigned long>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<long>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<long>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<float>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<float>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<double>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<double>(field_name, v, h5, compress);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<bool>& v,
                                H5::Group* h5,
                                const bool compress)
{
  std::vector<unsigned char> tmp;
  tmp.reserve(v.size());

  for (const auto& b : v)
  {
    tmp.push_back(b);
  }

  return WriteVectorH5(field_name, tmp, h5, compress);
}

#ifdef _WIN32

H5::DataSet xreg::WriteSingleScalarH5(const std::string& field_name,
                                      const size_type& field_val,
                                      H5::Group* h5)
{
  return detail::WriteSingleScalarH5Helper<size_type>(field_name, field_val, h5);
}

H5::DataSet xreg::WriteVectorH5(const std::string& field_name,
                                const std::vector<size_type>& v,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteVectorH5Helper<size_type>(field_name, v, h5, compress);
}

#endif

void xreg::WriteVectorElemH5(const unsigned char& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<unsigned char>(x, i, h5);
}

void xreg::WriteVectorElemH5(const char& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<char>(x, i, h5);
}

void xreg::WriteVectorElemH5(const unsigned short& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<unsigned short>(x, i, h5);
}

void xreg::WriteVectorElemH5(const short& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<short>(x, i, h5);
}

void xreg::WriteVectorElemH5(const unsigned int& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<unsigned int>(x, i, h5);
}

void xreg::WriteVectorElemH5(const int& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<int>(x, i, h5);
}

void xreg::WriteVectorElemH5(const unsigned long& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<unsigned long>(x, i, h5);
}

void xreg::WriteVectorElemH5(const long& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<long>(x, i, h5);
}

void xreg::WriteVectorElemH5(const float& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<float>(x, i, h5);
}

void xreg::WriteVectorElemH5(const double& x, const unsigned long i, H5::DataSet* h5)
{
  detail::WriteVectorElemH5Helper<double>(x, i, h5);
}

void xreg::WriteVectorElemH5(const bool& x, const unsigned long i, H5::DataSet* h5)
{
  WriteVectorElemH5(static_cast<unsigned char>(x), i, h5);
}

H5::DataSet xreg::CreateMatrixH5Float(const std::string& field_name,
                                      const unsigned long num_rows,
                                      const unsigned long num_cols,
                                      H5::Group* h5,
                                      const bool compress)
{
  return detail::CreateMatrixH5Helper<float>(field_name, num_rows, num_cols, h5, compress);
}

H5::DataSet xreg::CreateMatrixH5Double(const std::string& field_name,
                                       const unsigned long num_rows,
                                       const unsigned long num_cols,
                                       H5::Group* h5,
                                       const bool compress)
{
  return detail::CreateMatrixH5Helper<double>(field_name, num_rows, num_cols, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Pt2& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Pt3& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Pt4& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Pt5& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Pt6& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const PtN& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Mat2x2& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Mat3x3& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Mat4x4& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const Mat3x4& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

H5::DataSet xreg::WriteMatrixH5(const std::string& field_name,
                                const MatMxN& mat,
                                H5::Group* h5,
                                const bool compress)
{
  return detail::WriteMatrixH5Helper(field_name, mat, h5, compress);
}

void xreg::WriteMatrixRowH5(const float* row_buf, const unsigned long row_idx, H5::DataSet* h5)
{
  detail::WriteMatrixRowH5Helper(row_buf, row_idx, h5);
}

void xreg::WriteMatrixRowH5(const double* row_buf, const unsigned long row_idx, H5::DataSet* h5)
{
  detail::WriteMatrixRowH5Helper(row_buf, row_idx, h5);
}

H5::DataSet xreg::WriteAffineTransform4x4(const std::string& field_name,
                                          const FrameTransform& xform,
                                          H5::Group* h5,
                                          const bool compress)
{
  return WriteMatrixH5(field_name, xform.matrix(), h5, compress);
}

void xreg::WriteImageH5(const itk::Image<unsigned char,2>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<unsigned short,2>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<short,2>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<float,2>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<double,2>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<unsigned char,3>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<unsigned short,3>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<short,3>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<float,3>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteImageH5(const itk::Image<double,3>* img, 
                        H5::Group* h5,
                        const bool compress)
{
  detail::WriteNDImageH5Helper(img, h5, compress);
}

void xreg::WriteLandmarksMapH5(const LandMap2& m, H5::Group* h5)
{
  detail::WriteLandmarksMapH5Helper(m, h5);
}

void xreg::WriteLandmarksMapH5(const LandMap3& m, H5::Group* h5)
{
  detail::WriteLandmarksMapH5Helper(m, h5);
}

void xreg::WriteLandmarksMapH5(const LandMap4& m, H5::Group* h5)
{
  detail::WriteLandmarksMapH5Helper(m, h5);
}

H5::DataSet xreg::WriteListOfPointsAsMatrixH5(const std::string& field_name,
                                              const Pt2List& pts,
                                              H5::Group* h5,
                                              const bool compress)
{
  return detail::WriteListOfPointsAsMatrixH5Helper(field_name, pts, h5, compress);
}

H5::DataSet xreg::WriteListOfPointsAsMatrixH5(const std::string& field_name,
                                              const Pt3List& pts,
                                              H5::Group* h5,
                                              const bool compress)
{
  return detail::WriteListOfPointsAsMatrixH5Helper(field_name, pts, h5, compress);
}

H5::DataSet xreg::WriteListOfArraysToMatrixH5(const std::string& field_name,
                                              const std::vector<std::array<size_type,3>>& arrays,
                                              H5::Group* h5,
                                              const bool compress)
{
  return detail::WriteListOfArraysAsMatrixH5Helper(field_name, arrays, h5, compress);
}

void xreg::WriteSegImageH5(const itk::Image<unsigned char,2>* img, 
                           H5::Group* h5,
                           const std::unordered_map<unsigned char,std::string>& seg_labels_def,
                           const bool compress)
{
  detail::WriteSegNDImageH5Helper(img, h5, seg_labels_def, compress);
}

void xreg::WriteSegImageH5(const itk::Image<unsigned short,2>* img, 
                           H5::Group* h5,
                           const std::unordered_map<unsigned short,std::string>& seg_labels_def,
                           const bool compress)
{
  detail::WriteSegNDImageH5Helper(img, h5, seg_labels_def, compress);
}

void xreg::WriteSegImageH5(const itk::Image<unsigned char,3>* img, 
                           H5::Group* h5,
                           const std::unordered_map<unsigned char,std::string>& seg_labels_def,
                           const bool compress)
{
  detail::WriteSegNDImageH5Helper(img, h5, seg_labels_def, compress);
}

void xreg::WriteSegImageH5(const itk::Image<unsigned short,3>* img, 
                           H5::Group* h5,
                           const std::unordered_map<unsigned short,std::string>& seg_labels_def,
                           const bool compress)
{
  detail::WriteSegNDImageH5Helper(img, h5, seg_labels_def, compress);
}

unsigned char xreg::ReadSingleScalarH5UChar(const std::string& field_name,
                                            const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<unsigned char>(field_name, h5);
}

char xreg::ReadSingleScalarH5Char(const std::string& field_name,
                                  const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<char>(field_name, h5);
}

unsigned short xreg::ReadSingleScalarH5UShort(const std::string& field_name,
                                              const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<unsigned short>(field_name, h5);
}

short xreg::ReadSingleScalarH5Short(const std::string& field_name,
                                    const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<short>(field_name, h5);
}

unsigned int xreg::ReadSingleScalarH5UInt(const std::string& field_name,
                                          const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<unsigned int>(field_name, h5);
}

int xreg::ReadSingleScalarH5Int(const std::string& field_name,
                                const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<int>(field_name, h5);
}

unsigned long xreg::ReadSingleScalarH5ULong(const std::string& field_name,
                                            const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<unsigned long>(field_name, h5);
}

long xreg::ReadSingleScalarH5Long(const std::string& field_name,
                                  const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<long>(field_name, h5);
}

float xreg::ReadSingleScalarH5Float(const std::string& field_name,
                                    const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<float>(field_name, h5);
}

double xreg::ReadSingleScalarH5Double(const std::string& field_name,
                                      const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<double>(field_name, h5);
}

bool xreg::ReadSingleScalarH5Bool(const std::string& field_name,
                                  const H5::Group& h5)
{
  return static_cast<bool>(ReadSingleScalarH5UChar(field_name, h5));
}

xreg::CoordScalar xreg::ReadSingleScalarH5CoordScalar(const std::string& field_name,
                                                      const H5::Group& h5)
{
  return detail::ReadSingleScalarH5Helper<CoordScalar>(field_name, h5);
}

std::vector<unsigned char>
xreg::ReadVectorH5UChar(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<unsigned char>(field_name, h5);
}

std::vector<char>
xreg::ReadVectorH5Char(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<char>(field_name, h5);
}

std::vector<unsigned short>
xreg::ReadVectorH5UShort(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<unsigned short>(field_name, h5);
}

std::vector<short>
xreg::ReadVectorH5Short(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<short>(field_name, h5);
}

std::vector<unsigned int>
xreg::ReadVectorH5UInt(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<unsigned int>(field_name, h5);
}

std::vector<int>
xreg::ReadVectorH5Int(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<int>(field_name, h5);
}

std::vector<unsigned long>
xreg::ReadVectorH5ULong(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<unsigned long>(field_name, h5);
}

std::vector<long>
xreg::ReadVectorH5Long(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<long>(field_name, h5);
}

std::vector<float>
xreg::ReadVectorH5Float(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<float>(field_name, h5);
}

std::vector<double>
xreg::ReadVectorH5Double(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<double>(field_name, h5);
}

std::vector<bool>
xreg::ReadVectorH5Bool(const std::string& field_name, const H5::Group& h5)
{
  const auto hbool_vec = ReadVectorH5UChar(field_name, h5);

  std::vector<bool> v;
  v.reserve(hbool_vec.size());

  for (const auto& b : hbool_vec)
  {
    v.push_back(b);
  }

  return v;
}

std::vector<xreg::CoordScalar>
xreg::ReadVectorH5CoordScalar(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<CoordScalar>(field_name, h5);
}

std::vector<xreg::size_type>
xreg::ReadVectorH5SizeType(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadVectorH5Helper<size_type>(field_name, h5);
}

xreg::MatMxN xreg::ReadMatrixH5CoordScalar(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadMatrixH5Helper<CoordScalar>(field_name, h5);
}

xreg::MatMxN xreg::ReadMatrixH5Float(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadMatrixH5Helper<float>(field_name, h5);
}

xreg::MatMxN_d xreg::ReadMatrixH5Double(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadMatrixH5Helper<double>(field_name, h5);
}

xreg::FrameTransform
xreg::ReadAffineTransform4x4H5(const std::string& field_name,
                               const H5::Group& h5)
{
  FrameTransform xform;

  xform.matrix() = detail::ReadMatrixH5Helper<CoordScalar>(field_name, h5);

  return xform;
}

itk::Image<unsigned char,2>::Pointer
xreg::ReadITKImageH5UChar2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<unsigned char,2>(h5);
}

itk::Image<char,2>::Pointer
xreg::ReadITKImageH5Char2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<char,2>(h5);
}

itk::Image<unsigned short,2>::Pointer
xreg::ReadITKImageH5UShort2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<unsigned short,2>(h5);
}

itk::Image<short,2>::Pointer
xreg::ReadITKImageH5Short2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<short,2>(h5);
}

itk::Image<float,2>::Pointer
xreg::ReadITKImageH5Float2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<float,2>(h5);
}

itk::Image<double,2>::Pointer
xreg::ReadITKImageH5Double2D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<double,2>(h5);
}

itk::Image<unsigned char,3>::Pointer
xreg::ReadITKImageH5UChar3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<unsigned char,3>(h5);
}

itk::Image<char,3>::Pointer
xreg::ReadITKImageH5Char3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<char,3>(h5);
}

itk::Image<unsigned short,3>::Pointer
xreg::ReadITKImageH5UShort3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<unsigned short,3>(h5);
}

itk::Image<short,3>::Pointer
xreg::ReadITKImageH5Short3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<short,3>(h5);
}

itk::Image<float,3>::Pointer
xreg::ReadITKImageH5Float3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<float,3>(h5);
}

itk::Image<double,3>::Pointer
xreg::ReadITKImageH5Double3D(const H5::Group& h5)
{
  return detail::ReadNDImageH5Helper<double,3>(h5);
}

xreg::LandMap2 xreg::ReadLandmarksMapH5Pt2(const H5::Group& h5)
{
  return detail::ReadLandmarksMapH5Helper<Pt2>(h5);
}

xreg::LandMap3 xreg::ReadLandmarksMapH5Pt3(const H5::Group& h5)
{
  return detail::ReadLandmarksMapH5Helper<Pt3>(h5);
}

xreg::LandMap4 xreg::ReadLandmarksMapH5Pt4(const H5::Group& h5)
{
  return detail::ReadLandmarksMapH5Helper<Pt4>(h5);
}

xreg::Pt2List xreg::ReadLandsAsPtCloudH5Pt2(const H5::Group& h5)
{
  return detail::ReadLandsAsPtCloudH5Helper<Pt2>(h5);
}

xreg::Pt3List xreg::ReadLandsAsPtCloudH5Pt3(const H5::Group& h5)
{
  return detail::ReadLandsAsPtCloudH5Helper<Pt3>(h5);
}

xreg::Pt4List xreg::ReadLandsAsPtCloudH5Pt4(const H5::Group& h5)
{
  return detail::ReadLandsAsPtCloudH5Helper<Pt4>(h5);
}

xreg::Pt2List
xreg::ReadListOfPointsFromMatrixH5Pt2(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadListOfPointsFromMatrixH5Helper<CoordScalar,2>(field_name, h5);
}

xreg::Pt3List
xreg::ReadListOfPointsFromMatrixH5Pt3(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadListOfPointsFromMatrixH5Helper<CoordScalar,3>(field_name, h5);
}

xreg::Pt4List
xreg::ReadListOfPointsFromMatrixH5Pt4(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadListOfPointsFromMatrixH5Helper<CoordScalar,4>(field_name, h5);
}

xreg::PtNList
xreg::ReadListOfPointsFromMatrixH5PtN(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadListOfPointsFromMatrixH5Helper<CoordScalar>(field_name, h5);
}

std::vector<std::array<xreg::size_type,3>>
xreg::ReadListOfArraysFromMatrixH5Sizes3(const std::string& field_name, const H5::Group& h5)
{
  return detail::ReadListOfArraysFromMatrixH5Helper<size_type,3>(field_name, h5);
}

std::string xreg::ReadStringH5(const std::string& field_name,
                               const H5::Group& h5)
{
  const H5::DataSet data_set = h5.openDataSet(field_name);
  
  std::string s;
  data_set.read(s, data_set.getStrType());

  return s;
}

std::vector<std::string> xreg::GetH5ObjNames(const H5::Group& h5)
{
  const hsize_t num_objs = h5.getNumObjs();

  std::vector<std::string> names;
  names.reserve(num_objs);
  
  for (hsize_t i = 0; i < num_objs; ++i)
  {
    names.push_back(h5.getObjnameByIdx(i));
  }

  return names;
}

bool xreg::ObjectInGroupH5(const std::string& obj_name, const H5::Group& h5)
{
  const hsize_t num_objs = h5.getNumObjs();

  bool obj_exists = false;
  
  for (hsize_t i = 0; (i < num_objs) && !obj_exists; ++i)
  {
    if (obj_name == h5.getObjnameByIdx(i))
    {
      obj_exists = true;
    }
  }
  
  return obj_exists;
}

H5::Group xreg::OpenOrCreateGroupH5(const std::string& group_name, H5::Group* h5)
{
  return ObjectInGroupH5(group_name, *h5) ? h5->openGroup(group_name) : h5->createGroup(group_name);
}

std::tuple<std::vector<std::string>,std::vector<std::string>>
xreg::GetH5GroupAndDatasetNames(const H5::Group& h5)
{
  const hsize_t num_objs = h5.getNumObjs();

  std::vector<std::string> g_names;
  g_names.reserve(num_objs);
  
  std::vector<std::string> ds_names;
  ds_names.reserve(num_objs);
  
  for (hsize_t i = 0; i < num_objs; ++i)
  {
    const std::string n = h5.getObjnameByIdx(i);

    const auto t = h5.childObjType(n);

    if (t == H5O_TYPE_GROUP)
    {
      g_names.push_back(n);
    }
    else if (t == H5O_TYPE_DATASET)
    {
      ds_names.push_back(n);
    }
    else
    {
      xregThrow("UNKNOWN/UNEXPECTED H5O Type! (%d)", static_cast<int>(t));
    }
  }

  return std::make_tuple(g_names, ds_names);
}

xreg::HideH5ExceptionPrints::HideH5ExceptionPrints()
{
  H5::Exception::getAutoPrint(auto_print_fn, &auto_print_data);
  H5::Exception::dontPrint();
}

xreg::HideH5ExceptionPrints::~HideH5ExceptionPrints()
{
  H5::Exception::setAutoPrint(auto_print_fn, auto_print_data);
}
