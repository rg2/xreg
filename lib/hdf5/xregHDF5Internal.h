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

#ifndef XREGHDF5INTERNAL_H_
#define XREGHDF5INTERNAL_H_

#include "xregHDF5.h"

namespace xreg
{
namespace detail
{

template <class tScalar>
H5::DataSet WriteSingleScalarH5Helper(const std::string& field_name,
                                      const tScalar& field_val,
                                      H5::Group* h5)
{
  const auto data_type = LookupH5DataType<tScalar>();

  H5::DataSet data_set = h5->createDataSet(field_name, data_type, H5S_SCALAR);  

  data_set.write(&field_val, data_type);

  return data_set;
}

template <class T>
H5::DataSet CreateVectorH5Helper(const std::string& field_name,
                                 const unsigned long len,
                                 H5::Group* h5,
                                 const bool compress)
{
  using Scalar = typename std::conditional<std::is_same<bool,T>::value,unsigned char,T>::type;
  
  H5::DSetCreatPropList props;
  props.copy(H5::DSetCreatPropList::DEFAULT);
  
  if (compress)
  {
    // Maximum ~1 MB chunk size (fits in default chunk cache)
    const hsize_t chunk_dims = std::min(static_cast<size_type>(len), size_type((1024 * 1024 * 1) / sizeof(Scalar)));
    props.setChunk(1, &chunk_dims);
    props.setDeflate(9);
  }
  
  const hsize_t len_tmp = len;
  H5::DataSpace data_space(1, &len_tmp);

  const auto data_type = LookupH5DataType<Scalar>();

  return h5->createDataSet(field_name, data_type, data_space, props);
}

template <class T, class A>
H5::DataSet WriteVectorH5Helper(const std::string& field_name,
                                const std::vector<T,A>& v,
                                H5::Group* h5,
                                const bool compress)
{
  using Scalar = T;
  
  H5::DataSet data_set = CreateVectorH5Helper<Scalar>(field_name, v.size(), h5, compress);
  data_set.write(&v[0], LookupH5DataType<Scalar>());

  return data_set; 
}

template <class T>
void WriteVectorElemH5Helper(const T& x, const unsigned long i, H5::DataSet* h5)
{
  H5::DataSpace f_ds = h5->getSpace();
  
  const hsize_t cur_idx = i;

  f_ds.selectElements(H5S_SELECT_SET, 1, &cur_idx);

  const hsize_t single_elem = 1;
  H5::DataSpace m_ds(1, &single_elem);

  h5->write(&x, LookupH5DataType<T>(), m_ds, f_ds);
}

template <class tScalar>
H5::DataSet CreateMatrixH5Helper(const std::string& field_name,
                                 const unsigned long num_rows,
                                 const unsigned long num_cols,
                                 H5::Group* h5,
                                 const bool compress)
{
  using Scalar = tScalar;

  H5::DSetCreatPropList props;
  props.copy(H5::DSetCreatPropList::DEFAULT);
  
  if (compress)
  {
    // Maximum ~1 MB chunk size (fits in default chunk cache)
    
    constexpr hsize_t max_elems_per_dim = 1024 / sizeof(Scalar);
    const std::array<hsize_t,2> chunk_dims = { std::min(static_cast<hsize_t>(num_rows), max_elems_per_dim),
                                               std::min(static_cast<hsize_t>(num_cols), max_elems_per_dim) };

    props.setChunk(2, chunk_dims.data());
    props.setDeflate(9);
  }
  
  const std::array<hsize_t,2> mat_dims = { static_cast<hsize_t>(num_rows),
                                           static_cast<hsize_t>(num_cols) };
  H5::DataSpace data_space(2, mat_dims.data());

  const auto data_type = LookupH5DataType<Scalar>();

  return h5->createDataSet(field_name, data_type, data_space, props);
}

template <class tScalar, int tRows, int tCols, int tOpts, int tMaxRows, int tMaxCols>
H5::DataSet WriteMatrixH5Helper(const std::string& field_name,
                                const Eigen::Matrix<tScalar,tRows,tCols,tOpts,tMaxRows,tMaxCols>& mat,
                                H5::Group* h5,
                                const bool compress)
{
  using Scalar = tScalar;

  H5::DataSet data_set = CreateMatrixH5Helper<Scalar>(field_name, mat.rows(), mat.cols(), h5, compress);

  const auto data_type = LookupH5DataType<Scalar>();

  // HDF5 is row-major

  if (tOpts & Eigen::RowMajor)
  {
    // input matrix is row-major, we can just write the entire memory block
    data_set.write(&mat(0,0), data_type);
  }
  else
  {
    // input matrix is column-major, we need to write a column at a time

    H5::DataSpace data_space = data_set.getSpace();

    const size_type nr = static_cast<size_type>(mat.rows());
    const size_type nc = static_cast<size_type>(mat.cols());
 
    std::array<hsize_t,2> file_start = { 0, 0 };

    const std::array<hsize_t,2> file_count = { nr, 1 };

    // data space for a single column
    const H5::DataSpace mem_col_data_space(2, file_count.data());

    for (size_type c = 0; c < nc; ++c)
    {
      // select the current column in the hdf5 file
      file_start[1] = c;
      data_space.selectHyperslab(H5S_SELECT_SET, file_count.data(), file_start.data());
      
      data_set.write(&mat(0,c), data_type, mem_col_data_space, data_space);
    }
  }

  return data_set; 
}

template <class tScalar>
void WriteMatrixRowH5Helper(const tScalar* row_buf, const unsigned long row_idx, H5::DataSet* h5)
{
  using Scalar = tScalar;

  H5::DataSpace ds_f = h5->getSpace();
  xregASSERT(ds_f.getSimpleExtentNdims() == 2);

  std::array<hsize_t,2> f_dims;
  ds_f.getSimpleExtentDims(f_dims.data());
  xregASSERT(row_idx < f_dims[0]);

  const std::array<hsize_t,2> m_dims = { 1, f_dims[1] };
  H5::DataSpace ds_m(2, m_dims.data());
  
  const std::array<hsize_t,2> f_start = { row_idx, 0 };

  ds_f.selectHyperslab(H5S_SELECT_SET, m_dims.data(), f_start.data());

  h5->write(row_buf, LookupH5DataType<Scalar>(), ds_m, ds_f);
}

template <class tScalar, unsigned int tN>
void WriteNDImageH5Helper(const itk::Image<tScalar,tN>* img, 
                          H5::Group* h5,
                          const bool compress)
{
  using PixelScalar = tScalar;

  constexpr unsigned int kDIM = tN;

  // Set an attribute that indicates this is an N-D image
  SetStringAttr("xreg-type", fmt::format("image-{}D", kDIM), h5);

  // first write the image metadata

  WriteMatrixH5("dir-mat", GetITKDirectionMatrix(img), h5, false);

  WriteMatrixH5("origin", GetITKOriginPoint(img), h5, false);

  const auto itk_spacing = img->GetSpacing();
  Eigen::Matrix<CoordScalar,kDIM,1> spacing;
  for (unsigned int i = 0; i < kDIM; ++i)
  {
    spacing[i] = itk_spacing[i];
  }
  WriteMatrixH5("spacing", spacing, h5, false);

  // Now write the pixel data

  const auto itk_size = img->GetLargestPossibleRegion().GetSize();

  std::array<hsize_t,kDIM> img_dims;
  for (unsigned int i = 0; i < kDIM; ++i)
  {
    // reverse the order - ITK buffer is "row major" but the index and
    // size orderings are reversed.
    img_dims[i] = itk_size[kDIM - 1 - i];
  }
  
  H5::DSetCreatPropList props;
  props.copy(H5::DSetCreatPropList::DEFAULT);

  if (compress)
  {
    std::array<hsize_t,kDIM> chunk_dims = img_dims;

    // for now 1 MB max chunk size
    constexpr unsigned long max_num_elems_for_chunk = (1 * 1024 * 1024)
                                                          / sizeof(PixelScalar);

    bool chunk_ok = false;

    while (!chunk_ok)
    {
      bool found_non_one_chunk_size = false;
      unsigned long elems_per_chunk = 1;
      for (const auto& cd : chunk_dims)
      {
        elems_per_chunk *= cd;
        
        if (cd != 1)
        {
          found_non_one_chunk_size = true;
        }
      }

      if (!found_non_one_chunk_size || (elems_per_chunk <= max_num_elems_for_chunk))
      {
        chunk_ok = true;
      }
      else
      {
        for (unsigned int cur_dim = 0; cur_dim < kDIM; ++cur_dim)
        {
          if (chunk_dims[cur_dim] != 1)
          {
            --chunk_dims[cur_dim];
            break;
          }
        }
      }
    }
    
    props.setChunk(kDIM, chunk_dims.data());
    props.setDeflate(9);
  }
  
  const auto data_type = LookupH5DataType<PixelScalar>();
  
  H5::DataSpace data_space(kDIM, img_dims.data());

  H5::DataSet data_set = h5->createDataSet("pixels", data_type, data_space, props);

  data_set.write(img->GetBufferPointer(), data_type);
}

template <class tMapIt>
void WriteLandmarksMapH5Helper(tMapIt map_begin, tMapIt map_end, H5::Group* h5)
{
  using MapIt = tMapIt;

  SetStringAttr("xreg-type", "lands-map", h5);

  for (MapIt it = map_begin; it != map_end; ++it)
  {
    WriteMatrixH5(it->first, it->second, h5, false);
  }
}

template <class tKey, class tVal>
void WriteLandmarksMapH5Helper(const std::unordered_map<tKey,tVal>& m, H5::Group* h5)
{
  WriteLandmarksMapH5Helper(m.begin(), m.end(), h5);
}

template <class tScalar, int tRows, int tCols, int tOpts, int tMaxRows, int tMaxCols, class A>
H5::DataSet WriteListOfPointsAsMatrixH5Helper(const std::string& field_name,
                          const std::vector<Eigen::Matrix<tScalar,tRows,tCols,tOpts,tMaxRows,tMaxCols>,A>& pts,
                          H5::Group* h5,
                          const bool compress)
{
  using Pt  = Eigen::Matrix<tScalar,tRows,tCols,tOpts,tMaxRows,tMaxCols>;
  using Mat = Eigen::Matrix<tScalar,Eigen::Dynamic,Eigen::Dynamic>;

  xregASSERT(!pts.empty());

  const int nr = pts[0].rows();
  const int nc = pts[0].cols();

  xregASSERT(nc == 1);
  
  const int num_pts = pts.size();

  Mat mat(nr,num_pts);

  for (int i = 0; i < num_pts; ++i)
  {
    mat.block(0,i,nr,1) = pts[i];
  }
 
  return WriteMatrixH5(field_name, mat, h5, compress);
}

template <class tScalar, unsigned long tDim>
H5::DataSet WriteListOfArraysAsMatrixH5Helper(const std::string& field_name,
                                              const std::vector<std::array<tScalar,tDim>>& arrays,
                                              H5::Group* h5,
                                              const bool compress)
{
  constexpr int kDIM = tDim;

  using Mat = Eigen::Matrix<tScalar,Eigen::Dynamic,Eigen::Dynamic>;

  xregASSERT(!arrays.empty());

  const int num_arrays = static_cast<int>(arrays.size());
  
  Mat mat(kDIM, num_arrays);

  for (int array_idx = 0; array_idx < num_arrays; ++array_idx)
  {
    const auto& cur_array = arrays[array_idx];

    for (int r = 0; r < kDIM; ++r)
    {
      mat(r,array_idx) = cur_array[r];
    }
  }
  
  return WriteMatrixH5Helper(field_name, mat, h5, compress);
}

template <class tLabelScalar, unsigned int tN>
void WriteSegNDImageH5Helper(const itk::Image<tLabelScalar,tN>* img, 
                             H5::Group* h5,
                             const std::unordered_map<tLabelScalar,std::string>& seg_labels_def,
                             const bool compress)
{
  using LabelScalar = tLabelScalar;

  constexpr unsigned int kDIM = tN;

  // Set an attribute that indicates this is an N-D image segmentation
  SetStringAttr("xreg-type", fmt::format("image-seg-{}D", kDIM), h5);

  // write the label image
  {
    H5::Group seg_img_g = h5->createGroup("image");
    WriteImageH5(img, &seg_img_g, compress);
  }
  
  // write the labels def if available
  if (!seg_labels_def.empty())
  {
    H5::Group seg_labs_def_g = h5->createGroup("labels-def");
    
    for (const auto& kv : seg_labels_def)
    {
      WriteStringH5(fmt::format("{}", kv.first), kv.second, &seg_labs_def_g);
    }
  }
}

template <class tKey, class tVal>
void WriteMapAsArraysHelper(const std::unordered_map<tKey,tVal>& m, H5::Group* h5,
                            const std::string& keys_g_name = "keys",
                            const std::string& vals_g_name = "vals",
                            const bool compress = true)
{
  using MapKey  = tKey;
  using KeyList = std::vector<MapKey>;
  using MapVal  = tVal;
  using ValList = std::vector<MapVal>;
  
  SetStringAttr("xreg-type", "map-as-arrays", h5);
  SetStringAttr("xreg-keys-g-name", keys_g_name, h5);
  SetStringAttr("xreg-vals-g-name", vals_g_name, h5);

  const auto len = m.size();
  
  KeyList keys;
  ValList vals;

  keys.reserve(len);
  vals.reserve(len);
  
  for (const auto& kv : m)
  {
    keys.push_back(kv.first);
    vals.push_back(kv.second);
  }

  WriteVectorH5(keys_g_name, keys, h5, compress);
  WriteVectorH5(vals_g_name, vals, h5, compress);
}

template <class tScalar>
tScalar ReadSingleScalarH5Helper(const std::string& field_name,
                                 const H5::Group& h5)
{
  using Scalar = tScalar;

  const H5::DataSet data_set = h5.openDataSet(field_name);
  
  Scalar x;
  data_set.read(&x, LookupH5DataType<Scalar>());

  return x;
}

template <class T>
std::vector<T> ReadVectorH5Helper(const std::string& field_name,
                                  const H5::Group& h5)
{
  using Scalar = T;
  using Vec    = std::vector<Scalar>;

  const H5::DataSet data_set = h5.openDataSet(field_name);

  const H5::DataSpace data_space = data_set.getSpace();

  xregASSERT(data_space.getSimpleExtentNdims() == 1);

  hsize_t len = 0;
  data_space.getSimpleExtentDims(&len);

  Vec v(len);
  data_set.read(&v[0], LookupH5DataType<Scalar>());

  return v;
}

template <class tScalar>
Eigen::Matrix<tScalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor|Eigen::AutoAlign>
ReadMatrixH5Helper(const std::string& field_name, const H5::Group& h5)
{
  using Scalar = tScalar;
  using Mat    = Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,
                               Eigen::RowMajor|Eigen::AutoAlign>;

  const H5::DataSet data_set = h5.openDataSet(field_name);

  const H5::DataSpace data_space = data_set.getSpace();

  xregASSERT(data_space.getSimpleExtentNdims() == 2);

  std::array<hsize_t,2> dims = { 0, 0 };
  data_space.getSimpleExtentDims(dims.data());

  static_assert(!std::is_same<Scalar,bool>::value,
                "read bool matrix not implemented! need to read unsigned char!");

  Mat m(dims[0], dims[1]);
  data_set.read(&m(0,0), LookupH5DataType<Scalar>());

  return m;
}

template <class tScalar, unsigned int tN>
typename itk::Image<tScalar,tN>::Pointer
ReadNDImageH5Helper(const H5::Group& h5)
{
  using PixelScalar = tScalar;

  constexpr unsigned int kDIM = tN;

  using Img            = itk::Image<PixelScalar,kDIM>;
  using ImgCoordScalar = typename Img::SpacingValueType;

  const auto dir_mat   = ReadMatrixH5Float("dir-mat", h5);
  const auto origin_pt = ReadMatrixH5Float("origin",  h5);
  const auto spacing   = ReadMatrixH5Float("spacing", h5);
  
  const H5::DataSet data_set = h5.openDataSet("pixels");

  const H5::DataSpace data_space = data_set.getSpace();

  xregASSERT(data_space.getSimpleExtentNdims() == kDIM);

  std::array<hsize_t,kDIM> dims;
  data_space.getSimpleExtentDims(dims.data());

  auto img = Img::New();

  SetITKDirectionMatrix(img.GetPointer(), dir_mat);
  SetITKOriginPoint(img.GetPointer(), origin_pt);

  typename Img::SpacingType itk_spacing;
  for (unsigned int i = 0; i < kDIM; ++i)
  {
    itk_spacing[i] = static_cast<ImgCoordScalar>(spacing(i));
  }
  img->SetSpacing(itk_spacing);

  typename Img::RegionType reg;
  for (unsigned int i = 0; i < kDIM; ++i)
  {
    reg.SetIndex(i,0);
    reg.SetSize(i, dims[kDIM - 1 - i]);  // See WriteITKImageH5
  }
  img->SetRegions(reg);

  img->Allocate();

  data_set.read(img->GetBufferPointer(), LookupH5DataType<PixelScalar>());

  return img;
}

template <class tPt>
std::unordered_map<std::string,tPt>
ReadLandmarksMapH5Helper(const H5::Group& h5)
{
  using Pt = tPt;
  using LandsMap = std::unordered_map<std::string,Pt>;
  using Scalar = typename Pt::Scalar;

  const hsize_t num_lands = h5.getNumObjs();

  LandsMap lands;

  if (num_lands)
  {
    lands.reserve(num_lands);

    for (hsize_t l = 0; l < num_lands; ++l)
    {
      const std::string land_name = h5.getObjnameByIdx(l);

      lands.insert(typename LandsMap::value_type(land_name,
                        ReadMatrixH5Helper<Scalar>(land_name, h5)));
    }
  }

  return lands;
}

template <class tPt>
std::vector<tPt> ReadLandsAsPtCloudH5Helper(const H5::Group& h5)
{
  using Pt     = tPt;
  using PtList = std::vector<Pt>;
  using Scalar = typename Pt::Scalar;

  const hsize_t num_lands = h5.getNumObjs();

  PtList pts;

  if (num_lands)
  {
    pts.reserve(num_lands);

    for (hsize_t l = 0; l < num_lands; ++l)
    {
      pts.push_back(ReadMatrixH5Helper<Scalar>(h5.getObjnameByIdx(l), h5));
    }
  }

  return pts;
}

template <class tScalar, int tDim = Eigen::Dynamic>
std::vector<Eigen::Matrix<tScalar,tDim,1>>
ReadListOfPointsFromMatrixH5Helper(const std::string& field_name, const H5::Group& h5)
{
  using Scalar = tScalar;

  constexpr int kDIM = tDim;

  using Pt = Eigen::Matrix<Scalar,kDIM,1>;
  
  using PtList = std::vector<Pt>;

  const auto mat = ReadMatrixH5Helper<Scalar>(field_name, h5);
  
  const int dim = mat.rows();

  xregASSERT((kDIM == Eigen::Dynamic) || (kDIM == dim));

  const int num_pts = mat.cols();

  PtList pts;
  pts.reserve(num_pts);

  for (int i = 0; i < num_pts; ++i)
  {
    pts.push_back(mat.block(0,i,dim,1));
  }

  return pts;
}

template <class tScalar, unsigned long tDim>
std::vector<std::array<tScalar,tDim>>
ReadListOfArraysFromMatrixH5Helper(const std::string& field_name, const H5::Group& h5)
{
  using Scalar = tScalar;

  constexpr unsigned long kDIM = tDim;

  using Array        = std::array<Scalar,kDIM>;
  using ListOfArrays = std::vector<Array>;

  const auto mat = ReadMatrixH5Helper<Scalar>(field_name, h5);

  xregASSERT(static_cast<unsigned long>(mat.rows() == kDIM));

  const int num_arrays = mat.cols();

  ListOfArrays arrays(num_arrays);
  
  for (int array_idx = 0; array_idx < num_arrays; ++array_idx)
  {
    auto& cur_array = arrays[array_idx];

    for (unsigned long i = 0; i < kDIM; ++i)
    {
      cur_array[i] = mat(i,array_idx);
    }
  }

  return arrays;
}


}  // detail
}  // xreg

#endif

