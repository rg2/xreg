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

#ifndef XREGITKBASICIMAGEUTILS_H_
#define XREGITKBASICIMAGEUTILS_H_

#include <type_traits>

#include <itkAffineTransform.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>
#include <itkRGBPixel.h>
#include <itkCastImageFilter.h>

#include "xregCommon.h"
#include "xregAssert.h"

namespace xreg
{

/// \brief Convert an Eigen Affine Transformation object to an ITK Affine Transformation object
itk::AffineTransform<CoordScalar,3>::Pointer
ConvertEigenAffineXformToITK(const FrameTransform& eigen_xform);

/// \brief Create an Eigen::Map that references an ITK array
template <class T, unsigned N>
Eigen::Map<Eigen::Matrix<T,N,1>> MapITKToEigen(itk::FixedArray<T,N>& itk_arr)
{
  return Eigen::Map<Eigen::Matrix<T,N,1>>(itk_arr.GetDataPointer());
}

/// \brief Retrieve the minimum and maximum valid indices of an ITK volume - e.g. the valid index bounds.
template <class T, unsigned N>
std::tuple<Eigen::Matrix<CoordScalar,N,1>,Eigen::Matrix<CoordScalar,N,1>>
ITKImageIndexBoundsAsEigen(const itk::Image<T,N>* img)
{
  using ImageType     = itk::Image<T,N>;
  using ImageSizeType = typename ImageType::SizeType;
  
  using Pt = Eigen::Matrix<CoordScalar,N,1>;

  const ImageSizeType img_size = img->GetLargestPossibleRegion().GetSize();

  Pt min_inds;
  Pt max_inds;

  for (unsigned i = 0; i < N; ++i)
  {
    min_inds[i] = 0;  // this should always be the case, to be perfectly correct, should probably look at the index component of img->GetLargestPossibleRegion()
    max_inds[i] = img_size[i] - 1;
  }
  
  return std::make_tuple(min_inds,max_inds);
}

/// \brief Retrieve the extents of the image in terms of the physical units.
template <class T, unsigned N>
Eigen::Matrix<CoordScalar,N,1>
ITKImagePhysicalExtentsAsEigen(const itk::Image<T,N>* img)
{
  using ImageType        = itk::Image<T,N>;
  using ImageSizeType    = typename ImageType::SizeType;
  using ImageSpacingType = typename ImageType::SpacingType;

  const ImageSizeType    img_size    = img->GetLargestPossibleRegion().GetSize();
  const ImageSpacingType img_spacing = img->GetSpacing();

  Eigen::Matrix<CoordScalar,N,1> extents;

  for (unsigned i = 0; i < N; ++i)
  {
    extents[i] = img_size[i] * img_spacing[i];
  }

  return extents;
}

/// \brief Compute the center point of a volume in the volume's physical
///        coordinate frame.
template <class T, unsigned N>
Eigen::Matrix<CoordScalar,N,1>
ITKVol3DCenterAsPhysPt(const itk::Image<T,N>* img)
{
  using ImageType  = itk::Image<T,N>;
  using RegionType = typename ImageType::RegionType;
  
  const RegionType& reg = img->GetLargestPossibleRegion();

  itk::ContinuousIndex<CoordScalar,N> center_idx;
  for (unsigned i = 0; i < N; ++i)
  {
    center_idx[i] = static_cast<CoordScalar>(reg.GetSize(i) / 2.0);
  }
  
  itk::Point<CoordScalar,N> center_pt_itk;
  img->TransformContinuousIndexToPhysicalPoint(center_idx, center_pt_itk);

  Eigen::Matrix<CoordScalar,N,1> center_pt;
  
  for (unsigned i = 0; i < N; ++i)
  {
    center_pt[i] = center_pt_itk[i];
  }
  
  return center_pt;
}

/// \brief Retrieve the transformation from ITK volume indices to the ITK
///        volume's physical coordinate space.
template <class TPixel, unsigned int N>
Eigen::Transform<CoordScalar,N,Eigen::Affine>
ITKImagePhysicalPointTransformsAsEigen(const itk::Image<TPixel,N>* img)
{
  using size_type = unsigned;
  const size_type kDIM = N;

  using ImageType          = itk::Image<TPixel,kDIM>;
  using ImageDirectionType = typename ImageType::DirectionType;
  using ImageSpacingType   = typename ImageType::SpacingType;
  using ImagePoint         = typename ImageType::PointType;

  using OutputTransform = Eigen::Transform<CoordScalar,N,Eigen::Affine>;
  using TransformMatrix = typename OutputTransform::MatrixType;

  const ImagePoint img_origin_pt = img->GetOrigin();

  const ImageSpacingType img_spacing = img->GetSpacing();

  const ImageDirectionType img_mat = img->GetDirection();

  OutputTransform inds_to_phys_pt = OutputTransform::Identity();

  TransformMatrix& dst_mat = inds_to_phys_pt.matrix();
  dst_mat.setIdentity();

  for (size_type row_idx = 0; row_idx < kDIM; ++row_idx)
  {
    for (size_type col_idx = 0; col_idx < kDIM; ++col_idx)
    {
      dst_mat(row_idx,col_idx) = static_cast<CoordScalar>(img_mat(row_idx,col_idx) * img_spacing[col_idx]);
    }

    dst_mat(row_idx,kDIM) = static_cast<CoordScalar>(img_origin_pt[row_idx]);
  }

  return inds_to_phys_pt;
}

/// \brief Retrieve the ITK direction matrix
///
template <class tPixel, unsigned int N>
Eigen::Matrix<CoordScalar,N,N> GetITKDirectionMatrix(const itk::Image<tPixel,N>* img)
{
  using ImageType          = itk::Image<tPixel,N>;
  using ImageDirectionType = typename ImageType::DirectionType;

  using MatrixType = Eigen::Matrix<CoordScalar,N,N>;

  const ImageDirectionType img_dir = img->GetDirection();

  MatrixType mat;

  for (unsigned int r = 0; r < N; ++r)
  {
    for (unsigned int c = 0; c < N; ++c)
    {
      mat(r,c) = static_cast<CoordScalar>(img_dir(r,c));
    }
  }

  return mat;
}

/// \brief Sets the ITK direction matrix
///
template <class tPixel, unsigned N>
void SetITKDirectionMatrix(itk::Image<tPixel,N>* img,
                           const MatMxN& mat)
{
  using ImageType            = itk::Image<tPixel,N>;
  using ImageDirectionType   = typename ImageType::DirectionType;
  using ImageCoordScalarType = typename ImageType::PointValueType;

  xregASSERT((mat.rows() == static_cast<int>(N)) && (mat.rows() == mat.cols()));

  ImageDirectionType img_dir;

  for (unsigned int r = 0; r < N; ++r)
  {
    for (unsigned int c = 0; c < N; ++c)
    {
      img_dir(r,c) = static_cast<ImageCoordScalarType>(mat(r,c));
    }
  }

  img->SetDirection(img_dir);
}

/// \brief Retrieve the ITK origin point
///
template <class tPixel, unsigned int N>
Eigen::Matrix<CoordScalar,N,1> GetITKOriginPoint(const itk::Image<tPixel,N>* img)
{
  using ImageType      = itk::Image<tPixel,N>;
  using ImagePointType = typename ImageType::PointType;

  const ImagePointType img_origin = img->GetOrigin();

  Eigen::Matrix<CoordScalar,N,1> pt;

  for (unsigned int i = 0; i < N; ++i)
  {
    pt(i) = static_cast<CoordScalar>(img_origin[i]);
  }

  return pt;
}

namespace detail
{

template <class tPixel, unsigned int N, int N1, int N2, int N3, int N4, int N5>
void SetITKOriginPointHelper(itk::Image<tPixel,N>* img,
                             const Eigen::Matrix<CoordScalar,N1,N2,N3,N4,N5>& orig_pt)
{
  using ImageType            = itk::Image<tPixel,N>;
  using ImagePointType       = typename ImageType::PointType;
  using ImageCoordScalarType = typename ImageType::PointValueType;

  ImagePointType img_orig_pt;

  for (unsigned int i = 0; i < N; ++i)
  {
    img_orig_pt[i] = static_cast<ImageCoordScalarType>(orig_pt(i));
  }

  img->SetOrigin(img_orig_pt);
}

}  // detail

/// \brief Sets the ITK origin point
///
template <class tPixel, unsigned int N>
void SetITKOriginPoint(itk::Image<tPixel,N>* img,
                       const Eigen::Matrix<CoordScalar,static_cast<unsigned int>(N),1>& orig_pt)
{
  detail::SetITKOriginPointHelper(img, orig_pt);
}

/// \brief Sets the ITK origin point
///
template <class tPixel, unsigned int N>
void SetITKOriginPoint(itk::Image<tPixel,N>* img,
                       const Eigen::Matrix<CoordScalar,1,static_cast<unsigned int>(N)>& orig_pt)
{
  detail::SetITKOriginPointHelper(img, orig_pt);
}

/// \brief Sets the ITK origin point
///
template <class tPixel, unsigned int N>
void SetITKOriginPoint(itk::Image<tPixel,N>* img,
                       const Eigen::Matrix<CoordScalar,Eigen::Dynamic,1>& orig_pt)
{
  xregASSERT(orig_pt.rows() == static_cast<int>(N));
  detail::SetITKOriginPointHelper(img, orig_pt);
}

/// \brief Sets the ITK origin point
///
template <class tPixel, unsigned int N>
void SetITKOriginPoint(itk::Image<tPixel,N>* img,
                       const Eigen::Matrix<CoordScalar,1,Eigen::Dynamic>& orig_pt)
{
  xregASSERT(orig_pt.cols() == static_cast<int>(N));
  detail::SetITKOriginPointHelper(img, orig_pt);
}

/// \brief Sets the ITK origin point
///
template <class tPixel, unsigned int N>
void SetITKOriginPoint(itk::Image<tPixel,N>* img,
                       const Eigen::Matrix<CoordScalar,Eigen::Dynamic,Eigen::Dynamic>& orig_pt)
{
  xregASSERT(((orig_pt.rows() == static_cast<int>(N)) && (orig_pt.cols() == 1)) ||
             ((orig_pt.rows() == 1) && (orig_pt.cols() == static_cast<int>(N))));
  detail::SetITKOriginPointHelper(img, orig_pt);
}

/// \brief Deep copies an ITK image's pixels and metadata.
///
/// Very dumb/slow implementation... TODO: improve it!
template <class T, unsigned int N>
typename itk::Image<T,N>::Pointer
ITKImageDeepCopy(const itk::Image<T,N>* src)
{
  using ImageType    = itk::Image<T,N>;
  using ImagePointer = typename ImageType::Pointer;

  ImagePointer dst = ImageType::New();

  dst->SetDirection(src->GetDirection());
  dst->SetSpacing(src->GetSpacing());
  dst->SetOrigin(src->GetOrigin());
  dst->SetRegions(src->GetLargestPossibleRegion());
  dst->Allocate();

  using PixelType = typename ImageType::PixelType;

  // Do a memcpy if possible
  if (std::is_trivial<PixelType>::value && std::is_standard_layout<PixelType>::value)
  {
    const auto sz = src->GetLargestPossibleRegion().GetSize();
    
    unsigned long num_pix = 1;

    for (typename ImageType::ImageDimensionType i = 0; i < ImageType::ImageDimension; ++i)
    {
      num_pix *= sz[i];
    }

    std::memcpy(dst->GetBufferPointer(), src->GetBufferPointer(),
                sizeof(PixelType) * num_pix);
  }
  else
  {
    // otherwise just loop through each pixel and use the copy operator

    itk::ImageRegionConstIterator<ImageType> src_it(src, src->GetLargestPossibleRegion());
    itk::ImageRegionIterator<ImageType> dst_it(dst, dst->GetLargestPossibleRegion());

    src_it.GoToBegin();
    dst_it.GoToBegin();

    for (; !src_it.IsAtEnd(); ++src_it, ++dst_it)
    {
      dst_it.Set(src_it.Get());
    }
  }

  return dst;
}

/// \brief Checks if two images have the same coordinate frame metadata
///
/// Checks number of pixels in each dimension, pixel spacings, origin points,
/// and direction matrices.
template <class T, unsigned int N, class U>
bool ImagesHaveSameCoords(const itk::Image<T,N>* img1, const itk::Image<U,N>* img2)
{
  return (img1->GetLargestPossibleRegion().GetSize() == img2->GetLargestPossibleRegion().GetSize()) &&   // same number of pixels and layout
         ((img1->GetSpacing() - img2->GetSpacing()).GetNorm() < 1.0e-6) &&   // same pixel spacing
         ((img1->GetOrigin() - img2->GetOrigin()).GetNorm() < 1.0e-6) &&     // same origin point
         ((GetITKDirectionMatrix(img1) * GetITKDirectionMatrix(img2).transpose()).isIdentity(1.0e-6));  // same rotation matrices to physical coords
}

namespace detail
{

template <class tScalar>
struct ItkZero
{
  static tScalar value()
  {
    return tScalar(0);
  }
};

template <class tScalar>
struct ItkZero<itk::RGBPixel<tScalar>>
{
  static itk::RGBPixel<tScalar> value()
  {
    itk::RGBPixel<tScalar> p;
    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    
    return p;
  }
};

}  // detail

/// \brief Create a ND ITK image volume with specified size.
///
/// The pixels are filled with a default value. The spacings
/// and coordinate frames are not set.
template <class tPixelType, unsigned int tN>
typename itk::Image<tPixelType,tN>::Pointer
MakeITKNDVol(const itk::ImageRegion<tN>& img_reg,
             const tPixelType default_val = detail::ItkZero<tPixelType>::value())
{
  using ImageType = itk::Image<tPixelType,tN>;
  using ImagePtr  = typename ImageType::Pointer;

  ImagePtr img = ImageType::New();
  
  img->SetRegions(img_reg);

  img->Allocate();

  img->FillBuffer(default_val);

  return img;
}

/// \brief Create a 2D ITK image volume with specified size.
///
/// The pixels are filled with a default value. The spacings
/// and coordinate frames are not set.
template <class tPixelType>
typename itk::Image<tPixelType,2>::Pointer
MakeITK2DVol(const itk::ImageRegion<2>& img_reg,
             const tPixelType default_val = detail::ItkZero<tPixelType>::value())
{
  return MakeITKNDVol<tPixelType,2>(img_reg, default_val);
}

/// \brief Create a 2D ITK image volume with specified size.
///
/// The pixels are filled with a default value. The spacings
/// and coordinate frames are not set.
template <class tPixelType>
typename itk::Image<tPixelType,2>::Pointer
MakeITK2DVol(const unsigned long x_len, const unsigned long y_len,
             const tPixelType default_val = detail::ItkZero<tPixelType>::value())
{
  using ImageType = itk::Image<tPixelType,2>;

  typename ImageType::RegionType reg;
  reg.SetIndex(0,0);
  reg.SetIndex(1,0);
  reg.SetSize(0, x_len);
  reg.SetSize(1, y_len);

  return MakeITK2DVol<tPixelType>(reg, default_val);
}

/// \brief Create a 3D ITK image volume with specified size.
///
/// The pixels are filled with a default value. The spacings
/// and coordinate frames are not set.
template <class tPixelType>
typename itk::Image<tPixelType,3>::Pointer
MakeITK3DVol(const itk::ImageRegion<3>& img_reg,
             const tPixelType default_val = detail::ItkZero<tPixelType>::value())
{
  return MakeITKNDVol<tPixelType,3>(img_reg, default_val);
}

/// \brief Create a 3D ITK image volume with specified size.
///
/// The pixels are filled with a default value. The spacings
/// and coordinate frames are not set.
template <class tPixelType>
typename itk::Image<tPixelType,3>::Pointer
MakeITK3DVol(const unsigned long x_len, const unsigned long y_len,
             const unsigned long z_len,
             const tPixelType default_val = detail::ItkZero<tPixelType>::value())
{
  using ImageType = itk::Image<tPixelType,3>;

  typename ImageType::RegionType reg;
  reg.SetIndex(0,0);
  reg.SetIndex(1,0);
  reg.SetIndex(2,0);
  reg.SetSize(0, x_len);
  reg.SetSize(1, y_len);
  reg.SetSize(2, z_len);

  return MakeITK3DVol(reg, default_val);
}

template <class tDstScalar, unsigned int tN, class tSrcScalar>
typename itk::Image<tDstScalar,tN>::Pointer
MakeITKVolWithSameCoords(const itk::Image<tSrcScalar,tN>* src,
                         const tDstScalar default_val = detail::ItkZero<tDstScalar>::value())
{
  using DstImg = itk::Image<tDstScalar,tN>;
  using DstPtr = typename DstImg::Pointer;

  DstPtr dst = DstImg::New();

  dst->SetDirection(src->GetDirection());
  dst->SetSpacing(src->GetSpacing());
  dst->SetOrigin(src->GetOrigin());
  dst->SetRegions(src->GetLargestPossibleRegion());
  dst->Allocate();
  
  dst->FillBuffer(default_val);

  return dst;
}

/// \brief Performs a shallow copy from a buffer containing many images,
///        to a list of ITK Image smart pointers.
template <class tScalar, unsigned int tN>
std::vector<typename itk::Image<tScalar,tN>::Pointer>
GetShallowITKImageCopiesFromBuffer(const size_type num_imgs,
                                   const typename itk::Image<tScalar,tN>::SizeType img_size,
                                   void* buf)
{
  using Scalar       = tScalar;
  using Img          = itk::Image<Scalar,tN>;
  using ImgPtr       = typename Img::Pointer;
  using ImgPixelCont = typename Img::PixelContainer;

  std::vector<ImgPtr> imgs(num_imgs);

  constexpr size_type img_dim = tN;

  size_type img_num_pix = 1;
  for (size_type cur_dim = 0; cur_dim < img_dim; ++cur_dim)
  {
    img_num_pix *= img_size[cur_dim];
  }

  for (size_type img_idx = 0; img_idx < num_imgs; ++img_idx)
  {
    ImgPtr& cur_img = imgs[img_idx];

    cur_img = Img::New();

    auto img_pixel_container = ImgPixelCont::New();
    img_pixel_container->SetImportPointer(static_cast<Scalar*>(buf) + (img_num_pix * img_idx), img_num_pix, false);

    cur_img->SetPixelContainer(img_pixel_container);

    typename Img::RegionType img_region;
    img_region.GetModifiableIndex().Fill(0);
    img_region.SetSize(img_size);

    cur_img->SetRegions(img_region);
  }

  return imgs;
}

namespace detail
{

template <class tDstScalar, unsigned int tN, class tSrcScalar>
typename itk::Image<tDstScalar,tN>::Pointer
CastITKImageIfNeeded(itk::Image<tSrcScalar,tN>* src, const std::true_type)
{
  return src;
}

template <class tDstScalar, unsigned int tN, class tSrcScalar>
typename itk::Image<tDstScalar,tN>::Pointer
CastITKImageIfNeeded(itk::Image<tSrcScalar,tN>* src, const std::false_type)
{
  using SrcImg = itk::Image<tSrcScalar,tN>;
  using DstImg = itk::Image<tDstScalar,tN>;

  using Caster = itk::CastImageFilter<SrcImg,DstImg>;
    
  typename Caster::Pointer c = Caster::New();

  c->SetInput(src);
  c->Update();

  return c->GetOutput();
}

}  // detail

template <class tDstScalar, unsigned int tN, class tSrcScalar>
typename itk::Image<tDstScalar,tN>::Pointer
CastITKImageIfNeeded(itk::Image<tSrcScalar,tN>* src)
{
  return detail::CastITKImageIfNeeded<tDstScalar,tN,tSrcScalar>(src, std::is_same<tSrcScalar,tDstScalar>());
}

/// \brief Sets pixels at specific indices in an image to a specific value.
template <class tPixelType, unsigned tN>
void SetIndsToValue(itk::Image<tPixelType,tN>* img, const tPixelType s,
                    const std::vector<typename itk::Image<tPixelType,tN>::IndexType>& inds)
{
  typename itk::Image<tPixelType,tN>::IndexType tmp_idx;

  const size_type num_inds = inds.size();

  for (size_type cur_idx_idx = 0; cur_idx_idx < num_inds; ++cur_idx_idx)
  {
    for (size_type i = 0; i < tN; ++i)
    {
      tmp_idx[i] = inds[cur_idx_idx][i];
    }

    img->SetPixel(tmp_idx, s);
  }
}

}  // xreg

#endif

