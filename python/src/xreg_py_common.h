/*
 * MIT License
 *
 * Copyright (c) 2026 Robert Grupp
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

#ifndef XREG_PY_COMMON_H_
#define XREG_PY_COMMON_H_

#include <algorithm>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <itkImage.h>

#include "xregCommon.h"

namespace nb = nanobind;

namespace xreg
{
namespace py
{

////////////////////////////////////////////////////////////////////////////////
// Images
//
// The Python image classes (Image3DF32, Image2DU8, etc.) are small wrappers
// about an ITK smart pointer, so that images are passed about the Python
// interface without copying pixel data.
//
// A type caster (below) converts between ITK image smart pointers and these
// wrappers, so binding code may directly use ITK image pointers in function
// signatures and struct fields. A null pointer corresponds to None.
//
// Array index ordering follows the conventions of SimpleITK and ITK Python:
// a numpy array of shape (nz, ny, nx) maps to an ITK image of size (nx, ny, nz).
// e.g. the fastest moving index of the numpy array is the first ITK index.

template <class tPixel, unsigned int tN>
struct PyImage
{
  using Pixel   = tPixel;
  using Image   = itk::Image<tPixel,tN>;
  using Pointer = typename Image::Pointer;

  static constexpr unsigned int kDIM = tN;

  Pointer img;

  PyImage() = default;

  explicit PyImage(Pointer p)
    : img(std::move(p))
  { }
};

/// \brief Stand-in for an image object of another Python package (SimpleITK or
///        ITK Python); used to implicitly convert these objects to xreg images.
struct ForeignImage
{
  nb::object obj;
};

template <unsigned int tN>
using VecNd = Eigen::Matrix<double,tN,1>;

template <unsigned int tN>
using MatNd = Eigen::Matrix<double,tN,tN>;

template <class tPixel, unsigned int tN>
using ArrayIn = nb::ndarray<const tPixel, nb::ndim<tN>, nb::c_contig, nb::device::cpu>;

/// \brief Creates an ITK image by copying a numpy array.
template <class tPixel, unsigned int tN>
typename itk::Image<tPixel,tN>::Pointer
ITKImageFromArray(const ArrayIn<tPixel,tN>& arr,
                  const std::optional<VecNd<tN>>& spacing,
                  const std::optional<VecNd<tN>>& origin,
                  const std::optional<MatNd<tN>>& direction)
{
  using Image = itk::Image<tPixel,tN>;

  typename Image::RegionType reg;

  size_type num_pix = 1;

  for (unsigned int i = 0; i < tN; ++i)
  {
    reg.SetIndex(i, 0);
    reg.SetSize(i, arr.shape(tN - 1 - i));

    num_pix *= arr.shape(tN - 1 - i);
  }

  auto img = Image::New();
  img->SetRegions(reg);

  if (spacing)
  {
    typename Image::SpacingType s;
    for (unsigned int i = 0; i < tN; ++i)
    {
      if (!((*spacing)[i] > 0))
      {
        throw nb::value_error("image spacing must be positive");
      }

      s[i] = (*spacing)[i];
    }
    img->SetSpacing(s);
  }

  if (origin)
  {
    typename Image::PointType o;
    for (unsigned int i = 0; i < tN; ++i)
    {
      o[i] = (*origin)[i];
    }
    img->SetOrigin(o);
  }

  if (direction)
  {
    typename Image::DirectionType d;
    for (unsigned int r = 0; r < tN; ++r)
    {
      for (unsigned int c = 0; c < tN; ++c)
      {
        d(r,c) = (*direction)(r,c);
      }
    }
    img->SetDirection(d);
  }

  img->Allocate();

  if (num_pix)
  {
    std::memcpy(img->GetBufferPointer(), arr.data(), num_pix * sizeof(tPixel));
  }

  return img;
}

/// \brief Creates a numpy array which references (does not copy) the pixel
///        buffer of an ITK image. The array keeps the image alive.
template <class tPixel, unsigned int tN>
nb::ndarray<nb::numpy, tPixel, nb::ndim<tN>>
ITKImageToArrayView(const typename itk::Image<tPixel,tN>::Pointer& img)
{
  using Pointer = typename itk::Image<tPixel,tN>::Pointer;

  const auto itk_size = img->GetBufferedRegion().GetSize();

  size_t shape[tN];

  for (unsigned int i = 0; i < tN; ++i)
  {
    shape[i] = itk_size[tN - 1 - i];
  }

  // The capsule owns a smart pointer to the image, which keeps the pixel
  // buffer valid for the lifetime of the numpy array.
  nb::capsule owner(new Pointer(img),
                    [](void* p) noexcept { delete static_cast<Pointer*>(p); });

  return nb::ndarray<nb::numpy, tPixel, nb::ndim<tN>>(img->GetBufferPointer(), tN, shape, owner);
}

/// \brief Creates a numpy array by copying an arbitrary contiguous buffer.
template <class tPixel, unsigned int tN>
nb::ndarray<nb::numpy, tPixel, nb::ndim<tN>>
CopyBufToArray(const tPixel* src, const std::array<size_t,tN>& shape)
{
  size_t num_elems = 1;
  for (const auto& s : shape)
  {
    num_elems *= s;
  }

  tPixel* buf = new tPixel[num_elems ? num_elems : 1];

  if (num_elems)
  {
    std::memcpy(buf, src, num_elems * sizeof(tPixel));
  }

  nb::capsule owner(buf, [](void* p) noexcept { delete[] static_cast<tPixel*>(p); });

  return nb::ndarray<nb::numpy, tPixel, nb::ndim<tN>>(buf, tN, shape.data(), owner);
}

/// \brief Raises a Python ValueError when an ITK image pointer is null.
template <class tPtr>
const tPtr& CheckedImg(const tPtr& img, const char* what = "image")
{
  if (!img)
  {
    throw nb::value_error((std::string(what) + " is None").c_str());
  }

  return img;
}

////////////////////////////////////////////////////////////////////////////////
// Misc. helpers

/// \brief Normalizes a name passed from Python: lower case and '-' -> '_'
inline std::string NormName(const std::string& s)
{
  std::string n = s;
  std::transform(n.begin(), n.end(), n.begin(),
                 [](const unsigned char c) { return (c == '-') ? '_' : static_cast<char>(std::tolower(c)); });
  return n;
}

/// \brief Enables/disables verbose output (to stdout) of an object derived from ObjWithOStream.
template <class tObj>
void SetVerbose(tObj& obj, const bool v)
{
  if (v)
  {
    obj.set_debug_output_stream(std::cout, true);
  }
  else
  {
    obj.disable_debug_output();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Binding functions for each portion of the module

void BindImage(nb::module_& m);

void BindProjData(nb::module_& m);

void BindIO(nb::module_& m);

void BindBackend(nb::module_& m);

void BindRayCast(nb::module_& m);

void BindSimMetrics(nb::module_& m);

void BindOpt(nb::module_& m);

void BindRegi(nb::module_& m);

void BindHip(nb::module_& m);

void BindReplay(nb::module_& m);

}  // py
}  // xreg

NAMESPACE_BEGIN(NB_NAMESPACE)
NAMESPACE_BEGIN(detail)

////////////////////////////////////////////////////////////////////////////////
// Type caster for the Eigen matrices and vectors used by xreg, e.g. Pt3, Mat4x4.
//
// The caster provided by nanobind only accepts array objects (e.g. numpy
// arrays). In addition to arrays, this caster
// accepts (nested) lists and tuples of numbers when implicit conversions are
// allowed, e.g. center=[1, 2, 3] instead of center=numpy.array([1, 2, 3]).
// Values are always copied in both directions.

template <class tScalar>
constexpr const char* EigenSeqDTypeName()
{
  if constexpr (std::is_same_v<tScalar,float>)
  {
    return "float32";
  }
  else if constexpr (std::is_same_v<tScalar,double>)
  {
    return "float64";
  }
  else
  {
    return nullptr;
  }
}

template <class tMat>
struct EigenSeqCaster
{
  using Mat     = tMat;
  using tScalar = typename tMat::Scalar;

  // A distinct, but otherwise equivalent, type which is handled by nanobind's caster
  using Proxy = Eigen::Matrix<tScalar,tMat::RowsAtCompileTime,tMat::ColsAtCompileTime,Eigen::DontAlign>;

  using ProxyCaster = make_caster<Proxy>;

  NB_TYPE_CASTER(Mat, ProxyCaster::Name)

  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    ProxyCaster caster;

    if (caster.from_python(src, flags, cleanup))
    {
      value = caster.value;
      return true;
    }

    const char* dtype_name = EigenSeqDTypeName<tScalar>();

    if (!(flags & (uint32_t) cast_flags::convert) || !dtype_name ||
        !(PyList_Check(src.ptr()) || PyTuple_Check(src.ptr())))
    {
      return false;
    }

    try
    {
      object arr = module_::import_("numpy").attr("asarray")(src, arg("dtype") = dtype_name);

      if (caster.from_python(arr, flags, cleanup))
      {
        value = caster.value;
        return true;
      }
    }
    catch (...)
    {
      // e.g. a ragged or non-numeric sequence
    }

    PyErr_Clear();

    return false;
  }

  static handle from_cpp(const Mat& m, rv_policy, cleanup_list* cleanup) noexcept
  {
    return ProxyCaster::from_cpp(Proxy(m), rv_policy::move, cleanup);
  }
};

// nanobind's caster for Eigen types is a partial specialization which matches all
// plain Eigen types, so full specializations are needed for each type used.
// (variadic, since the commas of the template arguments split the macro argument)
#define XREG_PY_EIGEN_SEQ_CASTER(...) \
  template <> struct type_caster<__VA_ARGS__> : EigenSeqCaster<__VA_ARGS__> { }

XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,2,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,3,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,4,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,Eigen::Dynamic,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,3,3>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,4,4>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,3,4>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<double,2,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<double,3,1>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<double,2,2>);
XREG_PY_EIGEN_SEQ_CASTER(Eigen::Matrix<double,3,3>);

#undef XREG_PY_EIGEN_SEQ_CASTER

////////////////////////////////////////////////////////////////////////////////
// Type caster for xreg::FrameTransform (Eigen::Transform<float,3,Affine>)
// Converts to/from a 4x4 numpy array. Any floating point dtype is accepted
// as input (when implicit conversions are allowed) and float32 is output.

template <>
struct type_caster<xreg::FrameTransform>
{
  using Mat4 = Eigen::Matrix<xreg::CoordScalar,4,4>;

  NB_TYPE_CASTER(xreg::FrameTransform, make_caster<Mat4>::Name)

  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    make_caster<Mat4> mat_caster;

    if (!mat_caster.from_python(src, flags, cleanup))
    {
      return false;
    }

    value.matrix() = mat_caster.value;

    return true;
  }

  static handle from_cpp(const xreg::FrameTransform& xform, rv_policy, cleanup_list* cleanup) noexcept
  {
    return make_caster<Mat4>::from_cpp(Mat4(xform.matrix()), rv_policy::move, cleanup);
  }
};

////////////////////////////////////////////////////////////////////////////////
// Type caster for ITK image smart pointers: converts to/from the Python image
// wrapper classes, with a null pointer corresponding to None.

template <class tPixel, unsigned int tN>
struct type_caster<itk::SmartPointer<itk::Image<tPixel,tN>>>
{
  using Wrapper       = xreg::py::PyImage<tPixel,tN>;
  using WrapperCaster = make_caster<Wrapper>;

  // (alias needed since the commas of the template arguments would split the macro argument)
  using ImgPtr = itk::SmartPointer<itk::Image<tPixel,tN>>;

  NB_TYPE_CASTER(ImgPtr, WrapperCaster::Name)

  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    if (src.is_none())
    {
      value = nullptr;
      return true;
    }

    WrapperCaster caster;

    if (!caster.from_python(src, flags, cleanup))
    {
      return false;
    }

    Wrapper* w = caster.operator Wrapper*();

    if (!w)
    {
      return false;
    }

    value = w->img;

    return true;
  }

  static handle from_cpp(const Value& img, rv_policy, cleanup_list* cleanup) noexcept
  {
    if (!img)
    {
      return none().release();
    }

    return WrapperCaster::from_cpp(Wrapper(img), rv_policy::move, cleanup);
  }
};

// None is accepted for ITK image pointer arguments without needing to annotate
// each argument with .none()
template <class tPixel, unsigned int tN>
struct has_arg_defaults<itk::SmartPointer<itk::Image<tPixel,tN>>> : std::true_type { };

////////////////////////////////////////////////////////////////////////////////
// Type caster for xreg::py::ForeignImage: accepts image objects created by the
// SimpleITK or ITK Python packages (determined using the module name of the
// object's type, so that neither package needs to be imported by xreg).

template <>
struct type_caster<xreg::py::ForeignImage>
{
  NB_TYPE_CASTER(xreg::py::ForeignImage, const_name("object"))

  bool from_python(handle src, uint32_t, cleanup_list*) noexcept
  {
    try
    {
      const std::string mod_name = cast<std::string>(src.type().attr("__module__"));

      if ((mod_name.compare(0, 9, "SimpleITK") == 0) || (mod_name.compare(0, 3, "itk") == 0))
      {
        value.obj = borrow(src);
        return true;
      }
    }
    catch (...)
    {
      PyErr_Clear();
    }

    return false;
  }

  static handle from_cpp(const xreg::py::ForeignImage& img, rv_policy, cleanup_list*) noexcept
  {
    return handle(img.obj).inc_ref();
  }
};

NAMESPACE_END(detail)
NAMESPACE_END(NB_NAMESPACE)

#endif
