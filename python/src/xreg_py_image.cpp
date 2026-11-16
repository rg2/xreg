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

#include "xreg_py_common.h"

#include <sstream>

#include <nanobind/operators.h>

#include "xregITKBasicImageUtils.h"
#include "xregITKLabelUtils.h"
#include "xregHUToLinAtt.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

template <class tPixel>
struct PixelInfo;

template <>
struct PixelInfo<float>
{
  static constexpr const char* kDTYPE = "float32";
  static constexpr const char* kDESC  = "Single precision floating point";
};

template <>
struct PixelInfo<unsigned char>
{
  static constexpr const char* kDTYPE = "uint8";
  static constexpr const char* kDESC  = "Unsigned 8-bit integer (e.g. label map or mask)";
};

// Converts an image object from SimpleITK or ITK Python by calling into the
// pure Python portion of the package, which extracts a numpy array and the
// image geometry.
template <class tPixel, unsigned int tN>
typename itk::Image<tPixel,tN>::Pointer
ITKImageFromForeign(const ForeignImage& src)
{
  nb::object info = nb::module_::import_("xreg._image_interop")
                        .attr("_foreign_to_array_and_geom")(src.obj, tN, PixelInfo<tPixel>::kDTYPE);

  auto t = nb::cast<nb::tuple>(info);

  return ITKImageFromArray<tPixel,tN>(nb::cast<ArrayIn<tPixel,tN>>(t[0]),
                                      nb::cast<VecNd<tN>>(t[1]),
                                      nb::cast<VecNd<tN>>(t[2]),
                                      nb::cast<MatNd<tN>>(t[3]));
}

template <class tPixel, unsigned int tN>
void BindPyImage(nb::module_& m, const char* name)
{
  using Img     = PyImage<tPixel,tN>;
  using ITKImg  = itk::Image<tPixel,tN>;
  using ITKPtr  = typename ITKImg::Pointer;
  using ContIdx = itk::ContinuousIndex<double,tN>;

  const std::string class_doc = std::string(PixelInfo<tPixel>::kDESC) + " " + std::to_string(tN) +
    "D image (wraps an ITK image). Create with xreg.from_numpy(), xreg.from_sitk(), "
    "xreg.from_itk(), or xreg.read_image(). SimpleITK and ITK Python images are also converted "
    "automatically when passed to a function expecting this type (pixel data is copied and cast). "
    "Array index order matches SimpleITK and ITK Python: a numpy array of shape (nz, ny, nx) "
    "corresponds to an image of size (nx, ny, nz). Spacing, origin, direction, and size use ITK "
    "order (x first).";

  nb::class_<Img>(m, name, class_doc.c_str())
    .def("__init__",
        [](Img* img, const ForeignImage& src)
        {
          new (img) Img(ITKImageFromForeign<tPixel,tN>(src));
        },
        "image"_a,
        "Converts a SimpleITK or ITK Python image, pixel data is copied and cast to the pixel "
        "type of this class.")
    .def_static("from_numpy",
        [](ArrayIn<tPixel,tN> arr,
           const std::optional<VecNd<tN>>& spacing,
           const std::optional<VecNd<tN>>& origin,
           const std::optional<MatNd<tN>>& direction)
        {
          return ITKImageFromArray<tPixel,tN>(arr, spacing, origin, direction);
        },
        "array"_a, "spacing"_a = nb::none(), "origin"_a = nb::none(), "direction"_a = nb::none(),
        "Creates an image by copying a numpy array (numpy/SimpleITK index order). "
        "The spacing, origin, and direction are specified in ITK order (e.g. x first) "
        "and default to unit spacing, zero origin, and identity direction. "
        "See xreg.from_numpy() for a version which also converts the dtype.")
    .def("numpy",
        [](const Img& img)
        {
          return ITKImageToArrayView<tPixel,tN>(img.img);
        },
        "Returns a numpy array that shares memory with this image (numpy/SimpleITK index "
        "order); no pixel data is copied and modifying the array modifies the image. "
        "numpy.asarray(img) is equivalent, use numpy.array(img) for a copy.")
    .def("__array__",
        [](const Img& img, nb::object dtype, nb::object copy)
        {
          nb::object arr = nb::cast(ITKImageToArrayView<tPixel,tN>(img.img));

          if (!dtype.is_none())
          {
            arr = arr.attr("astype")(dtype, "copy"_a = false);
          }

          if (!copy.is_none() && nb::cast<bool>(copy))
          {
            arr = arr.attr("copy")();
          }

          return arr;
        },
        "dtype"_a = nb::none(), "copy"_a = nb::none())
    .def_prop_rw("spacing",
        [](const Img& img)
        {
          const auto s = img.img->GetSpacing();
          VecNd<tN> v;
          for (unsigned int i = 0; i < tN; ++i)
          {
            v[i] = s[i];
          }
          return v;
        },
        [](Img& img, const VecNd<tN>& v)
        {
          typename ITKImg::SpacingType s;
          for (unsigned int i = 0; i < tN; ++i)
          {
            if (!(v[i] > 0))
            {
              throw nb::value_error("image spacing must be positive");
            }

            s[i] = v[i];
          }
          img.img->SetSpacing(s);
        },
        "Pixel spacing (ITK order).")
    .def_prop_rw("origin",
        [](const Img& img)
        {
          const auto o = img.img->GetOrigin();
          VecNd<tN> v;
          for (unsigned int i = 0; i < tN; ++i)
          {
            v[i] = o[i];
          }
          return v;
        },
        [](Img& img, const VecNd<tN>& v)
        {
          typename ITKImg::PointType o;
          for (unsigned int i = 0; i < tN; ++i)
          {
            o[i] = v[i];
          }
          img.img->SetOrigin(o);
        },
        "Physical location of the first pixel (ITK order).")
    .def_prop_rw("direction",
        [](const Img& img)
        {
          const auto& d = img.img->GetDirection();
          MatNd<tN> dm;
          for (unsigned int r = 0; r < tN; ++r)
          {
            for (unsigned int c = 0; c < tN; ++c)
            {
              dm(r,c) = d(r,c);
            }
          }
          return dm;
        },
        [](Img& img, const MatNd<tN>& dm)
        {
          typename ITKImg::DirectionType d;
          for (unsigned int r = 0; r < tN; ++r)
          {
            for (unsigned int c = 0; c < tN; ++c)
            {
              d(r,c) = dm(r,c);
            }
          }
          img.img->SetDirection(d);
        },
        "Direction cosine matrix (ITK convention, columns are the image axes).")
    .def_prop_ro("size",
        [](const Img& img)
        {
          const auto s = img.img->GetLargestPossibleRegion().GetSize();
          nb::list l;
          for (unsigned int i = 0; i < tN; ++i)
          {
            l.append(s[i]);
          }
          return nb::tuple(l);
        },
        "Number of pixels along each dimension (ITK order, e.g. (nx, ny, nz)).")
    .def_prop_ro("shape",
        [](const Img& img)
        {
          const auto s = img.img->GetLargestPossibleRegion().GetSize();
          nb::list l;
          for (unsigned int i = 0; i < tN; ++i)
          {
            l.append(s[tN - 1 - i]);
          }
          return nb::tuple(l);
        },
        "Shape of the corresponding numpy array (numpy order, e.g. (nz, ny, nx)).")
    .def_prop_ro("ndim", [](const Img&) { return tN; }, "Number of image dimensions.")
    .def_prop_ro("dtype", [](const Img&) { return PixelInfo<tPixel>::kDTYPE; },
                 "Name of the numpy dtype of the pixels.")
    .def("index_to_physical_point",
        [](const Img& img, const VecNd<tN>& idx)
        {
          ContIdx ci;
          for (unsigned int i = 0; i < tN; ++i)
          {
            ci[i] = idx[i];
          }

          typename ITKImg::PointType p;
          img.img->TransformContinuousIndexToPhysicalPoint(ci, p);

          VecNd<tN> v;
          for (unsigned int i = 0; i < tN; ++i)
          {
            v[i] = p[i];
          }
          return v;
        },
        "index"_a,
        "Maps a (continuous) pixel index in ITK order (x first) to a physical point.")
    .def("physical_point_to_index",
        [](const Img& img, const VecNd<tN>& pt)
        {
          typename ITKImg::PointType p;
          for (unsigned int i = 0; i < tN; ++i)
          {
            p[i] = pt[i];
          }

          const ContIdx ci = img.img->template TransformPhysicalPointToContinuousIndex<double>(p);

          VecNd<tN> v;
          for (unsigned int i = 0; i < tN; ++i)
          {
            v[i] = ci[i];
          }
          return v;
        },
        "point"_a,
        "Maps a physical point to a continuous pixel index in ITK order (x first).")
    .def("copy",
        [](const Img& img) -> ITKPtr
        {
          return ITKImageDeepCopy(img.img.GetPointer());
        },
        "Returns a deep copy of this image.")
    .def("__eq__",
        [](const Img& a, const Img& b) { return a.img == b.img; },
        nb::is_operator(),
        "True when both objects reference the same underlying image (pixel data is not compared).")
    .def("__hash__",
        [](const Img& img) { return std::hash<const void*>()(img.img.GetPointer()); })
    .def("__repr__",
        [name](const Img& img)
        {
          std::stringstream ss;
          ss << name << "(size=(";

          const auto s  = img.img->GetLargestPossibleRegion().GetSize();
          const auto sp = img.img->GetSpacing();

          for (unsigned int i = 0; i < tN; ++i)
          {
            ss << s[i] << ((i + 1) < tN ? ", " : ")");
          }
          ss << ", spacing=(";
          for (unsigned int i = 0; i < tN; ++i)
          {
            ss << sp[i] << ((i + 1) < tN ? ", " : ")");
          }
          ss << ')';
          return ss.str();
        });

  // Allows SimpleITK and ITK Python images to be passed directly where this type is expected
  nb::implicitly_convertible<ForeignImage,Img>();
}

}  // un-named

void xreg::py::BindImage(nb::module_& m)
{
  using VolF32    = itk::Image<float,3>;
  using VolF32Ptr = VolF32::Pointer;
  using VolU8Ptr  = itk::Image<unsigned char,3>::Pointer;

  BindPyImage<float,3>(m, "Image3DF32");
  BindPyImage<unsigned char,3>(m, "Image3DU8");
  BindPyImage<float,2>(m, "Image2DF32");
  BindPyImage<unsigned char,2>(m, "Image2DU8");

  //////////////////////////////////////////////////////////////////////////////
  // Volume Utilities

  m.def("hu_to_lin_att",
      [](const VolF32Ptr& vol, const float hu_lower) -> VolF32Ptr
      {
        return HUToLinAtt(CheckedImg(vol, "vol").GetPointer(), hu_lower);
      },
      "vol"_a, "hu_lower"_a = -1000.0f,
      "Converts a CT volume from Hounsfield units to linear attenuation (for ray casting DRRs). "
      "Returns a new volume.");

  m.def("make_vol_list_from_vol_and_labels",
      [](const VolF32Ptr& vol, const VolU8Ptr& labels, const std::vector<unsigned char>& labels_to_use,
         const float masked_out_val)
      {
        return MakeVolListFromVolAndLabels(CheckedImg(vol, "vol").GetPointer(),
                                           CheckedImg(labels, "labels").GetPointer(),
                                           labels_to_use, masked_out_val);
      },
      "vol"_a, "labels"_a, "labels_to_use"_a, "masked_out_val"_a = 0.0f,
      "Creates a separate volume for each label, each tightly cropped about its label with "
      "voxels outside of the label set to masked_out_val.");

  m.def("vol_center_as_phys_pt",
      [](const VolF32Ptr& vol)
      {
        return Pt3(ITKVol3DCenterAsPhysPt(CheckedImg(vol, "vol").GetPointer()));
      },
      "vol"_a, "The center of a volume in physical coordinates.");

  m.def("vol_center_as_phys_pt",
      [](const VolU8Ptr& vol)
      {
        return Pt3(ITKVol3DCenterAsPhysPt(CheckedImg(vol, "vol").GetPointer()));
      },
      "vol"_a);
}
