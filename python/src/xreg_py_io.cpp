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

#include "xregITKIOUtils.h"
#include "xregLandmarkFiles.h"
#include "xregH5ProjDataIO.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

template <class tPixel, unsigned int tN>
void BindWriteImage(nb::module_& m)
{
  m.def("write_image",
      [](const typename itk::Image<tPixel,tN>::Pointer& img, const std::string& path,
         const bool force_no_compression)
      {
        CheckedImg(img, "img");

        nb::gil_scoped_release no_gil;

        WriteITKImageToDisk(img.GetPointer(), path, force_no_compression);
      },
      "img"_a, "path"_a, "force_no_compression"_a = false,
      "Writes an image to disk using ITK, the format is determined by the file extension.");
}

template <class tPixel, unsigned int tN>
nb::object ReadImage(const std::string& path)
{
  typename itk::Image<tPixel,tN>::Pointer img;

  {
    nb::gil_scoped_release no_gil;

    img = ReadITKImageFromDisk<itk::Image<tPixel,tN>>(path);
  }

  return nb::cast(img);
}

ProjDataF32List ReadProjData(const std::string& path,
                             const std::optional<std::vector<size_type>>& proj_idxs,
                             const bool read_pixels)
{
  nb::gil_scoped_release no_gil;

  if (!proj_idxs)
  {
    return ReadProjDataH5F32FromDisk(path, read_pixels);
  }

  // Only read the pixel data of the requested projections
  DeferredProjReader proj_reader(path);

  const auto& proj_metas = proj_reader.proj_data_F32();

  ProjDataF32List projs;
  projs.reserve(proj_idxs->size());

  for (const auto& proj_idx : *proj_idxs)
  {
    if (proj_idx >= proj_metas.size())
    {
      throw std::out_of_range("projection index " + std::to_string(proj_idx) +
                              " is out of range (number of projections: " +
                              std::to_string(proj_metas.size()) + ")");
    }

    projs.push_back(proj_metas[proj_idx]);

    if (read_pixels)
    {
      projs.back().img = proj_reader.read_proj_F32(proj_idx);
    }
  }

  return projs;
}

}  // un-named

void xreg::py::BindIO(nb::module_& m)
{
  m.def("read_image",
      [](const std::string& path, const unsigned int ndim, const std::string& dtype) -> nb::object
      {
        const std::string dt = NormName(dtype);

        const bool is_f32 = (dt == "float32") || (dt == "f32") || (dt == "float");
        const bool is_u8  = (dt == "uint8") || (dt == "u8");

        if (!is_f32 && !is_u8)
        {
          throw nb::value_error("dtype must be \"float32\" or \"uint8\"");
        }

        if (ndim == 3)
        {
          return is_f32 ? ReadImage<float,3>(path) : ReadImage<unsigned char,3>(path);
        }
        else if (ndim == 2)
        {
          return is_f32 ? ReadImage<float,2>(path) : ReadImage<unsigned char,2>(path);
        }

        throw nb::value_error("ndim must be 2 or 3");
      },
      "path"_a, "ndim"_a = 3, "dtype"_a = "float32",
      "Reads an image from disk using xreg's ITK (e.g. NIfTI, NRRD, MHA). Pixels are cast to "
      "dtype (\"float32\" or \"uint8\", e.g. use \"uint8\" for label maps). "
      "Images may also be read with SimpleITK or ITK Python.");

  BindWriteImage<float,3>(m);
  BindWriteImage<unsigned char,3>(m);
  BindWriteImage<float,2>(m);
  BindWriteImage<unsigned char,2>(m);

  m.def("read_proj_data", &ReadProjData,
        "path"_a, "proj_idxs"_a = nb::none(), "read_pixels"_a = true,
        "Reads projection data (camera models, images, landmarks) from an xreg HDF5 file. "
        "proj_idxs: indices of the projections to read (default: all projections).");

  m.def("write_proj_data",
      [](const ProjDataF32List& projs, const std::string& path, const bool compress)
      {
        nb::gil_scoped_release no_gil;

        WriteProjDataH5ToDisk(projs, path, compress);
      },
      "projs"_a, "path"_a, "compress"_a = true,
      "Writes projection data to an xreg HDF5 file.");

  m.def("read_landmarks", &ReadLandmarksFileNamePtMap, "path"_a, "output_in_lps"_a = true,
        "Reads a landmarks file (FCSV or 3D Slicer markups JSON) into a dict of name -> 3D point. "
        "Points are converted to LPS by default.");

  m.def("read_transform", &ReadITKAffineTransformFromFile, "path"_a,
        "Reads a rigid/affine transform from an ITK transform file (4x4 matrix).");

  m.def("write_transform", &WriteITKAffineTransform, "path"_a, "xform"_a,
        "Writes a rigid/affine transform (4x4 matrix) to an ITK transform file.");
}
