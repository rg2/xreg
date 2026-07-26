/*
 * MIT License
 *
 * Copyright (c) 2022 Robert Grupp
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

#include "xregpy_convert.h"

#include "xregITKIOUtils.h"

// Bindings for 3D volume/image file I/O. Volumes are read into a NumPy-backed
// Volume (float32 voxels + spacing/origin/direction). Backs the volume-reading
// half of convert_sta_raw_to_itk, crop_vol, etc. Any ITK-supported format is
// accepted for read/write (.nii/.nii.gz, .mha/.mhd, .nrrd, ...).
namespace xregpy
{

void RegisterImageIO(py::module_& parent)
{
  auto m = parent.def_submodule("image_io", "3D volume/image file I/O.");

  m.def("read_volume",
        [](const std::string& path)
        {
          auto img = xreg::ReadITKImageFromDisk<Vol3D>(path);
          return ItkVolToVolume(img.GetPointer());
        },
        py::arg("path"),
        "Read a 3D volume from any ITK-supported format into an xreg.Volume "
        "(voxels cast to float32).");

  m.def("write_volume",
        [](const Volume& vol, const std::string& path, const bool force_no_compression)
        {
          auto img = VolumeToItkVol(vol);
          xreg::WriteITKImageToDisk(img.GetPointer(), path, force_no_compression);
        },
        py::arg("volume"), py::arg("path"), py::arg("force_no_compression") = false,
        "Write an xreg.Volume to disk; format is chosen by the file extension.");

  m.def("read_dicom_volume",
        [](const std::string& path)
        {
          auto img = xreg::ReadDICOM3DFromDisk<xreg::RayCastPixelScalar>(path);
          return ItkVolToVolume(img.GetPointer());
        },
        py::arg("path"),
        "Read a single 3D DICOM volume file into an xreg.Volume (float32).");
}

}  // namespace xregpy
