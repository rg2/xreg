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

// Entry point for the `pyxreg` extension module. Each feature area registers its
// own submodule; this file only wires them together so the individual binding
// translation units stay focused and independently compilable.
namespace xregpy
{

void RegisterTransforms(py::module_& parent);
void RegisterLandmarks(py::module_& parent);
void RegisterImageIO(py::module_& parent);
void RegisterMesh(py::module_& parent);
void RegisterProjData(py::module_& parent);

}  // namespace xregpy

PYBIND11_MODULE(pyxreg, m)
{
  m.doc() = "Python bindings for xreg (2D/3D X-ray registration library).";

  xregpy::RegisterConvertTypes(m);

  xregpy::RegisterTransforms(m);
  xregpy::RegisterLandmarks(m);
  xregpy::RegisterImageIO(m);
  xregpy::RegisterMesh(m);
  xregpy::RegisterProjData(m);
}
