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

#ifndef XREG_PY_BACKEND_H_
#define XREG_PY_BACKEND_H_

#include "xreg_py_common.h"

namespace xreg
{

// Forward declarations
class ProgOpts;
class RayCaster;
class ImgSimMetric2D;

namespace py
{

/// \brief A compute backend (CPU, OpenCL, or Metal) and the GPU resources
///        (context/queue) that are shared among all objects (ray casters,
///        similarity metrics) created using it.
///
/// This wraps a ProgOpts object which has been populated as if the --backend,
/// --ocl-id, and --metal-id flags had been passed on the command line. The
/// factory functions of the xreg library that take a ProgOpts object are then
/// used, so the behavior is identical to the xreg command line tools.
struct Backend
{
  /// "cpu", "ocl", or "metal"
  std::string kind;

  /// The device identifier substring requested ("" -> default device)
  std::string device_id;

  /// Name of the device that was selected
  std::string device_name;

  std::shared_ptr<ProgOpts> po;
};

using BackendPtr = std::shared_ptr<Backend>;

/// \brief A backend argument passed from Python: either a Backend object or a
///        string (e.g. "cpu", "ocl", "metal", "ocl:amd", "metal:intel", or ""
///        for the default backend).
using BackendArg = std::variant<BackendPtr,std::string>;

/// \brief Creates a new backend
BackendPtr MakeBackend(const std::string& kind, const std::string& device_id);

/// \brief Retrieves a backend object, strings are parsed and backends created
///        from strings are cached so that all objects created using the same
///        string share the same GPU context/queue.
BackendPtr ResolveBackend(const BackendArg& b);

/// \brief Creates a ray caster of a specific type ("line_int" or "depth") for a backend.
std::shared_ptr<RayCaster> MakeRayCaster(const std::string& kind, const BackendArg& backend);

}  // py
}  // xreg

#endif
