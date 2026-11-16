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

#include "xreg_py_backend.h"

#include <map>

#include "xregProgOptUtils.h"
#include "xregOpenCLSys.h"

#ifdef XREG_HAS_METAL
#include "xregMetalSys.h"
#endif

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

std::string JoinStrs(const std::vector<std::string>& strs)
{
  std::string s;

  for (size_type i = 0; i < strs.size(); ++i)
  {
    s += (i ? ", \"" : "\"") + strs[i] + '\"';
  }

  return s;
}

std::vector<std::string> AvailableDevices(const std::string& kind_arg)
{
  const std::string kind = NormName(kind_arg);

  if (kind == "ocl")
  {
    std::vector<std::string> ids;
    for (const auto& kv : BuildDevIDStrsToDevMap())
    {
      ids.push_back(kv.first);
    }
    return ids;
  }
#ifdef XREG_HAS_METAL
  else if (kind == "metal")
  {
    return MetalDevIDStrs();
  }
#endif
  else if (kind == "cpu")
  {
    return { "cpu" };
  }

  throw nb::value_error(("Unsupported backend: \"" + kind_arg + "\"").c_str());
}

}  // un-named

xreg::py::BackendPtr xreg::py::MakeBackend(const std::string& kind_arg, const std::string& device_id)
{
  const auto valid_names = ValidBackendNames();

  std::string kind = NormName(kind_arg);

  if (kind.empty() || (kind == "default") || (kind == "auto"))
  {
    // the first backend is the default
    kind = valid_names[0];
  }
  else if (kind == "opencl")
  {
    kind = "ocl";
  }

  if (std::find(valid_names.begin(), valid_names.end(), kind) == valid_names.end())
  {
    throw nb::value_error(("Invalid or unavailable backend: \"" + kind_arg +
                           "\" (available backends: " + JoinStrs(valid_names) + ")").c_str());
  }

  // Populate a program options object as if the backend flags were passed to
  // a command line tool.
  std::vector<std::string> args = { "xreg-python", "--backend", kind };

  if (!device_id.empty())
  {
    if (kind == "cpu")
    {
      throw nb::value_error("a device may not be specified for the \"cpu\" backend");
    }

    args.push_back((kind == "ocl") ? "--ocl-id" : "--metal-id");
    args.push_back(device_id);
  }

  std::vector<char*> argv;
  argv.reserve(args.size());

  for (auto& a : args)
  {
    argv.push_back(a.data());
  }

  auto b = std::make_shared<Backend>();
  b->kind      = kind;
  b->device_id = device_id;
  b->po        = std::make_shared<ProgOpts>();

  b->po->add_backend_flags();
  b->po->parse(static_cast<int>(argv.size()), argv.data());

  // Select the device and create the context/queue now, so that an invalid
  // device string is reported here, rather than at the first use of the backend.
  try
  {
    if (kind == "ocl")
    {
      b->device_name = b->po->selected_ocl().name();
      b->po->selected_ocl_ctx_queue();
    }
#ifdef XREG_HAS_METAL
    else if (kind == "metal")
    {
      b->device_name = b->po->selected_metal().name();
      b->po->selected_metal_queue();
    }
#endif
    else
    {
      b->device_name = "cpu";
    }
  }
  catch (const std::exception& e)
  {
    throw nb::value_error((std::string("Failed to set up the \"") + kind + "\" backend: " + e.what()).c_str());
  }

  return b;
}

xreg::py::BackendPtr xreg::py::ResolveBackend(const BackendArg& b)
{
  if (std::holds_alternative<BackendPtr>(b))
  {
    const auto& p = std::get<BackendPtr>(b);
    if (!p)
    {
      throw nb::value_error("backend is None");
    }
    return p;
  }

  // Backends created from strings are cached, so that the same context/queue
  // is used for all objects that specify the backend using the same string.
  // NOTE: this is only called while the GIL is held, which protects the cache.
  static std::map<std::string,BackendPtr> backend_cache;

  const std::string& s = std::get<std::string>(b);

  auto it = backend_cache.find(s);

  if (it == backend_cache.end())
  {
    std::string kind = s;
    std::string dev_id;

    const auto colon_pos = s.find(':');
    if (colon_pos != std::string::npos)
    {
      kind   = s.substr(0, colon_pos);
      dev_id = s.substr(colon_pos + 1);
    }

    it = backend_cache.emplace(s, MakeBackend(kind, dev_id)).first;
  }

  return it->second;
}

void xreg::py::BindBackend(nb::module_& m)
{
  nb::class_<Backend>(m, "Backend",
      "A compute backend (\"cpu\", \"ocl\", or \"metal\") and its GPU device context/queue. "
      "Objects created with the same Backend share GPU resources. Anywhere a Backend is "
      "accepted, a string may be passed instead, e.g. \"metal\", \"ocl\", \"cpu\", \"ocl:amd\" "
      "(device ID substring after the colon), or \"\" for the default backend; Backend objects "
      "created from strings are cached and re-used.")
    .def("__init__",
        [](Backend* b, const std::string& kind, const std::string& device)
        {
          new (b) Backend(*MakeBackend(kind, device));
        },
        "kind"_a = "", "device"_a = "",
        "kind: \"cpu\", \"ocl\", \"metal\", or \"\" for the default. "
        "device: case-insensitive substring of the device ID (see available_devices()), "
        "or \"\" for the default device.")
    .def_ro("kind", &Backend::kind)
    .def_ro("device_id", &Backend::device_id, "The device ID substring that was requested.")
    .def_ro("device_name", &Backend::device_name, "Name of the device that was selected.")
    .def("__repr__",
        [](const Backend& b)
        {
          return "Backend(kind=\"" + b.kind + "\", device=\"" + b.device_name + "\")";
        });

  m.def("available_backends", &ValidBackendNames,
        "Names of the compute backends available on this system; the first is the default.");

  m.def("available_devices", &AvailableDevices, "kind"_a,
        "Device ID strings available for a backend (\"ocl\" or \"metal\").");

  m.def("get_backend", &ResolveBackend, "backend"_a = "",
        "Returns the (cached) Backend object for a backend string.");
}
