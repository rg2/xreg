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

#include "xregOpenCLSys.h"

#include <boost/compute/system.hpp>

#include "xregStringUtils.h"

std::map<std::string,boost::compute::device> xreg::BuildDevIDStrsToDevMap()
{
  OpenCLNameDevMap id_str_to_devs;

  const auto all_devs = boost::compute::system::devices();

  for (const auto& cur_dev : all_devs)
  {
    std::string cur_dev_id = StringRemoveAll(cur_dev.name());  // removes any whitespace

    const auto exts = cur_dev.extensions();

    // check for NVIDIA extensions
    const auto nv_ext = std::find(exts.begin(), exts.end(), "cl_nv_device_attribute_query");
    if (nv_ext != exts.end())
    {
      int pci_bus_id_nv = 0;
      size_t pci_bus_id_nv_ret_size = 0;
      if (CL_SUCCESS == clGetDeviceInfo(cur_dev.id(),
                                        static_cast<cl_device_info>(0x4008),
                                        sizeof(pci_bus_id_nv), &pci_bus_id_nv,
                                        &pci_bus_id_nv_ret_size))
      {
        cur_dev_id += "-";
        cur_dev_id += ToString(pci_bus_id_nv);
      }
    }
    // TODO: AMD
    // TODO: Intel

    id_str_to_devs.emplace(cur_dev_id, cur_dev);
  }

  return id_str_to_devs;
}

std::vector<std::string> xreg::DevIDStrs(const bool use_cpu)
{
  const auto id_dev_map = BuildDevIDStrsToDevMap();

  std::vector<std::string> id_strs;
  id_strs.reserve(id_dev_map.size());

  for (const auto& id_dev_kv : id_dev_map)
  {
    if (use_cpu || (id_dev_kv.second.type() == boost::compute::device::gpu))
    {
      id_strs.push_back(id_dev_kv.first);
    }
  }

  return id_strs;
}

