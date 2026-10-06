
#include <algorithm>
#include <iostream>

#include "xregAssert.h"
#include "xregMetalSys.h"

namespace
{

using namespace xreg;

const char* LocationStr(const MetalDeviceLocation loc)
{
  switch (loc)
  {
  case MetalDeviceLocation::kBUILT_IN:
    return "built-in";
  case MetalDeviceLocation::kSLOT:
    return "slot";
  case MetalDeviceLocation::kEXTERNAL:
    return "external";
  default:
    return "unspecified";
  }
}

}  // un-named

int main(int argc, char* argv[])
{
  using namespace xreg;

  const auto all_devs = MetalAllDevices();

  std::cout << "Number of Metal devices: " << all_devs.size() << std::endl;

  for (const auto& d : all_devs)
  {
    xregASSERT(d.valid());

    std::cout << "  " << d.id_str() << '\n'
              << "      name:           " << d.name() << '\n'
              << "      location:       " << LocationStr(d.location()) << '\n'
              << "      unified memory: " << d.has_unified_memory() << '\n'
              << "      low power:      " << d.is_low_power() << '\n'
              << "      headless:       " << d.is_headless() << '\n'
              << "      removable:      " << d.is_removable() << '\n'
              << "      working set:    " << (d.recommended_max_working_set_size() / (1024 * 1024)) << " MB\n"
              << "      max buffer:     " << (d.max_buffer_length() / (1024 * 1024)) << " MB" << std::endl;

    xregASSERT(!d.id_str().empty());
    xregASSERT(d.max_buffer_length() > 0);
  }

  xregASSERT(!all_devs.empty());

  // the default device is one of the devices listed
  const MetalDevice default_dev = MetalDevice::Default();

  std::cout << "Default device: " << default_dev.id_str() << std::endl;

  xregASSERT(std::any_of(all_devs.begin(), all_devs.end(),
                         [&default_dev] (const MetalDevice& d)
                         {
                           return d.registry_id() == default_dev.registry_id();
                         }));

  // ID strings are unique
  const auto id_dev_map = BuildMetalDevIDStrsToDevMap();
  xregASSERT(id_dev_map.size() == all_devs.size());

  for (const auto& id_dev_kv : id_dev_map)
  {
    xregASSERT(id_dev_kv.first == id_dev_kv.second.id_str());
  }

  // the list of ID strings matches the map keys
  const auto id_strs = MetalDevIDStrs();
  xregASSERT(id_strs.size() == id_dev_map.size());

  for (const auto& s : id_strs)
  {
    xregASSERT(id_dev_map.count(s) == 1);
  }

  // an invalid device has default info values
  const MetalDevice invalid_dev;
  xregASSERT(!invalid_dev.valid());
  xregASSERT(invalid_dev.id_str().empty());
  xregASSERT(invalid_dev.registry_id() == 0);

  std::cout << "PASSED" << std::endl;

  return 0;
}
