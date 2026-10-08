
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "xregAssert.h"
#include "xregMetalSys.h"
#include "xregProgOptUtils.h"
#include "xregStringUtils.h"

namespace
{

using namespace xreg;

bool Throws(const MetalIDStrDevMap& m, const std::string& id_substr)
{
  bool threw = false;

  try
  {
    FindMetalDevByIDSubstr(m, id_substr);
  }
  catch (const std::exception& e)
  {
    std::cout << "  \"" << id_substr << "\" threw: " << e.what() << std::endl;
    threw = true;
  }

  return threw;
}

const std::string& Match(const MetalIDStrDevMap& m, const std::string& id_substr)
{
  return FindMetalDevByIDSubstr(m, id_substr)->first;
}

void TestMatching()
{
  std::cout << "testing ID matching..." << std::endl;

  MetalIDStrDevMap m;
  m.emplace("AMDRadeonPro5500M-100000759", MetalDevice());
  m.emplace("Intel(R)UHDGraphics630-100000710", MetalDevice());
  m.emplace("Foo-10", MetalDevice());
  m.emplace("Foo-100", MetalDevice());

  xregASSERT(Match(m, "amd") == "AMDRadeonPro5500M-100000759");
  xregASSERT(Match(m, "RADEON") == "AMDRadeonPro5500M-100000759");
  xregASSERT(Match(m, "intel(r)uhd") == "Intel(R)UHDGraphics630-100000710");
  xregASSERT(Match(m, "INTEL(R)UHDGRAPHICS630-100000710") == "Intel(R)UHDGraphics630-100000710");

  // "Foo-10" is a substring of "Foo-100", but an exact match takes precedence
  xregASSERT(Match(m, "foo-10") == "Foo-10");
  xregASSERT(Match(m, "foo-100") == "Foo-100");

  // ambiguous
  xregASSERT(Throws(m, "1000007"));
  xregASSERT(Throws(m, "foo"));
  xregASSERT(Throws(m, "-1"));

  // no match
  xregASSERT(Throws(m, "nvidia"));
  xregASSERT(Throws(MetalIDStrDevMap(), "amd"));
}

// Parses the provided arguments into new program options with backend flags
std::unique_ptr<ProgOpts> ParseArgs(const std::vector<std::string>& args)
{
  auto po = std::make_unique<ProgOpts>();
  po->add_backend_flags();

  std::vector<std::string> args_copy = args;
  args_copy.insert(args_copy.begin(), "test-prog");

  std::vector<char*> argv;
  for (auto& a : args_copy)
  {
    argv.push_back(&a[0]);
  }

  po->parse(static_cast<int>(argv.size()), argv.data());

  return po;
}

void TestProgOpts()
{
  std::cout << "testing program options..." << std::endl;

  const auto all_devs = MetalAllDevices();
  xregASSERT(!all_devs.empty());

  // the metal backend and each device are listed in the help
  {
    auto po = ParseArgs({});

    std::ostringstream oss;
    po->print_help(oss);

    const std::string help_str = oss.str();

    xregASSERT(help_str.find("metal-id") != std::string::npos);
    xregASSERT(help_str.find("Available Metal Devices") != std::string::npos);
    xregASSERT(help_str.find("metal: Apple Metal") != std::string::npos);

    xregASSERT(help_str.find("Metal Framework: " + MetalFrameworkVersion()) != std::string::npos);

    for (const auto& d : all_devs)
    {
      xregASSERT(help_str.find(d.id_str()) != std::string::npos);

      xregASSERT(help_str.find("Metal Device: " + d.name() + " (" +
                               MetalGPUFamilyStr(d.highest_gpu_family()) + ")") != std::string::npos);
    }

    // print the version section, which follows the OpenCL platforms
    std::cout << help_str.substr(help_str.find("OpenCL Platform:")) << std::flush;
  }

  // the metal backend is the default when a Metal device is available, and the
  // other backends may still be selected
  {
    xregASSERT(ParseArgs({})->get("backend").as_string() == "metal");

    auto po = ParseArgs({ "--backend", "metal" });
    xregASSERT(po->get("backend").as_string() == "metal");

    xregASSERT(ParseArgs({ "--backend", "cpu" })->get("backend").as_string() == "cpu");
  }

  // no ID selects the default device
  {
    auto po = ParseArgs({ "--backend", "metal" });
    xregASSERT(po->selected_metal().registry_id() == MetalDevice::Default().registry_id());
  }

  // each device may be selected with a full ID string with different case, or
  // with the unique registry ID portion of the ID string
  for (const auto& d : all_devs)
  {
    const std::string id_str = d.id_str();

    {
      auto po = ParseArgs({ "--backend", "metal", "--metal-id", ToUpperCase(id_str) });

      const MetalDevice sel_dev = po->selected_metal();
      std::cout << "  selected: " << sel_dev.id_str() << std::endl;

      xregASSERT(sel_dev.registry_id() == d.registry_id());

      // the same device and queue are returned on subsequent calls
      xregASSERT(po->selected_metal().native_handle() == sel_dev.native_handle());

      const MetalCmdQueue q = po->selected_metal_queue();
      xregASSERT(q.valid());
      xregASSERT(q.device().registry_id() == d.registry_id());
      xregASSERT(po->selected_metal_queue().native_handle() == q.native_handle());
    }

    {
      const std::string reg_id_str = id_str.substr(id_str.rfind('-'));

      auto po = ParseArgs({ "--backend", "metal", "--metal-id", reg_id_str });
      xregASSERT(po->selected_metal().registry_id() == d.registry_id());
    }
  }

  // invalid ID
  {
    auto po = ParseArgs({ "--backend", "metal", "--metal-id", "gpu-that-does-not-exist" });

    bool threw = false;
    try
    {
      po->selected_metal();
    }
    catch (const std::exception& e)
    {
      std::cout << "  invalid ID threw: " << e.what() << std::endl;
      threw = true;
    }
    xregASSERT(threw);
  }
}

}  // un-named

int main(int argc, char* argv[])
{
  TestMatching();

  TestProgOpts();

  std::cout << "PASSED" << std::endl;

  return 0;
}
