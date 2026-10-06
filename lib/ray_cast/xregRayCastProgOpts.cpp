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

#include "xregRayCastProgOpts.h"

#include <type_traits>

#include "xregExceptionUtils.h"
#include "xregProgOptUtils.h"
#include "xregRayCastLineIntCPU.h"
#include "xregRayCastLineIntOCL.h"
#include "xregRayCastDepthCPU.h"
#include "xregRayCastDepthOCL.h"

#ifdef XREG_HAS_METAL
#include "xregRayCastLineIntMetal.h"
#include "xregRayCastDepthMetal.h"
#endif

namespace  // un-named
{

using namespace xreg;

// tRayCasterMetal may be void when there is no Metal implementation
template <class tRayCasterCPU, class tRayCasterOCL, class tRayCasterMetal = void>
std::shared_ptr<RayCaster>
RayCasterFromProgOptsHelper(ProgOpts& po)
{
  const std::string backend_str = po.get("backend");

  std::shared_ptr<RayCaster> rc;

  if (backend_str == "cpu")
  {
    rc = std::make_shared<tRayCasterCPU>();
  }
  else if (backend_str == "ocl")
  {
    auto ocl_ctx_queue = po.selected_ocl_ctx_queue();
    rc = std::make_shared<tRayCasterOCL>(std::get<0>(ocl_ctx_queue), std::get<1>(ocl_ctx_queue));
  }
#ifdef XREG_HAS_METAL
  else if constexpr (!std::is_void<tRayCasterMetal>::value)
  {
    if (backend_str == "metal")
    {
      rc = std::make_shared<tRayCasterMetal>(po.selected_metal_queue());
    }
  }
#endif

  if (!rc)
  {
    xregThrow("Unsupported backend for Ray Caster: %s", backend_str.c_str());
  }

  return rc;
}

}  // un-named

std::shared_ptr<xreg::RayCaster>
xreg::LineIntRayCasterFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return RayCasterFromProgOptsHelper<RayCasterLineIntCPU,RayCasterLineIntOCL,RayCasterLineIntMetal>(po);
#else
  return RayCasterFromProgOptsHelper<RayCasterLineIntCPU,RayCasterLineIntOCL>(po);
#endif
}

std::shared_ptr<xreg::RayCaster>
xreg::DepthRayCasterFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return RayCasterFromProgOptsHelper<RayCasterDepthCPU,RayCasterDepthOCL,RayCasterDepthMetal>(po);
#else
  return RayCasterFromProgOptsHelper<RayCasterDepthCPU,RayCasterDepthOCL>(po);
#endif
}

