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

#include "xregImgSimMetric2DProgOpts.h"

#include "xregExceptionUtils.h"
#include "xregProgOptUtils.h"
#include "xregImgSimMetric2DSSDCPU.h"
#include "xregImgSimMetric2DSSDOCL.h"
#include "xregImgSimMetric2DNCCCPU.h"
#include "xregImgSimMetric2DNCCOCL.h"
#include "xregImgSimMetric2DGradNCCCPU.h"
#include "xregImgSimMetric2DGradNCCOCL.h"
#include "xregImgSimMetric2DPatchNCCCPU.h"
#include "xregImgSimMetric2DPatchNCCOCL.h"
#include "xregImgSimMetric2DPatchGradNCCCPU.h"
#include "xregImgSimMetric2DPatchGradNCCOCL.h"

#ifdef XREG_HAS_METAL
#include "xregImgSimMetric2DSSDMetal.h"
#include "xregImgSimMetric2DNCCMetal.h"
#include "xregImgSimMetric2DGradNCCMetal.h"
#include "xregImgSimMetric2DPatchNCCMetal.h"
#include "xregImgSimMetric2DPatchGradNCCMetal.h"
#endif

namespace  // un-named
{

using namespace xreg;

// tMetalSim may be void when there is no Metal implementation
template <class tCPUSim, class tOCLSim, class tMetalSim = void>
std::shared_ptr<ImgSimMetric2D> SimMetricFromProgOptsHelper(ProgOpts& po)
{
  const std::string backend_str = po.get("backend");
  
  std::shared_ptr<ImgSimMetric2D> sm;

  if (backend_str == "cpu")
  {
    sm = std::make_shared<tCPUSim>();
  }
  else if (backend_str == "ocl")
  {
    auto ocl_ctx_queue = po.selected_ocl_ctx_queue();
    sm = std::make_shared<tOCLSim>(std::get<0>(ocl_ctx_queue), std::get<1>(ocl_ctx_queue));
  }
#ifdef XREG_HAS_METAL
  else if constexpr (!std::is_void<tMetalSim>::value)
  {
    if (backend_str == "metal")
    {
      sm = std::make_shared<tMetalSim>(po.selected_metal_queue());
    }
  }
#endif

  if (!sm)
  {
    xregThrow("Unsupported backend for Sim Metric: %s", backend_str.c_str());
  }

  return sm;
}

}  // un-named

std::shared_ptr<xreg::ImgSimMetric2D> xreg::SSDSimMetricFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return SimMetricFromProgOptsHelper<ImgSimMetric2DSSDCPU,ImgSimMetric2DSSDOCL,ImgSimMetric2DSSDMetal>(po);
#else
  return SimMetricFromProgOptsHelper<ImgSimMetric2DSSDCPU,ImgSimMetric2DSSDOCL>(po);
#endif
}

std::shared_ptr<xreg::ImgSimMetric2D> xreg::NCCSimMetricFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return SimMetricFromProgOptsHelper<ImgSimMetric2DNCCCPU,ImgSimMetric2DNCCOCL,ImgSimMetric2DNCCMetal>(po);
#else
  return SimMetricFromProgOptsHelper<ImgSimMetric2DNCCCPU,ImgSimMetric2DNCCOCL>(po);
#endif
}

std::shared_ptr<xreg::ImgSimMetric2D> xreg::GradNCCSimMetricFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return SimMetricFromProgOptsHelper<ImgSimMetric2DGradNCCCPU,ImgSimMetric2DGradNCCOCL,ImgSimMetric2DGradNCCMetal>(po);
#else
  return SimMetricFromProgOptsHelper<ImgSimMetric2DGradNCCCPU,ImgSimMetric2DGradNCCOCL>(po);
#endif
}

std::shared_ptr<xreg::ImgSimMetric2D> xreg::PatchNCCSimMetricFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return SimMetricFromProgOptsHelper<ImgSimMetric2DPatchNCCCPU,ImgSimMetric2DPatchNCCOCL,ImgSimMetric2DPatchNCCMetal>(po);
#else
  return SimMetricFromProgOptsHelper<ImgSimMetric2DPatchNCCCPU,ImgSimMetric2DPatchNCCOCL>(po);
#endif
}

std::shared_ptr<xreg::ImgSimMetric2D> xreg::PatchGradNCCSimMetricFromProgOpts(ProgOpts& po)
{
#ifdef XREG_HAS_METAL
  return SimMetricFromProgOptsHelper<ImgSimMetric2DPatchGradNCCCPU,ImgSimMetric2DPatchGradNCCOCL,ImgSimMetric2DPatchGradNCCMetal>(po);
#else
  return SimMetricFromProgOptsHelper<ImgSimMetric2DPatchGradNCCCPU,ImgSimMetric2DPatchGradNCCOCL>(po);
#endif
}
