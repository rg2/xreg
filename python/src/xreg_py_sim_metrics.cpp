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

#include "xregProgOptUtils.h"
#include "xregImgSimMetric2D.h"
#include "xregImgSimMetric2DPatchCommon.h"
#include "xregImgSimMetric2DGradImgParamInterface.h"
#include "xregImgSimMetric2DProgOpts.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

ImgSimMetric2DPatchCommon* AsPatchSimMetric(ImgSimMetric2D& sm)
{
  auto* p = dynamic_cast<ImgSimMetric2DPatchCommon*>(&sm);
  if (!p)
  {
    throw nb::type_error("similarity metric is not patch-based");
  }
  return p;
}

ImgSimMetric2DGradImgParamInterface* AsGradSimMetric(ImgSimMetric2D& sm)
{
  auto* p = dynamic_cast<ImgSimMetric2DGradImgParamInterface*>(&sm);
  if (!p)
  {
    throw nb::type_error("similarity metric is not gradient-based");
  }
  return p;
}

std::shared_ptr<ImgSimMetric2D>
MakeSimMetric(const std::string& name_arg, const BackendArg& backend_arg,
              const std::optional<size_type>& patch_radius,
              const std::optional<size_type>& patch_stride,
              const std::optional<size_type>& smooth_img_before_sobel_kernel_radius)
{
  const std::string name = NormName(name_arg);

  const auto b = ResolveBackend(backend_arg);

  std::shared_ptr<ImgSimMetric2D> sm;

  if (name == "ssd")
  {
    sm = SSDSimMetricFromProgOpts(*b->po);
  }
  else if (name == "ncc")
  {
    sm = NCCSimMetricFromProgOpts(*b->po);
  }
  else if (name == "grad_ncc")
  {
    sm = GradNCCSimMetricFromProgOpts(*b->po);
  }
  else if (name == "patch_ncc")
  {
    sm = PatchNCCSimMetricFromProgOpts(*b->po);
  }
  else if (name == "patch_grad_ncc")
  {
    sm = PatchGradNCCSimMetricFromProgOpts(*b->po);
  }
  else
  {
    throw nb::value_error(("Unsupported similarity metric type: \"" + name_arg +
                           "\" (valid types: \"ssd\", \"ncc\", \"grad_ncc\", "
                           "\"patch_ncc\", \"patch_grad_ncc\")").c_str());
  }

  if (patch_radius)
  {
    AsPatchSimMetric(*sm)->set_patch_radius(*patch_radius);
  }

  if (patch_stride)
  {
    AsPatchSimMetric(*sm)->set_patch_stride(*patch_stride);
  }

  if (smooth_img_before_sobel_kernel_radius)
  {
    AsGradSimMetric(*sm)->set_smooth_img_before_sobel_kernel_radius(*smooth_img_before_sobel_kernel_radius);
  }

  return sm;
}

}  // un-named

void xreg::py::BindSimMetrics(nb::module_& m)
{
  nb::class_<ImgSimMetric2D>(m, "ImgSimMetric2D",
      "2D image similarity metric. Create with make_sim_metric(). Fixed images, masks, and "
      "moving images are set automatically when used in a MultiLevelMultiObjRegi.")
    .def_prop_ro("is_patch_based",
        [](ImgSimMetric2D& sm) { return dynamic_cast<ImgSimMetric2DPatchCommon*>(&sm) != nullptr; })
    .def_prop_ro("is_grad_based",
        [](ImgSimMetric2D& sm) { return dynamic_cast<ImgSimMetric2DGradImgParamInterface*>(&sm) != nullptr; })
    .def("set_patch_radius",
        [](ImgSimMetric2D& sm, const size_type r) { AsPatchSimMetric(sm)->set_patch_radius(r); },
        "radius"_a, "(Patch-based metrics only)")
    .def("set_patch_stride",
        [](ImgSimMetric2D& sm, const size_type s) { AsPatchSimMetric(sm)->set_patch_stride(s); },
        "stride"_a, "(Patch-based metrics only)")
    .def("set_wgt_img",
        [](ImgSimMetric2D& sm, const ImgSimMetric2D::ImagePtr& img)
        {
          AsPatchSimMetric(sm)->set_wgt_img(img);
        },
        "img"_a,
        "(Patch-based metrics only) Sets an image used to weight each patch, or None. The weight "
        "of a patch is looked up at the pixel location of the patch center in the fixed image "
        "used by the metric (e.g. after any downsampling by a registration level) - no "
        "resampling is performed. NOTE: the weight image must be at least as large as that "
        "fixed image, this is not checked.")
    .def("set_smooth_img_before_sobel_kernel_radius",
        [](ImgSimMetric2D& sm, const size_type r)
        {
          AsGradSimMetric(sm)->set_smooth_img_before_sobel_kernel_radius(r);
        },
        "radius"_a, "(Gradient-based metrics only)")
    .def("set_mask", &ImgSimMetric2D::set_mask, "mask"_a,
         "Sets a mask (or None), pixels with value zero are not used. This is set automatically "
         "from MultiLevelMultiObjRegi.masks_2d when used in a multi-level registration.");

  m.def("make_sim_metric", &MakeSimMetric,
        "kind"_a = "patch_grad_ncc", "backend"_a = "",
        nb::kw_only(),
        "patch_radius"_a = nb::none(), "patch_stride"_a = nb::none(),
        "smooth_img_before_sobel_kernel_radius"_a = nb::none(),
        "Creates a 2D image similarity metric. kind: \"ssd\", \"ncc\", \"grad_ncc\", "
        "\"patch_ncc\", or \"patch_grad_ncc\". backend: a Backend or a backend string. "
        "Optionally set the patch radius/stride (patch metrics) and smoothing kernel radius "
        "(gradient metrics).");
}
