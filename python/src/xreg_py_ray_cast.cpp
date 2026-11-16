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

#include <nanobind/stl/function.h>

#include <opencv2/core.hpp>

#include "xregProgOptUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregRayCastInterface.h"
#include "xregRayCastProgOpts.h"
#include "xregProj3DLabelsTo2D.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

// Copies a single channel, single precision, OpenCV image into a numpy array.
nb::object CvMatF32ToArray(const cv::Mat& mat)
{
  if (mat.type() != CV_32FC1)
  {
    throw std::runtime_error("expected a single channel, single precision image");
  }

  const cv::Mat cont_mat = mat.isContinuous() ? mat : mat.clone();

  return nb::cast(CopyBufToArray<float,2>(cont_mat.ptr<float>(),
                                          { static_cast<size_t>(cont_mat.rows),
                                            static_cast<size_t>(cont_mat.cols) }));
}

std::vector<RayCaster::ProjPtr>
ComputeDRRs(const RayCaster::VolList& vols,
            const std::vector<CameraModel>& cams,
            const FrameTransformList& cam_to_vols,
            const BackendArg& backend,
            const std::optional<CoordScalar>& ray_step_size)
{
  if (vols.empty())
  {
    throw nb::value_error("at least one volume is required");
  }

  for (const auto& v : vols)
  {
    CheckedImg(v, "volume");
  }

  if (cams.empty())
  {
    throw nb::value_error("at least one camera model is required");
  }

  if (cam_to_vols.size() != vols.size())
  {
    throw nb::value_error("one pose (camera to volume) is required for each volume");
  }

  // Create the ray caster while holding the GIL, since it may create (and cache) a backend
  auto rc = MakeRayCaster("line_int", backend);

  RayCaster::ProjList projs;

  {
    nb::gil_scoped_release no_gil;

    rc->set_volumes(vols);
    rc->set_camera_models(cams);

    if (ray_step_size)
    {
      rc->set_ray_step_size(*ray_step_size);
    }

    rc->set_num_projs(cams.size());
    rc->allocate_resources();

    SimpleRayCasterWrapperFn drr_fn;
    drr_fn.ray_caster        = rc.get();
    drr_fn.cam_world_to_vols = cam_to_vols;

    drr_fn();

    projs.reserve(drr_fn.projs.size());

    // The projection objects reference the ray caster's buffer, so copy them
    for (const auto& p : drr_fn.projs)
    {
      projs.push_back(ITKImageDeepCopy(p.GetPointer()));
    }
  }

  return projs;
}

void SetSingleChanSegFn(Proj3DLabelsTo2D& p, nb::object fn)
{
  if (fn.is_none())
  {
    p.convert_to_single_chan_seg_fn = nullptr;
    return;
  }

  if (!PyCallable_Check(fn.ptr()))
  {
    throw nb::type_error("expected a callable or None");
  }

  p.convert_to_single_chan_seg_fn =
    [fn](const std::vector<cv::Mat>& depth_imgs, Proj3DLabelsTo2D::LabelImg2D* seg)
    {
      nb::gil_scoped_acquire gil;

      nb::list depths;
      for (const auto& d : depth_imgs)
      {
        depths.append(CvMatF32ToArray(d));
      }

      nb::object res = fn(depths);

      auto res_arr = nb::cast<ArrayIn<unsigned char,2>>(
                        nb::module_::import_("numpy").attr("ascontiguousarray")(res, "dtype"_a = "uint8"));

      const auto seg_size = seg->GetLargestPossibleRegion().GetSize();

      if ((res_arr.shape(0) != seg_size[1]) || (res_arr.shape(1) != seg_size[0]))
      {
        throw nb::value_error("the single channel segmentation must have the same shape as the depth images");
      }

      std::memcpy(seg->GetBufferPointer(), res_arr.data(), seg_size[0] * seg_size[1]);
    };
}

}  // un-named

std::shared_ptr<xreg::RayCaster>
xreg::py::MakeRayCaster(const std::string& name_arg, const BackendArg& backend_arg)
{
  const std::string name = NormName(name_arg);

  const auto b = ResolveBackend(backend_arg);

  if ((name == "line_int") || (name == "drr"))
  {
    return LineIntRayCasterFromProgOpts(*b->po);
  }
  else if (name == "depth")
  {
    return DepthRayCasterFromProgOpts(*b->po);
  }

  throw nb::value_error(("Unsupported ray caster type: \"" + name_arg +
                         "\" (valid types: \"line_int\", \"depth\")").c_str());
}

void xreg::py::BindRayCast(nb::module_& m)
{
  m.attr("RAY_CAST_MAX_DEPTH") = kRAY_CAST_MAX_DEPTH;

  nb::class_<RayCaster>(m, "RayCaster",
      "Ray caster (e.g. DRR generator). Create with make_ray_caster(). Volumes, camera "
      "models, etc. are set automatically when used in a MultiLevelMultiObjRegi.")
    .def_prop_rw("ray_step_size", &RayCaster::ray_step_size, &RayCaster::set_ray_step_size,
                 "Step size (mm) along each ray.")
    .def("set_interp_method",
        [](RayCaster& rc, const std::string& interp)
        {
          const std::string n = NormName(interp);
          if (n == "linear")
          {
            rc.use_linear_interp();
          }
          else if ((n == "nn") || (n == "nearest"))
          {
            rc.use_nn_interp();
          }
          else if (n == "sinc")
          {
            rc.use_sinc_interp();
          }
          else if (n == "bspline")
          {
            rc.use_bspline_interp();
          }
          else
          {
            throw nb::value_error("interpolation method must be one of: \"linear\", \"nn\", \"sinc\", \"bspline\"");
          }
        },
        "method"_a, "Volume interpolation: \"linear\" (default), \"nn\", \"sinc\", or \"bspline\".")
    .def_prop_rw("default_bg_pixel_val", &RayCaster::default_bg_pixel_val,
                 &RayCaster::set_default_bg_pixel_val)
    .def_prop_ro("num_vols", &RayCaster::num_vols)
    .def_prop_ro("num_camera_models", &RayCaster::num_camera_models);

  m.def("make_ray_caster", &MakeRayCaster, "kind"_a = "line_int", "backend"_a = "",
        "Creates a ray caster. kind: \"line_int\" (DRRs) or \"depth\". "
        "backend: a Backend or a backend string (\"\" -> default backend).");

  m.def("compute_drrs", &ComputeDRRs,
        "vols"_a, "cams"_a, "cam_to_vols"_a, "backend"_a = "", "ray_step_size"_a = nb::none(),
        "Computes one DRR (line integral) per camera model of the volumes at the given poses "
        "(one camera to volume pose per volume, contributions of each volume are summed). "
        "Volumes should be in units of linear attenuation (see hu_to_lin_att()).");

  //////////////////////////////////////////////////////////////////////////////
  // Projection of 3D labels into 2D

  using LabelProj = Proj3DLabelsTo2D;

  nb::class_<LabelProj>(m, "Proj3DLabelsTo2D",
      "Projects the labels of a 3D label map into 2D using a depth ray caster. Set "
      "depth_ray_caster, label_vol, labels_to_proj, cam (and optionally a single channel "
      "segmentation function), call init(), set obj_poses (one camera to volume pose per "
      "label), and then call run().")
    .def(nb::init<>())
    .def_prop_rw("depth_ray_caster",
        [](const LabelProj& p) { return p.depth_ray_caster; },
        [](LabelProj& p, const std::variant<std::shared_ptr<RayCaster>,std::string>& rc)
        {
          if (std::holds_alternative<std::string>(rc))
          {
            // interpret the string as a backend
            p.depth_ray_caster = MakeRayCaster("depth", BackendArg(std::get<std::string>(rc)));
          }
          else
          {
            p.depth_ray_caster = std::get<std::shared_ptr<RayCaster>>(rc);
          }
        },
        "A depth ray caster (see make_ray_caster(\"depth\", backend)); a backend string may be "
        "assigned, in which case a depth ray caster is created for that backend.")
    .def_rw("label_vol", &LabelProj::label_vol)
    .def_rw("labels_to_proj", &LabelProj::labels_to_proj)
    .def_rw("cam", &LabelProj::cam)
    .def_rw("obj_poses", &LabelProj::obj_poses,
            "Camera to volume pose of each label in labels_to_proj.")
    .def("set_single_chan_seg_fn", &SetSingleChanSegFn, "fn"_a.none(),
         "Sets a function (or None) used to create a single channel segmentation. The function "
         "is passed a list of depth images (2D float32 arrays): entry 0 is not used and entry i "
         "(i > 0) is the depth at which each ray intersects label labels_to_proj[i-1] "
         "(values >= RAY_CAST_MAX_DEPTH indicate no intersection). The function must return a "
         "uint8 array with the same shape. This must be set prior to calling init().")
    .def("set_verbose", [](LabelProj& p, const bool v) { SetVerbose(p, v); }, "verbose"_a = true)
    .def("init",
        [](LabelProj& p)
        {
          if (!p.depth_ray_caster)
          {
            throw nb::value_error("depth_ray_caster has not been set");
          }

          CheckedImg(p.label_vol, "label_vol");

          p.init();
        })
    .def("run",
        [](LabelProj& p)
        {
          if (p.obj_poses.size() != p.labels_to_proj.size())
          {
            throw nb::value_error("obj_poses must have one pose for each entry of labels_to_proj");
          }

          p.run();
        })
    .def_prop_ro("seg_channels",
        [](const LabelProj& p)
        {
          nb::list chans;
          for (const auto& c : p.seg_channels)
          {
            chans.append(CvMatF32ToArray(c));
          }
          return chans;
        },
        "Output: a list of 2D float32 arrays. Entry 0 is the background channel (1 when no "
        "labels are intersected). Entry i (i > 0) corresponds to labels_to_proj[i-1] and is "
        "1 / (number of labels intersected) when the label is intersected, 0 otherwise.")
    .def_ro("single_chan_seg", &LabelProj::single_chan_seg,
            "Output: the single channel segmentation, available when a single channel "
            "segmentation function was set.");
}
