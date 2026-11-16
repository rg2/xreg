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

#include "xreg_py_common.h"

#include <sstream>

#include <nanobind/operators.h>

#include "xregPerspectiveXform.h"
#include "xregProjData.h"
#include "xregProjPreProc.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

using RotToPatUpArg = std::variant<ProjDataRotToPatUp,int>;

std::optional<ProjDataRotToPatUp> ToRotToPatUp(const std::optional<RotToPatUpArg>& r)
{
  if (!r)
  {
    return std::nullopt;
  }

  if (std::holds_alternative<ProjDataRotToPatUp>(*r))
  {
    return std::get<ProjDataRotToPatUp>(*r);
  }

  switch (std::get<int>(*r))
  {
  case 0:
    return ProjDataRotToPatUp::kZERO;
  case 90:
    return ProjDataRotToPatUp::kNINETY;
  case 180:
    return ProjDataRotToPatUp::kONE_EIGHTY;
  case 270:
    return ProjDataRotToPatUp::kTWO_SEVENTY;
  default:
    throw nb::value_error("rot_to_pat_up must be None, a ProjDataRotToPatUp, or one of 0, 90, 180, 270");
  }
}

void RunProjPreProc(ProjPreProc& pp)
{
  for (size_type i = 0; i < pp.input_projs.size(); ++i)
  {
    if (!pp.input_projs[i].img)
    {
      throw nb::value_error(("input_projs[" + std::to_string(i) + "] has no image").c_str());
    }
  }

  nb::gil_scoped_release no_gil;

  pp();
}

}  // un-named

void xreg::py::BindProjData(nb::module_& m)
{
  //////////////////////////////////////////////////////////////////////////////
  // Camera Model

  nb::class_<CameraModel> cam_cls(m, "CameraModel",
      "Perspective projection camera model (e.g. an X-ray imager). "
      "See xregPerspectiveXform.h for a full description of the coordinate frames. "
      "Prefer the setup() methods to setting individual fields, which must remain consistent.");

  nb::enum_<CameraModel::CameraCoordFrame>(cam_cls, "CameraCoordFrame")
    .value("ORIGIN_AT_FOCAL_PT_DET_POS_Z", CameraModel::kORIGIN_AT_FOCAL_PT_DET_POS_Z)
    .value("ORIGIN_AT_FOCAL_PT_DET_NEG_Z", CameraModel::kORIGIN_AT_FOCAL_PT_DET_NEG_Z)
    .value("ORIGIN_ON_DETECTOR", CameraModel::kORIGIN_ON_DETECTOR);

  cam_cls
    .def(nb::init<>())
    .def("setup",
        nb::overload_cast<const CoordScalar,const size_type,const size_type,
                          const CoordScalar,const CoordScalar>(&CameraModel::setup),
        "focal_len"_a, "num_rows"_a, "num_cols"_a, "row_spacing"_a, "col_spacing"_a,
        "Basic camera model: principal point at the detector center, zero shear, identity extrinsic.")
    .def("setup",
        nb::overload_cast<const Mat3x4&,const size_type,const size_type,
                          const CoordScalar,const CoordScalar,const bool>(&CameraModel::setup),
        "proj_mat"_a, "num_rows"_a, "num_cols"_a, "row_spacing"_a, "col_spacing"_a,
        "use_extrins"_a = true,
        "Camera model from a 3x4 projection matrix.")
    .def("setup",
        nb::overload_cast<const Mat3x3&,const Mat4x4&,const size_type,const size_type,
                          const CoordScalar,const CoordScalar>(&CameraModel::setup),
        "intrins"_a, "extrins"_a, "num_rows"_a, "num_cols"_a, "row_spacing"_a, "col_spacing"_a,
        "Camera model from intrinsic (3x3) and extrinsic (4x4) matrices.")
    .def_rw("intrins", &CameraModel::intrins)
    .def_rw("intrins_inv", &CameraModel::intrins_inv)
    .def_rw("extrins", &CameraModel::extrins,
            "Transforms \"world\" coordinates to camera coordinates (4x4).")
    .def_rw("extrins_inv", &CameraModel::extrins_inv)
    .def_rw("pinhole_pt", &CameraModel::pinhole_pt)
    .def_rw("focal_len", &CameraModel::focal_len)
    .def_rw("num_det_rows", &CameraModel::num_det_rows)
    .def_rw("num_det_cols", &CameraModel::num_det_cols)
    .def_rw("det_row_spacing", &CameraModel::det_row_spacing)
    .def_rw("det_col_spacing", &CameraModel::det_col_spacing)
    .def_rw("coord_frame_type", &CameraModel::coord_frame_type)
    .def("proj_pt_to_det_pt", &CameraModel::proj_pt_to_det_pt, "pt"_a)
    .def("phys_pt_to_ind_pt", &CameraModel::phys_pt_to_ind_pt, "pt"_a,
         "Projects a 3D world point to a continuous (homogeneous) 2D pixel index (col, row, 1).")
    .def("phys_pts_to_ind_pts", &CameraModel::phys_pts_to_ind_pts, "pts"_a)
    .def("ind_pt_to_phys_det_pt",
         nb::overload_cast<const Pt2&>(&CameraModel::ind_pt_to_phys_det_pt, nb::const_), "ind"_a)
    .def(nb::self == nb::self)
    .def(nb::self != nb::self)
    .def("__repr__",
        [](const CameraModel& cam)
        {
          std::stringstream ss;
          ss << "CameraModel(num_det_rows=" << cam.num_det_rows
             << ", num_det_cols=" << cam.num_det_cols
             << ", det_row_spacing=" << cam.det_row_spacing
             << ", det_col_spacing=" << cam.det_col_spacing
             << ", focal_len=" << cam.focal_len << ')';
          return ss.str();
        });

  //////////////////////////////////////////////////////////////////////////////
  // Projection Data

  nb::enum_<ProjDataRotToPatUp>(m, "ProjDataRotToPatUp", nb::is_arithmetic(),
      "Rotation (degrees) required to orient the patient \"up\" in a projection image.")
    .value("ZERO", ProjDataRotToPatUp::kZERO)
    .value("NINETY", ProjDataRotToPatUp::kNINETY)
    .value("ONE_EIGHTY", ProjDataRotToPatUp::kONE_EIGHTY)
    .value("TWO_SEVENTY", ProjDataRotToPatUp::kTWO_SEVENTY);

  nb::class_<ProjDataF32>(m, "ProjData",
      "A single projection: camera model, image (single precision float), 2D landmarks, "
      "and some optional metadata.")
    .def("__init__",
        [](ProjDataF32* pd, const CameraModel& cam, const ProjDataF32::ProjPtr& img,
           const LandMap2& lands, const std::optional<RotToPatUpArg>& rot_to_pat_up)
        {
          new (pd) ProjDataF32();
          pd->cam           = cam;
          pd->img           = img;
          pd->landmarks     = lands;
          pd->rot_to_pat_up = ToRotToPatUp(rot_to_pat_up);
        },
        "cam"_a = CameraModel(), "img"_a = nb::none(), "landmarks"_a = LandMap2(),
        "rot_to_pat_up"_a = nb::none())
    .def_rw("cam", &ProjDataF32::cam)
    .def_rw("img", &ProjDataF32::img, "The projection image (shared, not copied), or None.")
    .def_rw("landmarks", &ProjDataF32::landmarks,
            "Mapping of landmark names to continuous 2D pixel indices (col, row). "
            "NOTE: assign an entire dict - modifying the returned dict in place has no effect.")
    .def_prop_rw("rot_to_pat_up",
        [](const ProjDataF32& pd) { return pd.rot_to_pat_up; },
        [](ProjDataF32& pd, const std::optional<RotToPatUpArg>& r) { pd.rot_to_pat_up = ToRotToPatUp(r); },
        "Rotation required to orient the patient \"up\" in the image: None when unknown, "
        "otherwise a ProjDataRotToPatUp (integers 0, 90, 180, 270 are accepted when setting).")
    .def_rw("det_spacings_from_orig_meta", &ProjDataF32::det_spacings_from_orig_meta)
    .def("__repr__",
        [](const ProjDataF32& pd)
        {
          std::stringstream ss;
          ss << "ProjData(rows=" << pd.cam.num_det_rows << ", cols=" << pd.cam.num_det_cols
             << ", has_img=" << (pd.img ? "True" : "False")
             << ", num_landmarks=" << pd.landmarks.size() << ')';
          return ss.str();
        });

  m.def("downsample_proj_data",
      [](const ProjDataF32List& projs, const CoordScalar ds_factor, const bool force_even_dims)
      {
        return DownsampleProjData(projs, ds_factor, force_even_dims);
      },
      "projs"_a, "ds_factor"_a, "force_even_dims"_a = false,
      "Downsamples projection images and updates the camera models (and landmarks) accordingly.");

  m.def("downsample_proj_data",
      [](const ProjDataF32& proj, const CoordScalar ds_factor, const bool force_even_dims)
      {
        return DownsampleProjData(proj, ds_factor, force_even_dims);
      },
      "proj"_a, "ds_factor"_a, "force_even_dims"_a = false);

  //////////////////////////////////////////////////////////////////////////////
  // Projection Pre-Processing

  nb::class_<ProjPreProcParams>(m, "ProjPreProcParams",
      "Parameters of the standard projection pre-processing (see ProjPreProc).")
    .def("__init__",
        [](ProjPreProcParams* p, const int crop_width, const bool auto_mask, const bool invert_mask,
           const double auto_mask_thresh, const double auto_mask_level,
           const bool allow_iterative_thresh, const bool no_log_remap)
        {
          new (p) ProjPreProcParams();
          p->crop_width             = crop_width;
          p->auto_mask              = auto_mask;
          p->invert_mask            = invert_mask;
          p->auto_mask_thresh       = auto_mask_thresh;
          p->auto_mask_level        = auto_mask_level;
          p->allow_iterative_thresh = allow_iterative_thresh;
          p->no_log_remap           = no_log_remap;
        },
        "crop_width"_a = ProjPreProcParams().crop_width,
        "auto_mask"_a = ProjPreProcParams().auto_mask,
        "invert_mask"_a = ProjPreProcParams().invert_mask,
        "auto_mask_thresh"_a = ProjPreProcParams().auto_mask_thresh,
        "auto_mask_level"_a = ProjPreProcParams().auto_mask_level,
        "allow_iterative_thresh"_a = ProjPreProcParams().allow_iterative_thresh,
        "no_log_remap"_a = ProjPreProcParams().no_log_remap)
    .def_rw("crop_width", &ProjPreProcParams::crop_width,
            "Number of pixels cropped from each image boundary.")
    .def_rw("auto_mask", &ProjPreProcParams::auto_mask)
    .def_rw("invert_mask", &ProjPreProcParams::invert_mask)
    .def_rw("auto_mask_thresh", &ProjPreProcParams::auto_mask_thresh)
    .def_rw("auto_mask_level", &ProjPreProcParams::auto_mask_level)
    .def_rw("allow_iterative_thresh", &ProjPreProcParams::allow_iterative_thresh)
    .def_rw("no_log_remap", &ProjPreProcParams::no_log_remap,
            "Do NOT perform log remapping of the projection intensities.");

  nb::class_<ProjPreProc>(m, "ProjPreProc",
      "Standard pre-processing of projections prior to intensity-based registration "
      "(boundary cropping, log remapping, optional automatic masking). "
      "NOTE: params is returned by reference, so pp.params.crop_width = 0 works as expected.")
    .def(nb::init<>())
    .def_rw("params", &ProjPreProc::params)
    .def_rw("input_projs", &ProjPreProc::input_projs,
            "NOTE: assign an entire list - modifying the returned list in place has no effect.")
    .def_ro("output_projs", &ProjPreProc::output_projs)
    .def_prop_ro("output_masks",
        [](const ProjPreProc& pp)
        {
          std::vector<ProjDataU8::ProjPtr> masks;
          masks.reserve(pp.output_mask_projs.size());

          for (const auto& pd : pp.output_mask_projs)
          {
            masks.push_back(pd.img);
          }

          return masks;
        },
        "Masks computed when params.auto_mask is enabled.")
    .def("set_verbose", [](ProjPreProc& pp, const bool v) { SetVerbose(pp, v); }, "verbose"_a = true)
    .def("run", &RunProjPreProc,
         "Runs pre-processing on input_projs, populating output_projs (and output_masks).")
    .def("__call__", &RunProjPreProc);
}
