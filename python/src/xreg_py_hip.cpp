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

#include "xregPnPUtils.h"
#include "xregAnatCoordFrames.h"
#include "xregHipSegUtils.h"
#include "xregPerspectiveXform.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

APPCenterPt StrToAPPCenterPt(const std::string& s)
{
  const std::string n = NormName(s);

  if (n == "median_iliac_crest")
  {
    return kAPP_ORIGIN_MEDIAN_ILIAC_CREST;
  }
  else if (n == "left_fh")
  {
    return kAPP_ORIGIN_LEFT_FH;
  }
  else if (n == "right_fh")
  {
    return kAPP_ORIGIN_RIGHT_FH;
  }

  throw nb::value_error("APP origin must be one of: \"median_iliac_crest\", \"left_fh\", \"right_fh\"");
}

}  // un-named

void xreg::py::BindHip(nb::module_& m)
{
  //////////////////////////////////////////////////////////////////////////////
  // Pose Initialization

  m.def("pnp_posit_and_reproj_cmaes",
      [](const CameraModel& cam, const LandMap3& lands_3d, const LandMap2& lands_2d)
      {
        nb::gil_scoped_release no_gil;

        return PnPPOSITAndReprojCMAES(cam, lands_3d, lands_2d);
      },
      "cam"_a, "lands_3d"_a, "lands_2d"_a,
      "Solves a PnP problem (POSIT followed by re-projection distance minimization) using "
      "corresponding 3D and 2D landmarks (dicts of name -> point). Returns the camera to "
      "volume pose (4x4).");

  m.def("pnp_reproj_cmaes",
      [](const CameraModel& cam, const LandMap3& lands_3d, const LandMap2& lands_2d,
         const FrameTransform& init_cam_to_vol)
      {
        nb::gil_scoped_release no_gil;

        return PnPReprojCMAES(cam, lands_3d, lands_2d, init_cam_to_vol);
      },
      "cam"_a, "lands_3d"_a, "lands_2d"_a, "init_cam_to_vol"_a,
      "Solves a PnP problem by minimizing landmark re-projection distances, starting from an "
      "initial pose. Returns the camera to volume pose (4x4).");

  //////////////////////////////////////////////////////////////////////////////
  // Anatomical Coordinate Frames

  m.def("anterior_pelvic_plane_from_landmarks",
      [](const LandMap3& lands, const std::string& origin)
      {
        return AnteriorPelvicPlaneFromLandmarksMap(lands, StrToAPPCenterPt(origin));
      },
      "lands"_a, "origin"_a = "median_iliac_crest",
      "Computes the Anterior Pelvic Plane (APP) frame from landmarks (ASIS-{l,r}, IPS-{l,r}, and "
      "FH-{l,r} when used as the origin). Returns the transform mapping APP points to the volume. "
      "origin: \"median_iliac_crest\", \"left_fh\", or \"right_fh\".");

  m.def("create_ap_view_of_app",
      [](const CameraModel& cam, const CoordScalar intersect_pt_loc_on_src_det_line,
         const bool src_is_posterior, const bool pat_up,
         const std::optional<Pt3>& ref_pt_wrt_app, const std::optional<Pt2>& ref_pt_2d_idx)
      {
        if (ref_pt_wrt_app.has_value() != ref_pt_2d_idx.has_value())
        {
          throw nb::value_error("ref_pt_wrt_app and ref_pt_2d_idx must both be specified, or neither");
        }

        if (ref_pt_wrt_app)
        {
          return CreateAPViewOfAPP(cam, intersect_pt_loc_on_src_det_line, src_is_posterior, pat_up,
                                   *ref_pt_wrt_app, *ref_pt_2d_idx);
        }
        else
        {
          return CreateAPViewOfAPP(cam, intersect_pt_loc_on_src_det_line, src_is_posterior, pat_up);
        }
      },
      "cam"_a, "intersect_pt_loc_on_src_det_line"_a, "src_is_posterior"_a = true, "pat_up"_a = true,
      "ref_pt_wrt_app"_a = nb::none(), "ref_pt_2d_idx"_a = nb::none(),
      "Creates a nominal AP view of the APP. Returns the transform mapping camera world "
      "coordinates to the APP. Optionally, a 3D point (wrt APP) is aligned with a 2D pixel location.");

  //////////////////////////////////////////////////////////////////////////////
  // Segmentation Utilities

  m.def("guess_pelvis_femur_pao_frag_labels",
      [](const itk::Image<unsigned char,3>::Pointer& labels, const Pt3& femur_pt,
         const bool labels_has_frag, const bool labels_has_cut)
      {
        const auto t = GuessPelvisFemurPAOFragLabels(CheckedImg(labels, "labels").GetPointer(),
                                                     femur_pt, labels_has_frag, labels_has_cut);
        return std::make_tuple(static_cast<int>(std::get<0>(t)), static_cast<int>(std::get<1>(t)),
                               static_cast<int>(std::get<2>(t)), static_cast<int>(std::get<3>(t)));
      },
      "labels"_a, "femur_pt"_a, "labels_has_frag"_a = true, "labels_has_cut"_a = true,
      "Guesses the (pelvis, femur, fragment, cut) labels of a PAO label map, given a point in the femur.");
}
