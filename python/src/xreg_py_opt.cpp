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

#include "xreg_py_opt.h"

#include "xregDistInterface.h"
#include "xregNormDist.h"
#include "xregFoldNormDist.h"
#include "xregLogNormDist.h"
#include "xregNullDist.h"
#include "xregSE3OptVars.h"
#include "xregRegi2D3DPenaltyFn.h"
#include "xregRegi2D3DPenaltyFnSE3EulerDecomp.h"
#include "xregRegi2D3DPenaltyFnSE3Mag.h"
#include "xregRegi2D3DPenaltyFnGlobalPelvis.h"
#include "xregRegi2D3DPenaltyFnLandReproj.h"
#include "xregRegi2D3DPenaltyFnRelPose.h"
#include "xregRegi2D3DPenaltyFnCombo.h"

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

using DistPtr = std::shared_ptr<Dist>;

// A point at which a distribution is evaluated: a scalar is allowed for 1D distributions
using DistArg = std::variant<CoordScalar,PtN>;

PtN ToDistPt(const DistArg& x)
{
  if (std::holds_alternative<CoordScalar>(x))
  {
    PtN p(1);
    p[0] = std::get<CoordScalar>(x);
    return p;
  }

  return std::get<PtN>(x);
}

}  // un-named

xreg::py::OptVarsPtr xreg::py::MakeOptVars(const OptVarsArg& arg)
{
  if (std::holds_alternative<OptVarsPtr>(arg))
  {
    const auto& p = std::get<OptVarsPtr>(arg);
    if (!p)
    {
      throw nb::value_error("optimization variables object is None");
    }
    return p;
  }

  const std::string n = NormName(std::get<std::string>(arg));

  if ((n == "se3") || (n == "se3_lie"))
  {
    return std::make_shared<SE3OptVarsLieAlg>();
  }
  else if (n == "se3_euler")
  {
    return std::make_shared<SE3OptVarsEuler>();
  }
  else if ((n == "so3") || (n == "so3_lie"))
  {
    return std::make_shared<SO3OptVarsLieAlg>();
  }
  else if (n == "so3_euler")
  {
    return std::make_shared<SO3OptVarsEuler>();
  }
  else if (n == "trans")
  {
    return std::make_shared<SE3OptVarsTransOnly>();
  }
  else if (n == "trans_x")
  {
    return std::make_shared<SE3OptVarsTransXOnly>();
  }
  else if (n == "trans_y")
  {
    return std::make_shared<SE3OptVarsTransYOnly>();
  }
  else if (n == "trans_z")
  {
    return std::make_shared<SE3OptVarsTransZOnly>();
  }
  else if (n == "rot_x")
  {
    return std::make_shared<SO3OptVarsOnlyX>();
  }
  else if (n == "rot_y")
  {
    return std::make_shared<SO3OptVarsOnlyY>();
  }
  else if (n == "rot_z")
  {
    return std::make_shared<SO3OptVarsOnlyZ>();
  }

  throw nb::value_error(("Invalid optimization variables string: \"" + std::get<std::string>(arg) +
                         "\" (valid: \"se3\", \"se3_euler\", \"so3\", \"so3_euler\", \"trans\", "
                         "\"trans_x\", \"trans_y\", \"trans_z\", \"rot_x\", \"rot_y\", \"rot_z\")").c_str());
}

void xreg::py::BindOpt(nb::module_& m)
{
  //////////////////////////////////////////////////////////////////////////////
  // Probability Distributions (used by penalty functions/regularizers)

  nb::class_<Dist>(m, "Dist", "Probability distribution interface.")
    .def("density", [](const Dist& d, const DistArg& x) { return d.density(ToDistPt(x)); }, "x"_a,
         "Density at a point (a scalar may be passed for 1D distributions).")
    .def("log_density", [](const Dist& d, const DistArg& x) { return d.log_density(ToDistPt(x)); }, "x"_a,
         "Log density at a point (a scalar may be passed for 1D distributions).")
    .def_prop_ro("dim", &Dist::dim)
    .def_prop_ro("normalized", &Dist::normalized);

  nb::class_<NormalDist1D, Dist>(m, "NormalDist1D", "1D normal distribution.")
    .def(nb::init<const CoordScalar,const CoordScalar>(), "mean"_a = 0, "std_dev"_a = 1);

  nb::class_<FoldNormDist, Dist>(m, "FoldNormDist", "Folded normal distribution.")
    .def(nb::init<const CoordScalar,const CoordScalar>(), "mean"_a = 1, "std_dev"_a = 1);

  nb::class_<LogNormDist, Dist>(m, "LogNormDist", "Log-normal distribution.")
    .def(nb::init<const CoordScalar,const CoordScalar>(), "mean"_a = 0, "std_dev"_a = 1);

  nb::class_<NullDist, Dist>(m, "NullDist",
      "Null distribution: always has a density of one (e.g. used to disable a component of a penalty).")
    .def(nb::init<const size_type>(), "dim"_a = 1);

  //////////////////////////////////////////////////////////////////////////////
  // Penalty Functions/Regularizers

  nb::class_<Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFn",
      "Penalty function (regularizer) for intensity-based 2D/3D registration.")
    .def_prop_ro("reg_vals", &Regi2D3DPenaltyFn::reg_vals)
    .def_prop_ro("log_probs", &Regi2D3DPenaltyFn::log_probs);

  using PenEuler = Regi2D3DPenaltyFnSE3EulerDecomp;

  nb::class_<PenEuler, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnSE3EulerDecomp",
      "Penalizes each Euler angle and translation component of the pose change of a single object.")
    .def("__init__",
        [](PenEuler* p, const DistPtr& rot_x_pdf, const DistPtr& rot_y_pdf, const DistPtr& rot_z_pdf,
           const DistPtr& trans_x_pdf, const DistPtr& trans_y_pdf, const DistPtr& trans_z_pdf)
        {
          new (p) PenEuler();
          p->rot_x_pdf   = rot_x_pdf;
          p->rot_y_pdf   = rot_y_pdf;
          p->rot_z_pdf   = rot_z_pdf;
          p->trans_x_pdf = trans_x_pdf;
          p->trans_y_pdf = trans_y_pdf;
          p->trans_z_pdf = trans_z_pdf;
        },
        "rot_x_pdf"_a.none() = nb::none(), "rot_y_pdf"_a.none() = nb::none(),
        "rot_z_pdf"_a.none() = nb::none(), "trans_x_pdf"_a.none() = nb::none(),
        "trans_y_pdf"_a.none() = nb::none(), "trans_z_pdf"_a.none() = nb::none())
    .def_rw("rot_x_pdf", &PenEuler::rot_x_pdf)
    .def_rw("rot_y_pdf", &PenEuler::rot_y_pdf)
    .def_rw("rot_z_pdf", &PenEuler::rot_z_pdf)
    .def_rw("trans_x_pdf", &PenEuler::trans_x_pdf)
    .def_rw("trans_y_pdf", &PenEuler::trans_y_pdf)
    .def_rw("trans_z_pdf", &PenEuler::trans_z_pdf)
    .def_rw("obj_idx", &PenEuler::obj_idx)
    .def_rw("use_xforms_from_opt", &PenEuler::use_xforms_from_opt);

  using PenMag = Regi2D3DPenaltyFnSE3Mag;

  nb::class_<PenMag, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnSE3Mag",
      "Penalizes the magnitudes of the rotation and translation of the pose change of each object.")
    .def("__init__",
        [](PenMag* p, const std::vector<DistPtr>& rot_pdfs_per_obj,
           const std::vector<DistPtr>& trans_pdfs_per_obj)
        {
          new (p) PenMag();
          p->rot_pdfs_per_obj   = rot_pdfs_per_obj;
          p->trans_pdfs_per_obj = trans_pdfs_per_obj;
        },
        "rot_pdfs_per_obj"_a = std::vector<DistPtr>(),
        "trans_pdfs_per_obj"_a = std::vector<DistPtr>())
    .def_rw("rot_pdfs_per_obj", &PenMag::rot_pdfs_per_obj)
    .def_rw("trans_pdfs_per_obj", &PenMag::trans_pdfs_per_obj)
    .def_rw("apply_reg_for_obj", &PenMag::apply_reg_for_obj);

  nb::class_<Regi2D3DPenaltyFnGlobalPelvis, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnGlobalPelvis",
      "Heuristic penalty for global pelvis registration (e.g. an AP-like view is expected).")
    .def(nb::init<>())
    .def_rw("vol_idx", &Regi2D3DPenaltyFnGlobalPelvis::vol_idx)
    .def_rw("pat_is_up", &Regi2D3DPenaltyFnGlobalPelvis::pat_is_up)
    .def_rw("left_fh_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::left_fh_wrt_vol)
    .def_rw("right_fh_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::right_fh_wrt_vol)
    .def_rw("left_asis_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::left_asis_wrt_vol)
    .def_rw("right_asis_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::right_asis_wrt_vol)
    .def_rw("left_iof_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::left_iof_wrt_vol)
    .def_rw("right_iof_wrt_vol", &Regi2D3DPenaltyFnGlobalPelvis::right_iof_wrt_vol);

  nb::class_<Regi2D3DPenaltyFnLandReproj, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnLandReproj",
      "Penalizes the re-projection distances of corresponding 3D and 2D landmarks. "
      "NOTE: 2D landmarks must be adjusted for any cropping and downsampling of the level.")
    .def(nb::init<>())
    .def("set_lands", &Regi2D3DPenaltyFnLandReproj::set_lands, "lands_3d"_a, "lands_2d"_a,
         "Sets corresponding landmarks from name -> point mappings (only names in both are used).")
    .def_rw("vol_idx", &Regi2D3DPenaltyFnLandReproj::vol_idx)
    .def_rw("lands_3d", &Regi2D3DPenaltyFnLandReproj::lands_3d)
    .def_rw("lands_2d", &Regi2D3DPenaltyFnLandReproj::lands_2d)
    .def_rw("num_lands", &Regi2D3DPenaltyFnLandReproj::num_lands)
    .def_rw("std_dev", &Regi2D3DPenaltyFnLandReproj::std_dev)
    .def_rw("use_outlier_det", &Regi2D3DPenaltyFnLandReproj::use_outlier_det)
    .def_rw("outlier_thresh_num_stds", &Regi2D3DPenaltyFnLandReproj::outlier_thresh_num_stds);

  nb::class_<Regi2D3DPenaltyFnSE3RelPose, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnSE3RelPose",
      "Penalizes the relative pose between two objects.")
    .def(nb::init<>())
    .def_rw("rot_pdf", &Regi2D3DPenaltyFnSE3RelPose::rot_pdf)
    .def_rw("trans_pdf", &Regi2D3DPenaltyFnSE3RelPose::trans_pdf)
    .def_rw("vol_idx_1", &Regi2D3DPenaltyFnSE3RelPose::vol_idx_1)
    .def_rw("vol_idx_2", &Regi2D3DPenaltyFnSE3RelPose::vol_idx_2);

  nb::class_<Regi2D3DPenaltyFnCombo, Regi2D3DPenaltyFn>(m, "Regi2D3DPenaltyFnCombo",
      "Combination (sum) of several penalty functions.")
    .def(nb::init<>())
    .def_rw("pen_fns", &Regi2D3DPenaltyFnCombo::pen_fns);

  //////////////////////////////////////////////////////////////////////////////
  // Optimization Variables (SE(3) parameterizations)

  nb::class_<SE3OptVars>(m, "SE3OptVars",
      "Parameterization of the rigid transformation space used during optimization.")
    .def("__call__", &SE3OptVars::operator(), "x"_a,
         "Maps an optimization parameter vector to a 4x4 rigid transformation.")
    .def_prop_ro("num_params", &SE3OptVars::num_params);

  nb::class_<SE3OptVarsLieAlg, SE3OptVars>(m, "SE3OptVarsLieAlg",
      "se(3) Lie algebra: (rot. x, rot. y, rot. z, trans. x, trans. y, trans. z)")
    .def(nb::init<>());

  nb::class_<SE3OptVarsEuler, SE3OptVars>(m, "SE3OptVarsEuler", "Euler angles and translations.")
    .def(nb::init<const unsigned long,const unsigned long,const unsigned long,
                  const unsigned long,const unsigned long,const unsigned long>(),
         "rot_x_idx"_a = 5, "rot_y_idx"_a = 4, "rot_z_idx"_a = 3,
         "trans_x_idx"_a = 2, "trans_y_idx"_a = 1, "trans_z_idx"_a = 0);

  nb::class_<SE3OptVarsTransOnly, SE3OptVars>(m, "SE3OptVarsTransOnly", "Translation only.")
    .def(nb::init<const bool,const bool,const bool>(),
         "use_x"_a = true, "use_y"_a = true, "use_z"_a = true);

  nb::class_<SO3OptVarsLieAlg, SE3OptVars>(m, "SO3OptVarsLieAlg", "Rotation only, so(3) Lie algebra.")
    .def(nb::init<>());

  nb::class_<SO3OptVarsEuler, SE3OptVars>(m, "SO3OptVarsEuler", "Rotation only, Euler angles.")
    .def(nb::init<const unsigned long,const unsigned long,const unsigned long>(),
         "rot_x_idx"_a = 2, "rot_y_idx"_a = 1, "rot_z_idx"_a = 0);

  m.def("make_opt_vars", &MakeOptVars, "kind"_a,
        "Creates optimization variables from a string: \"se3\" (Lie algebra), \"se3_euler\", "
        "\"so3\", \"so3_euler\", \"trans\", \"trans_x\", \"trans_y\", \"trans_z\", \"rot_x\", "
        "\"rot_y\", \"rot_z\".");
}
