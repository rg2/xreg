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

#include <sstream>

#include <nanobind/stl/bind_vector.h>

#include "xregNDRange.h"
#include "xregSE3OptVars.h"
#include "xregRayCastInterface.h"
#include "xregImgSimMetric2D.h"
#include "xregRegi2D3DPenaltyFn.h"
#include "xregIntensity2D3DRegi.h"
#include "xregIntensity2D3DRegiCMAES.h"
#include "xregIntensity2D3DRegiNLOptInterface.h"
#include "xregIntensity2D3DRegiBOBYQA.h"
#include "xregIntensity2D3DRegiNelderMead.h"
#include "xregIntensity2D3DRegiSbplx.h"
#include "xregIntensity2D3DRegiNEWUOA.h"
#include "xregIntensity2D3DRegiPRAXIS.h"
#include "xregIntensity2D3DRegiCRS.h"
#include "xregIntensity2D3DRegiDIRECT.h"
#include "xregIntensity2D3DRegiDESCH.h"
#include "xregIntensity2D3DRegiDISRES.h"
#include "xregIntensity2D3DRegiDiffEvo.h"
#include "xregIntensity2D3DRegiPSO.h"
#include "xregIntensity2D3DRegiHillClimb.h"
#include "xregIntensity2D3DRegiExhaustive.h"
#include "xregMultiObjMultiLevel2D3DRegi.h"
#include "xregMultiObjMultiLevel2D3DRegiDebug.h"

namespace xreg
{
namespace py
{

using InitPose    = MultiLevelMultiObjRegi::Level::SingleRegi::InitPose;
using InitPosePtr = std::shared_ptr<InitPose>;
using InitPoseArg = std::variant<InitPosePtr,std::string>;

using RefFramePtr = std::shared_ptr<MultiLevelMultiObjRegi::RefFrame>;
using RefFrameArg = std::variant<RefFramePtr,std::string>;


using IndexList = MultiLevelMultiObjRegi::IndexList;

/// \brief Python facing version of MultiLevelMultiObjRegi::Level::SingleRegi.
///
/// Instances are held by shared pointers so that Python code may modify
/// registrations stored in a level, e.g. ml.levels[0].regis[1].regi = ...
/// These are copied into the C++ MultiLevelMultiObjRegi object just before running.
struct PySingleRegi
{
  IndexList mov_vols;

  IndexList static_vols;

  IndexList ref_frames;

  std::vector<InitPosePtr> init_mov_vol_poses;

  std::vector<InitPosePtr> static_vol_poses;

  std::shared_ptr<Intensity2D3DRegi> regi;
};

using PySingleRegiPtr  = std::shared_ptr<PySingleRegi>;
using PySingleRegiList = std::vector<PySingleRegiPtr>;

/// \brief Python facing version of MultiLevelMultiObjRegi::Level.
struct PyLevel
{
  double ds_factor = 1.0;

  std::shared_ptr<RayCaster> ray_caster;

  Intensity2D3DRegi::SimMetricList sim_metrics;

  IndexList fixed_imgs_to_use;

  PySingleRegiList regis;
};

using PyLevelPtr  = std::shared_ptr<PyLevel>;
using PyLevelList = std::vector<PyLevelPtr>;

/// \brief Python facing version of MultiLevelMultiObjRegi.
///
/// All members are used directly, except for the levels, which are stored
/// in py_levels and copied into levels when running.
struct PyMultiLevelMultiObjRegi : MultiLevelMultiObjRegi
{
  PyLevelList py_levels;
};

}  // py
}  // xreg

NB_MAKE_OPAQUE(xreg::py::PyLevelList)
NB_MAKE_OPAQUE(xreg::py::PySingleRegiList)

namespace  // un-named
{

using namespace xreg;
using namespace xreg::py;
using namespace nb::literals;

using MLMO      = MultiLevelMultiObjRegi;
using DebugML   = DebugRegiResultsMultiLevel;
using InitPoseId                   = MLMO::Level::SingleRegi::InitPoseId;
using InitPosePrevPoseEst          = MLMO::Level::SingleRegi::InitPosePrevPoseEst;
using InitPosePrevPoseEstTransOnly = MLMO::Level::SingleRegi::InitPosePrevPoseEstTransOnly;
using InitPosePrevPoseEstRotOnly   = MLMO::Level::SingleRegi::InitPosePrevPoseEstRotOnly;

// Parses "<prefix>:<index>"
std::optional<size_type> ParseIdxSuffix(const std::string& s, const std::string& prefix)
{
  const std::string p = prefix + ':';

  if ((s.size() > p.size()) && (s.compare(0, p.size(), p) == 0))
  {
    try
    {
      size_t num_chars = 0;
      const auto idx = std::stoul(s.substr(p.size()), &num_chars);
      if ((p.size() + num_chars) == s.size())
      {
        return static_cast<size_type>(idx);
      }
    }
    catch (...)
    { }

    throw nb::value_error(("Invalid index in string: \"" + s + "\"").c_str());
  }

  return std::nullopt;
}

InitPosePtr MakeInitPose(const InitPoseArg& arg)
{
  if (std::holds_alternative<InitPosePtr>(arg))
  {
    const auto& p = std::get<InitPosePtr>(arg);
    if (!p)
    {
      throw nb::value_error("initial pose object is None");
    }
    return p;
  }

  const std::string n = NormName(std::get<std::string>(arg));

  if ((n == "identity") || (n == "id"))
  {
    return std::make_shared<InitPoseId>();
  }
  else if (auto idx = ParseIdxSuffix(n, "prev"))
  {
    auto p = std::make_shared<InitPosePrevPoseEst>();
    p->vol_idx = *idx;
    return p;
  }
  else if (auto idx = ParseIdxSuffix(n, "prev_trans"))
  {
    auto p = std::make_shared<InitPosePrevPoseEstTransOnly>();
    p->vol_idx = *idx;
    return p;
  }
  else if (auto idx = ParseIdxSuffix(n, "prev_rot"))
  {
    auto p = std::make_shared<InitPosePrevPoseEstRotOnly>();
    p->vol_idx = *idx;
    return p;
  }

  throw nb::value_error(("Invalid initial pose string: \"" + std::get<std::string>(arg) +
                         "\" (valid: \"identity\", \"prev:<vol idx>\", \"prev_trans:<vol idx>\", "
                         "\"prev_rot:<vol idx>\")").c_str());
}

std::vector<InitPosePtr> MakeInitPoses(const std::vector<InitPoseArg>& args)
{
  std::vector<InitPosePtr> poses;
  poses.reserve(args.size());

  for (const auto& a : args)
  {
    poses.push_back(MakeInitPose(a));
  }

  return poses;
}

RefFramePtr MakeRefFrame(const RefFrameArg& arg)
{
  if (std::holds_alternative<RefFramePtr>(arg))
  {
    const auto& p = std::get<RefFramePtr>(arg);
    if (!p)
    {
      throw nb::value_error("reference frame object is None");
    }
    return p;
  }

  const std::string n = NormName(std::get<std::string>(arg));

  if ((n == "identity") || (n == "id"))
  {
    return MLMO::MakeIdRefFrame();
  }

  throw nb::value_error(("Invalid reference frame string: \"" + std::get<std::string>(arg) +
                         "\" (valid: \"identity\"; otherwise use a reference frame object)").c_str());
}

////////////////////////////////////////////////////////////////////////////////
// Running the multi-level registration

void CheckIdxs(const IndexList& inds, const size_type n, const std::string& what)
{
  for (const auto& i : inds)
  {
    if (i >= n)
    {
      throw nb::index_error((what + ": index " + std::to_string(i) +
                             " is out of range (size: " + std::to_string(n) + ")").c_str());
    }
  }
}

void ValidateMultiLevel(const PyMultiLevelMultiObjRegi& r)
{
  const size_type nv = r.vols.size();
  const size_type np = r.fixed_proj_data.size();
  const size_type nr = r.ref_frames.size();

  if (!nv)
  {
    throw nb::value_error("no volumes have been set (vols)");
  }

  for (size_type i = 0; i < nv; ++i)
  {
    if (!r.vols[i])
    {
      throw nb::value_error(("vols[" + std::to_string(i) + "] is None").c_str());
    }
  }

  if (!np)
  {
    throw nb::value_error("no fixed projections have been set (fixed_proj_data)");
  }

  for (size_type i = 0; i < np; ++i)
  {
    if (!r.fixed_proj_data[i].img)
    {
      throw nb::value_error(("fixed_proj_data[" + std::to_string(i) + "] has no image").c_str());
    }
  }

  if (r.init_cam_to_vols.size() != nv)
  {
    throw nb::value_error(("init_cam_to_vols must have one pose per volume (expected " +
                           std::to_string(nv) + ", got " +
                           std::to_string(r.init_cam_to_vols.size()) + ")").c_str());
  }

  if (!r.masks_2d.empty() && (r.masks_2d.size() != np))
  {
    throw nb::value_error("masks_2d must be empty or have one entry (possibly None) per fixed projection");
  }

  if (!r.vol_names.empty() && (r.vol_names.size() != nv))
  {
    throw nb::value_error("vol_names must be empty or have one name per volume");
  }

  for (size_type i = 0; i < nr; ++i)
  {
    if (!r.ref_frames[i])
    {
      throw nb::value_error(("ref_frames[" + std::to_string(i) + "] is None").c_str());
    }
  }

  for (size_type lvl_idx = 0; lvl_idx < r.py_levels.size(); ++lvl_idx)
  {
    const std::string lvl_str = "levels[" + std::to_string(lvl_idx) + "]";

    const auto& lvl_ptr = r.py_levels[lvl_idx];
    if (!lvl_ptr)
    {
      throw nb::value_error((lvl_str + " is None").c_str());
    }

    const auto& lvl = *lvl_ptr;

    if (!(lvl.ds_factor > 0))
    {
      throw nb::value_error((lvl_str + ".ds_factor must be positive").c_str());
    }

    if (!lvl.ray_caster)
    {
      throw nb::value_error((lvl_str + ".ray_caster has not been set").c_str());
    }

    if (lvl.fixed_imgs_to_use.empty())
    {
      throw nb::value_error((lvl_str + ".fixed_imgs_to_use is empty").c_str());
    }

    CheckIdxs(lvl.fixed_imgs_to_use, np, lvl_str + ".fixed_imgs_to_use");

    if (lvl.sim_metrics.size() != lvl.fixed_imgs_to_use.size())
    {
      throw nb::value_error((lvl_str + ".sim_metrics must have one similarity metric "
                             "per entry in fixed_imgs_to_use").c_str());
    }

    for (const auto& sm : lvl.sim_metrics)
    {
      if (!sm)
      {
        throw nb::value_error((lvl_str + ".sim_metrics has a None entry").c_str());
      }
    }

    for (size_type regi_idx = 0; regi_idx < lvl.regis.size(); ++regi_idx)
    {
      const std::string regi_str = lvl_str + ".regis[" + std::to_string(regi_idx) + "]";

      const auto& regi_ptr = lvl.regis[regi_idx];
      if (!regi_ptr)
      {
        throw nb::value_error((regi_str + " is None").c_str());
      }

      const auto& regi = *regi_ptr;

      if (!regi.regi)
      {
        throw nb::value_error((regi_str + ".regi (registration/optimizer object) has not been set").c_str());
      }

      if (regi.mov_vols.empty())
      {
        throw nb::value_error((regi_str + ".mov_vols is empty").c_str());
      }

      CheckIdxs(regi.mov_vols, nv, regi_str + ".mov_vols");
      CheckIdxs(regi.static_vols, nv, regi_str + ".static_vols");

      if (regi.init_mov_vol_poses.size() != regi.mov_vols.size())
      {
        throw nb::value_error((regi_str + ".init_mov_vol_poses must have one entry per moving volume").c_str());
      }

      if (regi.static_vol_poses.size() != regi.static_vols.size())
      {
        throw nb::value_error((regi_str + ".static_vol_poses must have one entry per static volume").c_str());
      }

      if (!regi.ref_frames.empty())
      {
        if (regi.ref_frames.size() != regi.mov_vols.size())
        {
          throw nb::value_error((regi_str + ".ref_frames must be empty or have one entry per moving volume").c_str());
        }

        for (const auto& rf : regi.ref_frames)
        {
          if ((rf != MLMO::Level::SingleRegi::kNO_REF_FRAME_USED) && (rf >= nr))
          {
            throw nb::index_error((regi_str + ".ref_frames: index " + std::to_string(rf) +
                                   " is out of range (number of ref_frames: " +
                                   std::to_string(nr) + ")").c_str());
          }
        }
      }

      for (const auto& p : regi.init_mov_vol_poses)
      {
        if (!p)
        {
          throw nb::value_error((regi_str + ".init_mov_vol_poses has a None entry").c_str());
        }
      }

      for (const auto& p : regi.static_vol_poses)
      {
        if (!p)
        {
          throw nb::value_error((regi_str + ".static_vol_poses has a None entry").c_str());
        }
      }
    }
  }
}

void RunMultiLevel(PyMultiLevelMultiObjRegi& r)
{
  ValidateMultiLevel(r);

  // When resources are deallocated as the registration progresses, the objects
  // are moved out of the Python facing levels, so that the Python wrappers do
  // not keep them (and their potentially large GPU allocations) alive.
  const bool move_objs = r.dealloc_resources;

  const size_type num_levels = r.py_levels.size();

  r.levels.clear();
  r.levels.resize(num_levels);

  for (size_type lvl_idx = 0; lvl_idx < num_levels; ++lvl_idx)
  {
    PyLevel& src_lvl = *r.py_levels[lvl_idx];
    auto&    dst_lvl = r.levels[lvl_idx];

    dst_lvl.ds_factor         = src_lvl.ds_factor;
    dst_lvl.fixed_imgs_to_use = src_lvl.fixed_imgs_to_use;

    if (move_objs)
    {
      dst_lvl.ray_caster  = std::move(src_lvl.ray_caster);
      dst_lvl.sim_metrics = std::move(src_lvl.sim_metrics);
      src_lvl.ray_caster  = nullptr;
      src_lvl.sim_metrics.clear();
    }
    else
    {
      dst_lvl.ray_caster  = src_lvl.ray_caster;
      dst_lvl.sim_metrics = src_lvl.sim_metrics;
    }

    const size_type num_regis = src_lvl.regis.size();

    dst_lvl.regis.resize(num_regis);

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      PySingleRegi& src_regi = *src_lvl.regis[regi_idx];
      auto&         dst_regi = dst_lvl.regis[regi_idx];

      dst_regi.mov_vols           = src_regi.mov_vols;
      dst_regi.static_vols        = src_regi.static_vols;
      dst_regi.ref_frames         = src_regi.ref_frames;
      dst_regi.init_mov_vol_poses = src_regi.init_mov_vol_poses;
      dst_regi.static_vol_poses   = src_regi.static_vol_poses;

      if (move_objs)
      {
        dst_regi.regi = std::move(src_regi.regi);
        src_regi.regi = nullptr;
      }
      else
      {
        dst_regi.regi = src_regi.regi;
      }
    }
  }

  try
  {
    nb::gil_scoped_release no_gil;

    r.run();
  }
  catch (...)
  {
    r.levels.clear();
    throw;
  }

  // drop the C++ side references to the per-level objects, the Python wrappers
  // keep them alive when needed.
  r.levels.clear();
}

}  // un-named

void xreg::py::BindRegi(nb::module_& m)
{
  //////////////////////////////////////////////////////////////////////////////
  // Intensity-Based 2D/3D Registration (Optimizers)

  using Regi = Intensity2D3DRegi;

  nb::class_<Regi>(m, "Intensity2D3DRegi",
      "Intensity-based 2D/3D registration (single optimization run). Use one of the "
      "derived optimizer classes. The ray caster, similarity metrics, volumes, initial "
      "guesses, and reference frames are set automatically by MultiLevelMultiObjRegi.")
    .def("set_opt_vars",
        [](Regi& r, const OptVarsArg& v) { r.set_opt_vars(MakeOptVars(v)); },
        "opt_vars"_a,
        "Sets the parameterization: an SE3OptVars object or a string (e.g. \"se3\", \"so3\").")
    .def("set_opt_x_tol", &Regi::set_opt_x_tol, "tol"_a)
    .def("set_opt_obj_fn_tol", &Regi::set_opt_obj_fn_tol, "tol"_a)
    .def("set_penalty_fn", &Regi::set_penalty_fn, "penalty_fn"_a.none())
    .def_prop_ro("penalty_fn", &Regi::penalty_fn)
    .def("set_img_sim_penalty_coefs",
        nb::overload_cast<const Regi::Scalar,const Regi::Scalar>(&Regi::set_img_sim_penalty_coefs),
        "img_sim_coef"_a, "penalty_fn_coef"_a,
        "Weights of the image similarity and penalty terms in the objective function. "
        "NOTE: call this after setting any population size.")
    .def("set_img_sim_penalty_coefs",
        nb::overload_cast<const Regi::ScalarList&,const Regi::ScalarList&>(&Regi::set_img_sim_penalty_coefs),
        "img_sim_coefs"_a, "penalty_fn_coefs"_a)
    .def_prop_rw("include_penalty_in_obj_fn", &Regi::include_penalty_in_obj_fn,
                 &Regi::set_include_penalty_in_obj_fn)
    .def_prop_ro("max_num_iters", &Regi::max_num_iters)
    .def("set_max_num_iters", &Regi::set_max_num_iters, "max_iters"_a)
    .def("max_num_projs_per_view_per_iter", &Regi::max_num_projs_per_view_per_iter)
    .def("regi_xform", &Regi::regi_xform, "vol_idx"_a = 0,
         "The registration result (camera to volume pose) of a moving volume, available after running.")
    .def("set_verbose", [](Regi& r, const bool v) { SetVerbose(r, v); }, "verbose"_a = true)
    .def("set_debug_output_dir_path", &Regi::set_debug_output_dir_path, "path"_a)
    .def("set_debug_write_remapped_drrs", &Regi::set_debug_write_remapped_drrs, "write"_a)
    .def("set_debug_write_raw_drrs", &Regi::set_debug_write_raw_drrs, "write"_a)
    .def("set_debug_write_fixed_img_edge_overlays",
         &Regi::set_debug_write_fixed_img_edge_overlays, "write"_a)
    .def("set_debug_write_combined_sim_scores_to_stream",
         &Regi::set_debug_write_combined_sim_scores_to_stream, "write"_a)
    .def("set_debug_write_opt_vars_to_stream", &Regi::set_debug_write_opt_vars_to_stream, "write"_a);

  nb::class_<Intensity2D3DRegiCMAES, Regi>(m, "Intensity2D3DRegiCMAES",
      "CMA-ES optimization.")
    .def(nb::init<>())
    .def("set_pop_size", &Intensity2D3DRegiCMAES::set_pop_size, "pop_size"_a)
    .def("set_bounds", &Intensity2D3DRegiCMAES::set_bounds, "bounds"_a)
    .def("remove_bounds", &Intensity2D3DRegiCMAES::remove_bounds)
    .def("set_sigma", &Intensity2D3DRegiCMAES::set_sigma, "sigma"_a,
         "Initial standard deviations of each optimization parameter.")
    .def("reset_sigma", &Intensity2D3DRegiCMAES::reset_sigma);

  using NLOptRegi = Intensity2D3DRegiNLOptInterface;

  nb::class_<NLOptRegi, Regi>(m, "Intensity2D3DRegiNLOptInterface",
      "Base class for optimizers implemented by NLopt.")
    .def("set_bounds", &NLOptRegi::set_bounds, "bounds"_a,
         "Box constraints: +/- bounds about the initial guess for each parameter.")
    .def("remove_bounds", &NLOptRegi::remove_bounds)
    .def("set_steps", &NLOptRegi::set_steps, "steps"_a)
    .def("remove_steps", &NLOptRegi::remove_steps)
    .def("set_stochastic_pop_size", &NLOptRegi::set_stochastic_pop_size, "pop_size"_a)
    .def("reset_stochastic_pop_size", &NLOptRegi::reset_stochastic_pop_size);

  nb::class_<Intensity2D3DRegiBOBYQA, NLOptRegi>(m, "Intensity2D3DRegiBOBYQA", "BOBYQA (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiNelderMead, NLOptRegi>(m, "Intensity2D3DRegiNelderMead", "Nelder-Mead (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiSbplx, NLOptRegi>(m, "Intensity2D3DRegiSbplx", "Subplex (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiNEWUOA, NLOptRegi>(m, "Intensity2D3DRegiNEWUOA", "NEWUOA (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiPRAXIS, NLOptRegi>(m, "Intensity2D3DRegiPRAXIS", "PRAXIS (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiCRS, NLOptRegi>(m, "Intensity2D3DRegiCRS", "Controlled Random Search (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiDIRECT, NLOptRegi>(m, "Intensity2D3DRegiDIRECT", "DIRECT (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiDIRECTL, NLOptRegi>(m, "Intensity2D3DRegiDIRECTL", "DIRECT-L (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiDIRECTRandL, NLOptRegi>(m, "Intensity2D3DRegiDIRECTRandL",
      "Randomized DIRECT-L (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiDESCH, NLOptRegi>(m, "Intensity2D3DRegiDESCH", "ESCH evolutionary (NLopt).")
    .def(nb::init<>());
  nb::class_<Intensity2D3DRegiDISRES, NLOptRegi>(m, "Intensity2D3DRegiDISRES", "ISRES evolutionary (NLopt).")
    .def(nb::init<>());

  nb::class_<Intensity2D3DRegiDiffEvo, Regi>(m, "Intensity2D3DRegiDiffEvo", "Differential evolution.")
    .def(nb::init<>())
    .def("set_evo_rate", &Intensity2D3DRegiDiffEvo::set_evo_rate, "F"_a)
    .def("set_cross_over_prob", &Intensity2D3DRegiDiffEvo::set_cross_over_prob, "cr"_a)
    .def("set_pop_size", &Intensity2D3DRegiDiffEvo::set_pop_size, "pop_size"_a)
    .def("set_max_num_iters", &Intensity2D3DRegiDiffEvo::set_max_num_iters, "max_iters"_a)
    .def("set_box_contraint", &Intensity2D3DRegiDiffEvo::set_box_contraint, "bounds"_a)
    .def("set_box_constraint", &Intensity2D3DRegiDiffEvo::set_box_contraint, "bounds"_a,
         "Box constraints: +/- bounds about the initial guess for each parameter.")
    .def("set_print_status_inc", &Intensity2D3DRegiDiffEvo::set_print_status_inc, "inc"_a);

  nb::class_<Intensity2D3DRegiPSO, Regi>(m, "Intensity2D3DRegiPSO", "Particle swarm optimization.")
    .def(nb::init<>())
    .def("set_num_particles", &Intensity2D3DRegiPSO::set_num_particles, "num_particles"_a)
    .def("set_max_num_iters", &Intensity2D3DRegiPSO::set_max_num_iters, "max_iters"_a)
    .def("set_box_contraint", &Intensity2D3DRegiPSO::set_box_contraint, "bounds"_a)
    .def("set_box_constraint", &Intensity2D3DRegiPSO::set_box_contraint, "bounds"_a,
         "Box constraints: +/- bounds about the initial guess for each parameter.")
    .def("set_print_status_inc", &Intensity2D3DRegiPSO::set_print_status_inc, "inc"_a);

  nb::class_<Intensity2D3DRegiHillClimb, Regi>(m, "Intensity2D3DRegiHillClimb", "Hill climbing.")
    .def(nb::init<>())
    .def("set_num_step_levels", &Intensity2D3DRegiHillClimb::set_num_step_levels, "num_step_levels"_a)
    .def("set_init_steps", &Intensity2D3DRegiHillClimb::set_init_steps, "init_steps"_a);

  nb::class_<Intensity2D3DRegiExhaustive, Regi>(m, "Intensity2D3DRegiExhaustive",
      "Exhaustive (grid) search.")
    .def(nb::init<>())
    .def("set_cam_wrt_vols",
        [](Intensity2D3DRegiExhaustive& r,
           const std::vector<std::tuple<CoordScalar,CoordScalar,CoordScalar>>& ranges,
           const size_type batch_size)
        {
          ConstSpacedMeshGrid::Range1DList r1d;
          r1d.reserve(ranges.size());

          for (const auto& t : ranges)
          {
            r1d.push_back({ std::get<0>(t), std::get<1>(t), std::get<2>(t) });
          }

          r.set_cam_wrt_vols(ConstSpacedMeshGrid(r1d), batch_size);
        },
        "ranges"_a, "batch_size"_a = 0,
        "Search over a grid in the optimization parameter space. ranges: one (start, stop, inc) "
        "tuple per optimization parameter. batch_size: max. number of poses evaluated "
        "simultaneously (0 -> all at once).")
    .def("set_cam_wrt_vols",
        nb::overload_cast<const Regi::ListOfFrameTransformLists&,const size_type>(
                                &Intensity2D3DRegiExhaustive::set_cam_wrt_vols),
        "xforms"_a, "batch_size"_a = 0,
        "Search over an explicit list of poses: xforms[i][j] is the jth candidate of the ith object.")
    .def("set_save_all_sim_vals", &Intensity2D3DRegiExhaustive::set_save_all_sim_vals, "save"_a)
    .def_prop_ro("all_sim_vals", &Intensity2D3DRegiExhaustive::all_sim_vals)
    .def_prop_ro("tot_num_xforms", &Intensity2D3DRegiExhaustive::tot_num_xforms)
    .def("set_print_status_inc", &Intensity2D3DRegiExhaustive::set_print_status_inc, "inc"_a);

  //////////////////////////////////////////////////////////////////////////////
  // Multi-Level, Multi-Object Registration

  nb::class_<PyMultiLevelMultiObjRegi> ml_cls(m, "MultiLevelMultiObjRegi",
      "Multiple resolution level, multiple object, multiple view intensity-based 2D/3D registration. "
      "Mirrors xreg::MultiLevelMultiObjRegi: set vols, fixed_proj_data, init_cam_to_vols, "
      "ref_frames, and levels, then call run(). Results are in cur_cam_to_vols.");

  // Reference frames

  using RefFrame = MLMO::RefFrame;

  nb::class_<RefFrame>(ml_cls, "RefFrame",
      "Reference frame in which a registration optimizes the pose of an object.")
    .def_prop_ro("standard", &RefFrame::standard);

  nb::class_<MLMO::StaticRefFrame, RefFrame>(ml_cls, "StaticRefFrame",
      "A fixed reference frame. When maps_vol_to_ref is true, ref_frame maps volume "
      "coordinates to the reference frame, otherwise it maps camera coordinates to the "
      "reference frame.")
    .def("__init__",
        [](MLMO::StaticRefFrame* rf, const FrameTransform& ref_frame, const bool maps_vol_to_ref)
        {
          new (rf) MLMO::StaticRefFrame();
          rf->info.ref_frame       = ref_frame;
          rf->info.maps_vol_to_ref = maps_vol_to_ref;
        },
        "ref_frame"_a = FrameTransform::Identity(), "maps_vol_to_ref"_a = true)
    .def_prop_rw("ref_frame",
        [](const MLMO::StaticRefFrame& rf) { return rf.info.ref_frame; },
        [](MLMO::StaticRefFrame& rf, const FrameTransform& x) { rf.info.ref_frame = x; })
    .def_prop_rw("maps_vol_to_ref",
        [](const MLMO::StaticRefFrame& rf) { return rf.info.maps_vol_to_ref; },
        [](MLMO::StaticRefFrame& rf, const bool b) { rf.info.maps_vol_to_ref = b; });

  nb::class_<MLMO::CamAlignRefFrameWithCurPose, RefFrame>(ml_cls, "CamAlignRefFrameWithCurPose",
      "Reference frame aligned with the camera axes and centered at a point (e.g. the volume center) "
      "using the current pose estimate of a volume.")
    .def("__init__",
        [](MLMO::CamAlignRefFrameWithCurPose* rf, const size_type vol_idx,
           const FrameTransform& cam_extrins, const Pt3& center_of_rot_wrt_vol)
        {
          new (rf) MLMO::CamAlignRefFrameWithCurPose();
          rf->vol_idx               = vol_idx;
          rf->cam_extrins           = cam_extrins;
          rf->center_of_rot_wrt_vol = center_of_rot_wrt_vol;
        },
        "vol_idx"_a, "cam_extrins"_a, "center_of_rot_wrt_vol"_a)
    .def_rw("vol_idx", &MLMO::CamAlignRefFrameWithCurPose::vol_idx)
    .def_rw("cam_extrins", &MLMO::CamAlignRefFrameWithCurPose::cam_extrins)
    .def_rw("center_of_rot_wrt_vol", &MLMO::CamAlignRefFrameWithCurPose::center_of_rot_wrt_vol);

  nb::class_<MLMO::DynVolRefFrame, RefFrame>(ml_cls, "DynVolRefFrame",
      "Dynamic reference frame attached to another volume being optimized simultaneously.")
    .def("__init__",
        [](MLMO::DynVolRefFrame* rf, const size_type dyn_vol_idx, const FrameTransform& inter_to_vol)
        {
          new (rf) MLMO::DynVolRefFrame();
          rf->dyn_vol_idx  = dyn_vol_idx;
          rf->inter_to_vol = inter_to_vol;
        },
        "dyn_vol_idx"_a, "inter_to_vol"_a = FrameTransform::Identity())
    .def_rw("dyn_vol_idx", &MLMO::DynVolRefFrame::dyn_vol_idx)
    .def_rw("inter_to_vol", &MLMO::DynVolRefFrame::inter_to_vol);

  // Levels and single registrations within a level

  nb::class_<PyLevel> lvl_cls(ml_cls, "Level",
      "A single resolution level: a downsampling factor, ray caster, one similarity metric per "
      "fixed image used, and a sequence of registrations.");

  nb::class_<PySingleRegi> single_regi_cls(lvl_cls, "SingleRegi",
      "A single registration (optimization) run within a level.");

  nb::class_<InitPose>(single_regi_cls, "InitPose", "Strategy for computing an initial/static pose.");

  nb::class_<InitPoseId, InitPose>(single_regi_cls, "InitPoseId", "Identity pose.")
    .def(nb::init<>());

  nb::class_<InitPosePrevPoseEst, InitPose>(single_regi_cls, "InitPosePrevPoseEst",
      "The current pose estimate of a volume.")
    .def("__init__",
        [](InitPosePrevPoseEst* p, const size_type vol_idx)
        {
          new (p) InitPosePrevPoseEst();
          p->vol_idx = vol_idx;
        },
        "vol_idx"_a)
    .def_rw("vol_idx", &InitPosePrevPoseEst::vol_idx);

  nb::class_<InitPosePrevPoseEstTransOnly, InitPose>(single_regi_cls, "InitPosePrevPoseEstTransOnly",
      "Translation component of the current pose estimate of a volume (identity rotation).")
    .def("__init__",
        [](InitPosePrevPoseEstTransOnly* p, const size_type vol_idx)
        {
          new (p) InitPosePrevPoseEstTransOnly();
          p->vol_idx = vol_idx;
        },
        "vol_idx"_a)
    .def_rw("vol_idx", &InitPosePrevPoseEstTransOnly::vol_idx);

  nb::class_<InitPosePrevPoseEstRotOnly, InitPose>(single_regi_cls, "InitPosePrevPoseEstRotOnly",
      "Rotation component of the current pose estimate of a volume (zero translation).")
    .def("__init__",
        [](InitPosePrevPoseEstRotOnly* p, const size_type vol_idx)
        {
          new (p) InitPosePrevPoseEstRotOnly();
          p->vol_idx = vol_idx;
        },
        "vol_idx"_a)
    .def_rw("vol_idx", &InitPosePrevPoseEstRotOnly::vol_idx);

  single_regi_cls.attr("NO_REF_FRAME_USED") = size_type(MLMO::Level::SingleRegi::kNO_REF_FRAME_USED);

  const char* init_pose_doc =
    "List of InitPose objects or strings: \"identity\", \"prev:<vol idx>\", "
    "\"prev_trans:<vol idx>\", \"prev_rot:<vol idx>\".";

  single_regi_cls
    .def("__init__",
        [](PySingleRegi* r, const IndexList& mov_vols, const IndexList& static_vols,
           const IndexList& ref_frames, const std::vector<InitPoseArg>& init_mov_vol_poses,
           const std::vector<InitPoseArg>& static_vol_poses,
           const std::shared_ptr<Intensity2D3DRegi>& regi)
        {
          new (r) PySingleRegi();
          r->mov_vols           = mov_vols;
          r->static_vols        = static_vols;
          r->ref_frames         = ref_frames;
          r->init_mov_vol_poses = MakeInitPoses(init_mov_vol_poses);
          r->static_vol_poses   = MakeInitPoses(static_vol_poses);
          r->regi               = regi;
        },
        "mov_vols"_a = IndexList(), "static_vols"_a = IndexList(), "ref_frames"_a = IndexList(),
        "init_mov_vol_poses"_a = std::vector<InitPoseArg>(),
        "static_vol_poses"_a = std::vector<InitPoseArg>(),
        "regi"_a.none() = nb::none())
    .def_rw("mov_vols", &PySingleRegi::mov_vols, "Indices of the volumes whose poses are optimized.")
    .def_rw("static_vols", &PySingleRegi::static_vols,
            "Indices of volumes projected in the background at fixed poses.")
    .def_rw("ref_frames", &PySingleRegi::ref_frames,
            "Index (into MultiLevelMultiObjRegi.ref_frames) of the reference frame used by "
            "each moving volume (or NO_REF_FRAME_USED); may be empty.")
    .def_prop_rw("init_mov_vol_poses",
        [](const PySingleRegi& r) { return r.init_mov_vol_poses; },
        [](PySingleRegi& r, const std::vector<InitPoseArg>& p) { r.init_mov_vol_poses = MakeInitPoses(p); },
        init_pose_doc)
    .def_prop_rw("static_vol_poses",
        [](const PySingleRegi& r) { return r.static_vol_poses; },
        [](PySingleRegi& r, const std::vector<InitPoseArg>& p) { r.static_vol_poses = MakeInitPoses(p); },
        init_pose_doc)
    .def_rw("regi", &PySingleRegi::regi, nb::for_setter(nb::arg("regi").none()),
            "The registration/optimizer object (Intensity2D3DRegi subclass).");

  nb::bind_vector<PySingleRegiList>(lvl_cls, "SingleRegiList");

  lvl_cls
    .def("__init__",
        [](PyLevel* l, const double ds_factor, const std::shared_ptr<RayCaster>& ray_caster,
           const Intensity2D3DRegi::SimMetricList& sim_metrics, const IndexList& fixed_imgs_to_use,
           const PySingleRegiList& regis)
        {
          new (l) PyLevel();
          l->ds_factor         = ds_factor;
          l->ray_caster        = ray_caster;
          l->sim_metrics       = sim_metrics;
          l->fixed_imgs_to_use = fixed_imgs_to_use;
          l->regis             = regis;
        },
        "ds_factor"_a = 1.0, "ray_caster"_a.none() = nb::none(),
        "sim_metrics"_a = Intensity2D3DRegi::SimMetricList(),
        "fixed_imgs_to_use"_a = IndexList(), "regis"_a = PySingleRegiList())
    .def_rw("ds_factor", &PyLevel::ds_factor, "Downsampling factor applied to the fixed images.")
    .def_rw("ray_caster", &PyLevel::ray_caster, nb::for_setter(nb::arg("ray_caster").none()))
    .def_rw("sim_metrics", &PyLevel::sim_metrics,
            "One similarity metric per entry of fixed_imgs_to_use. "
            "NOTE: assign an entire list - modifying the returned list in place has no effect.")
    .def_rw("fixed_imgs_to_use", &PyLevel::fixed_imgs_to_use,
            "Indices into MultiLevelMultiObjRegi.fixed_proj_data used at this level.")
    .def_rw("regis", &PyLevel::regis, "Registrations run (in order) at this level.");

  nb::bind_vector<PyLevelList>(ml_cls, "LevelList");

  // Debug information

  nb::class_<DebugML> dbg_cls(m, "DebugRegiResultsMultiLevel",
      "Debug information collected by MultiLevelMultiObjRegi (when enabled with "
      "set_save_debug_info(True)). Written to disk with write() and may be replayed "
      "using the xreg-regi2d3d-replay tool.");

  nb::class_<DebugML::VolPathInfo>(dbg_cls, "VolPathInfo",
      "Paths to the volume (and optional label map) used, avoids storing volumes in the debug file.")
    .def("__init__",
        [](DebugML::VolPathInfo* v, const std::string& vol_path, const std::string& label_vol_path,
           const DebugML::IndexList& labels_used)
        {
          new (v) DebugML::VolPathInfo();
          v->vol_path       = vol_path;
          v->label_vol_path = label_vol_path;
          v->labels_used    = labels_used;
        },
        "vol_path"_a, "label_vol_path"_a = "", "labels_used"_a = DebugML::IndexList())
    .def_rw("vol_path", &DebugML::VolPathInfo::vol_path)
    .def_rw("label_vol_path", &DebugML::VolPathInfo::label_vol_path)
    .def_rw("labels_used", &DebugML::VolPathInfo::labels_used);

  nb::class_<DebugML::ProjDataPathInfo>(dbg_cls, "ProjDataPathInfo",
      "Path to the projection data file used, avoids storing the projections in the debug file.")
    .def("__init__",
        [](DebugML::ProjDataPathInfo* p, const std::string& path, const DebugML::IndexList& projs_used)
        {
          new (p) DebugML::ProjDataPathInfo();
          p->path       = path;
          p->projs_used = projs_used;
        },
        "path"_a, "projs_used"_a = DebugML::IndexList())
    .def_rw("path", &DebugML::ProjDataPathInfo::path)
    .def_rw("projs_used", &DebugML::ProjDataPathInfo::projs_used);

  dbg_cls
    .def_static("read", &ReadMultiLevel2D3DRegiDebugFromDisk, "path"_a,
                "Reads debug information from an HDF5 file.")
    .def("total_num_projs_per_view", &DebugML::total_num_projs_per_view,
         "The total number of projections, per view, of all registrations: the initial pose, "
         "each iteration, and the final pose of each registration.")
    .def_rw("regi_names", &DebugML::regi_names,
            "regi_names[i][j] is the name of the jth registration at the ith level.")
    .def_prop_rw("vols",
        [](const DebugML& d) { return d.vols; },
        [](DebugML& d, const std::vector<std::variant<RayCaster::VolPtr,DebugML::VolPathInfo>>& vols)
        {
          for (const auto& v : vols)
          {
            if (std::holds_alternative<RayCaster::VolPtr>(v))
            {
              CheckedImg(std::get<RayCaster::VolPtr>(v), "volume");
            }
          }

          d.vols = vols;
        },
        "Volumes used: Image3DF32 objects (serialized into the debug file, typically in HU) "
        "or VolPathInfo objects (paths on disk).")
    .def_rw("fixed_projs", &DebugML::fixed_projs,
            "Fixed projections (list of ProjData, prior to pre-processing) or a ProjDataPathInfo.")
    .def_rw("proj_pre_proc_info", &DebugML::proj_pre_proc_info)
    .def_rw("pre_proc_time_secs", &DebugML::pre_proc_time_secs)
    .def_rw("tot_time_secs", &DebugML::tot_time_secs)
    .def_ro("multi_level_run_time_secs", &DebugML::multi_level_run_time_secs)
    .def_ro("multi_res_levels", &DebugML::multi_res_levels)
    .def("write",
        [](const DebugML& d, const std::string& path)
        {
          WriteMultiLevel2D3DRegiDebugToDisk(d, path);
        },
        "path"_a, "Writes the debug information to an HDF5 file.");

  // MultiLevelMultiObjRegi members

  ml_cls
    .def(nb::init<>())
    .def_rw("vols", &PyMultiLevelMultiObjRegi::vols,
            "Volumes (attenuation, e.g. see hu_to_lin_att()) of each object. "
            "NOTE: assign an entire list.")
    .def_rw("vol_names", &PyMultiLevelMultiObjRegi::vol_names)
    .def_rw("fixed_proj_data", &PyMultiLevelMultiObjRegi::fixed_proj_data,
            "Fixed (pre-processed) projections. NOTE: assign an entire list.")
    .def_rw("masks_2d", &PyMultiLevelMultiObjRegi::masks_2d,
            "Optional masks (Image2DU8): empty, or one (possibly None) per fixed projection. "
            "Pixels with value zero are not used in the similarity metrics.")
    .def_rw("init_cam_to_vols", &PyMultiLevelMultiObjRegi::init_cam_to_vols,
            "Initial pose (camera to volume, 4x4) of each volume.")
    .def_ro("cur_cam_to_vols", &PyMultiLevelMultiObjRegi::cur_cam_to_vols,
            "Current/final pose estimates (camera to volume, 4x4) of each volume.")
    .def_prop_rw("ref_frames",
        [](const PyMultiLevelMultiObjRegi& r) { return r.ref_frames; },
        [](PyMultiLevelMultiObjRegi& r, const std::vector<RefFrameArg>& rfs)
        {
          r.ref_frames.clear();
          for (const auto& rf : rfs)
          {
            r.ref_frames.push_back(MakeRefFrame(rf));
          }
        },
        "Reference frames (RefFrame objects or \"identity\") referenced by index from each SingleRegi.")
    .def_rw("levels", &PyMultiLevelMultiObjRegi::py_levels,
            "The resolution levels (a list of Level objects), run in order.")
    .def_rw("dealloc_resources", &PyMultiLevelMultiObjRegi::dealloc_resources,
            "(Advanced) Release ray casters, similarity metrics, and registration objects as the "
            "registration proceeds (default True). When True, the corresponding fields of each "
            "Level/SingleRegi are set to None by run().")
    .def_rw("ray_caster_needs_resources_alloc",
            &PyMultiLevelMultiObjRegi::ray_caster_needs_resources_alloc)
    .def_rw("sim_metrics_need_resources_alloc",
            &PyMultiLevelMultiObjRegi::sim_metrics_need_resources_alloc)
    .def_ro("debug_info", &PyMultiLevelMultiObjRegi::debug_info,
            "Debug information, available when set_save_debug_info(True) was called (otherwise None).")
    .def("set_save_debug_info", &PyMultiLevelMultiObjRegi::set_save_debug_info, "save"_a,
         "Enables/disables the collection of debug information (allocates debug_info).")
    .def("write_debug",
        [](const PyMultiLevelMultiObjRegi& r, const std::string& path)
        {
          if (!r.debug_info)
          {
            throw nb::value_error("debug info was not saved, call set_save_debug_info(True) before run()");
          }
          WriteMultiLevel2D3DRegiDebugToDisk(*r.debug_info, path);
        },
        "path"_a, "Writes the debug information to an HDF5 file.")
    .def("set_verbose", [](PyMultiLevelMultiObjRegi& r, const bool v) { SetVerbose(r, v); },
         "verbose"_a = true, "Print verbose status information to stdout.")
    .def("run", &RunMultiLevel, "Runs the registration (the GIL is released while running).")
    .def_static("make_static_ref_frame", &MLMO::MakeStaticRefFrame, "ref_frame"_a, "maps_vol_to_ref"_a)
    .def_static("make_id_ref_frame", &MLMO::MakeIdRefFrame);

  m.def("make_init_pose", &MakeInitPose, "pose"_a,
        "Creates an InitPose from a string: \"identity\", \"prev:<vol idx>\", "
        "\"prev_trans:<vol idx>\", or \"prev_rot:<vol idx>\".");
}
