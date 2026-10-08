
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregRayCastOccContourCPU.h"
#include "xregRayCastOccContourMetal.h"
#include "xregRayCastOccContourOCL.h"
#include "xregRayCastSurRenderCPU.h"
#include "xregRayCastSurRenderMetal.h"
#include "xregRayCastSurRenderOCL.h"
#include "xregRayCastTestUtils.h"

namespace
{

using namespace xreg;
using namespace xreg::test;

constexpr size_type kNUM_PROJS = 6;

/// \brief Surfaces are located where the blobs of the test volume reach this
///        value, which gives closed surfaces that some rays miss.
constexpr float kSUR_THRESH = 0.5f;

/// \brief Backtracking refines the surface locations of every ray caster to
///        nearly the same location, so comparisons are much closer than without.
constexpr size_type kNUM_BACKTRACKING_STEPS = 8;

using ConfigFn = std::function<void(RayCaster&)>;

/// \brief Set the scene and the collision parameters of a ray caster, with an
///        optional configuration of other parameters.
void SetupSurface(RayCaster& rc, const Scene& s, const size_type num_backtracking_steps,
                  const ConfigFn& config = {})
{
  SetupRayCaster(rc, s, 1,
                 [num_backtracking_steps, &config] (RayCaster& r)
                 {
                   auto& coll = dynamic_cast<RayCasterCollisionParamInterface&>(r);
                   coll.set_render_thresh(kSUR_THRESH);
                   coll.set_num_backtracking_steps(num_backtracking_steps);

                   if (config)
                   {
                     config(r);
                   }
                 });
}

template <class tRayCasterMetal>
std::unique_ptr<tRayCasterMetal> MakeMetal(const MetalDevice& dev, const bool force_sw)
{
  auto rc = std::make_unique<tRayCasterMetal>(dev);
  rc->set_force_sw_interp(force_sw);
  return rc;
}

/// \brief Computes the projections of a ray caster.
PixelBuf Compute(RayCaster& rc, const Scene& s, const size_type num_backtracking_steps,
                 const ConfigFn& config = {})
{
  SetupSurface(rc, s, num_backtracking_steps, config);
  rc.compute();

  return GetProjs(rc, s.poses.size());
}

/// \brief Both ray casters add to the pixels where a surface is found (surface
///        rendering) or a contour is found (occluding contours), other pixels
///        keep the background value of zero.
bool IsHit(const float v)
{
  return v != 0;
}

struct HitDiffStats
{
  size_type num_pix = 0;

  size_type num_hits = 0;

  /// \brief number of pixels with a hit in only one of the inputs
  size_type num_hit_mismatches = 0;

  /// \brief statistics of the differences for pixels with a hit in both
  float max_abs_diff  = 0;
  float mean_abs_diff = 0;
};

HitDiffStats CompareHits(const PixelBuf& a, const PixelBuf& b)
{
  xregASSERT(a.size() == b.size());

  HitDiffStats stats;
  stats.num_pix = a.size();

  double sum_abs_diff = 0;

  for (size_type i = 0; i < a.size(); ++i)
  {
    const bool hit_a = IsHit(a[i]);
    const bool hit_b = IsHit(b[i]);

    if (hit_a && hit_b)
    {
      const float d = std::abs(a[i] - b[i]);

      stats.max_abs_diff = std::max(stats.max_abs_diff, d);
      sum_abs_diff += d;

      ++stats.num_hits;
    }
    else if (hit_a != hit_b)
    {
      ++stats.num_hit_mismatches;
    }
  }

  stats.mean_abs_diff = stats.num_hits ? static_cast<float>(sum_abs_diff / stats.num_hits) : 0.0f;

  return stats;
}

/// \brief Checks the fraction of pixels with a hit in only one input, and the
///        differences of pixels with a hit in both.
void CheckHitsClose(const std::string& desc, const PixelBuf& a, const PixelBuf& b,
                    const float max_mismatch_frac, const float max_abs_tol, const float mean_abs_tol)
{
  const HitDiffStats stats = CompareHits(a, b);

  const float mismatch_frac = static_cast<float>(stats.num_hit_mismatches) / stats.num_pix;

  std::cout << fmt::format("    {:<44} hits: {:6.2f}%  mismatches: {:6.3f}%  max diff: {:.2e}  "
                           "mean diff: {:.2e}",
                           desc, (100.0 * stats.num_hits) / stats.num_pix, 100 * mismatch_frac,
                           stats.max_abs_diff, stats.mean_abs_diff) << std::endl;

  // the scene should have pixels with and without hits
  xregASSERT(stats.num_hits > 0);
  xregASSERT(stats.num_hits < stats.num_pix);

  for (const float v : a)
  {
    xregASSERT(std::isfinite(v));
  }

  xregASSERT(mismatch_frac <= max_mismatch_frac);
  xregASSERT(stats.max_abs_diff <= max_abs_tol);
  xregASSERT(stats.mean_abs_diff <= mean_abs_tol);
}

/// \brief Checks the store methods of a ray caster that adds its values to the
///        existing pixel values.
///
/// The reference results are computed with the default background value of
/// zero, every pixel is exactly the sum of the existing value and the
/// reference value.
void CheckAdditiveStoreMethods(const std::function<std::unique_ptr<RayCaster>()>& make_rc,
                               const Scene& s, const PixelBuf& ref)
{
  const size_type num_pix_per_proj = s.cams[0].num_det_rows * s.cams[0].num_det_cols;

  // accumulating with the same result doubles it
  {
    auto rc = make_rc();
    SetupSurface(*rc, s, kNUM_BACKTRACKING_STEPS);
    rc->compute();
    rc->use_proj_store_accum_method();
    rc->compute();

    const PixelBuf result = GetProjs(*rc, kNUM_PROJS);

    for (size_type i = 0; i < ref.size(); ++i)
    {
      xregASSERT(result[i] == (ref[i] + ref[i]));
    }
  }

  // background projections, a different background is used for each camera
  {
    const std::vector<float> bg_vals = { 3.0f, 5.0f };

    auto rc = make_rc();
    rc->set_camera_models(s.cams);

    RayCaster::ProjList bg_projs;
    for (size_type cam_idx = 0; cam_idx < s.cams.size(); ++cam_idx)
    {
      auto bg = Proj::New();
      Proj::RegionType region;
      region.SetSize({ s.cams[0].num_det_cols, s.cams[0].num_det_rows });
      bg->SetRegions(region);
      bg->Allocate();
      bg->FillBuffer(bg_vals[cam_idx]);

      bg_projs.push_back(bg);
    }

    rc->set_bg_projs(bg_projs);

    SetupSurface(*rc, s, kNUM_BACKTRACKING_STEPS);
    rc->compute();

    const PixelBuf result = GetProjs(*rc, kNUM_PROJS);

    for (size_type proj_idx = 0; proj_idx < kNUM_PROJS; ++proj_idx)
    {
      const float bg_val = bg_vals[s.cam_for_proj[proj_idx]];

      for (size_type i = 0; i < num_pix_per_proj; ++i)
      {
        const size_type pix_idx = (proj_idx * num_pix_per_proj) + i;

        xregASSERT(result[pix_idx] == (bg_val + ref[pix_idx]));
      }
    }
  }

  std::cout << "    accumulate and background projections: exact" << std::endl;
}

//////////////////////////////////////////////////////////////////////
// Surface rendering

// Tolerances, the shaded intensities are in [0,1] for the default parameters.
//
// Without backtracking, interpolation precision may move the first sample
// greater than or equal to the threshold by one step along a ray, which
// changes the surface normal (up to 0.06 differences measured for a few
// pixels). With backtracking, every implementation refines the location to
// nearly the same position (up to 0.006 measured with hardware interpolation,
// 0.003 with software interpolation). The CPU implementation is only compared
// with backtracking, since it moves the start of each ray slightly inside the
// volume, which changes every sample location.
constexpr float kSUR_MAX_MISMATCH_FRAC  = 1.0e-3f;
constexpr float kSUR_NO_BT_MAX_ABS_TOL  = 0.1f;
constexpr float kSUR_NO_BT_MEAN_ABS_TOL = 1.0e-3f;
constexpr float kSUR_BT_MAX_ABS_TOL     = 2.0e-2f;
constexpr float kSUR_BT_MEAN_ABS_TOL    = 5.0e-4f;

/// \brief Shading parameters that differ from the defaults, to check that they
///        are used.
void SetNonDefaultShading(RayCaster& rc)
{
  auto& sr = dynamic_cast<RayCasterSurRenderParamInterface&>(rc);

  sr.set_ambient_reflection_ratio(0.1f);
  sr.set_diffuse_reflection_ratio(0.5f);
  sr.set_specular_reflection_ratio(0.4f);
  sr.set_alpha_shininess(4.0f);
}

void TestSurRender(const MetalDevice& dev, const bool force_sw, const Scene& s,
                   const std::optional<boost::compute::device>& ocl_ref_dev)
{
  std::cout << "    surface rendering:" << std::endl;

  auto make_metal = [&dev, force_sw] () { return MakeMetal<RayCasterSurRenderMetal>(dev, force_sw); };

  const PixelBuf metal_bt = Compute(*make_metal(), s, kNUM_BACKTRACKING_STEPS);

  if (ocl_ref_dev)
  {
    RayCasterSurRenderOCL ocl_rc(*ocl_ref_dev);
    CheckHitsClose("no backtracking, vs. OpenCL", Compute(*make_metal(), s, 0), Compute(ocl_rc, s, 0),
                   kSUR_MAX_MISMATCH_FRAC, kSUR_NO_BT_MAX_ABS_TOL, kSUR_NO_BT_MEAN_ABS_TOL);

    RayCasterSurRenderOCL ocl_bt_rc(*ocl_ref_dev);
    CheckHitsClose(fmt::format("{} backtracking, vs. OpenCL", kNUM_BACKTRACKING_STEPS), metal_bt,
                   Compute(ocl_bt_rc, s, kNUM_BACKTRACKING_STEPS),
                   kSUR_MAX_MISMATCH_FRAC, kSUR_BT_MAX_ABS_TOL, kSUR_BT_MEAN_ABS_TOL);
  }

  {
    RayCasterSurRenderCPU cpu_rc;
    CheckHitsClose(fmt::format("{} backtracking, vs. CPU", kNUM_BACKTRACKING_STEPS), metal_bt,
                   Compute(cpu_rc, s, kNUM_BACKTRACKING_STEPS),
                   kSUR_MAX_MISMATCH_FRAC, kSUR_BT_MAX_ABS_TOL, kSUR_BT_MEAN_ABS_TOL);
  }

  {
    RayCasterSurRenderCPU cpu_rc;
    CheckHitsClose(fmt::format("{} backtracking, other shading, vs. CPU", kNUM_BACKTRACKING_STEPS),
                   Compute(*make_metal(), s, kNUM_BACKTRACKING_STEPS, &SetNonDefaultShading),
                   Compute(cpu_rc, s, kNUM_BACKTRACKING_STEPS, &SetNonDefaultShading),
                   kSUR_MAX_MISMATCH_FRAC, kSUR_BT_MAX_ABS_TOL, kSUR_BT_MEAN_ABS_TOL);
  }

  // the intensities are within the range of the illumination model
  {
    const RayCasterSurRenderShadingParams shading = RayCasterSurRenderMetal().surface_render_params();

    const float max_intensity = shading.ambient_reflection_ratio + shading.diffuse_reflection_ratio +
                                shading.specular_reflection_ratio;

    for (const float v : metal_bt)
    {
      xregASSERT(!IsHit(v) || ((v >= shading.ambient_reflection_ratio) && (v <= (max_intensity + 1.0e-5f))));
    }

    std::cout << fmt::format("    intensities in [{}, {}]", shading.ambient_reflection_ratio, max_intensity)
              << std::endl;
  }

  CheckAdditiveStoreMethods(make_metal, s, metal_bt);
}

//////////////////////////////////////////////////////////////////////
// Occluding contours

// Contours are pixels where the surface normal is within a threshold of
// perpendicular to the ray, so slight differences in the normal change whether
// pixels near the boundary of a contour are included. Up to 0.04% of the pixels
// (about 6% of the contour pixels) differed between implementations without
// backtracking and 0.01% with backtracking. The CPU implementation is only
// compared with backtracking (see the surface rendering tolerances).
constexpr float kCONTOUR_MAX_MISMATCH_FRAC = 5.0e-4f;

// When continuing after a collision, every sample inside an object is checked
// for a contour, and the CPU implementation moves every sample location (see
// the surface rendering tolerances), so more pixels differ: 0.11% of the pixels
// (0.4% of the contour pixels) were measured.
constexpr float kCONTOUR_CONTINUE_MAX_MISMATCH_FRAC = 5.0e-3f;

void SetContinueAfterCollision(RayCaster& rc)
{
  dynamic_cast<RayCasterOccludingContours&>(rc).set_stop_after_collision(false);
}

void SetLargerContourAngle(RayCaster& rc)
{
  dynamic_cast<RayCasterOccludingContours&>(rc).set_occlusion_angle_thresh_deg(15);
}

void TestOccContours(const MetalDevice& dev, const bool force_sw, const Scene& s,
                     const std::optional<boost::compute::device>& ocl_ref_dev)
{
  std::cout << "    occluding contours:" << std::endl;

  auto make_metal = [&dev, force_sw] () { return MakeMetal<RayCasterOccludingContoursMetal>(dev, force_sw); };

  const PixelBuf metal_no_bt = Compute(*make_metal(), s, 0);
  const PixelBuf metal_bt    = Compute(*make_metal(), s, kNUM_BACKTRACKING_STEPS);

  // the OpenCL ray caster does not support backtracking
  if (ocl_ref_dev)
  {
    RayCasterOccludingContoursOCL ocl_rc(*ocl_ref_dev);
    CheckHitsClose("no backtracking, vs. OpenCL", metal_no_bt, Compute(ocl_rc, s, 0),
                   kCONTOUR_MAX_MISMATCH_FRAC, 0, 0);
  }

  {
    RayCasterOccludingContoursCPU cpu_rc;
    CheckHitsClose(fmt::format("{} backtracking, vs. CPU", kNUM_BACKTRACKING_STEPS), metal_bt,
                   Compute(cpu_rc, s, kNUM_BACKTRACKING_STEPS),
                   kCONTOUR_MAX_MISMATCH_FRAC, 0, 0);
  }

  {
    RayCasterOccludingContoursCPU cpu_rc;
    CheckHitsClose(fmt::format("{} backtracking, 15 deg., vs. CPU", kNUM_BACKTRACKING_STEPS),
                   Compute(*make_metal(), s, kNUM_BACKTRACKING_STEPS, &SetLargerContourAngle),
                   Compute(cpu_rc, s, kNUM_BACKTRACKING_STEPS, &SetLargerContourAngle),
                   kCONTOUR_MAX_MISMATCH_FRAC, 0, 0);
  }

  // Nearly every pixel with a surface has a contour when continuing after a
  // collision, since the gradient inside a blob is perpendicular to the ray at
  // some location along the ray.
  {
    RayCasterOccludingContoursCPU cpu_rc;
    CheckHitsClose(fmt::format("{} backtracking, continue, vs. CPU", kNUM_BACKTRACKING_STEPS),
                   Compute(*make_metal(), s, kNUM_BACKTRACKING_STEPS, &SetContinueAfterCollision),
                   Compute(cpu_rc, s, kNUM_BACKTRACKING_STEPS, &SetContinueAfterCollision),
                   kCONTOUR_CONTINUE_MAX_MISMATCH_FRAC, 0, 0);
  }

  // continuing after a collision finds the same contours as stopping, since
  // the first surface location is checked in the same way, and possibly more
  {
    const PixelBuf metal_continue = Compute(*make_metal(), s, 0, &SetContinueAfterCollision);

    size_type num_contours = 0;
    size_type num_contours_continue = 0;

    for (size_type i = 0; i < metal_no_bt.size(); ++i)
    {
      xregASSERT((metal_no_bt[i] == 0) || (metal_no_bt[i] == 1));
      xregASSERT(!IsHit(metal_no_bt[i]) || IsHit(metal_continue[i]));

      num_contours          += IsHit(metal_no_bt[i]) ? 1 : 0;
      num_contours_continue += IsHit(metal_continue[i]) ? 1 : 0;
    }

    xregASSERT(num_contours_continue > num_contours);

    std::cout << fmt::format("    contours stopping after collision: {}, continuing: {} (a superset)",
                             num_contours, num_contours_continue) << std::endl;
  }

  CheckAdditiveStoreMethods(make_metal, s, metal_bt);
}

//////////////////////////////////////////////////////////////////////

template <class tRayCasterMetal, class tRayCasterOCL>
void PrintTimings(const std::string& desc, const MetalDevice& dev, const Scene& s, const bool have_ocl)
{
  for (const bool force_sw : { false, true })
  {
    auto metal_rc = MakeMetal<tRayCasterMetal>(dev, force_sw);

    if (!force_sw && !metal_rc->use_hw_interp())
    {
      continue;
    }

    SetupSurface(*metal_rc, s, 0);

    std::cout << fmt::format("      {:<20} Metal ({} interp.): {:8.2f} ms", desc,
                             metal_rc->use_hw_interp() ? "HW" : "SW", TimeCompute(*metal_rc)) << std::endl;
  }

  if (have_ocl)
  {
    if (OpenCLRayCastDevicesMatching(dev).size() < OpenCLDevicesMatching(dev).size())
    {
      std::cout << fmt::format("      {:<20} OpenCL:             skipped (invalid ray casts, see "
                               "OpenCLRayCastersInvalid())", desc) << std::endl;
    }

    for (const auto& ocl_dev : OpenCLRayCastDevicesMatching(dev))
    {
      tRayCasterOCL ocl_rc(ocl_dev);
      SetupSurface(ocl_rc, s, 0);

      std::cout << fmt::format("      {:<20} OpenCL:             {:8.2f} ms", desc, TimeCompute(ocl_rc))
                << std::endl;
    }
  }
}

}  // un-named

int main(int argc, char* argv[])
{
  const bool have_ocl = HaveOpenCL();

  const std::optional<boost::compute::device> ocl_ref_dev = have_ocl ? OpenCLRayCastRefDevice() :
                                                                        std::nullopt;

  if (ocl_ref_dev)
  {
    std::cout << "OpenCL reference device: " << ocl_ref_dev->name() << std::endl;
  }
  else
  {
    std::cout << "No valid OpenCL device available, skipping OpenCL comparisons." << std::endl;
  }

  // the detector is larger than the projected volume, so some rays do not
  // intersect the volume
  const Scene scene        = MakeScene(kNUM_PROJS, 96, 80, 64, 120, 150, 1.2);
  const Scene timing_scene = MakeScene(50, 256, 256, 256, 256, 256, 1.2);

  for (const auto& dev : MetalAllDevices())
  {
    std::cout << "Metal device: " << dev.id_str() << std::endl;

    for (const bool force_sw : { false, true })
    {
      if (!force_sw && !dev.supports_32bit_float_filtering())
      {
        std::cout << "  device does not support 32-bit float filtering, skipping HW interp." << std::endl;
        continue;
      }

      std::cout << "  " << (force_sw ? "SW" : "HW") << " interpolation:" << std::endl;

      TestSurRender(dev, force_sw, scene, ocl_ref_dev);

      TestOccContours(dev, force_sw, scene, ocl_ref_dev);
    }

    std::cout << fmt::format("  timings, {} projections of {}x{} through a {}^3 volume:",
                             timing_scene.poses.size(), timing_scene.cams[0].num_det_cols,
                             timing_scene.cams[0].num_det_rows,
                             timing_scene.vol->GetLargestPossibleRegion().GetSize()[0]) << std::endl;

    PrintTimings<RayCasterSurRenderMetal,RayCasterSurRenderOCL>("surface rendering", dev,
                                                                 timing_scene, have_ocl);

    PrintTimings<RayCasterOccludingContoursMetal,RayCasterOccludingContoursOCL>("occluding contours", dev,
                                                                                 timing_scene, have_ocl);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}
