
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregRayCastDepthCPU.h"
#include "xregRayCastDepthMetal.h"
#include "xregRayCastDepthOCL.h"
#include "xregRayCastProgOpts.h"
#include "xregRayCastTestUtils.h"

namespace
{

using namespace xreg;
using namespace xreg::test;

constexpr size_type kNUM_PROJS = 6;

/// \brief Surfaces are located where the blobs of the test volume reach this
///        value, which gives closed surfaces that some rays miss.
constexpr float kSUR_THRESH = 0.5f;

void SetupDepth(RayCaster& rc, const Scene& s, const CoordScalar step_size,
                const size_type num_backtracking_steps = 0)
{
  SetupRayCaster(rc, s, step_size,
                 [num_backtracking_steps] (RayCaster& r)
                 {
                   auto& coll = dynamic_cast<RayCasterCollisionParamInterface&>(r);
                   coll.set_render_thresh(kSUR_THRESH);
                   coll.set_num_backtracking_steps(num_backtracking_steps);
                 });
}

std::unique_ptr<RayCasterDepthMetal> MakeMetal(const MetalDevice& dev, const bool force_sw)
{
  auto rc = std::make_unique<RayCasterDepthMetal>(dev);
  rc->set_force_sw_interp(force_sw);
  return rc;
}

/// \brief true when a surface was found for a pixel, pixels without a surface
///        keep the background value of kRAY_CAST_MAX_DEPTH.
bool IsHit(const float depth)
{
  return depth < kRAY_CAST_MAX_DEPTH;
}

struct DepthDiffStats
{
  size_type num_pix = 0;

  /// \brief number of pixels with a surface in both
  size_type num_hits = 0;

  /// \brief number of pixels with a surface in only one of the inputs
  size_type num_hit_mismatches = 0;

  /// \brief statistics of the depth differences for pixels with a surface in both
  float max_abs_diff  = 0;
  float mean_abs_diff = 0;

  /// \brief range of depths for pixels with a surface in both
  float min_depth = kRAY_CAST_MAX_DEPTH;
  float max_depth = 0;
};

DepthDiffStats CompareDepths(const PixelBuf& a, const PixelBuf& b)
{
  xregASSERT(a.size() == b.size());

  DepthDiffStats stats;
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

      stats.min_depth = std::min(stats.min_depth, std::min(a[i], b[i]));
      stats.max_depth = std::max(stats.max_depth, std::max(a[i], b[i]));
    }
    else if (hit_a != hit_b)
    {
      ++stats.num_hit_mismatches;
    }
  }

  stats.mean_abs_diff = stats.num_hits ? static_cast<float>(sum_abs_diff / stats.num_hits) : 0.0f;

  return stats;
}

/// \brief Checks the fraction of pixels with a surface in only one input, and
///        the depth differences (mm) of pixels with a surface in both.
void CheckDepthsClose(const std::string& desc, const PixelBuf& a, const PixelBuf& b,
                      const float max_mismatch_frac, const float max_abs_tol,
                      const float mean_abs_tol)
{
  const DepthDiffStats stats = CompareDepths(a, b);

  const float mismatch_frac = static_cast<float>(stats.num_hit_mismatches) / stats.num_pix;

  std::cout << fmt::format("    {:<40} hits: {:5.1f}%  mismatches: {:6.3f}%  depths: [{:6.1f}, {:6.1f}]  "
                           "max diff: {:.2e}  mean diff: {:.2e}",
                           desc, (100.0 * stats.num_hits) / stats.num_pix, 100 * mismatch_frac,
                           stats.min_depth, stats.max_depth,
                           stats.max_abs_diff, stats.mean_abs_diff) << std::endl;

  // the scene should have pixels with and without surfaces
  xregASSERT(stats.num_hits > 0);
  xregASSERT(stats.num_hits < stats.num_pix);

  xregASSERT(mismatch_frac <= max_mismatch_frac);
  xregASSERT(stats.max_abs_diff <= max_abs_tol);
  xregASSERT(stats.mean_abs_diff <= mean_abs_tol);
}

/// \brief Checks that two results are identical.
void CheckDepthsEqual(const std::string& desc, const PixelBuf& a, const PixelBuf& b)
{
  CheckDepthsClose(desc, a, b, 0, 0, 0);
}

// Tolerances, depths and differences are in mm:
//
// The Metal and OpenCL kernels perform the same computations without
// backtracking. Interpolation differences (e.g. reduced precision interpolation
// weights in texture hardware) may move a sample across the threshold, which
// moves the surface by one step for those pixels, or adds/removes a surface for
// a grazing ray.
constexpr float kOCL_MAX_MISMATCH_FRAC = 1.0e-3f;
constexpr float kOCL_MAX_ABS_TOL_STEPS = 1.01f;  // multiple of the step size
constexpr float kOCL_MEAN_ABS_TOL      = 1.0e-2f;

// With backtracking, the surface is located within step_size / 2^N of the
// threshold crossing, which also removes the effect of the different sample
// locations of the CPU ray caster (it nudges the start of each ray). The
// remaining differences come from interpolation: with software interpolation
// the maximum difference was ~0.01 mm. Hardware interpolation weights have
// reduced precision, and the resulting error in the volume value moves the
// threshold crossing furthest for rays that are nearly tangent to a surface:
// the maximum difference was ~0.09 mm on an AMD Radeon Pro 5500M and ~0.2 mm on
// an Intel UHD 630.
constexpr size_type kNUM_BACKTRACKING_STEPS = 8;

constexpr float kCPU_MAX_MISMATCH_FRAC = 1.0e-3f;
constexpr float kCPU_MAX_ABS_TOL_SW    = 5.0e-2f;
constexpr float kCPU_MAX_ABS_TOL_HW    = 0.5f;
constexpr float kCPU_MEAN_ABS_TOL      = 1.0e-2f;

void TestAgainstReferences(const MetalDevice& dev, const bool force_sw, const Scene& s,
                           const bool have_ocl)
{
  if (have_ocl)
  {
    for (const CoordScalar step_size : { 1.0f, 0.5f })
    {
      auto metal_rc = MakeMetal(dev, force_sw);
      SetupDepth(*metal_rc, s, step_size);
      metal_rc->compute();

      RayCasterDepthOCL ocl_rc;
      SetupDepth(ocl_rc, s, step_size);
      ocl_rc.compute();

      CheckDepthsClose(fmt::format("step {}, vs. OpenCL", step_size),
                       GetProjs(*metal_rc, kNUM_PROJS), GetProjs(ocl_rc, kNUM_PROJS),
                       kOCL_MAX_MISMATCH_FRAC, kOCL_MAX_ABS_TOL_STEPS * step_size,
                       kOCL_MEAN_ABS_TOL);
    }
  }

  {
    auto metal_rc = MakeMetal(dev, force_sw);
    SetupDepth(*metal_rc, s, 1, kNUM_BACKTRACKING_STEPS);
    metal_rc->compute();

    RayCasterDepthCPU cpu_rc;
    SetupDepth(cpu_rc, s, 1, kNUM_BACKTRACKING_STEPS);
    cpu_rc.compute();

    CheckDepthsClose(fmt::format("step 1, {} backtracking, vs. CPU", kNUM_BACKTRACKING_STEPS),
                     GetProjs(*metal_rc, kNUM_PROJS), GetProjs(cpu_rc, kNUM_PROJS),
                     kCPU_MAX_MISMATCH_FRAC,
                     metal_rc->use_hw_interp() ? kCPU_MAX_ABS_TOL_HW : kCPU_MAX_ABS_TOL_SW,
                     kCPU_MEAN_ABS_TOL);
  }
}

void TestBacktracking(const MetalDevice& dev, const bool force_sw, const Scene& s)
{
  constexpr CoordScalar kSTEP_SIZE = 1;

  auto rc_no_bt = MakeMetal(dev, force_sw);
  SetupDepth(*rc_no_bt, s, kSTEP_SIZE);
  rc_no_bt->compute();

  auto rc_bt = MakeMetal(dev, force_sw);
  SetupDepth(*rc_bt, s, kSTEP_SIZE, kNUM_BACKTRACKING_STEPS);
  rc_bt->compute();

  const PixelBuf no_bt = GetProjs(*rc_no_bt, kNUM_PROJS);
  const PixelBuf bt    = GetProjs(*rc_bt, kNUM_PROJS);

  // Backtracking only refines a location between the first sample at or above
  // the threshold and the previous sample: the same pixels have surfaces, the
  // depths do not increase, and they decrease by less than one step.
  CheckDepthsClose("backtracking vs. no backtracking", bt, no_bt, 0, kSTEP_SIZE, kSTEP_SIZE);

  bool any_changed = false;

  for (size_type i = 0; i < bt.size(); ++i)
  {
    if (IsHit(bt[i]))
    {
      xregASSERT(bt[i] <= (no_bt[i] + 1.0e-3f));

      any_changed = any_changed || (bt[i] != no_bt[i]);
    }
  }

  xregASSERT(any_changed);
}

void TestStoreMethods(const MetalDevice& dev, const bool force_sw, const Scene& s)
{
  const size_type num_pix_per_proj = s.cams[0].num_det_rows * s.cams[0].num_det_cols;

  auto ref_rc = MakeMetal(dev, force_sw);
  SetupDepth(*ref_rc, s, 1);
  ref_rc->compute();

  const PixelBuf ref = GetProjs(*ref_rc, kNUM_PROJS);

  // pixels without a surface are exactly the background value
  for (const float d : ref)
  {
    xregASSERT(IsHit(d) || (d == static_cast<float>(kRAY_CAST_MAX_DEPTH)));
  }

  // accumulating with the same result does not change it (min(d,d) == d)
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupDepth(*rc, s, 1);
    rc->compute();
    rc->use_proj_store_accum_method();
    rc->compute();

    CheckDepthsEqual("accumulate", GetProjs(*rc, kNUM_PROJS), ref);
  }

  // background projections: the smaller of the background and surface depths
  // is kept, a different background is used for each camera
  {
    const DepthDiffStats ref_stats = CompareDepths(ref, ref);

    // between the nearest and farthest surfaces, so that the background
    // replaces some surfaces
    const std::vector<float> bg_depths = { ref_stats.min_depth + 0.25f * (ref_stats.max_depth - ref_stats.min_depth),
                                           ref_stats.min_depth + 0.5f * (ref_stats.max_depth - ref_stats.min_depth) };

    auto rc = MakeMetal(dev, force_sw);
    rc->set_camera_models(s.cams);

    RayCaster::ProjList bg_projs;
    for (size_type cam_idx = 0; cam_idx < s.cams.size(); ++cam_idx)
    {
      auto bg = Proj::New();
      Proj::RegionType region;
      region.SetSize({ s.cams[0].num_det_cols, s.cams[0].num_det_rows });
      bg->SetRegions(region);
      bg->Allocate();
      bg->FillBuffer(bg_depths[cam_idx]);

      bg_projs.push_back(bg);
    }

    rc->set_bg_projs(bg_projs);

    SetupDepth(*rc, s, 1);
    rc->compute();

    const PixelBuf result = GetProjs(*rc, kNUM_PROJS);

    size_type num_bg = 0;

    for (size_type proj_idx = 0; proj_idx < kNUM_PROJS; ++proj_idx)
    {
      const float bg_depth = bg_depths[s.cam_for_proj[proj_idx]];

      for (size_type i = 0; i < num_pix_per_proj; ++i)
      {
        const size_type pix_idx = (proj_idx * num_pix_per_proj) + i;

        xregASSERT(result[pix_idx] == std::min(ref[pix_idx], bg_depth));

        num_bg += (result[pix_idx] == bg_depth) ? 1 : 0;
      }
    }

    std::cout << fmt::format("    {:<40} {} pixels set to the background", "background projections", num_bg)
              << std::endl;

    // both the background and surfaces are present
    xregASSERT(num_bg > 0);
    xregASSERT(num_bg < result.size());
  }

  // only linear interpolation is supported
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupDepth(*rc, s, 1);
    rc->use_nn_interp();

    bool threw = false;
    try
    {
      rc->compute();
    }
    catch (const RayCaster::UnsupportedOperationException&)
    {
      threw = true;
    }
    xregASSERT(threw);
  }
}

void TestProgOpts(const MetalDevice& dev)
{
  auto po = ParseArgs({ "--backend", "metal", "--metal-id", dev.id_str() });

  auto rc = DepthRayCasterFromProgOpts(*po);

  auto* metal_rc = dynamic_cast<RayCasterDepthMetal*>(rc.get());
  xregASSERT(metal_rc);
  xregASSERT(metal_rc->device().registry_id() == dev.registry_id());

  std::cout << "    DepthRayCasterFromProgOpts() created a Metal ray caster" << std::endl;
}

void PrintTimings(const MetalDevice& dev, const Scene& s, const bool have_ocl)
{
  std::cout << fmt::format("    {} projections of {}x{} through a {}^3 volume:",
                           s.poses.size(), s.cams[0].num_det_cols, s.cams[0].num_det_rows,
                           s.vol->GetLargestPossibleRegion().GetSize()[0]) << std::endl;

  for (const bool force_sw : { false, true })
  {
    auto metal_rc = MakeMetal(dev, force_sw);

    if (!force_sw && !metal_rc->use_hw_interp())
    {
      continue;
    }

    SetupDepth(*metal_rc, s, 1);

    std::cout << fmt::format("      Metal ({} interp.): {:8.2f} ms",
                             metal_rc->use_hw_interp() ? "HW" : "SW", TimeCompute(*metal_rc)) << std::endl;
  }

  if (have_ocl)
  {
    for (const auto& ocl_dev : OpenCLDevicesMatching(dev))
    {
      RayCasterDepthOCL ocl_rc(ocl_dev);
      SetupDepth(ocl_rc, s, 1);

      std::cout << fmt::format("      OpenCL:             {:8.2f} ms", TimeCompute(ocl_rc)) << std::endl;
    }
  }
}

}  // un-named

int main(int argc, char* argv[])
{
  const bool have_ocl = HaveOpenCL();

  if (!have_ocl)
  {
    std::cout << "No OpenCL platform available, skipping OpenCL comparisons." << std::endl;
  }

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

      TestAgainstReferences(dev, force_sw, scene, have_ocl);

      TestBacktracking(dev, force_sw, scene);

      TestStoreMethods(dev, force_sw, scene);
    }

    std::cout << "  program options:" << std::endl;
    TestProgOpts(dev);

    std::cout << "  timings:" << std::endl;
    PrintTimings(dev, timing_scene, have_ocl);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}
