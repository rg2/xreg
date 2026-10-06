
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <boost/compute/system.hpp>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregMetalSys.h"
#include "xregProgOptUtils.h"
#include "xregRayCastLineIntCPU.h"
#include "xregRayCastLineIntMetal.h"
#include "xregRayCastLineIntOCL.h"
#include "xregRayCastProgOpts.h"
#include "xregRayCastSyncBuf.h"
#include "xregRigidUtils.h"

namespace
{

using namespace xreg;

using Vol    = RayCaster::Vol;
using VolPtr = RayCaster::VolPtr;
using Proj   = RayCaster::Proj;
using ProjPtr = RayCaster::ProjPtr;
using PixelBuf = std::vector<RayCaster::PixelScalar2D>;

constexpr size_type kNUM_PROJS = 6;

/// \brief A volume of smooth blobs, which is zero at the boundary.
///
/// The spacing is anisotropic, and the direction and origin are not trivial,
/// so that the transformation from physical points to indices is exercised.
VolPtr MakeVolume(const size_type nx, const size_type ny, const size_type nz)
{
  auto vol = Vol::New();

  Vol::RegionType region;
  region.SetIndex({ 0, 0, 0 });
  region.SetSize({ nx, ny, nz });
  vol->SetRegions(region);

  const double spacing[3] = { 0.8, 1.0, 1.25 };
  vol->SetSpacing(spacing);

  const double origin[3] = { -30, 12, 5 };
  vol->SetOrigin(origin);

  const Mat3x3 rot = (EulerRotZFrame(20 * kDEG2RAD) * EulerRotXFrame(-10 * kDEG2RAD)).matrix().block(0,0,3,3);

  Vol::DirectionType dir;
  for (unsigned r = 0; r < 3; ++r)
  {
    for (unsigned c = 0; c < 3; ++c)
    {
      dir(r,c) = rot(r,c);
    }
  }
  vol->SetDirection(dir);

  vol->Allocate();

  // blob centers (as fractions of the volume size), widths (in voxels) and amplitudes
  const std::vector<std::array<float,5>> blobs = { { 0.5f, 0.5f, 0.5f, 0.18f, 1.0f },
                                                   { 0.3f, 0.6f, 0.4f, 0.08f, 2.0f },
                                                   { 0.7f, 0.35f, 0.6f, 0.06f, 1.5f } };

  const float dims[3] = { float(nx), float(ny), float(nz) };

  float* buf = vol->GetBufferPointer();

  for (size_type k = 0; k < nz; ++k)
  {
    for (size_type j = 0; j < ny; ++j)
    {
      for (size_type i = 0; i < nx; ++i, ++buf)
      {
        const float idx[3] = { float(i), float(j), float(k) };

        float val = 0;

        for (const auto& b : blobs)
        {
          float d_sq = 0;
          for (int d = 0; d < 3; ++d)
          {
            const float diff = (idx[d] - (b[d] * dims[d])) / (b[3] * dims[0]);
            d_sq += diff * diff;
          }

          val += b[4] * std::exp(-0.5f * d_sq);
        }

        // ensure zero values at the volume boundary
        const bool on_boundary = (i == 0) || (j == 0) || (k == 0) ||
                                 (i == (nx - 1)) || (j == (ny - 1)) || (k == (nz - 1));

        *buf = on_boundary ? 0.0f : val;
      }
    }
  }

  return vol;
}

/// \brief Two cameras with the same detector size and different focal lengths.
RayCaster::CameraModelList MakeCameras(const size_type num_rows, const size_type num_cols,
                                       const CoordScalar pixel_spacing)
{
  RayCaster::CameraModelList cams(2);

  cams[0].setup(1000, num_rows, num_cols, pixel_spacing, pixel_spacing);
  cams[1].setup(900, num_rows, num_cols, pixel_spacing, pixel_spacing);

  return cams;
}

/// \brief Scene to ray cast: a volume, cameras and a pose for each projection.
struct Scene
{
  VolPtr vol;

  RayCaster::CameraModelList cams;

  FrameTransformList poses;

  /// \brief camera model index of each projection
  RayCaster::CamModelAssocList cam_for_proj;
};

Scene MakeScene(const size_type num_projs, const size_type vol_dim_x, const size_type vol_dim_y,
                const size_type vol_dim_z, const size_type num_det_rows, const size_type num_det_cols,
                const CoordScalar pixel_spacing)
{
  Scene s;

  s.vol  = MakeVolume(vol_dim_x, vol_dim_y, vol_dim_z);
  s.cams = MakeCameras(num_det_rows, num_det_cols, pixel_spacing);

  // A CPU ray caster is used to compute the pose helpers
  RayCasterLineIntCPU rc;
  rc.set_camera_models(s.cams);
  rc.set_volume(s.vol);

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    const size_type cam_idx = proj_idx % s.cams.size();

    // rotations about the volume center, with a small offset so that the
    // central ray is not aligned with the volume
    const FrameTransform rot = EulerRotYFrame((proj_idx * 37.0f + 5) * kDEG2RAD) *
                               EulerRotXFrame((proj_idx * 11.0f - 20) * kDEG2RAD) *
                               EulerRotZFrame((proj_idx * 23.0f) * kDEG2RAD);

    FrameTransform trans = FrameTransform::Identity();
    trans.translation() = Pt3(3, -2, 4);

    // The cameras use kORIGIN_AT_FOCAL_PT_DET_NEG_Z, so the midpoint between
    // the focal point and detector, which is placed at the volume center, is
    // located at z = -focal_len / 2 with respect to the camera.
    FrameTransform cam_wrt_midpoint = FrameTransform::Identity();
    cam_wrt_midpoint.translation() = Pt3(0, 0, s.cams[cam_idx].focal_len / 2);

    s.poses.push_back(rc.xform_img_center_to_itk_phys() * trans * rot * cam_wrt_midpoint);

    s.cam_for_proj.push_back(cam_idx);
  }

  return s;
}

void SetupRayCaster(RayCaster& rc, const Scene& s, const CoordScalar step_size,
                    const RayCastLineIntKernel kernel = kRAY_CAST_LINE_INT_SUM_KERNEL)
{
  rc.set_camera_models(s.cams);
  rc.set_volume(s.vol);
  rc.set_num_projs(s.poses.size());
  rc.set_ray_step_size(step_size);

  if (auto* li = dynamic_cast<RayCastLineIntParamInterface*>(&rc))
  {
    li->set_kernel_id(kernel);
  }

  rc.allocate_resources();

  for (size_type proj_idx = 0; proj_idx < s.poses.size(); ++proj_idx)
  {
    rc.xform_cam_to_itk_phys(proj_idx) = s.poses[proj_idx];
    rc.set_proj_cam_model(proj_idx, s.cam_for_proj[proj_idx]);
  }
}

/// \brief Copy all projections of a ray caster into a buffer (synchronizing
///        with the host when necessary).
PixelBuf GetProjs(RayCaster& rc, const size_type num_projs)
{
  PixelBuf pixels;

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    const cv::Mat p = rc.proj_ocv(proj_idx);

    pixels.insert(pixels.end(), p.ptr<float>(), p.ptr<float>() + p.total());
  }

  return pixels;
}

struct DiffStats
{
  float max_abs_diff  = 0;
  float mean_abs_diff = 0;
  float max_abs_val   = 0;
};

DiffStats CompareProjs(const PixelBuf& a, const PixelBuf& b)
{
  xregASSERT(a.size() == b.size());

  DiffStats stats;

  double sum_abs_diff = 0;

  for (size_type i = 0; i < a.size(); ++i)
  {
    const float d = std::abs(a[i] - b[i]);

    stats.max_abs_diff = std::max(stats.max_abs_diff, d);
    stats.max_abs_val  = std::max(stats.max_abs_val, std::max(std::abs(a[i]), std::abs(b[i])));

    sum_abs_diff += d;
  }

  stats.mean_abs_diff = static_cast<float>(sum_abs_diff / a.size());

  return stats;
}

/// \brief Checks that the max and mean absolute differences, relative to the
///        maximum absolute pixel value, are within tolerances.
void CheckClose(const std::string& desc, const PixelBuf& a, const PixelBuf& b,
                const float max_rel_tol, const float mean_rel_tol)
{
  const DiffStats stats = CompareProjs(a, b);

  const float max_rel  = stats.max_abs_diff  / stats.max_abs_val;
  const float mean_rel = stats.mean_abs_diff / stats.max_abs_val;

  std::cout << fmt::format("    {:<44} max val: {:9.3f}  max rel. diff: {:.2e}  mean rel. diff: {:.2e}",
                           desc, stats.max_abs_val, max_rel, mean_rel) << std::endl;

  xregASSERT(stats.max_abs_val > 0);
  xregASSERT(max_rel <= max_rel_tol);
  xregASSERT(mean_rel <= mean_rel_tol);
}

/// \brief The OpenCL ray casters are only used for comparisons when an OpenCL
///        platform is available.
bool HaveOpenCL()
{
  try
  {
    return !boost::compute::system::platforms().empty();
  }
  catch (...)
  {
    return false;
  }
}

std::unique_ptr<RayCasterLineIntMetal> MakeMetal(const MetalDevice& dev, const bool force_sw)
{
  auto rc = std::make_unique<RayCasterLineIntMetal>(dev);
  rc->set_force_sw_interp(force_sw);
  return rc;
}

// Tolerances (relative to the maximum pixel value):
//
// The Metal and OpenCL kernels perform the same computations, the remaining
// differences come from interpolation; texture hardware uses reduced precision
// interpolation weights (e.g. 8-bit sub-voxel weights) and the software
// trilinear path does not.
constexpr float kOCL_MAX_REL_TOL  = 5.0e-3f;
constexpr float kOCL_MEAN_REL_TOL = 5.0e-4f;

// The CPU ray caster additionally nudges the start and stop of each ray into
// the volume, which shifts the sample locations along each ray.
constexpr float kCPU_MAX_REL_TOL  = 2.0e-2f;
constexpr float kCPU_MEAN_REL_TOL = 2.0e-3f;

// Consistency checks of a Metal ray caster with itself
constexpr float kSELF_MAX_REL_TOL  = 1.0e-5f;
constexpr float kSELF_MEAN_REL_TOL = 1.0e-6f;

void TestAgainstReferences(const MetalDevice& dev, const bool force_sw, const Scene& s,
                           const bool have_ocl)
{
  // step size of 1 so that the CPU results are comparable (the CPU ray caster
  // scales by the step size, while the OpenCL and Metal ray casters do not)
  {
    auto metal_rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*metal_rc, s, 1);
    metal_rc->compute();

    RayCasterLineIntCPU cpu_rc;
    SetupRayCaster(cpu_rc, s, 1);
    cpu_rc.compute();

    CheckClose("sum, step 1, vs. CPU", GetProjs(*metal_rc, kNUM_PROJS), GetProjs(cpu_rc, kNUM_PROJS),
               kCPU_MAX_REL_TOL, kCPU_MEAN_REL_TOL);
  }

  {
    auto metal_rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*metal_rc, s, 1, kRAY_CAST_LINE_INT_MAX_KERNEL);
    metal_rc->compute();

    RayCasterLineIntCPU cpu_rc;
    SetupRayCaster(cpu_rc, s, 1, kRAY_CAST_LINE_INT_MAX_KERNEL);
    cpu_rc.compute();

    CheckClose("max, step 1, vs. CPU", GetProjs(*metal_rc, kNUM_PROJS), GetProjs(cpu_rc, kNUM_PROJS),
               kCPU_MAX_REL_TOL, kCPU_MEAN_REL_TOL);
  }

  if (have_ocl)
  {
    for (const CoordScalar step_size : { 1.0f, 0.5f })
    {
      for (const auto kernel : { kRAY_CAST_LINE_INT_SUM_KERNEL, kRAY_CAST_LINE_INT_MAX_KERNEL })
      {
        auto metal_rc = MakeMetal(dev, force_sw);
        SetupRayCaster(*metal_rc, s, step_size, kernel);
        metal_rc->compute();

        RayCasterLineIntOCL ocl_rc;
        SetupRayCaster(ocl_rc, s, step_size, kernel);
        ocl_rc.compute();

        CheckClose(fmt::format("{}, step {}, vs. OpenCL",
                               (kernel == kRAY_CAST_LINE_INT_SUM_KERNEL) ? "sum" : "max", step_size),
                   GetProjs(*metal_rc, kNUM_PROJS), GetProjs(ocl_rc, kNUM_PROJS),
                   kOCL_MAX_REL_TOL, kOCL_MEAN_REL_TOL);
      }
    }
  }
}

PixelBuf AddConstant(PixelBuf buf, const float c)
{
  for (auto& x : buf)
  {
    x += c;
  }

  return buf;
}

void TestStoreMethodsAndBuffers(const MetalDevice& dev, const bool force_sw, const Scene& s)
{
  const size_type num_pix_per_proj = s.cams[0].num_det_rows * s.cams[0].num_det_cols;

  // reference result
  auto ref_rc = MakeMetal(dev, force_sw);
  SetupRayCaster(*ref_rc, s, 1);
  ref_rc->compute();

  const PixelBuf ref = GetProjs(*ref_rc, kNUM_PROJS);

  // computing again gives the same result (the projections are replaced)
  ref_rc->compute();
  CheckClose("repeated compute", GetProjs(*ref_rc, kNUM_PROJS), ref,
             kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);

  // non-zero default background value
  {
    auto rc = MakeMetal(dev, force_sw);
    rc->set_default_bg_pixel_val(3);
    SetupRayCaster(*rc, s, 1);
    rc->compute();

    CheckClose("default background value", GetProjs(*rc, kNUM_PROJS), AddConstant(ref, 3),
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // accumulation
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*rc, s, 1);
    rc->compute();
    rc->use_proj_store_accum_method();
    rc->compute();

    PixelBuf ref_x2 = ref;
    for (auto& x : ref_x2)
    {
      x *= 2;
    }

    CheckClose("accumulate", GetProjs(*rc, kNUM_PROJS), ref_x2,
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // background projections, a different background for each camera
  {
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
      bg->FillBuffer(5.0f + cam_idx);

      bg_projs.push_back(bg);
    }

    rc->set_bg_projs(bg_projs);

    SetupRayCaster(*rc, s, 1);
    rc->compute();

    PixelBuf expected = ref;
    for (size_type proj_idx = 0; proj_idx < kNUM_PROJS; ++proj_idx)
    {
      for (size_type i = 0; i < num_pix_per_proj; ++i)
      {
        expected[(proj_idx * num_pix_per_proj) + i] += 5.0f + s.cam_for_proj[proj_idx];
      }
    }

    CheckClose("background projections", GetProjs(*rc, kNUM_PROJS), expected,
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // fewer projections than allocated
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*rc, s, 1);
    rc->set_num_projs(kNUM_PROJS / 2);
    rc->compute();

    CheckClose("fewer projections than allocated", GetProjs(*rc, kNUM_PROJS / 2),
               PixelBuf(ref.begin(), ref.begin() + ((kNUM_PROJS / 2) * num_pix_per_proj)),
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // external host buffer
  {
    PixelBuf ext_buf(kNUM_PROJS * num_pix_per_proj, -1);

    auto rc = MakeMetal(dev, force_sw);
    rc->use_external_host_pixel_buf(ext_buf.data());
    SetupRayCaster(*rc, s, 1);
    rc->compute();

    xregASSERT(rc->raw_host_pixel_buf() == ext_buf.data());
    xregASSERT(rc->proj_ocv(1).ptr<float>() == (ext_buf.data() + num_pix_per_proj));

    CheckClose("external host buffer", ext_buf, ref, kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // synchronization objects
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*rc, s, 1);
    rc->compute();

    RayCastSyncHostBuf* to_host = rc->to_host_buf();
    to_host->alloc();
    to_host->sync();

    CheckClose("to_host_buf()", PixelBuf(to_host->host_buf().buf, to_host->host_buf().buf + ref.size()),
               ref, kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);

    RayCastSyncMetalBuf* to_metal = rc->to_metal_buf();
    to_metal->alloc();
    to_metal->sync();

    xregASSERT(to_metal->metal_buf_valid());

    PixelBuf from_dev(ref.size());
    CopyMetalToHost(to_metal->metal_buf(), 0, ref.size(), from_dev.data(), to_metal->queue());

    CheckClose("to_metal_buf()", from_dev, ref, kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);
  }

  // sharing a projection buffer
  {
    auto rc_a = MakeMetal(dev, force_sw);
    SetupRayCaster(*rc_a, s, 1);

    auto rc_b = MakeMetal(dev, force_sw);
    rc_b->use_other_proj_buf(rc_a.get());
    SetupRayCaster(*rc_b, s, 1);
    rc_b->compute();

    CheckClose("shared projection buffer", GetProjs(*rc_b, kNUM_PROJS), ref,
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);

    // the results were written into the buffer of the first ray caster
    PixelBuf from_a(ref.size());
    MetalCmdQueue queue_a = rc_a->queue();
    CopyMetalToHost(rc_a->to_metal_buf()->metal_buf(), 0, ref.size(), from_a.data(), queue_a);

    CheckClose("shared projection buffer (other ray caster)", from_a, ref,
               kSELF_MAX_REL_TOL, kSELF_MEAN_REL_TOL);

    // a CPU ray caster's buffer cannot be shared
    RayCasterLineIntCPU cpu_rc;
    bool threw = false;
    try
    {
      rc_a->use_other_proj_buf(&cpu_rc);
    }
    catch (const std::exception&)
    {
      threw = true;
    }
    xregASSERT(threw);
  }

  // only linear interpolation is supported
  {
    auto rc = MakeMetal(dev, force_sw);
    SetupRayCaster(*rc, s, 1);
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

std::unique_ptr<ProgOpts> ParseArgs(std::vector<std::string> args)
{
  auto po = std::make_unique<ProgOpts>();
  po->add_backend_flags();

  args.insert(args.begin(), "test-prog");

  std::vector<char*> argv;
  for (auto& a : args)
  {
    argv.push_back(&a[0]);
  }

  po->parse(static_cast<int>(argv.size()), argv.data());

  return po;
}

void TestProgOpts(const MetalDevice& dev)
{
  auto po = ParseArgs({ "--backend", "metal", "--metal-id", dev.id_str() });

  auto rc = LineIntRayCasterFromProgOpts(*po);

  auto* metal_rc = dynamic_cast<RayCasterLineIntMetal*>(rc.get());
  xregASSERT(metal_rc);
  xregASSERT(metal_rc->device().registry_id() == dev.registry_id());

  std::cout << "    LineIntRayCasterFromProgOpts() created a Metal ray caster" << std::endl;

  // there is no Metal depth ray caster
  bool threw = false;
  try
  {
    DepthRayCasterFromProgOpts(*po);
  }
  catch (const std::exception&)
  {
    threw = true;
  }
  xregASSERT(threw);
}

/// \brief Average time of compute(), after a warm up call.
double TimeCompute(RayCaster& rc, const size_type num_runs = 5)
{
  using Clock = std::chrono::steady_clock;

  rc.compute();

  const auto start = Clock::now();

  for (size_type i = 0; i < num_runs; ++i)
  {
    rc.compute();
  }

  return std::chrono::duration<double,std::milli>(Clock::now() - start).count() / num_runs;
}

void PrintTimings(const MetalDevice& dev, const Scene& s, const bool have_ocl)
{
  auto metal_rc = MakeMetal(dev, false);
  SetupRayCaster(*metal_rc, s, 1);

  std::cout << fmt::format("    {} projections of {}x{} through a {}^3 volume:",
                           s.poses.size(), s.cams[0].num_det_cols, s.cams[0].num_det_rows,
                           s.vol->GetLargestPossibleRegion().GetSize()[0]) << std::endl;

  std::cout << fmt::format("      Metal ({} interp.): {:8.2f} ms",
                           metal_rc->use_hw_interp() ? "HW" : "SW", TimeCompute(*metal_rc)) << std::endl;

  if (metal_rc->use_hw_interp())
  {
    auto metal_sw_rc = MakeMetal(dev, true);
    SetupRayCaster(*metal_sw_rc, s, 1);

    std::cout << fmt::format("      Metal (SW interp.): {:8.2f} ms", TimeCompute(*metal_sw_rc)) << std::endl;
  }

  if (have_ocl)
  {
    // compare with OpenCL on the same device, when it can be found (OpenCL
    // device names may have a suffix, e.g. "Compute Engine")
    for (const auto& ocl_dev : boost::compute::system::devices())
    {
      if (ocl_dev.name().find(dev.name()) == 0)
      {
        RayCasterLineIntOCL ocl_rc(ocl_dev);
        SetupRayCaster(ocl_rc, s, 1);

        std::cout << fmt::format("      OpenCL:             {:8.2f} ms", TimeCompute(ocl_rc)) << std::endl;
      }
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

      TestAgainstReferences(dev, force_sw, scene, have_ocl);

      TestStoreMethodsAndBuffers(dev, force_sw, scene);
    }

    std::cout << "  program options:" << std::endl;
    TestProgOpts(dev);

    std::cout << "  timings:" << std::endl;
    PrintTimings(dev, timing_scene, have_ocl);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}
