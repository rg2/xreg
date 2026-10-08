
#ifndef XREGRAYCASTTESTUTILS_H_
#define XREGRAYCASTTESTUTILS_H_

// Utilities shared by the ray caster unit tests: a synthetic scene (volume,
// cameras and poses), ray caster setup, and timing.

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <boost/compute/device.hpp>
#include <boost/compute/system.hpp>

#include "xregMetalSys.h"
#include "xregProgOptUtils.h"
#include "xregRayCastLineIntCPU.h"
#include "xregRigidUtils.h"

namespace xreg
{
namespace test
{

using Vol    = RayCaster::Vol;
using VolPtr = RayCaster::VolPtr;
using Proj   = RayCaster::Proj;
using ProjPtr = RayCaster::ProjPtr;
using PixelBuf = std::vector<RayCaster::PixelScalar2D>;

/// \brief A volume of smooth blobs, which is zero at the boundary.
///
/// The spacing is anisotropic, and the direction and origin are not trivial,
/// so that the transformation from physical points to indices is exercised.
inline VolPtr MakeVolume(const size_type nx, const size_type ny, const size_type nz)
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
inline RayCaster::CameraModelList MakeCameras(const size_type num_rows, const size_type num_cols,
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

inline Scene MakeScene(const size_type num_projs, const size_type vol_dim_x, const size_type vol_dim_y,
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

/// \brief Set the scene of a ray caster, allocate resources and set the poses.
///
/// The optional configuration function is called prior to allocating resources,
/// e.g. to set ray caster specific parameters.
inline void SetupRayCaster(RayCaster& rc, const Scene& s, const CoordScalar step_size,
                           const std::function<void(RayCaster&)>& config = {})
{
  rc.set_camera_models(s.cams);
  rc.set_volume(s.vol);
  rc.set_num_projs(s.poses.size());
  rc.set_ray_step_size(step_size);

  if (config)
  {
    config(rc);
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
inline PixelBuf GetProjs(RayCaster& rc, const size_type num_projs)
{
  PixelBuf pixels;

  for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
  {
    const cv::Mat p = rc.proj_ocv(proj_idx);

    pixels.insert(pixels.end(), p.ptr<float>(), p.ptr<float>() + p.total());
  }

  return pixels;
}

/// \brief The OpenCL ray casters are only used for comparisons when an OpenCL
///        platform is available.
inline bool HaveOpenCL()
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

/// \brief Parse command line arguments with program options that include the
///        backend flags.
inline std::unique_ptr<ProgOpts> ParseArgs(std::vector<std::string> args)
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

/// \brief Average time of compute(), after a warm up call.
inline double TimeCompute(RayCaster& rc, const size_type num_runs = 5)
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

/// \brief The OpenCL device corresponding to a Metal device, if found.
///
/// The devices are matched by name, OpenCL device names may have a suffix, e.g.
/// "Compute Engine".
inline std::vector<boost::compute::device> OpenCLDevicesMatching(const MetalDevice& dev)
{
  std::vector<boost::compute::device> ocl_devs;

  for (const auto& ocl_dev : boost::compute::system::devices())
  {
    if (ocl_dev.name().find(dev.name()) == 0)
    {
      ocl_devs.push_back(ocl_dev);
    }
  }

  return ocl_devs;
}

/// \brief true when an OpenCL device returns invalid volume samples in the
///        xreg OpenCL ray casters.
///
/// The AMD OpenCL driver on macOS (e.g. for a Radeon Pro 5500M) returns zeros
/// or garbage when sampling a 3D image whose kernel argument follows a struct
/// passed by value of at least 32 bytes. Every xreg OpenCL ray caster passes
/// its arguments struct (120 bytes) prior to the volume image, so all of the
/// samples are invalid and these devices cannot be used as references.
inline bool OpenCLRayCastersInvalid(const boost::compute::device& ocl_dev)
{
  return ocl_dev.vendor().find("AMD") != std::string::npos;
}

/// \brief The OpenCL device used for reference ray casts, if any.
///
/// This is the default device, unless its ray casts are invalid (see
/// OpenCLRayCastersInvalid()), in which case another GPU, and lastly any other
/// device, is used. The reference device may differ from the Metal device that
/// is compared with it, the ray casting algorithms are the same on every device.
inline std::optional<boost::compute::device> OpenCLRayCastRefDevice()
{
  const auto dflt = boost::compute::system::default_device();

  if (!OpenCLRayCastersInvalid(dflt))
  {
    return dflt;
  }

  std::optional<boost::compute::device> ref;

  for (const auto& ocl_dev : boost::compute::system::devices())
  {
    if (!OpenCLRayCastersInvalid(ocl_dev) &&
        (!ref || ((ocl_dev.type() & CL_DEVICE_TYPE_GPU) && !(ref->type() & CL_DEVICE_TYPE_GPU))))
    {
      ref = ocl_dev;
    }
  }

  return ref;
}

/// \brief The OpenCL devices corresponding to a Metal device whose ray casts
///        are valid, e.g. for comparing timings.
///
/// \see OpenCLDevicesMatching
/// \see OpenCLRayCastersInvalid
inline std::vector<boost::compute::device> OpenCLRayCastDevicesMatching(const MetalDevice& dev)
{
  std::vector<boost::compute::device> ocl_devs;

  for (const auto& ocl_dev : OpenCLDevicesMatching(dev))
  {
    if (!OpenCLRayCastersInvalid(ocl_dev))
    {
      ocl_devs.push_back(ocl_dev);
    }
  }

  return ocl_devs;
}

}  // test
}  // xreg

#endif
