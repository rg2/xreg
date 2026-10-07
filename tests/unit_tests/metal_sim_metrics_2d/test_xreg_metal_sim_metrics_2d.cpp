
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <boost/compute/command_queue.hpp>
#include <boost/compute/context.hpp>

#include <fmt/format.h>

#include "xregAssert.h"
#include "xregITKBasicImageUtils.h"
#include "xregImgSimMetric2DGradNCCCPU.h"
#include "xregImgSimMetric2DGradNCCMetal.h"
#include "xregImgSimMetric2DGradNCCOCL.h"
#include "xregImgSimMetric2DNCCCPU.h"
#include "xregImgSimMetric2DNCCMetal.h"
#include "xregImgSimMetric2DNCCOCL.h"
#include "xregImgSimMetric2DPatchGradNCCCPU.h"
#include "xregImgSimMetric2DPatchGradNCCMetal.h"
#include "xregImgSimMetric2DPatchGradNCCOCL.h"
#include "xregImgSimMetric2DPatchNCCCPU.h"
#include "xregImgSimMetric2DPatchNCCMetal.h"
#include "xregImgSimMetric2DPatchNCCOCL.h"
#include "xregImgSimMetric2DProgOpts.h"
#include "xregImgSimMetric2DSSDCPU.h"
#include "xregImgSimMetric2DSSDMetal.h"
#include "xregRayCastLineIntMetal.h"
#include "xregRayCastTestUtils.h"

namespace
{

using namespace xreg;
using namespace xreg::test;

using Scalar       = ImgSimMetric2D::Scalar;
using ScalarList   = ImgSimMetric2D::ScalarList;
using Image        = ImgSimMetric2D::Image;
using ImagePtr     = ImgSimMetric2D::ImagePtr;
using ImageMask    = ImgSimMetric2D::ImageMask;
using ImageMaskPtr = ImgSimMetric2D::ImageMaskPtr;

using SimPtr   = std::unique_ptr<ImgSimMetric2D>;
using ConfigFn = std::function<void(ImgSimMetric2D&)>;

using MetalDevBuf = ImgSimMetric2DMetal::DevBuf;

/// \brief A fixed image and moving images of the same scene.
///
/// The moving images are rendered with small perturbations of the fixed pose,
/// the first moving image uses the fixed pose.
struct SimScene
{
  Scene scene;

  ImagePtr fixed_img;

  size_type num_mov_imgs = 0;

  PixelBuf mov_imgs;

  ImageMaskPtr mask;

  size_type num_rows() const
  {
    return fixed_img->GetLargestPossibleRegion().GetSize()[1];
  }

  size_type num_cols() const
  {
    return fixed_img->GetLargestPossibleRegion().GetSize()[0];
  }

  size_type num_pix() const
  {
    return num_rows() * num_cols();
  }
};

/// \brief Deep copy of pixels into a new image.
ImagePtr MakeImage(const float* pixels, const size_type num_rows, const size_type num_cols)
{
  ImagePtr img = MakeITK2DVol<Scalar>(num_cols, num_rows);

  std::copy(pixels, pixels + (num_rows * num_cols), img->GetBufferPointer());

  return img;
}

ImagePtr CopyImage(const ImagePtr& src)
{
  const auto size = src->GetLargestPossibleRegion().GetSize();

  return MakeImage(src->GetBufferPointer(), size[1], size[0]);
}

/// \brief An elliptical mask with a rectangular hole.
ImageMaskPtr MakeMask(const size_type num_rows, const size_type num_cols)
{
  ImageMaskPtr mask = MakeITK2DVol<ImgSimMetric2D::MaskScalar>(num_cols, num_rows);

  auto* buf = mask->GetBufferPointer();

  for (size_type r = 0; r < num_rows; ++r)
  {
    for (size_type c = 0; c < num_cols; ++c, ++buf)
    {
      const double x = (c - (0.5 * num_cols)) / (0.42 * num_cols);
      const double y = (r - (0.5 * num_rows)) / (0.45 * num_rows);

      const bool in_hole = (r > (num_rows / 3)) && (r < (num_rows / 2)) &&
                           (c > (num_cols / 2)) && (c < ((3 * num_cols) / 4));

      *buf = (((x * x) + (y * y)) <= 1) && !in_hole;
    }
  }

  return mask;
}

SimScene MakeSimScene(const size_type num_mov_imgs, const size_type num_rows, const size_type num_cols)
{
  SimScene ss;

  ss.scene = MakeScene(1, 96, 80, 64, num_rows, num_cols, 1.2);

  const FrameTransform fixed_pose = ss.scene.poses[0];

  RayCasterLineIntCPU rc;
  rc.set_camera_models(ss.scene.cams);
  rc.set_volume(ss.scene.vol);

  // perturbations of the object about the volume center
  const FrameTransform center     = rc.xform_img_center_to_itk_phys();
  const FrameTransform center_inv = center.inverse();

  for (size_type i = 0; i < num_mov_imgs; ++i)
  {
    const float a = static_cast<float>(i);

    const FrameTransform rot = EulerRotXFrame(2.5f * std::sin(1.3f * a) * kDEG2RAD) *
                               EulerRotYFrame(2.5f * std::sin(0.7f * a) * kDEG2RAD) *
                               EulerRotZFrame(1.5f * std::sin(2.1f * a) * kDEG2RAD);

    FrameTransform pert = rot;
    pert.translation() = Pt3(3 * std::sin(a), 2 * std::sin(1.7f * a), 4 * std::sin(0.4f * a));

    ss.scene.poses.push_back(center * pert * center_inv * fixed_pose);
  }

  ss.scene.cam_for_proj.assign(ss.scene.poses.size(), 0);

  SetupRayCaster(rc, ss.scene, 1);
  rc.compute();

  const PixelBuf projs = GetProjs(rc, ss.scene.poses.size());

  const size_type num_pix = num_rows * num_cols;

  ss.fixed_img = MakeImage(projs.data(), num_rows, num_cols);

  ss.num_mov_imgs = num_mov_imgs;
  ss.mov_imgs.assign(projs.begin() + num_pix, projs.end());

  ss.mask = MakeMask(num_rows, num_cols);

  return ss;
}

/// \brief Computes similarity values with moving images from the host.
///
/// The moving images are copied, since some implementations modify them. The
/// copy starts with proj_off images that should not be used.
ScalarList ComputeSims(ImgSimMetric2D& sim, const SimScene& ss, const bool use_mask,
                       const ConfigFn& config, const size_type proj_off = 0,
                       std::shared_ptr<MetalDevBuf> fixed_dev = nullptr)
{
  PixelBuf mov_imgs(proj_off * ss.num_pix(), -1000.0f);
  mov_imgs.insert(mov_imgs.end(), ss.mov_imgs.begin(), ss.mov_imgs.end());

  sim.set_num_moving_images(ss.num_mov_imgs);
  sim.set_fixed_image(CopyImage(ss.fixed_img));

  if (use_mask)
  {
    sim.set_mask(ss.mask);
  }

  if (config)
  {
    config(sim);
  }

  if (fixed_dev)
  {
    dynamic_cast<ImgSimMetric2DMetal&>(sim).set_fixed_image_dev(fixed_dev);
  }

  sim.set_mov_imgs_host_buf(mov_imgs.data(), proj_off);

  sim.allocate_resources();

  sim.compute();

  if (dynamic_cast<ImgSimMetric2DMetal*>(&sim))
  {
    // the Metal similarity metrics never modify the fixed image
    const Scalar* fixed_after = sim.fixed_image()->GetBufferPointer();
    xregASSERT(std::equal(fixed_after, fixed_after + ss.num_pix(), ss.fixed_img->GetBufferPointer()));
  }

  return ScalarList(sim.sim_vals().begin(), sim.sim_vals().begin() + ss.num_mov_imgs);
}

struct SimDiffs
{
  float max_abs_diff = 0;

  /// \brief The maximum of |a - b| / (abs_tol + (rel_tol * |b|)), which is at
  ///        most 1 when the values are close.
  float max_tol_frac = 0;
};

SimDiffs CompareSims(const ScalarList& a, const ScalarList& b, const float abs_tol, const float rel_tol)
{
  xregASSERT(a.size() == b.size());

  SimDiffs d;

  for (size_type i = 0; i < a.size(); ++i)
  {
    const float diff = std::abs(a[i] - b[i]);

    d.max_abs_diff = std::max(d.max_abs_diff, diff);
    d.max_tol_frac = std::max(d.max_tol_frac, diff / (abs_tol + (rel_tol * std::abs(b[i]))));
  }

  return d;
}

void CheckSimsClose(const std::string& desc, const ScalarList& a, const ScalarList& b,
                    const float abs_tol, const float rel_tol)
{
  const SimDiffs d = CompareSims(a, b, abs_tol, rel_tol);

  const auto minmax = std::minmax_element(b.begin(), b.end());

  std::cout << fmt::format("      {:<34} values: [{:10.4e}, {:10.4e}]  max diff: {:.2e}  "
                           "(fraction of tol.: {:.3f})",
                           desc, *minmax.first, *minmax.second, d.max_abs_diff, d.max_tol_frac)
            << std::endl;

  xregASSERT(d.max_tol_frac <= 1);
}

void CheckSimsEqual(const ScalarList& a, const ScalarList& b)
{
  xregASSERT(a.size() == b.size());

  for (size_type i = 0; i < a.size(); ++i)
  {
    xregASSERT(a[i] == b[i]);
  }
}

/// \brief The moving image with the fixed pose should be the most similar.
void CheckFixedPoseMostSimilar(const ScalarList& sims)
{
  xregASSERT(std::min_element(sims.begin(), sims.end()) == sims.begin());
}

/// \brief Reference SSD values computed on the host in double precision.
///
/// This is the mean of the squared differences over the pixels that are not
/// masked out, which is how the OpenCL implementation is intended to compute
/// SSD. ImgSimMetric2DSSDOCL is not used as a reference, since it does not
/// compute SSD: the image length kernel argument is set from a buffer that has
/// not been allocated yet (and is therefore zero), so the squared differences
/// are never computed and the result is the mean of each moving image.
ScalarList HostSSD(const SimScene& ss, const bool use_mask)
{
  const Scalar* fixed_img = ss.fixed_img->GetBufferPointer();

  const ImgSimMetric2D::MaskScalar* mask = ss.mask->GetBufferPointer();

  ScalarList sims;

  for (size_type mov_idx = 0; mov_idx < ss.num_mov_imgs; ++mov_idx)
  {
    const Scalar* mov_img = ss.mov_imgs.data() + (mov_idx * ss.num_pix());

    double ssd = 0;

    size_type num_pix = 0;

    for (size_type i = 0; i < ss.num_pix(); ++i)
    {
      if (!use_mask || mask[i])
      {
        const double d = static_cast<double>(mov_img[i]) - fixed_img[i];

        ssd += d * d;

        ++num_pix;
      }
    }

    sims.push_back(static_cast<Scalar>(ssd / num_pix));
  }

  return sims;
}

/// \brief The OpenCL objects of a device used for reference computations.
///
/// The similarity metrics of a device share a context and queue, and use a
/// unique ViennaCL context index, since a ViennaCL context index should only
/// be associated with a single OpenCL context.
struct OCLRef
{
  boost::compute::context ctx;

  boost::compute::command_queue queue;

  long vienna_cl_ctx_idx;

  template <class tSim>
  SimPtr make() const
  {
    auto s = std::make_unique<tSim>(ctx, queue);
    s->set_vienna_cl_ctx_idx(vienna_cl_ctx_idx);
    return s;
  }
};

/// \brief A similarity metric configuration and how to create each implementation.
struct MetricCase
{
  std::string desc;

  std::function<SimPtr(const MetalDevice&)> make_metal;

  /// \brief Empty when the OpenCL implementation should not be compared.
  std::function<SimPtr(const OCLRef&)> make_ocl;

  /// \brief Reference values computed on the host, when not empty.
  std::function<ScalarList(const SimScene&, bool)> host_ref;

  /// \brief Empty when the CPU implementation is not equivalent, e.g. when the
  ///        OpenCL and CPU implementations differ.
  std::function<SimPtr()> make_cpu;

  bool use_mask = false;

  ConfigFn config;
};

template <class tMetal, class tOCL, class tCPU = void>
MetricCase MakeCase(const std::string& desc, const bool use_mask, const ConfigFn& config = {})
{
  MetricCase c;

  c.desc     = desc;
  c.use_mask = use_mask;
  c.config   = config;

  c.make_metal = [] (const MetalDevice& dev) -> SimPtr { return std::make_unique<tMetal>(dev); };

  if constexpr (!std::is_void<tOCL>::value)
  {
    c.make_ocl = [] (const OCLRef& ocl) { return ocl.make<tOCL>(); };
  }

  if constexpr (!std::is_void<tCPU>::value)
  {
    c.make_cpu = [] () -> SimPtr { return std::make_unique<tCPU>(); };
  }

  return c;
}

MetricCase WithHostRef(MetricCase c, ScalarList (*host_ref)(const SimScene&, bool))
{
  c.host_ref = host_ref;
  return c;
}

ConfigFn SmoothWidth(const size_type w)
{
  return [w] (ImgSimMetric2D& s)
  {
    dynamic_cast<ImgSimMetric2DGradImgParamInterface&>(s).set_smooth_img_before_sobel_kernel_radius(w);
  };
}

ConfigFn PatchCombine(const bool weight, const bool mean)
{
  return [weight, mean] (ImgSimMetric2D& s)
  {
    auto& p = dynamic_cast<ImgSimMetric2DPatchCommon&>(s);
    p.set_weight_patch_sims_in_combine(weight);
    p.set_compute_mean_of_patch_sims(mean);
  };
}

/// \brief Use every 7th patch (of the default patches: radius 5, stride 1).
ConfigFn PatchSubset(const SimScene& ss)
{
  constexpr size_type kPATCH_RAD = 5;

  const size_type num_patches = (ss.num_rows() - (2 * kPATCH_RAD)) * (ss.num_cols() - (2 * kPATCH_RAD));

  ImgSimMetric2DPatchCommon::PatchIndexList inds;

  for (size_type i = 0; i < num_patches; i += 7)
  {
    inds.push_back(i);
  }

  return [inds] (ImgSimMetric2D& s)
  {
    dynamic_cast<ImgSimMetric2DPatchCommon&>(s).set_patches_to_use(inds);
  };
}

std::vector<MetricCase> MakeCases(const SimScene& ss)
{
  using PatchNCCMtl  = ImgSimMetric2DPatchNCCMetal;
  using PatchNCCOCL  = ImgSimMetric2DPatchNCCOCL;
  using PatchGradMtl = ImgSimMetric2DPatchGradNCCMetal;
  using PatchGradOCL = ImgSimMetric2DPatchGradNCCOCL;

  return {
    WithHostRef(MakeCase<ImgSimMetric2DSSDMetal, void, ImgSimMetric2DSSDCPU>("SSD", false), &HostSSD),
    WithHostRef(MakeCase<ImgSimMetric2DSSDMetal, void>("SSD (mask)", true), &HostSSD),
    MakeCase<ImgSimMetric2DNCCMetal, ImgSimMetric2DNCCOCL, ImgSimMetric2DNCCCPU>("NCC", false),
    MakeCase<ImgSimMetric2DNCCMetal, ImgSimMetric2DNCCOCL, ImgSimMetric2DNCCCPU>("NCC (mask)", true),
    MakeCase<ImgSimMetric2DGradNCCMetal, ImgSimMetric2DGradNCCOCL>("Grad-NCC", false),
    MakeCase<ImgSimMetric2DGradNCCMetal, ImgSimMetric2DGradNCCOCL>("Grad-NCC (mask)", true),
    MakeCase<ImgSimMetric2DGradNCCMetal, ImgSimMetric2DGradNCCOCL>("Grad-NCC (no smoothing)", false,
                                                                   SmoothWidth(0)),
    MakeCase<PatchNCCMtl, PatchNCCOCL, ImgSimMetric2DPatchNCCCPU>("Patch NCC", false),
    MakeCase<PatchNCCMtl, PatchNCCOCL>("Patch NCC (mask)", true),
    MakeCase<PatchNCCMtl, PatchNCCOCL>("Patch NCC (mean, no weights)", true, PatchCombine(false, true)),
    MakeCase<PatchNCCMtl, PatchNCCOCL>("Patch NCC (sum)", false, PatchCombine(false, false)),
    MakeCase<PatchNCCMtl, PatchNCCOCL>("Patch NCC (subset, mask)", true, PatchSubset(ss)),
    MakeCase<PatchGradMtl, PatchGradOCL>("Patch Grad-NCC", false),
    MakeCase<PatchGradMtl, PatchGradOCL>("Patch Grad-NCC (mask)", true),
    MakeCase<PatchGradMtl, PatchGradOCL>("Patch Grad-NCC (subset)", false, PatchSubset(ss))
  };
}

// Tolerances, |metal - ref| <= abs_tol + (rel_tol * |ref|), the values are
// sums over many pixels computed in single precision with different orders of
// operations.
constexpr float kOCL_ABS_TOL = 1.0e-5f;
constexpr float kOCL_REL_TOL = 1.0e-4f;
constexpr float kCPU_ABS_TOL = 1.0e-5f;
constexpr float kCPU_REL_TOL = 1.0e-4f;
constexpr float kHOST_ABS_TOL = 1.0e-5f;
constexpr float kHOST_REL_TOL = 1.0e-4f;

/// \brief Compares each Metal similarity metric with the OpenCL and CPU
///        implementations, and checks variants of the Metal inputs.
void TestMetrics(const MetalDevice& dev, const SimScene& ss, const std::vector<OCLRef>& ocl_refs)
{
  for (const auto& c : MakeCases(ss))
  {
    std::cout << "    " << c.desc << ":" << std::endl;

    SimPtr metal_sim = c.make_metal(dev);

    const ScalarList metal_vals = ComputeSims(*metal_sim, ss, c.use_mask, c.config);

    CheckFixedPoseMostSimilar(metal_vals);

    if (c.host_ref)
    {
      CheckSimsClose("vs host reference", metal_vals, c.host_ref(ss, c.use_mask),
                     kHOST_ABS_TOL, kHOST_REL_TOL);
    }

    for (const auto& ocl : c.make_ocl ? ocl_refs : std::vector<OCLRef>())
    {
      SimPtr ocl_sim = c.make_ocl(ocl);

      CheckSimsClose("vs OpenCL (" + ocl.queue.get_device().name() + ")", metal_vals,
                     ComputeSims(*ocl_sim, ss, c.use_mask, c.config),
                     kOCL_ABS_TOL, kOCL_REL_TOL);
    }

    if (c.make_cpu)
    {
      SimPtr cpu_sim = c.make_cpu();

      CheckSimsClose("vs CPU", metal_vals, ComputeSims(*cpu_sim, ss, c.use_mask, c.config),
                     kCPU_ABS_TOL, kCPU_REL_TOL);
    }

    // The same computations with a projection offset and the fixed image on the
    // device, which should not be modified
    {
      const Scalar* fixed_host = ss.fixed_img->GetBufferPointer();

      MetalCmdQueue queue(dev);

      auto fixed_dev = std::make_shared<MetalDevBuf>(dev, ss.num_pix());
      CopyHostToMetal(fixed_host, fixed_host + ss.num_pix(), *fixed_dev, 0, queue);

      SimPtr metal_sim2 = c.make_metal(dev);

      CheckSimsEqual(ComputeSims(*metal_sim2, ss, c.use_mask, c.config, 3, fixed_dev), metal_vals);

      PixelBuf fixed_after(ss.num_pix());
      CopyMetalToHost(*fixed_dev, 0, ss.num_pix(), fixed_after.data(), queue);

      xregASSERT(std::equal(fixed_after.begin(), fixed_after.end(), fixed_host));

      // fewer moving images than allocated, as when re-using a similarity metric
      const size_type num_mov_imgs_subset = ss.num_mov_imgs / 2;

      metal_sim2->set_num_moving_images(num_mov_imgs_subset);
      metal_sim2->compute();

      CheckSimsEqual(ScalarList(metal_sim2->sim_vals().begin(),
                                metal_sim2->sim_vals().begin() + num_mov_imgs_subset),
                     ScalarList(metal_vals.begin(), metal_vals.begin() + num_mov_imgs_subset));

      std::cout << "      projection offset, fixed image on device, fewer moving images: equal" << std::endl;
    }
  }
}

/// \brief Each moving image's similarity, computed with all of the moving images,
///        is identical to its similarity computed alone.
///
/// This does not depend on other implementations, e.g. it checks that each
/// moving image is processed independently and its result is stored in the
/// correct location. The computations for a moving image do not depend on the
/// number of moving images, so the values should be equal.
void TestEachMovImgAlone(const MetalDevice& dev, const SimScene& ss,
                         const std::vector<size_type>& mov_inds)
{
  for (const auto& c : MakeCases(ss))
  {
    SimPtr all_sim = c.make_metal(dev);

    const ScalarList all_vals = ComputeSims(*all_sim, ss, c.use_mask, c.config);

    // the moving images should not all have the same similarity
    xregASSERT(*std::min_element(all_vals.begin(), all_vals.end()) <
               *std::max_element(all_vals.begin(), all_vals.end()));

    for (const size_type mov_idx : mov_inds)
    {
      SimScene single_ss = ss;

      single_ss.num_mov_imgs = 1;
      single_ss.mov_imgs.assign(ss.mov_imgs.begin() + (mov_idx * ss.num_pix()),
                                ss.mov_imgs.begin() + ((mov_idx + 1) * ss.num_pix()));

      SimPtr single_sim = c.make_metal(dev);

      CheckSimsEqual(ComputeSims(*single_sim, single_ss, c.use_mask, c.config),
                     ScalarList(1, all_vals[mov_idx]));
    }
  }

  std::cout << fmt::format("    similarities of {} moving images computed alone are equal to "
                           "those computed with all {} moving images",
                           mov_inds.size(), ss.num_mov_imgs) << std::endl;
}

/// \brief Similarity metrics using the projection buffer of a Metal ray caster.
void TestRayCasterBuf(const MetalDevice& dev, const SimScene& ss)
{
  RayCasterLineIntMetal rc(dev);
  SetupRayCaster(rc, ss.scene, 1);
  rc.compute();

  // the first projection is rendered with the fixed pose, use the ray caster's
  // projections, which differ slightly from the CPU projections
  SimScene metal_ss = ss;

  const PixelBuf projs = GetProjs(rc, ss.scene.poses.size());

  metal_ss.fixed_img = MakeImage(projs.data(), ss.num_rows(), ss.num_cols());
  metal_ss.mov_imgs.assign(projs.begin() + ss.num_pix(), projs.end());

  for (const auto& c : MakeCases(ss))
  {
    SimPtr host_sim = c.make_metal(dev);

    const ScalarList host_vals = ComputeSims(*host_sim, metal_ss, c.use_mask, c.config);

    SimPtr rc_sim = c.make_metal(dev);

    rc_sim->set_num_moving_images(ss.num_mov_imgs);
    rc_sim->set_fixed_image(CopyImage(metal_ss.fixed_img));

    if (c.use_mask)
    {
      rc_sim->set_mask(ss.mask);
    }

    if (c.config)
    {
      c.config(*rc_sim);
    }

    // skip the fixed projection
    rc_sim->set_mov_imgs_buf_from_ray_caster(&rc, 1);

    rc_sim->allocate_resources();
    rc_sim->compute();

    CheckSimsEqual(ScalarList(rc_sim->sim_vals().begin(), rc_sim->sim_vals().begin() + ss.num_mov_imgs),
                   host_vals);
  }

  std::cout << "    ray caster buffer and host buffer similarities are equal" << std::endl;
}

template <class tMetalSim>
void CheckProgOpts(ProgOpts& po, const MetalDevice& dev,
                   std::shared_ptr<ImgSimMetric2D> (*from_po)(ProgOpts&))
{
  auto sim = from_po(po);

  auto* metal_sim = dynamic_cast<tMetalSim*>(sim.get());
  xregASSERT(metal_sim);
  xregASSERT(metal_sim->device().registry_id() == dev.registry_id());
}

void TestProgOpts(const MetalDevice& dev)
{
  auto po = ParseArgs({ "--backend", "metal", "--metal-id", dev.id_str() });

  CheckProgOpts<ImgSimMetric2DSSDMetal>(*po, dev, &SSDSimMetricFromProgOpts);
  CheckProgOpts<ImgSimMetric2DNCCMetal>(*po, dev, &NCCSimMetricFromProgOpts);
  CheckProgOpts<ImgSimMetric2DGradNCCMetal>(*po, dev, &GradNCCSimMetricFromProgOpts);
  CheckProgOpts<ImgSimMetric2DPatchNCCMetal>(*po, dev, &PatchNCCSimMetricFromProgOpts);
  CheckProgOpts<ImgSimMetric2DPatchGradNCCMetal>(*po, dev, &PatchGradNCCSimMetricFromProgOpts);

  std::cout << "    *SimMetricFromProgOpts() created Metal similarity metrics" << std::endl;
}

/// \brief Average time of compute(), after a warm up call.
double TimeCompute(ImgSimMetric2D& sim, const size_type num_runs = 5)
{
  using Clock = std::chrono::steady_clock;

  sim.compute();

  const auto start = Clock::now();

  for (size_type i = 0; i < num_runs; ++i)
  {
    sim.compute();
  }

  return std::chrono::duration<double,std::milli>(Clock::now() - start).count() / num_runs;
}

void PrintTimings(const MetalDevice& dev, const SimScene& ss, const std::vector<OCLRef>& ocl_refs)
{
  std::cout << fmt::format("    {} moving images of {}x{}:", ss.num_mov_imgs, ss.num_cols(), ss.num_rows())
            << std::endl;

  for (const auto& c : MakeCases(ss))
  {
    if (c.use_mask || c.config)
    {
      // only the default configurations
      continue;
    }

    SimPtr metal_sim = c.make_metal(dev);
    ComputeSims(*metal_sim, ss, false, {});

    std::string line = fmt::format("      {:<16} Metal: {:8.2f} ms", c.desc, TimeCompute(*metal_sim));

    for (const auto& ocl : c.make_ocl ? ocl_refs : std::vector<OCLRef>())
    {
      SimPtr ocl_sim = c.make_ocl(ocl);
      ComputeSims(*ocl_sim, ss, false, {});

      line += fmt::format("  OpenCL: {:8.2f} ms", TimeCompute(*ocl_sim));
    }

    std::cout << line << std::endl;
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

  // the dimensions are not multiples of the threadgroup sizes
  const SimScene ss        = MakeSimScene(8, 120, 150);
  const SimScene timing_ss = MakeSimScene(50, 256, 256);

  long vienna_cl_ctx_idx = 1;

  for (const auto& dev : MetalAllDevices())
  {
    std::cout << "Metal device: " << dev.id_str() << std::endl;

    std::vector<OCLRef> ocl_refs;

    if (have_ocl)
    {
      for (const auto& ocl_dev : OpenCLDevicesMatching(dev))
      {
        boost::compute::context ctx(ocl_dev);

        ocl_refs.push_back({ ctx, boost::compute::command_queue(ctx, ocl_dev), vienna_cl_ctx_idx++ });
      }
    }

    std::cout << "  similarity metrics:" << std::endl;
    TestMetrics(dev, ss, ocl_refs);

    std::cout << "  each moving image alone:" << std::endl;
    {
      std::vector<size_type> all_mov_inds(ss.num_mov_imgs);
      std::iota(all_mov_inds.begin(), all_mov_inds.end(), size_type(0));

      TestEachMovImgAlone(dev, ss, all_mov_inds);
    }

    std::cout << "  ray caster buffer:" << std::endl;
    TestRayCasterBuf(dev, ss);

    std::cout << "  program options:" << std::endl;
    TestProgOpts(dev);

    std::cout << "  timings:" << std::endl;
    PrintTimings(dev, timing_ss, ocl_refs);
  }

  std::cout << "PASSED" << std::endl;

  return 0;
}
