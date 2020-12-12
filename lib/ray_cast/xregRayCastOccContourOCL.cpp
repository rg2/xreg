/*
 * MIT License
 *
 * Copyright (c) 2020 Robert Grupp
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

#include "xregRayCastOccContourOCL.h"

#include <boost/compute/utility/source.hpp>
#include <boost/compute/types/struct.hpp>

#include "xregAssert.h"
#include "xregExceptionUtils.h"

//////////////////////////////////////////////////////////////////////

struct ContourSurCollArgs
{
  float sur_coll_thresh;
  float occluding_ang_thresh_rad;
};

BOOST_COMPUTE_ADAPT_STRUCT(ContourSurCollArgs, ContourSurCollArgs,
                           (sur_coll_thresh, occluding_ang_thresh_rad))

//////////////////////////////////////////////////////////////////////

namespace  // un-named
{

const char* kRAY_CASTING_OCCLUDING_CONTOUR_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void xregOccludingContourKernel(const RayCastArgs args,
                                         const ContourSurCollArgs sur_coll_args,
                                         __global const float4* det_pts,
                                         image3d_t vol_tex,
                                         __global const float16* cam_to_itk_phys_xforms,
                                         __global float* dst_contours_buf,
                                         __global const ulong* cam_model_for_proj,
                                         __global const float4* cam_focal_pts)
{
  const ulong idx = get_global_id(0);

  const ulong num_rays = args.num_projs * args.num_det_pts;

  if (idx < num_rays)
  {
    const ulong proj_idx   = idx / args.num_det_pts;
    const ulong det_pt_idx = idx - (proj_idx * args.num_det_pts);
    const ulong cam_idx    = cam_model_for_proj[proj_idx];

    const float4 focal_pt_wrt_cam = cam_focal_pts[cam_idx];

    const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR;
    
    const float4 tex_coords_off = (float4) (0.5f, 0.5f, 0.5f, 0);

    const float16 xform_cam_to_itk_idx = xregFrm4x4Composition(args.itk_phys_pt_to_itk_idx_xform, cam_to_itk_phys_xforms[proj_idx]);

    const float4 pinhole_wrt_itk_idx = xregFrm4x4XformFloat4Pt(xform_cam_to_itk_idx, focal_pt_wrt_cam);

    const float4 cur_det_pt_wrt_cam = det_pts[(cam_idx * args.num_det_pts) + det_pt_idx];

    const float4 pinhole_to_det_wrt_itk_idx = xregFrm4x4XformFloat4Pt(xform_cam_to_itk_idx, cur_det_pt_wrt_cam) - pinhole_wrt_itk_idx;

    // check intersection with the image/volume boundary
    // these pointer were passed as float4's since boost::compute does not have a float3
    float2 t = xregRayRectIntersect(xregFloat4HmgToFloat3(args.img_aabb_min),
                                    xregFloat4HmgToFloat3(args.img_aabb_max),
                                    xregFloat4HmgToFloat3(pinhole_wrt_itk_idx),
                                    xregFloat4HmgToFloat3(pinhole_to_det_wrt_itk_idx));
    // t.x indicates the first intersection with the volume along the source to detector ray,
    // t.y indicates the exit of the volume along the source to detector ray
    // t.x, t.y are in [0,inf)

    const float4 start_pt_wrt_itk_idx = pinhole_wrt_itk_idx + (t.x * pinhole_to_det_wrt_itk_idx);

    const float pinhole_to_det_len_wrt_itk_idx = xregFloat4HmgNorm(pinhole_to_det_wrt_itk_idx);
    const float intersect_len_wrt_itk_idx = (t.y - t.x) * pinhole_to_det_len_wrt_itk_idx;

    const float4 focal_pt_to_det_wrt_cam = cur_det_pt_wrt_cam - focal_pt_wrt_cam;
    // NOTE: since we are transforming a vector with a scaling transform (points to indices),
    //       the output vector may not have the same norm, thus we take the norm
    const float step_len_wrt_itk_idx = xregFloat4HmgNorm(
                                          xregFrm4x4XformFloat4Vec(xform_cam_to_itk_idx,
                                                                   (focal_pt_to_det_wrt_cam / xregFloat4HmgNorm(focal_pt_to_det_wrt_cam)) * args.step_size));

    const ulong num_steps = (ulong)(intersect_len_wrt_itk_idx / step_len_wrt_itk_idx);

    float4 cur_cont_vol_idx = (float4) (start_pt_wrt_itk_idx.x, start_pt_wrt_itk_idx.y, start_pt_wrt_itk_idx.z, 0);

    cur_cont_vol_idx += tex_coords_off;

    // "/ pinhole_to_det_len_wrt_itk_idx" makes pinhole_to_det_wrt_itk_idx a unit vector
    const float scale_to_step = step_len_wrt_itk_idx / pinhole_to_det_len_wrt_itk_idx;

    const float4 step_vec_wrt_itk_idx = (float4) (pinhole_to_det_wrt_itk_idx.x * scale_to_step, pinhole_to_det_wrt_itk_idx.y * scale_to_step, pinhole_to_det_wrt_itk_idx.z * scale_to_step, 0);

    int is_edge = 0;

    for (ulong step_idx = 0; step_idx <= num_steps; ++step_idx, cur_cont_vol_idx += step_vec_wrt_itk_idx)
    {
      if (read_imagef(vol_tex, sampler, cur_cont_vol_idx).x >= sur_coll_args.sur_coll_thresh)
      {
        // approximate the derivative at this point with finite differencing adjacent indices
        // NOTE: these are wrt image (index) axes.

        float3 grad_vec;

        // Gradient in X Direction:
        grad_vec.x = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x + 1, cur_cont_vol_idx.y, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x -
                     read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x - 1, cur_cont_vol_idx.y, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x;

        // Gradient in Y Direction:
        grad_vec.y = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y + 1, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x -
                     read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y - 1, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x;

        // Gradient in Z Direction:
        grad_vec.z = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y, cur_cont_vol_idx.z + 1, cur_cont_vol_idx.w)).x -
                     read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y, cur_cont_vol_idx.z - 1, cur_cont_vol_idx.w)).x;

        grad_vec /= xregFloat3Norm(grad_vec);

        // 1.5707963267948966 == pi/2
        is_edge = fabs(acos(xregFloat3Inner(grad_vec,
                     xregFloat3Normalize(xregFloat4HmgToFloat3(pinhole_to_det_wrt_itk_idx))))
                                            - 1.5707963267948966f) < sur_coll_args.occluding_ang_thresh_rad;

        break;
      }
    }

    dst_contours_buf[idx] += is_edge ? 1 : 0;
  }  // if (idx < num_rays)
}

);

//////////////////////////////////////////////////////////////////////

}  // un-named

xreg::RayCasterOccludingContoursOCL::RayCasterOccludingContoursOCL(const boost::compute::device& dev)
  : RayCasterOCL(dev)
{ }

xreg::RayCasterOccludingContoursOCL::RayCasterOccludingContoursOCL(const boost::compute::context& ctx,
                                                                   const boost::compute::command_queue& queue)
  : RayCasterOCL(ctx, queue)
{ }


void xreg::RayCasterOccludingContoursOCL::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  // setup kernel arguments and launch
  ContourSurCollArgs coll_args = { this->render_thresh(), this->occlusion_angle_thresh_rad() };

  dev_kernel_.set_arg(0, sizeof(ray_cast_kernel_args_), &ray_cast_kernel_args_);
  
  dev_kernel_.set_arg(1, sizeof(coll_args), &coll_args);

  dev_kernel_.set_arg(2, det_pts_dev_);

  dev_kernel_.set_arg(3, vol_texs_dev_[vol_idx]);

  dev_kernel_.set_arg(4, cam_to_itk_phys_xforms_dev_);
  
  dev_kernel_.set_arg(5, *proj_pixels_dev_to_use_);

  dev_kernel_.set_arg(6, cam_model_for_proj_dev_);

  dev_kernel_.set_arg(7, focal_pts_dev_);

  std::size_t global_work_size = ray_cast_kernel_args_.num_det_pts * this->num_projs_;

  cmd_queue_.enqueue_nd_range_kernel(dev_kernel_,
                                     1, // dim
                                     0, // null offset -> start at 0
                                     &global_work_size,
                                     0  // passing null lets open CL pick a local size
                                    ).wait();

  compute_helper_post_kernels(vol_idx);
}

void xreg::RayCasterOccludingContoursOCL::allocate_resources()
{
  namespace bc = boost::compute;

  if (this->num_backtracking_steps() > 0)
  {
    xregThrow("For GPU Occluding Contours - Backtracking is NOT supported!");
  }

  if (!this->stop_after_collision())
  {
    xregThrow("For GPU Occluding Contours - Continuing after collision is NOT supported!");
  }

  RayCasterOCL::allocate_resources();

  // Build the surface rendering ray casting program
  std::stringstream ss;
  ss << RayCastBaseOCLStr()
     << boost::compute::type_definition<ContourSurCollArgs>()
     << kRAY_CASTING_OCCLUDING_CONTOUR_OPENCL_SRC;

  bc::program prog = bc::program::create_with_source(ss.str(), ctx_);

  try
  {
    prog.build();
  }
  catch (bc::opencl_error &)
  {
    std::cerr << "OpenCL Kernel Compile Error (RayCasterOccludingContoursOCL):\n"
              << prog.build_log() << std::endl;
    throw;
  }

  dev_kernel_ = prog.create_kernel("xregOccludingContourKernel");
}

