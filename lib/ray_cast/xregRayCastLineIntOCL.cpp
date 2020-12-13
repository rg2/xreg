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

#include "xregRayCastLineIntOCL.h"

#include <boost/compute/utility/source.hpp>

#include "xregExceptionUtils.h"
#include "xregAssert.h"

namespace  // un-named
{

//////////////////////////////////////////////////////////////////////

const char* kRAY_CASTING_LINE_INT_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void xregLineIntegralKernel(const RayCastArgs args,
                                     __global const float4* det_pts,
                                     image3d_t vol_tex,
                                     __global float* dst_line_integral_sums,
                                     __global const float16* cam_to_itk_phys_xforms,
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

    const float16 xform_cam_to_itk_idx = xregFrm4x4Composition(args.itk_phys_pt_to_itk_idx_xform,
                                                               cam_to_itk_phys_xforms[proj_idx]);

    const float4 pinhole_wrt_itk_idx = xregFrm4x4XformFloat4Pt(xform_cam_to_itk_idx, focal_pt_wrt_cam);

    const float4 cur_det_pt_wrt_cam = det_pts[(cam_idx * args.num_det_pts) + det_pt_idx];

    const float4 pinhole_to_det_wrt_itk_idx = xregFrm4x4XformFloat4Pt(xform_cam_to_itk_idx, cur_det_pt_wrt_cam)
                                                - pinhole_wrt_itk_idx;

    // check intersection
    float2 t = xregLineSegmentRectIntersect(xregFloat4HmgToFloat3(args.img_aabb_min),
                                            xregFloat4HmgToFloat3(args.img_aabb_max),
                                            xregFloat4HmgToFloat3(pinhole_wrt_itk_idx),
                                            xregFloat4HmgToFloat3(pinhole_to_det_wrt_itk_idx));
    // t.x indicates the first intersection with the volume along the source to detector line,
    // t.y indicates the exit of the volume along the source to detector line

    // a float4 is returned from read_imagef, but for our image all components
    // are equal
    float4 dst_val = XREG_LINE_INT_KERNEL_INIT;

    const float4 start_pt_wrt_itk_idx = pinhole_wrt_itk_idx + (t.x * pinhole_to_det_wrt_itk_idx);

    const float pinhole_to_det_len_wrt_itk_idx  = xregFloat4HmgNorm(pinhole_to_det_wrt_itk_idx);
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

    const float4 step_vec_wrt_itk_idx = pinhole_to_det_wrt_itk_idx * scale_to_step;

    for (ulong step_idx = 0; step_idx <= num_steps; ++step_idx)
    {
      dst_val = XREG_LINE_INT_KERNEL_OP(dst_val, read_imagef(vol_tex, sampler, cur_cont_vol_idx));

      cur_cont_vol_idx += step_vec_wrt_itk_idx;
    }

    dst_line_integral_sums[idx] = XREG_LINE_INT_KERNEL_OP(dst_val.x, dst_line_integral_sums[idx]);
  }
}

);

//////////////////////////////////////////////////////////////////////

}  // un-named

xreg::RayCasterLineIntOCL::RayCasterLineIntOCL(const boost::compute::device& dev)
  : RayCasterOCL(dev)
{ }

xreg::RayCasterLineIntOCL::RayCasterLineIntOCL(const boost::compute::context& ctx,
                                               const boost::compute::command_queue& queue)
  : RayCasterOCL(ctx, queue)
{ }

void xreg::RayCasterLineIntOCL::allocate_resources()
{
  namespace bc = boost::compute;

  RayCasterOCL::allocate_resources();
  
  std::string kernel_op_ocl_src;
  
  switch (this->kernel_id())
  {
    case kRAY_CAST_LINE_INT_SUM_KERNEL:
      kernel_op_ocl_src = "\n\n#define XREG_LINE_INT_KERNEL_INIT 0\n"
                          "#define XREG_LINE_INT_KERNEL_OP(X,Y) (X) + (Y)\n\n";
      break;
    case kRAY_CAST_LINE_INT_MAX_KERNEL:
      kernel_op_ocl_src = "\n\n#define XREG_LINE_INT_KERNEL_INIT -FLT_MAX\n"
                          "#define XREG_LINE_INT_KERNEL_OP(X,Y) max((X),(Y))\n\n";
      break;
    default:
      xregThrow("Unsupported Line Integral Kernel!");
  }

  // Build the line integral ray casting program
  std::stringstream ss;
  ss << RayCastBaseOCLStr()
     << kernel_op_ocl_src
     << kRAY_CASTING_LINE_INT_OPENCL_SRC;

  bc::program prog = bc::program::create_with_source(ss.str(), ctx_);
  prog.build();

  dev_kernel_ = prog.create_kernel("xregLineIntegralKernel");
}

void xreg::RayCasterLineIntOCL::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);

  compute_helper_pre_kernels(vol_idx);

  // setup kernel arguments and launch

  dev_kernel_.set_arg(0, sizeof(ray_cast_kernel_args_), &ray_cast_kernel_args_);

  dev_kernel_.set_arg(1, det_pts_dev_);

  dev_kernel_.set_arg(2, vol_texs_dev_[vol_idx]);

  dev_kernel_.set_arg(3, *proj_pixels_dev_to_use_);

  dev_kernel_.set_arg(4, cam_to_itk_phys_xforms_dev_);

  dev_kernel_.set_arg(5, cam_model_for_proj_dev_);

  dev_kernel_.set_arg(6, focal_pts_dev_);

  std::size_t global_work_size = ray_cast_kernel_args_.num_det_pts * this->num_projs_;

  cmd_queue_.enqueue_nd_range_kernel(dev_kernel_,
                                     1, // dim
                                     0, // null offset -> start at 0
                                     &global_work_size,
                                     0  // passing null lets open CL pick a local size
                                    ).wait();

  compute_helper_post_kernels(vol_idx);
}

