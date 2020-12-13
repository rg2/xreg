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

#include "xregRayCastSurRenderOCL.h"

#include <boost/compute/utility/source.hpp>
#include <boost/compute/types/struct.hpp>

#include "xregAssert.h"

//////////////////////////////////////////////////////////////////////

BOOST_COMPUTE_ADAPT_STRUCT(xreg::RayCasterSurRenderOCL::RayCastSurRenderArgs, RayCastSurRenderArgs,
                           (thresh,
                            pad1,
                            num_backtracking_steps,
                            ambient_reflection_ratio,
                            diffuse_reflection_ratio,
                            specular_reflection_ratio,
                            alpha_shininess))

//////////////////////////////////////////////////////////////////////

namespace  // un-named
{

const char* kRAY_CASTING_SUR_RENDER_OPENCL_SRC = BOOST_COMPUTE_STRINGIZE_SOURCE(

__kernel void xregSurRenderKernel1(const RayCastArgs args,
                                   const RayCastSurRenderArgs sur_render_args,
                                   __global const float4* det_pts,
                                   image3d_t vol_tex,
                                   __global const float16* cam_to_itk_phys_xforms,
                                   __global float8* dst_step_vecs_and_intersect_pts_wrt_itk_idx,
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

    // "/ pinhole_to_det_len_wrt_itk_idx" makes pinhole_to_det_wrt_itk_idx a unit vector
    const float scale_to_step = step_len_wrt_itk_idx / pinhole_to_det_len_wrt_itk_idx;

    const float4 step_vec_wrt_itk_idx = (float4) (pinhole_to_det_wrt_itk_idx.x * scale_to_step, pinhole_to_det_wrt_itk_idx.y * scale_to_step, pinhole_to_det_wrt_itk_idx.z * scale_to_step, 0);

    // The first three elements will store the step direction in ITK index space
    dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s0 = step_vec_wrt_itk_idx.x;
    dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s1 = step_vec_wrt_itk_idx.y;
    dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s2 = step_vec_wrt_itk_idx.z;

    // The fourth element will indicate whether this ray intersected the surface.
    dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s3 = 0;  //  0 -> no intersection found

    for (ulong step_idx = 0; step_idx <= num_steps; ++step_idx, cur_cont_vol_idx += step_vec_wrt_itk_idx)
    {
      if (read_imagef(vol_tex, sampler, cur_cont_vol_idx + tex_coords_off).x >= sur_render_args.thresh)
      {
        // The 5th, 6th, and 7th elements will store the initial point of intersection, before any backtracking
        dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s4 = cur_cont_vol_idx.x;
        dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s5 = cur_cont_vol_idx.y;
        dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s6 = cur_cont_vol_idx.z;
        dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s7 = 0;

        dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s3 = 1;  // 1 -> intersection found
        break;
      }
    }
  }
}

__kernel void xregSurRenderKernel2(const RayCastArgs args,
                                   const RayCastSurRenderArgs sur_render_args,
                                   image3d_t vol_tex,
                                   __global const float8* dst_step_vecs_and_intersect_pts_wrt_itk_idx,
                                   __global float* dst_intensities)
{
  const ulong idx = get_global_id(0);

  const ulong num_rays = args.num_projs * args.num_det_pts;

  if (idx < num_rays)
  {
    if (dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s3 > 1.0e-6)
    {
      const sampler_t sampler = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP | CLK_FILTER_LINEAR;
      
      const float4 tex_coords_off = (float4) (0.5f, 0.5f, 0.5f, 0);

      float3 light_src_vec = (float3) (dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s0, dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s1, dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s2);
      // right now, this light source vector points from the light source, it needs to be negated, and it is when normalizing a few lines below

      // This vector will always point towards the pinhole.
      float4 step_vec_wrt_itk_idx = (float4) (light_src_vec.x, light_src_vec.y, light_src_vec.z, 0);
      step_vec_wrt_itk_idx *= 0.5f;

      // normalize the direction vector to the light source, and negate to get the right orientation
      light_src_vec /= -xregFloat3Norm(light_src_vec);

      float4 cur_cont_vol_idx = (float4) (dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s4, dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s5, dst_step_vecs_and_intersect_pts_wrt_itk_idx[idx].s6, 0);
      
      cur_cont_vol_idx += tex_coords_off;

      // we know that we should start out backtracking.
      cur_cont_vol_idx -= step_vec_wrt_itk_idx;

      for (ulong backtrack_idx = 0; backtrack_idx < sur_render_args.num_backtracking_steps; ++backtrack_idx)
      {
        step_vec_wrt_itk_idx *= 0.5f;

        cur_cont_vol_idx += (read_imagef(vol_tex, sampler, cur_cont_vol_idx).x >= sur_render_args.thresh) ?
                                  step_vec_wrt_itk_idx : -step_vec_wrt_itk_idx;
      }

      // compute lighting/shading according to the phong model

      // approximate the derivative at this point with finite differencing adjacent indices
      // NOTE: these are wrt image (index) axes.

      float3 tmp_vec;

      // Gradient in X Direction:
      tmp_vec.x = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x + 1, cur_cont_vol_idx.y, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x -
                  read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x - 1, cur_cont_vol_idx.y, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x;

      // Gradient in Y Direction:
      tmp_vec.y = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y + 1, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x -
                  read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y - 1, cur_cont_vol_idx.z, cur_cont_vol_idx.w)).x;

      // Gradient in Z Direction:
      tmp_vec.z = read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y, cur_cont_vol_idx.z + 1, cur_cont_vol_idx.w)).x -
                  read_imagef(vol_tex, sampler, (float4) (cur_cont_vol_idx.x, cur_cont_vol_idx.y, cur_cont_vol_idx.z - 1, cur_cont_vol_idx.w)).x;

      tmp_vec /= (-0.5f * xregFloat3Norm(tmp_vec));

      float val = sur_render_args.ambient_reflection_ratio;

      // Treating the pinhole as the light source

      float tmp_dot = xregFloat3Inner(light_src_vec, tmp_vec);
      const int diffuse_valid = tmp_dot > 1.0e-6;

      val += diffuse_valid ? (sur_render_args.diffuse_reflection_ratio * tmp_dot) : 0;

      tmp_vec *= 2 * tmp_dot;
      tmp_vec -= light_src_vec;

      tmp_dot = xregFloat3Inner(light_src_vec, tmp_vec);

      val += (tmp_dot > 1.0e-6) ?
              (sur_render_args.specular_reflection_ratio * pow(tmp_dot, sur_render_args.alpha_shininess)) : 0;

      dst_intensities[idx] += val;
    }
  }
}

);

//////////////////////////////////////////////////////////////////////

}  // un-named

xreg::RayCasterSurRenderOCL::RayCasterSurRenderOCL()
  : RayCasterOCL(), step_vecs_and_intersect_pts_wrt_itk_idx_dev_(ctx_)
{ }

xreg::RayCasterSurRenderOCL::RayCasterSurRenderOCL(const boost::compute::device& dev)
  : RayCasterOCL(dev), step_vecs_and_intersect_pts_wrt_itk_idx_dev_(ctx_)
{ }

xreg::RayCasterSurRenderOCL::RayCasterSurRenderOCL(const boost::compute::context& ctx,
                                                   const boost::compute::command_queue& queue)
  : RayCasterOCL(ctx, queue), step_vecs_and_intersect_pts_wrt_itk_idx_dev_(ctx_)
{ }

void xreg::RayCasterSurRenderOCL::allocate_resources()
{
  namespace bc = boost::compute;

  RayCasterOCL::allocate_resources();

  // Build the surface rendering ray casting program
  std::stringstream ss;
  ss << RayCastBaseOCLStr()
     << boost::compute::type_definition<RayCasterSurRenderOCL::RayCastSurRenderArgs>()
     << kRAY_CASTING_SUR_RENDER_OPENCL_SRC;

  bc::program prog = bc::program::create_with_source(ss.str(), ctx_);
  prog.build();

  dev_kernel1_ = prog.create_kernel("xregSurRenderKernel1");

  dev_kernel2_ = prog.create_kernel("xregSurRenderKernel2");

  step_vecs_and_intersect_pts_wrt_itk_idx_dev_.resize(this->camera_models_[0].num_det_rows *
                                                      this->camera_models_[0].num_det_cols *
                                                      this->num_projs_,
                                                      cmd_queue_);
}

void xreg::RayCasterSurRenderOCL::compute(const size_type vol_idx)
{
  xregASSERT(this->resources_allocated_);
  
  compute_helper_pre_kernels(vol_idx);

  RayCastSurRenderArgs sur_args;
  
  sur_args.thresh = render_thresh();
  
  sur_args.num_backtracking_steps = num_backtracking_steps();
  
  sur_args.ambient_reflection_ratio =
                  surface_render_params().ambient_reflection_ratio;
  
  sur_args.diffuse_reflection_ratio =
                  surface_render_params().diffuse_reflection_ratio;
  
  sur_args.specular_reflection_ratio =
                  surface_render_params().specular_reflection_ratio;
  
  sur_args.alpha_shininess = surface_render_params().alpha_shininess;

  // setup kernel arguments and launch

  dev_kernel1_.set_arg(0, sizeof(ray_cast_kernel_args_), &ray_cast_kernel_args_);

  dev_kernel1_.set_arg(1, sizeof(sur_args), &sur_args);

  dev_kernel1_.set_arg(2, det_pts_dev_);

  dev_kernel1_.set_arg(3, vol_texs_dev_[vol_idx]);

  dev_kernel1_.set_arg(4, cam_to_itk_phys_xforms_dev_);

  dev_kernel1_.set_arg(5, step_vecs_and_intersect_pts_wrt_itk_idx_dev_);

  dev_kernel1_.set_arg(6, cam_model_for_proj_dev_);

  dev_kernel1_.set_arg(7, focal_pts_dev_);

  std::size_t global_work_size = ray_cast_kernel_args_.num_det_pts * this->num_projs_;
  //std::size_t local_work_size = 512;  passing null lets open CL pick a local size

  cmd_queue_.enqueue_nd_range_kernel(dev_kernel1_,
                                     1, // dim
                                     0, // null offset -> start at 0
                                     &global_work_size,
                                     0  // passing null lets open CL pick a local size
                                    ).wait();


  dev_kernel2_.set_arg(0, sizeof(ray_cast_kernel_args_), &ray_cast_kernel_args_);

  dev_kernel2_.set_arg(1, sizeof(sur_args), &sur_args);

  dev_kernel2_.set_arg(2, vol_texs_dev_[vol_idx]);

  dev_kernel2_.set_arg(3, step_vecs_and_intersect_pts_wrt_itk_idx_dev_);

  dev_kernel2_.set_arg(4, *proj_pixels_dev_to_use_);

  cmd_queue_.enqueue_nd_range_kernel(dev_kernel2_,
                                     1, // dim
                                     0, // null offset -> start at 0
                                     &global_work_size,
                                     0  // passing null lets open CL pick a local size
                                    ).wait();

  compute_helper_post_kernels(vol_idx);
}

