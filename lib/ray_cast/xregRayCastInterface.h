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

#ifndef XREGRAYCASTINTERFACE_H_
#define XREGRAYCASTINTERFACE_H_

#include <opencv2/core/core.hpp>

#include "xregPerspectiveXform.h"

namespace xreg
{

// Forward Declarations
class RayCastSyncHostBuf;
class RayCastSyncOCLBuf;

/// \brief Parent class for a perspective camera ray caster.
///
/// This can be used to model a traditional optical camera or something like
/// an X-ray imager.
class RayCaster
{
public:
  using PixelScalar2D = RayCastPixelScalar;
  using PixelScalar3D = RayCastPixelScalar;
  using Vol           = itk::Image<PixelScalar3D,3>;
  using VolPtr        = Vol::Pointer;
  using VolList       = std::vector<VolPtr>;
  using Proj          = itk::Image<PixelScalar2D,2>;
  using ProjPtr       = Proj::Pointer;
  using ProjList      = std::vector<ProjPtr>;

  using CameraModelList = std::vector<CameraModel>;

  using CamModelAssocList = std::vector<size_type>;

  class UnsupportedOperationException { };

  enum InterpMethod
  {
    kRAY_CAST_INTERP_LINEAR = 0,
    kRAY_CAST_INTERP_NN,
    kRAY_CAST_INTERP_SINC,
    kRAY_CAST_INTERP_BSPLINE
  };

  /// \brief Used to indicate how ray casted values should be inserted into the
  ///        projection.
  enum ProjPixelStoreMethod
  {
    kRAY_CAST_PIXEL_REPLACE = 0,  ///< Replace existing pixels
    kRAY_CAST_PIXEL_ACCUM         ///< Accumulate with existing pixels
  };

  RayCaster() = default;

  /// \brief Destructor
  virtual ~RayCaster() = default;

  // no copying
  RayCaster(const RayCaster&)            = delete;
  RayCaster& operator=(const RayCaster&) = delete;

  /// \brief Sets a single 3D volume to be used
  void set_volume(VolPtr img_vol);

  /// \brief Sets a collection of 3D volumes that may be ray casted
  void set_volumes(const VolList& vols);

  /// \brief Retrieves the number of 3D volumes that may be ray casted
  size_type num_vols() const;

  /// \brief Retrieves the number of camera models to be used.
  ///
  /// Trivial getter.
  size_type num_camera_models() const;

  /// \brief Sets the camera model to be used. Implies a single camera.
  ///
  /// This should be called prior to calling allocate_resources()
  /// The camera model should be completely initialized. Trivial setter.
  void set_camera_model(const CameraModel& camera_model);

  /// \brief Sets the camera models that will be used.
  ///
  /// This should be called prior to calling allocate_resources()
  /// The camera models should be completely initialized.
  void set_camera_models(const CameraModelList& camera_models);

  /// \brief Retrieves the entire list of camera models stored by this
  ///        ray caster.
  const CameraModelList& camera_models() const;

  /// \brief Retrieve the camera model currently being used.
  const CameraModel& camera_model(const size_type cam_idx = 0) const;

  /// \brief Sets the camera model associated with a projection
  void set_proj_cam_model(const size_type proj_idx, const size_type cam_idx);

  /// \brief Retrieves the current assocation of each projection to a camera model.
  const CamModelAssocList& camera_model_proj_associations() const;

  /// \brief Sets the association of each projection to a camera model
  void set_camera_model_proj_associations(const CamModelAssocList& cam_model_for_proj);

  /// \brief Assign each pose in a list to each camera model.
  ///
  /// This is useful for multiview registration when computing many candidate
  /// poses in one process call to the ray caster. The transforms are distributed
  /// in "camera model order," e.g. the first N projections are the N transforms
  /// associated with camera model 1, the second N projections are the N
  /// transformations assocaited with camera model 2, and so forth.
  /// Requires xforms_cam_to_itk_phys.size() * num_camera_models == num_projs
  /// This should be called after calling allocate_resources()
  void distribute_xforms_among_cam_models(const FrameTransformList& xforms_cam_to_itk_phys);

  /// \brief Assign a single pose to each camera model.
  ///
  /// \see distribute_xforms_among_cam_models
  void distribute_xform_among_cam_models(const FrameTransform& xform_cam_to_itk_phys);

  /// \brief Sets the ray casting step size to be used
  ///
  /// Trivial setter.
  void set_ray_step_size(const CoordScalar& step_size);

  /// \brief Retrieves the ray casting step size used.
  CoordScalar ray_step_size() const;

  /// \brief Sets the number of projections that shall be computed
  ///
  /// This will also conservatively (not reduce already allocated capacity)
  /// resize other members, such as the projection -> camera model mapping.
  /// A sub-class can add functionality, such as changing the range on a
  /// synchronization buffer.
  virtual void set_num_projs(const size_type num_projs);

  /// \brief Retrieves the number of projections computed.
  ///
  /// Trivial getter.
  size_type num_projs() const;

  /// \brief Sets the volume interpolation method to be used.
  ///
  /// Trivial setter.
  void set_interp_method(const InterpMethod& interp_method);

  /// \brief Gets the volume interpolation method used.
  InterpMethod interp_method() const;

  /// \brief Sets the volume interpolation method to tri-linear.
  ///
  /// Trivial setter.
  void use_linear_interp();

  /// \brief Sets the volume interpolation method to nearest neighbor.
  ///
  /// Trivial setter.
  void use_nn_interp();

  /// \brief Sets the volume interpolation method to a sinc method.
  ///
  /// Trivial setter.
  void use_sinc_interp();

  /// \brief Sets the volume interpolation method to B-Spline.
  ///
  /// Trivial setter.
  void use_bspline_interp();

  /// \brief Sets the poses to be used - this will update the number of
  ///        projections!
  ///
  /// The user needs to ensure that this does not exceed the maximum number
  /// of projections allowed (e.g. larger than the number set prior to calling
  /// allocate_resources).
  /// This does not update any camera model associations.
  void set_xforms_cam_to_itk_phys(const FrameTransformList& xforms);

  /// \brief Gets the poses to be used
  ///
  /// This is const so that the number of projections is not changed.
  const FrameTransformList& xforms_cam_to_itk_phys() const;

  /// \brief Retrieves the non-const pose of a specific ray casting.
  ///
  /// This should be used when setting the ray casting poses prior to the
  /// compute() call.
  /// This is the pose of the camera with respect to the ITK volume
  /// physical coordinates.
  /// A valid number of projections must be set and allocate_resources() must be
  /// called prior to any call to this.
  FrameTransform& xform_cam_to_itk_phys(const size_type proj_idx);

  /// \brief Retrieves the const pose of a specific ray casting.
  ///
  /// This is the pose of the camera with respect to the ITK volume
  /// physical coordinates.
  /// A valid number of projections must be set and allocate_resources() must be
  /// called prior to any call to this.
  const FrameTransform& xform_cam_to_itk_phys(const size_type proj_idx) const;

  /// \brief Retrieve the pose of a 3D volume center with respect to the
  ///        existing ITK physical coordinate system.
  ///
  /// Useful for centering the volume.
  /// This is a translation only - therefore the original ITK physical
  /// coordinate system direction axes are preserved.
  FrameTransform xform_img_center_to_itk_phys(const size_type vol_idx = 0) const;

  /// \brief translation vector to move the C-Arm center of rotation to the origin.
  ///
  /// This is a translation along the z-axis equal to minus half of the source
  /// to detector distance.
  FrameTransform xform_cam_wrt_carm_center_of_rot(const size_type cam_idx = 0) const;

  /// \brief Post multiply all frame transforms by a specified transform.
  ///
  /// This is useful for applying an extrinsic transformation of the C-Arm.
  /// This is equivalent to: xform_cam_to_itk_phys(i) = xform_cam_to_itk_phys(i) * post_xform
  void post_multiply_all_xforms(const FrameTransform& post_xform);

  /// \brief Pre multiply all frame transforms by a specified transform.
  ///
  /// This is equivalent to: xform_cam_to_itk_phys(i) =  pre_xform * xform_cam_to_itk_phys(i)
  void pre_multiply_all_xforms(const FrameTransform& pre_xform);

  /// \brief Allocates resources required to complete the ray casting.
  ///
  /// The CameraRayCasting base allocates only the list of ray casting poses.
  /// Any child class should first allocate the resources needed by the parent.
  /// This may be done by calling ParentType::allocate_resources() before any
  /// other work.
  virtual void allocate_resources();

  /// \brief Perform the ray casting - needs to be implemented by the child class
  virtual void compute(const size_type vol_idx = 0) = 0;

  /// \brief Retrieve a ray casting result - needs to be implemented by the child class
  ///
  /// The underlying buffer of the returned object may be modified by subsequent
  /// calls to compute(), therefore the user should perform a deep copy if a
  /// particular result needs to be available for arbitrary lengths of time.
  virtual ProjPtr proj(const size_type proj_idx) = 0;

  /// \brief Retrieve a ray casting result, storing output in OpenCV format.
  ///
  /// \see proj for memory buffer details.
  virtual cv::Mat proj_ocv(const size_type proj_idx) = 0;

  /// \brief Retrieve the raw HOST buffer used to store projections.
  ///
  /// TODO: re-use this comment elsewhere... :)
  /// This returns an implementation specific buffer, e.g. a CPU implementation
  /// will most likely return a buffer from malloc/new/std::vector, while a
  /// GPU implementation is free to return a device buffer.
  virtual PixelScalar2D* raw_host_pixel_buf() = 0;

  /// \brief Use an external buffer for storing computed DRRs on the HOST
  ///
  /// The default behavior when this is not called, or a null pointer is provided,
  /// is to use an internally allocated buffer.
  /// The user is responsible for ensuring it has sufficient capacity and for
  /// managing the memory properly (e.g. deallocating when appropriate).
  virtual void use_external_host_pixel_buf(void* buf) = 0;

  /// \brief The maximum number of projections that is possible to allocate
  ///        resources for (as limited by hardware or the system).
  ///
  /// This is determined by the device's maximum buffer allocation size,
  /// the size of a projection, and the fraction of the maximum buffer allocation
  /// size set. This is available after setting the camera information and
  /// initializing any devices (such as a GPU).
  virtual size_type max_num_projs_possible() const = 0;

  /// \brief Set the method that will be used to update a 2D pixel value with the
  ///        next value computed by the next set of ray ray casting.
  ///
  /// Can be used to overwrite values, accumulate, etc.
  void set_proj_store_method(const ProjPixelStoreMethod m);

  /// \brief Gets the method that will be used to update a 2D pixel value with the
  ///        next value computed by the next set of ray ray casting.
  ProjPixelStoreMethod proj_store_method() const;

  /// \brief Update 2D pixel values by replacing/overwriting them. 
  void use_proj_store_replace_method();

  /// \brief Update 2D pixel values by accumulating them.
  void use_proj_store_accum_method();

  /// \brief Use a buffer from another ray caster instead of allocating another.
  ///
  /// May be useful when combining several objects - although that application
  /// is probably better suited to setting multiple volumes for a single ray
  /// casting instance.
  /// This is allowed to throw if the functionality is not supported.
  virtual void use_other_proj_buf(RayCaster* other_ray_caster) = 0;

  /// \brief Retrieve the 2D projection buffer synchronization object for
  ///        moving data to an OpenCL "device," e.g. a GPU.
  virtual RayCastSyncOCLBuf* to_ocl_buf();

  /// \brief Retrieve the 2D projection buffer synchronization object for
  ///        moving data to the "host."
  virtual RayCastSyncHostBuf* to_host_buf();

  /// \brief Set whether or not a background image should be set in each
  ///        projection before ray casting.
  ///
  /// This should be called prior to calling allocate resources
  void set_use_bg_projs(const bool use_bg_projs);

  bool use_bg_projs() const;

  /// \brief Sets a background projection image to be used for every camera view.
  ///
  /// This should be called prior to calling allocate resources, BUT AFTER
  /// the camera models have been set (so that the number of copies to be made
  /// is known).
  void set_bg_proj(ProjPtr& proj, const bool use_bg_projs = true);

  /// \brief Sets background projection images for each camera view.
  ///
  /// This should be called prior to calling allocate resources, BUT AFTER
  /// the camera models have been set
  void set_bg_projs(ProjList& projs, const bool use_bg_projs = true);

  /// \brief The maximum number of projections with the allocated resources.
  size_type max_num_projs() const;

  PixelScalar2D default_bg_pixel_val() const;

  void set_default_bg_pixel_val(const PixelScalar2D bg_val);

protected:

  /// \brief The 3D volumes that may be ray casted at/on/in.
  VolList vols_;

  /// \brief The camera parameters used for computing projections (intrinsics)
  ///
  /// NOTE: The indices of this list to not correspond to projection indices.
  /// For example, in a registration problem, we may have two intraoperative
  /// views and thus two camera models, however we may compute two pertubations
  /// for each view for a total of four projections.
  /// Multiple camera models allows for implicitly representing relative poses
  /// between multiple C-Arm views.
  /// NOTE: Camera models with varying image dimensions (e.g. one with 512x512
  /// and a second with 768x768 is not currently supported).
  CameraModelList camera_models_;

  /// \brief The number of ray castings to perform.
  ///
  /// Representing the number of different camera extrinsics.
  size_type num_projs_ = 0;

  /// \brief The maximum number of projections that the allocated resources can hold
  size_type max_num_projs_ = 0;

  /// \brief The list of camera extrinsics for each ray casting to be computed.
  ///
  /// These are the poses of the camera with respect to the
  /// ITK physical coordinates.
  FrameTransformList xforms_cam_to_itk_phys_;

  /// \brief The step size used for ray casting
  CoordScalar ray_step_size_ = 1;

  /// \brief The interpolation method to be used on the 3D volume.
  InterpMethod interp_method_ = kRAY_CAST_INTERP_LINEAR;

  /// \brief Stores the camera model to use for each projection.
  ///
  /// Projection j uses xforms_cam_to_itk_phys_[j] and camera_models_[cam_model_for_proj_[j]]
  CamModelAssocList cam_model_for_proj_;

  /// \brief The default pixel value to use in the projections.
  ///
  /// Defaults to zero.
  PixelScalar2D default_bg_pixel_val_ = 0;

  /// \brief Determines how the projection images store new values.
  ///
  /// E.g. replace all pixels, or accumulate with previous values
  /// Accumulation is useful when projecting multiple objects
  ProjPixelStoreMethod proj_store_meth_ = kRAY_CAST_PIXEL_REPLACE;

  bool use_bg_projs_ = false;

  bool bg_projs_updated_ = false;
  ProjList bg_projs_for_each_cam_;

  /// \brief Indicates that resources have been allocated successfully.
  bool resources_allocated_ = false;

  /// \brief Called whenever the image volumes are changed
  ///
  /// e.g. when calling set_volume(), set_volumes()
  /// The default implementation is a no-op, however a sub-class, such as a GPU
  /// implementation, may be interested in providing its own implementation to
  /// move things to device memory. 
  virtual void vols_changed() { }

  /// \brief Called whenever the camera models are changed
  ///
  /// e.g. when calling set_camera_model(), set_camera_models()
  /// The default implementation is a no-op, however a sub-class, such as a GPU
  /// implementation, may be interested in providing its own implementation to
  /// move things to device memory. 
  virtual void camera_models_changed() { }
};

struct RayCasterCollisionParams
{
  using PixelScalar = RayCaster::PixelScalar3D;

  PixelScalar thresh;

  size_type num_backtracking_steps;
};

/// \brief Parameters used when rendering a surface with a ray caster.
struct RayCasterSurRenderShadingParams
{
  using PixelScalar = RayCaster::PixelScalar3D;

  // Phong illumination model params
  // https://en.wikipedia.org/wiki/Phong_reflection_model
  PixelScalar ambient_reflection_ratio;
  PixelScalar diffuse_reflection_ratio;
  PixelScalar specular_reflection_ratio;
  PixelScalar alpha_shininess;
};

class RayCasterCollisionParamInterface
{
public:
  using PixelScalar = RayCasterCollisionParams::PixelScalar;

  void set_render_thresh(const PixelScalar t);

  void set_num_backtracking_steps(const size_type num_steps);

  PixelScalar render_thresh() const;

  size_type num_backtracking_steps() const;

  const RayCasterCollisionParams& collision_params() const;

private:
  RayCasterCollisionParams collision_params_ = { 150, 0 };
};

/// \brief Interface for incorporating surface rendering parameters into a
///        ray caster.
class RayCasterSurRenderParamInterface : public RayCasterCollisionParamInterface
{
public:

  RayCasterSurRenderParamInterface();
 
  void set_sur_render_params(const RayCasterSurRenderShadingParams& sur_render_params);

  const RayCasterSurRenderShadingParams& surface_render_params() const;

  void set_ambient_reflection_ratio(const PixelScalar amb_ref_ratio);

  void set_diffuse_reflection_ratio(const PixelScalar diff_ref_ratio);

  void set_specular_reflection_ratio(const PixelScalar spec_ref_ratio);

  void set_alpha_shininess(const PixelScalar alpha);

private:
  RayCasterSurRenderShadingParams sur_render_params_;
};

class RayCasterOccludingContours : public RayCasterCollisionParamInterface
{
public:
  RayCasterOccludingContours();

  CoordScalar occlusion_angle_thresh_rad() const;

  void set_occlusion_angle_thresh_rad(const CoordScalar ang_rad);

  void set_occlusion_angle_thresh_deg(const CoordScalar ang_deg);

  bool stop_after_collision() const;

  void set_stop_after_collision(const bool stop_after_coll);
  
private:

  CoordScalar occlusion_angle_thresh_rad_ = 7.5 * kDEG2RAD; 

  bool stop_after_collision_ = true;
};

/// \brief Simplified interface for computed ray casting projections.
///
/// This interface will compute ray casted projections at multiple views with
/// multiple objects at arbitrary poses. There is one projection per view.
///
/// 
/// A ray caster instance that has had volumes, cameras, maximum number of
/// projections set, and allocated is required.
/// It is assumed that the camera models share the same camera world frame,
/// so that a frame transform from camera world to volume, specifies the position
/// of a volume with respect to all cameras/views.
/// A list of camera world to volume transforms, cam_world_to_vols, is required.
///
/// A list of volumes to project, vols_to_proj, may be provided. It specifies
/// the order in which volumes are projected.
/// It is possible for a volume to be projected several times via this interface
/// with differing poses each time.
/// If vols_to_proj is empty, then the default behavior is to project each volume
/// of the ray caster object once in the order they are stored.
///
/// Intermediate frames may also be provided.
/// If inter_frames, inter_frames_wrt_vol, and ref_frames_cam_world_to_vol are
/// all empty, than intermediate frames are not used. If one is non-empty, than
/// each of them must be the same length as the number of volumes to project through.
/// TODO: Elaborate on intermediate frames.
///
/// NOTE: The output projections are shallow copies into the ray caster's interal
///       Host memory buffer, and should be copied if the user wants to retain their
///       contains between repeated ray casting computations.
struct SimpleRayCasterWrapperFn
{
  using ProjList  = RayCaster::ProjList;
  using IndexList = std::vector<size_type>;
  using BoolList  = std::vector<bool>;

  RayCaster* ray_caster;

  FrameTransformList cam_world_to_vols;

  FrameTransformList inter_frames;
  
  BoolList inter_frames_wrt_vol;

  FrameTransformList ref_frames_cam_world_to_vol;

  IndexList vols_to_proj;

  ProjList projs;

  void operator()();
};

enum RayCastLineIntKernel
{
  kRAY_CAST_LINE_INT_SUM_KERNEL = 0,
  kRAY_CAST_LINE_INT_MAX_KERNEL
};

class RayCastLineIntParamInterface
{
public:
  
  RayCastLineIntKernel kernel_id() const;

  void set_kernel_id(const RayCastLineIntKernel k);

protected:
  RayCastLineIntKernel kernel_id_ = kRAY_CAST_LINE_INT_SUM_KERNEL;
};

// This is a large number which is too large to represent a valid depth, and
// therefore indicates that no object intersection was found and the depth
// is not defined. This is smaller than, and used in place of,
// std::numeric_limits<CoordScalar>::max() (for CoordScalar == float) as we
// have found that some NVIDIA cards will convert a
// std::numeric_limits<CoordScalar>::max() to a smaller value, which leads
// to bugs and inconsistencies when mixing with CPU code that uses the
// correct number.
constexpr CoordScalar kRAY_CAST_MAX_DEPTH = 1.0e37;

}  // xreg

#endif
