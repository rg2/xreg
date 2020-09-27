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

#ifndef XREGINTENSITY2D3DREGI_H_
#define XREGINTENSITY2D3DREGI_H_

#include "xregCommon.h"
#include "xregObjWithOStream.h"
#include "xregRayCastInterface.h"

namespace xreg
{

// Forward declarations
class ImgSimMetric2D;
class SE3OptVars;
class CamSourceObjPoseOptVars;
class ImgSimMetric2DCombineMean;
class Regi2D3DPenaltyFn;
struct SingleRegiDebugResults;

/// \brief General class for intensity-based 2D/3D registration.
///
/// The type of images are implicitly defined by the choice of ray caster and
/// similarity metric. For example, a line integral ray caster may be set to
/// indicate that this is a 2D/3D X-Ray registration, or a surface ray caster
/// may be set to indicate a 2D/3D camera registration.
/// This method provides a generic interface that must be completed in a derived
/// class. The derived class will typically wrap an interface to some other
/// optimizer.
///
/// The number of views is equal to the number of similarity metrics used. If
/// only rigid pose is being optimized over, then the number of cameras in the
/// ray caster should be equal to the number of views. When the camera source,
/// or entire projection matrix, is being optimized over, then the number of
/// cameras in the ray caster must be equal to the total number of projections
/// being computed over all views (e.g. each projection has its own camera model).
///
/// TODO: Add some pure virtual methods that the derived classes must implement
///       to indicate the current, best, pose and associated similarity metric.
/// TODO: Allow a different SE3 parameterization per volume
class Intensity2D3DRegi : public ObjWithOStream
{
public:
  using RayCasterPtr  = std::shared_ptr<RayCaster>;
  using SimMetricPtr  = std::shared_ptr<ImgSimMetric2D>;
  using SimMetricList = std::vector<SimMetricPtr>;
  
  using Scalar                   = CoordScalar;
  using ScalarList               = CoordScalarList;
  using ListOfScalarLists        = std::vector<ScalarList>;
  using ListOfListsOfScalarLists = std::vector<ListOfScalarLists>;

  using IndexList = std::vector<size_type>;

  using ListOfFrameTransformLists = std::vector<FrameTransformList>;

  using CamModelList = RayCaster::CameraModelList;

  using SE3OptVarsPtr = std::shared_ptr<SE3OptVars>;

  using ProjList = RayCaster::ProjList;

  using PenaltyFnPtr = std::shared_ptr<Regi2D3DPenaltyFn>;

  using CallbackFn = std::function<void(Intensity2D3DRegi*)>;

  using DynRefFrameFn = std::function<FrameTransformList(const size_type,
                                                         const ListOfFrameTransformLists&,  // before any inter frames
                                                         const ListOfFrameTransformLists&,  // including standard inter frames
                                                         const FrameTransformList&,
                                                         const std::vector<bool>&,
                                                         const FrameTransformList&)>;

  using SingleRegiDebugResultsPtr = std::shared_ptr<SingleRegiDebugResults>;

  class UnsupportedOperationException { };

  /// \brief Value indicating that the index is not valid (binary all ones).
  static const size_type kNPOS = ~size_type(0);

  /// \brief Default constructor - does not perform any work.
  Intensity2D3DRegi() = default;

  /// \brief Destructor
  virtual ~Intensity2D3DRegi() = default;
  
  // no copying
  Intensity2D3DRegi(const Intensity2D3DRegi&) = delete;
  Intensity2D3DRegi& operator=(const Intensity2D3DRegi&) = delete;

  /// \brief Allocates resources needed and connects ray casters to similarity
  ///        metrics.
  ///
  /// This must be called before running the registration (calling run()), however
  /// a ray caster object and similarity metric objects need to be set before
  /// this call.
  virtual void setup();

  /// \brief Performs the registration
  virtual void run() = 0;

  /// \brief Returns the number of volumes this object is attempting to register.
  size_type num_vols() const;

  /// \brief Sets the volumes that should be used in the ray caster.
  void set_multi_vol_inds_in_ray_caster(const IndexList& vol_inds);

  /// \brief Sets the volume that should be used in the ray caster.
  void set_single_vol_ind_in_ray_caster(const size_type vol_idx);

  /// \brief Use all volumes available in the ray caster for registration.
  void set_use_all_vols_in_ray_caster();

  /// \brief Retrieves the computed registration transformation.
  FrameTransform regi_xform(const size_type vol_idx = 0) const;

  /// \brief Sets an initial guess (starting transformation) for the registration.
  ///
  /// NOTE: This must be called after calling setup().
  /// This is typically computed by another registration, or manually (by "hand")
  void set_regi_xform_guess(const FrameTransform& regi_xform_guess,
                            const size_type vol_idx = kNPOS);

  /// \brief Retrieve the final, registered/optimized, camera model. Should only
  ///        be called when optimizing over tha camera model in some way.
  const CameraModel& regi_cam() const;

  // We don't need this, the reference camera in the optimization object serves
  // this purpose
  //void set_cam_guess(const CameraModel& cam_guess);

  /// \brief Sets an intermediate frame to optimize over.
  ///
  /// If wrt_vol is true then, the intermediate frame should define the pose of
  /// the intermediate frame with respect to the volume frame, otherwise the
  /// intermediate frame is treated with respect to the camera.
  /// WRT to volume: T^VOL_CAM = T^VOL_INTER * delta T * T^INTER_VOL * guess(T^VOL_CAM)
  /// WRT to camera: T^VOL_CAM = guess(T^VOL_CAM) * T^CAM_INTER * delta T * T^INTER_CAM
  /// NOTE: This must be called after calling setup().
  void set_intermediate_frame(const FrameTransform& inter_frame, const bool wrt_vol,
                              const size_type vol_idx = kNPOS);
  
  void set_intermediate_frame(DynRefFrameFn dyn_ref_frame_fn, const size_type vol_idx);

  /// \brief Sets the ray caster object to use
  ///
  /// All ray casting parameters, such as the volume, camera models, interpolator,
  /// etc. need to be set by the user.
  /// If the volume index is set to kNPOS, then all volumes are used,
  /// otherwise a single volume is used.
  void set_ray_caster(RayCasterPtr ray_caster,
                      const size_type vol_idx = kNPOS,
                      const bool need_to_alloc = true);

  /// \brief Sets the ray caster and a specific sub-set of volumes to be used.
  void set_ray_caster(RayCasterPtr ray_caster, const IndexList& vol_inds,
                      const bool need_to_alloc = true);

  /// \brief Retrieves the current ray caster in use
  RayCasterPtr ray_caster();

  /// \brief Sets a single similarity metric to use.
  ///
  /// With the current setup, this implies single view.
  void set_sim_metric(SimMetricPtr sim_metric, const bool need_to_alloc = true);

  /// \brief Sets the similarity metric objects to use
  ///
  /// The fixed images need to be set, independently, by the user on these objects.
  void set_sim_metrics(const SimMetricList& sim_metrics, const bool need_to_alloc = true);

  SimMetricList& sim_metrics();

  const SimMetricList& sim_metrics() const;

  SimMetricPtr sim_metric(const size_type view_idx = 0);

  /// \brief Re-use previously allocated ray caster from another registration object
  void use_ray_caster_from_other_regi(Intensity2D3DRegi* other_regi);

  /// \brief Re-use previously allocated ray caster and similarity metrics from
  ///        another registration object.
  void use_ray_caster_and_sim_metrics_from_other_regi(Intensity2D3DRegi* other_regi);

  /// \brief Sets the SE3 parameterization to use.
  ///
  /// This should be called prior to calling setup().
  void set_opt_vars(SE3OptVarsPtr opt_vars);

  /// \brief Sets the convergence tolerance on objective function values
  ///
  /// This must be implemented by a derived class, if it is used at all, since
  /// the derived class has knowledge of the optimization method being executed.
  virtual void set_opt_obj_fn_tol(const Scalar& tol);

  /// \brief Sets the convergence tolerance on search space values
  ///
  /// This must be implemented by a derived class, if it is used at all, since
  /// the derived class has knowledge of the optimization method being executed.
  virtual void set_opt_x_tol(const Scalar& tol);

  /// \brief Sets the directory to write debug outputs to.
  ///
  /// This directory should already exist.
  void set_debug_output_dir_path(const std::string& debug_output_dir_path);

  /// \brief Sets whether or not remapped (8 bpp) DRR images should be written
  ///        to disk (as PNG files).
  void set_debug_write_remapped_drrs(const bool write_remapped_drrs);

  /// \brief Sets whether or not raw DRR images should be written to disk
  ///        (as .nii.gz files).
  void set_debug_write_raw_drrs(const bool write_raw_drrs);

  /// \brief Sets whether or not the (8 bpp remapped) fixed images with edges
  ///        from DRRs should be written to disk (as PNG files)
  void set_debug_write_fixed_img_edge_overlays(const bool write_fixed_img_edge_overlays);

  /// \brief Indicates that similarity scores should be written out to the debug stream at each iteration.
  void set_debug_write_combined_sim_scores_to_stream(const bool write_combined_sim_scores_to_stream);

  /// \brief Indicates that optimization pose vectors should be written out to the debug stream at each iteration.
  void set_debug_write_opt_vars_to_stream(const bool write_opt_vars_to_stream);

  /// \brief Given a set of single poses for each volume, compute a single DRR
  ///        for each view - useful for debugging or using as a
  ///        background image for the next stage in registration.
  ///
  /// xforms[i] is the pose of the ith volume
  /// projs[k] is the projection generated by the kith view
  /// intermediate_frame indicates if the poses are with respect to a previously
  /// specified intermediate frame.
  /// When cams is null, it is assumed that each view in the ray caster corresponds
  /// to a view, otherwise each view corresponds to a passed cams.
  void debug_compute_drrs_single_proj_per_view(const FrameTransformList& xforms,
                                               ProjList* projs,
                                               const bool intermediate_frame = true,
                                               const CamModelList* cams = nullptr);

  /// \brief Given a single pose for all volumes, compute a single DRR for each
  ///        camera model - useful for debugging or using as a background image
  ///        for the next stage in registration.
  ///
  /// projs[k] is the projection generated by the kith camera model
  /// intermediate_frame indicates if the pose is with respect to a previously
  /// specified intermediate frame.
  /// When cams is null, it is assumed that each view in the ray caster corresponds
  /// to a view, otherwise each view corresponds to a passed cams.
  void debug_compute_drrs_single_proj_per_view(const FrameTransform& xform,
                                               ProjList* projs,
                                               const bool intermediate_frame = true,
                                               const CamModelList* cams = nullptr);

  /// \brief Sets whether per iteration statistics should be retained.
  void set_debug_save_iter_debug_info(const bool save_iter_info);

  void set_sim_metric_debug_save_info(const bool save_sm_info);

  /// \brief Retrieves the per iteration statistics
  SingleRegiDebugResultsPtr debug_info() const;

  /// \brief Returns the maximum number of projections required for each view
  ///        on each iteration.
  ///
  /// This should be used to allocate the maximum number of resources for a
  /// ray caster, or similarity metric, when they are shared amongst several
  /// registration objects. This will be called after optimization parameters
  /// have been set on this object, but before calling setup().
  virtual size_type max_num_projs_per_view_per_iter() const = 0;

  void set_penalty_fn(PenaltyFnPtr penalty_fn);

  PenaltyFnPtr penalty_fn();
  
  // This should only be called after setup
  void set_img_sim_penalty_coefs(const Scalar img_sim_coeff,
                                 const Scalar penalty_fn_coeff);

  void set_img_sim_penalty_coefs(const ScalarList& img_sim_coeffs,
                                 const ScalarList& penalty_fn_coeffs);

  bool include_penalty_in_obj_fn() const;

  void set_include_penalty_in_obj_fn(const bool b);

  const std::vector<bool>& intermediate_frames_wrt_vol() const;

  const FrameTransformList& intermediate_frames() const;
  
  const FrameTransformList& regi_xform_guesses() const;
  
  void add_begin_of_iter_callback(CallbackFn& fn);

  void reset_begin_of_iter_callbacks();
  
  void add_end_of_iter_callback(CallbackFn& fn);

  void reset_end_of_iter_callbacks();

  size_type max_num_iters() const;

  void set_max_num_iters(const size_type max_iters);

  bool has_a_static_vol() const;

  void set_has_a_static_vol(const bool b);

protected:

  /// \brief Initialization of the optimization algorithm.
  ///
  /// This is mainly to setup any memory or construct an object instance. A
  /// derived class must implement this.
  virtual void init_opt() = 0;

  virtual void obj_fn(const ListOfFrameTransformLists& frame_xforms_per_object,
                      const CamModelList* cams_per_proj,
                      ScalarList* sim_vals_ptr); 

  /// \brief Objective function that computes DRRs and similarity metrics.
  ///
  /// This should be called by the optimizer in some way, maybe not directly,
  /// depending on the optimization interface.
  /// opt_vec_space_vals[i][j][k] is the kth parameter of the jth SE(3) (projection) element for the ith object
  /// (*sim_vals_ptr)[j] is the computed similarity value corresponding to the jth projection element
  virtual void obj_fn(const ListOfListsOfScalarLists& opt_vec_space_vals,
                      ScalarList* sim_vals_ptr);

  /// \brief This should be called by the derived class before entering the main
  ///        loop of the algorithm.
  virtual void before_first_iteration();

  /// \brief This should be called by the derived class after leaving the main
  ///        loop of the algorithm.
  virtual void after_last_iteration();

  /// \brief This should be called by the derived class at the beginning of
  ///        each algorithm iteration
  virtual void begin_of_iteration(const ScalarList& x);

  /// \brief This should be called by the derived class at the beginning of
  ///        each algorithm iteration
  virtual void begin_of_iteration(const FrameTransformList& delta_xforms);

  /// \brief This should be called by the derived class at the end of each
  ///        algorithm iteration.
  virtual void end_of_iteration();

  /// \brief Returns true when at least one DRR is required for writing debug images.
  virtual bool write_debug_requires_drr() const;

  /// \brief Calls the write debug image routines if the appropriate flags have
  ///        been set.
  virtual void write_debug();

  /// \brief Writes remapped (8 bpp) DRRs to disk.
  ///
  /// This base implementation will write every projection, for every view,
  /// and for every objective function invocation with the following file name
  /// pattern: "drr_remap_<obj fn #>_<view index>_<projection index>.png"
  /// This is made virtual, so that another implementation may override.
  /// For example, with the CMA-ES optimizer, most likely it is not desired that
  /// every projection be written out, but rather the DRR with the best similarity
  /// score so far.
  virtual void debug_write_remapped_drrs();

  /// \brief Writes raw DRRs to disk.
  ///
  /// This base implementation will write every projection, for every view,
  /// and for every objective function invocation with the following file name
  /// pattern: "drr_raw_<obj fn #>_<view index>_<projection index>.nii.gz"
  /// This is made virtual, so that another implementation may override.
  /// For example, with the CMA-ES optimizer, most likely it is not desired that
  /// every projection be written out, but rather the DRR with the best similarity
  /// score so far.
  virtual void debug_write_raw_drrs();

  /// \brief Writes remapped fixed images with edges from the DRRs overlaid to disk.
  ///
  /// This base implementation will write every projection, for every view,
  /// and for every objective function invocation with the following file name
  /// pattern: "edges_<obj fn #>_<view index>_<projection index>.png"
  /// This is made virtual, so that another implementation may override.
  /// For example, with the CMA-ES optimizer, most likely it is not desired that
  /// every projection be written out, but rather the DRR with the best similarity
  /// score so far.
  virtual void debug_write_fixed_img_edge_overlays();

  /// \brief Given current transformations estimated by an optimization, compute
  ///        the registration transformations of the cameras with respect to the
  ///        volumes.
  void update_regi_xforms(const FrameTransformList& delta_xforms);

  /// \brief Write the similarity score for the current pose estimate to a stream.
  ///
  /// This must be implemented by a derived class, if it is used at all, since
  /// the derived class has knowledge of the optimization method being executed.
  virtual void debug_write_comb_sim_score();

  /// \brief Write the current vector space pose parameters of each volume to a stream.
  ///
  /// This must be implemented by a derived class, if it is used at all, since
  /// the derived class has knowledge of the optimization method being executed.
  virtual void debug_write_opt_pose_vars();

  /// \brief Given the current optimization vector, return a list of the corresponding
  ///        frame transforms for each object/volume
  FrameTransformList opt_vec_to_frame_transforms(const ScalarList& x) const;

  FrameTransformList inter_transforms_to_regi(const FrameTransformList& delta_xforms) const;

  ListOfFrameTransformLists apply_inter_transforms_for_obj_fn(
                              const ListOfFrameTransformLists& src_frame_xforms_per_object) const;

  bool need_to_alloc_ray_caster_ = true;

  bool need_to_alloc_sim_metrics_ = true;

  bool need_to_setup_sim_combiner_ = true;

  bool need_to_init_opt_ = 0;

  FrameTransformList regi_xforms_;

  FrameTransformList regi_xform_guesses_;

  FrameTransformList intermediate_frames_;

  std::vector<bool> intermediate_frames_wrt_vol_;

  std::vector<DynRefFrameFn> dyn_ref_frame_fns_;

  RayCasterPtr ray_caster_;

  SimMetricList sim_metrics_;

  SE3OptVarsPtr opt_vars_;

  bool has_a_static_vol_ = false;

  /// \brief Stores the number of projections to simulate (number of object poses)
  ///        for each view.
  ///
  /// This is basically the population parameter used by stochastic routines
  /// (e.g. CMA-ES).
  /// This needs to be set by the child class!
  size_type num_projs_per_view_ = 0;

  size_type num_obj_fn_evals_ = 0;

  ListOfFrameTransformLists tmp_frame_xforms_;

  size_type max_num_iters_ = std::numeric_limits<size_type>::max();

  // Debug information/actions:

  /// \brief The output directory where debug data should be written.
  ///
  /// This directory should exist - defaults to "" so that the default directory
  /// is the current working directory of the executable.
  std::string debug_output_dir_path_;

  bool write_remapped_drrs_ = false;
  bool write_raw_drrs_ = false;
  bool write_fixed_img_edge_overlays_ = false;

  bool write_combined_sim_scores_to_stream_ = false;
  bool write_opt_vars_to_stream_ = false;

  std::shared_ptr<ImgSimMetric2DCombineMean> sim_metric_combiner_;

  IndexList vol_inds_in_ray_caster_;

  bool debug_save_iter_debug_info_ = false;
  SingleRegiDebugResultsPtr debug_info_;

  bool sim_metric_debug_save_info_ = false;

  // This will be set whenever the SE3 opt vars object is set, when it is non-null
  // then the camera models in the ray caster will be updated
  const CamSourceObjPoseOptVars* src_and_obj_pose_opt_vars_ = nullptr;

  CamModelList regi_cam_models_;

  // temporary storage, similar to tmp_frame_xforms_
  CamModelList tmp_cam_models_;

  ScalarList coeffs_img_sim_;
  ScalarList coeffs_penalty_fns_;

  PenaltyFnPtr penalty_fn_;

  // If this flag is set to false, then the penalty functions are still evaluated
  // (when available), however no weights are applied to the image similarity metrics
  // and regularization values, AND the regularization/penalties are not added into
  // the final similarity score array.
  // It is useful to set this to false when regularization/prior probabilities
  // are desired for sampling purposes and we do not want to include their
  // values in the energy term of the boltzmann distribution for the similarity metric.
  bool include_penalty_in_obj_fn_ = true;

  // each of these are called by begin_of_iteration()
  std::vector<CallbackFn> begin_of_iter_fns_;
  
  // each of these are called by end_of_iteration()
  std::vector<CallbackFn> end_of_iter_fns_;

private:
  enum { kDEFAULT_INTER_FRAMES_WRT_VOL = 0 };

  /// \brief Should be called whenever the size of vol_inds_in_ray_caster_ is
  ///        updated, so that other lists may be resized.
  void num_vols_updated();
};

}  // xreg

#endif

