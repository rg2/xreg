# xReg: Modeling and Registration Software for Surgery
This repository contains library routines and stand-alone programs for various modeling and registration tasks relating to surgery, with a particular focus on 2D/3D registration for intraoperative X-ray navigation.
Although xReg was developed from a practical approach and provides an out-of-the-box capability for intraoperative fluoroscopic navigation, it is also very much compatible with research-oriented goals.
In addition to accelerating the development of new registration components through the extendable interfaces for ray casting, image similarity computation and iterative optimization, xReg is also useful as an initial baseline for evaluating the neural network approaches *du jour*.

Much of the functionality provided by xReg is also useful for other computer-assisted surgery tasks.
A comprehensive listing of the features of the library are provided below, along with a list of the executable programs that are also included in this repository.
Please visit the [wiki](https://github.com/rg2/jhmr-v2/wiki) for descriptions on the use of the library and executable programs.
Basic usage of the executables is provided via a walkthrough/tutorial [here](https://github.com/rg2/jhmr-v2/wiki#walkthrough).
Please submit an [issue](https://github.com/rg2/jhmr-v2/issues) for any problems, feature requests, or suggestions.

This software was originally created while conducting research with [Russell Taylor](http://www.cs.jhu.edu/~rht), [Mehran Armand](https://bigss.lcsr.jhu.edu), and [Mathias Unberath](https://mathiasunberath.github.io/) within the [Laboratory for Computational Sensing and Robotics](https://lcsr.jhu.edu) at [Johns Hopkins University](https://www.jhu.edu).
The current repository represents a rewrite/refactor of the original, internal, version of this software, with a focus on minimizing compile times and being reusable to novice developers and researchers.

## Library Features:
* Registration:
  * Efficient ray casters for 2D/3D registration and visualization:
    * Line integral ([CPU](lib/ray_cast/xregRayCastLineIntCPU.h) and [OpenCL](lib/ray_cast/xregRayCastLineIntOCL.h))
    * Line integral approximation via splatting ([CPU](lib/ray_cast/xregSplatLineIntCPU.h))
    * Surface rendering ([CPU](lib/ray_cast/xregRayCastSurRenderCPU.h) and [OpenCL](lib/ray_cast/xregRayCastSurRenderOCL.h))
    * Depth maps ([CPU](lib/ray_cast/xregRayCastDepthCPU.h) and [OpenCL](lib/ray_cast/xregRayCastDepthOCL.h))
    * Occluding contours ([CPU](lib/ray_cast/xregRayCastOccContourCPU.h) and [OpenCL](lib/ray_cast/xregRayCastOccContourOCL.h))
    * Sparse collision detection ([CPU](lib/ray_cast/xregRayCastSparseCollCPU.h))
    * Extendable common interface ([CPU](lib/ray_cast/xregRayCastBaseCPU.h), [OpenCL](lib/ray_cast/xregRayCastBaseOCL.h), and [more](lib/ray_cast/xregRayCastInterface.h))
  * Image similarity metrics for driving 2D/3D registrations:
    * Sum of squared differences (SSD) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DSSDCPU.h) and [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DSSDOCL.h))
    * Normalized cross correlation (NCC) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DNCCCPU.h) and [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DNCCOCL.h))
    * NCC of Sobel Gradients (Grad-NCC) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DGradNCCCPU.h) and [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DGradNCCOCL.h))
    * Gradient orientation (GO) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DGradOrientCPU.h))
    * Gradient difference (Grad-Diff) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DGradDiffCPU.h))
    * Patch-wise NCC (Patch-NCC) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DPatchNCCCPU.h) and [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DPatchNCCOCL.h))
    * Patch-wise Grad-NCC (Patch-Grad-NCC) ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DPatchGradNCCCPU.h) and [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DPatchGradNCCOCL.h))
    * Boundary contour distance ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DBoundaryEdgesCPU.h))
    * Extendable common interface ([CPU](lib/regi/sim_metrics_2d/xregImgSimMetric2DCPU.h), [OpenCL](lib/regi/sim_metrics_2d/xregImgSimMetric2DOCL.h), and [more](lib/regi/sim_metrics_2d/xregImgSimMetric2D.h))
  * Various optimization strategies for 2D/3D intensity-based registration:
    * [CMA-ES](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiCMAES.h)
    * [Differential Evolution](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiDiffEvo.h)
    * [Exhaustive/Grid Search](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiExhaustive.h)
    * [Particle Swarm Optimization](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiPSO.h)
    * [Hill Climbing](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiHillClimb.h)
    * Wrappers around [NLOpt](https://github.com/stevengj/nlopt) routines:
      * [BOBYQA](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiBOBYQA.h)
      * [CRS](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiCRS.h)
      * [DESCH](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiDESCH.h)
      * [Variations of DIRECT](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiDIRECT.h)
      * [DISRES](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiDISRES.h)
      * [NEWUOA](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiNEWUOA.h)
      * [Nelder Mead](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiNelderMead.h)
      * [PRAXIS](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiPRAXIS.h)
      * [Sbplx](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegiSbplx.h)
    * [Extendable common interface](lib/regi/interfaces_2d_3d/xregIntensity2D3DRegi.h)
  * Regularizers for 2D/3D intensity-based registration:
    * [Rotation and translation magnitudes](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnSE3Mag.h)
    * [Euler decomposition magnitudes](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnSE3EulerDecomp.h)
    * [Rotation and translation magnitudes of relative pose between multiple objects](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnRelPose.h)
    * [Relative pose difference from nominal AP pose](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnPelvisAP.h)
    * [Landmark re-projection distances](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnLandReproj.h)
    * [Heuristics for automatic global pelvis registration](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnGlobalPelvis.h)
    * [Combination of regularizers](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFnCombo.h)
    * [Extendable common interface](lib/regi/penalty_fns_2d_3d/xregRegi2D3DPenaltyFn.h)
  * [Pipeline for chaining together 2D/3D registrations](lib/regi/interfaces_2d_3d/xregMultiObjMultiLevel2D3DRegi.h) (intensity-based and feature-based) for solving registration problems with multiple-resolutions and views
  * Perspective-n-Point (PnP) solvers (paired point 2D/3D)
    * [Minimization of re-projection distances](lib/regi/pnp_solvers/xregLandmark2D3DRegiReprojDistCMAES.h)
    * [POSIT](lib/regi/pnp_solvers/xregPOSIT.h)
    * [P3P using C-arm geometry assumptions](lib/regi/pnp_solvers/xregP3PCArm.h)
    * [RANSAC PnP wrapper](lib/regi/pnp_solvers/xregRANSACPnP.h)
  * [Paired Point 3D/3D](lib/regi/xregPairedPointRegi3D3D.h)
  * [3D Point Cloud to 3D Surface ICP](lib/regi/xregICP3D3D.h)
* Mesh Processing:
  * [Triangular and tetrahedral mesh representations](lib/common/xregMesh.h)
* Image/Volume Processing:
  * [Interpolation of non-uniform spaced slices](lib/image/xregVariableSpacedSlices.h)
  * Image processing operations leveraging lower-level [ITK](lib/itk) and [OpenCV](lib/opencv/xregOpenCVUtils.h) routines
  * [Conversion of Hounsfield units (HU) to linear attenuation](lib/image/xregHUToLinAtt.h)
  * [Poisson noise](lib/image/xregImageAddPoissonNoise.h)
  * [Image intensity log transform](lib/image/xregImageIntensLogTrans.h)
  * [Piecewise rigid volume warping using label maps](lib/image/xregLabelWarping.h)
* Numerical Optimization:
  * [Configurable line search implementation](lib/optim/xregLineSearchOpt.h)
  * Implementations of several derivative-free methods, including [Differential Evolution](lib/optim/xregDiffEvo.h), [Simulated Annealing](lib/optim/xregSimAnn.h), and [Particle Swarm Optimization](lib/optim/xregPSO.h)
  * [Wrapper around C implementation of CMA-ES optimization](lib/optim/xregCMAESInterface.h)
  * [Suite of test objective functions](lib/optim/xregOptimTestObjFns.h)
* Geometric Primitives and Spatial Data Structures
  * [KD-Tree for points or surfaces of arbitrary dimension](lib/spatial/xregKDTree.h)
  * [Primitives with support for intersection, etc.](lib/spatial/xregSpatialPrimitives.h)
  * Fitting primitives to data ([circle](lib/spatial/xregFitCircle.h), [plane](lib/spatial/xregFitPlane.h)) with robustness to outliers
* Spatial Transformation Utilities:
  * [Rotation](lib/transforms/xregRotUtils.h) and [rigid](lib/transforms/xregRigidUtils.h) transformation utilities, including lie group/algebra routines
  * [Perspective projection (3D to 2D)](lib/transforms/xregPerspectiveXform.h)
  * [Calculation of anatomical coordinate frames](lib/transforms/xregAnatCoordFrames.h)
  * [Point cloud manipulation](lib/transforms/xregPointCloudUtils.h)
* Visualization
  * [Interactive 3D scene plotting using VTK](lib/vtk/xregVTK3DPlotter.h)
* File I/O:
  * [Common DICOM fields](lib/file_formats/xregDICOMUtils.h)
  * [DICOM files from Siemens CIOS Fusion C-arm](lib/file_formats/xregCIOSFusionDICOM.h)
  * [Helpers for HDF5 reading/writing](lib/hdf5/xregHDF5.h)
  * Various mesh formats
  * Various image/volume formats (via ITK)
  * 3D Slicer annotations, [FCSV](lib/file_formats/xregFCSVUtils.h) and [ACSV](lib/file_formats/xregACSVUtils.h)
  * [Comma separated value (CSV) files](lib/file_formats/xregCSVUtils.h)
* Basic Math Utilities:
  * [Basic statistics](lib/basic_math/xregBasicStats.h)
  * [Distribution fitting](lib/basic_math/xregNormDistFit.h)
  * [Uniformly distributed N-D unit vector sampling](lib/basic_math/xregSampleUniformUnitVecs.h)
  * [Common interface for probability densities](lib/basic_math/xregDistInterface.h) and instances for common distributions:
    * [Normal](lib/basic_math/xregNormDist.h)
    * [Log-normal](lib/basic_math/xregLogNormDist.h)
    * [Folded normal](lib/basic_math/xregFoldNormDist.h)
* General/Common:
  * [String parsing/manipulation utilities](lib/common/xregStringUtils.h)
  * [Serialization streams](lib/common/xregStreams.h)
  * [Command line argument parsing](lib/common/xregProgOptUtils.h)
  * [Timer class for measuring runtimes with a stopwatch-like interface](lib/common/xregTimer.h)
  * [Basic filesystem utilities](lib/common/xregFilesystemUtils.h)
  * [Runtime assertions](lib/common/xregAssert.h)
* Hip Surgery:
  * [Guessing labels of bones from segmentation volumes](lib/hip_surgery/xregHipSegUtils.h)
  * [Planning and modeling of osteotomies](lib/hip_surgery/xregPAOCuts.h)
  * [Visualization of osteotomies in 3D](lib/hip_surgery/xregPAODrawBones.h)
  * [Modeling of surgical objects, such as screws and K-wires](lib/hip_surgery/xregMetalObjs.h)
  * Support for simulated data creation, including [randomized screw and K-wire shapes and poses](lib/hip_surgery/xregMetalObjSampling.h), and [volumetric data incorporating osteotomies, repositioned bones, and inserted screws and K-wires](lib/hip_surgery/xregPAOVolAfterRepo.h)

## Programs
Some of the capabilities provided by individual programs contained with the apps directory include:
* Image I/O:
  * [DICOM conversion and resampling](apps/image_io/convert_resample_dicom)
  * [Volume cropping](apps/image_io/crop_vol)
  * [Printing DICOM metadata](apps/image_io/report_dicom)
* Mesh processing:
  * [Mesh creation](apps/mesh/create_mesh)
  * [Mesh display](apps/mesh/show_mesh)
* Basic point cloud operations:
  * [Printing FCSV contents](apps/point_clouds/print_fcsv)
  * [Warping FCSV files](apps/point_clouds/xform_fcsv)
* Registration
  * [ICP for 3D point cloud to 3D surface registration](apps/mesh/sur_regi)
* General utilities for projection data:
  * [Advanced visualization of projective geometry coordinate frames with a scene of 3D objects](apps/image_io/draw_xray_scene)
  * [Remap and tile projection data for visualization](apps/image_io/remap_and_tile_proj_data)
  * [Tool for creating movie replays of 2D/3D registration processing](apps/image_io/regi2d3d_replay)
  * [Extract projection into NIFTY format (.nii/.nii.gz)](apps/image_io/extract_nii_from_proj_data)
  * [Insert landmarks (FCSV) into HDF5 projection data](apps/image_io/add_lands_to_proj_data)
* Hip Surgery: Periacetabular Osteotomy (PAO)
  * [Osteotomy planning and modeling](apps/hip_surgery/pao/create_fragment)
  * [Osteotomy 3D visualization](apps/hip_surgery/pao/draw_bones)
  * [Randomized simulation of fragment adjustments](apps/hip_surgery/pao/sample_frag_moves)
  * [Volumetric modeling of fragment adjustments](apps/hip_surgery/pao/create_repo_vol)
  * [Volumetric modeling of fragment fixation using screws and K-wires](apps/hip_surgery/pao/add_screw_kwires_to_vol)
  * [Creation of simulated fluoroscopy for 2D/3D registration experiments](apps/hip_surgery/pao/create_synthetic_fluoro)
  * Examples of 2D/3D, fluoroscopy to CT, registration
    * [Single-view pelvis registration](apps/hip_surgery/pelvis_single_view_regi_2d_3d)
    * [Multiple-view, pelvis, femur PAO fragment registration](apps/hip_surgery/pao/frag_multi_view_regi_2d_3d)

## Planned Work
Although the following capabilities currently only exist in an internal version of the xReg software, they will be incorporated into this repository at a future date:
* Executable for running a multiple-view/multiple-resolution 2D/3D registration pipeline defined using a configuration file
* Intraoperative reconstruction of PAO bone fragments
* Utilities for creation and manipulation of statistical shape models
* Shape completion from partial shapes and statistical models
* More point cloud manipulation utilities
* Python bindings, conda integration
* And more...

## Dependencies
* C++ 11 compatible compiler
* External libraries (compatible versions are listed):
  * OpenCL (1.x) (typically provided with your graphics drivers or CUDA SDK)
  * [Intel Threading Building Blocks (TBB)](https://github.com/oneapi-src/oneTBB) (20170919oss)
  * [Boost](https://www.boost.org) (header only) (1.65)
  * [Eigen3](http://eigen.tuxfamily.org) (3.3.4)
  * [fmt](https://fmt.dev) (5.3.0)
  * [NLOpt](https://github.com/stevengj/nlopt) (2.5.0)
  * [ITK](https://itk.org) (4.13.2)
  * [VTK](https://vtk.org) (7.1.1)
  * [OpenCV](https://opencv.org) (3.2.0)
  * [ViennaCL](http://viennacl.sourceforge.net) (1.7.1)
  * Optional: [ffmpeg](https://ffmpeg.org) is used for writing videos when it is found in the system path. The OpenCV video writer is used if ffmpeg is not available.

## Building
A standard CMake configure/generate process is used.
It is recommended to generate Ninja build files for fast and efficient compilation. 
An example script for building all dependencies (except OpenCL) and the xReg repository is also provided [here](example_build_script).
The [docker](docker) directory demonstrates how Docker may be used to build the software.

## License and Attribution
The software is available for use under the [MIT License](LICENSE).

If you have found this software useful in your work, we kindly ask that you cite the most appropriate references below:
```
Grupp, Robert B., et al. "Pose estimation of periacetabular osteotomy fragments with intraoperative X-ray navigation." IEEE Transactions on Biomedical Engineering 67.2 (2019): 441-452.
----------------------------------------------------------------------
@article{grupp2019pose,
  title={Pose estimation of periacetabular osteotomy fragments with intraoperative {X}-ray navigation},
  author={Grupp, Robert B and Hegeman, Rachel A and Murphy, Ryan J and Alexander, Clayton P and Otake, Yoshito and McArthur, Benjamin A and Armand, Mehran and Taylor, Russell H},
  journal={IEEE Transactions on Biomedical Engineering},
  volume={67},
  number={2},
  pages={441--452},
  year={2019},
  publisher={IEEE}
}
```
```
Grupp, Robert B., Mehran Armand, and Russell H. Taylor. "Patch-based image similarity for intraoperative 2D/3D pelvis registration during periacetabular osteotomy." OR 2.0 Context-Aware Operating Theaters, Computer Assisted Robotic Endoscopy, Clinical Image-Based Procedures, and Skin Image Analysis. Springer, Cham, 2018. 153-163.
----------------------------------------------------------------------
@incollection{grupp2018patch,
  title={Patch-based image similarity for intraoperative {2D}/{3D} pelvis registration during periacetabular osteotomy},
  author={Grupp, Robert B and Armand, Mehran and Taylor, Russell H},
  booktitle={OR 2.0 Context-Aware Operating Theaters, Computer Assisted Robotic Endoscopy, Clinical Image-Based Procedures, and Skin Image Analysis},
  pages={153--163},
  year={2018},
  publisher={Springer}
}
```
```
Grupp, Robert B., et al. "Automatic annotation of hip anatomy in fluoroscopy for robust and efficient 2D/3D registration." International Journal of Computer Assisted Radiology and Surgery (2020): 1-11.
----------------------------------------------------------------------
@article{grupp2020automatic,
  title={Automatic annotation of hip anatomy in fluoroscopy for robust and efficient {2D}/{3D} registration},
  author={Grupp, Robert B and Unberath, Mathias and Gao, Cong and Hegeman, Rachel A and Murphy, Ryan J and Alexander, Clayton P and Otake, Yoshito and McArthur, Benjamin A and Armand, Mehran and Taylor, Russell H},
  journal={International Journal of Computer Assisted Radiology and Surgery},
  pages={1--11},
  publisher={Springer}
}
```
```
Grupp, Robert, et al. "Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy." Physics in Medicine & Biology (2020).
----------------------------------------------------------------------
@article{grupp2020fast,
  title={Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy},
  author={Grupp, Robert and Murphy, Ryan and Hegeman, Rachel and Alexander, Clayton and Unberath, Mathias and Otake, Yoshito and McArthur, Benjamin and Armand, Mehran and Taylor, Russell H},
  journal={Physics in Medicine \& Biology},
  year={2020},
  publisher={IOP Publishing}
}
```
```
Grupp, R., et al. "Pelvis surface estimation from partial CT for computer-aided pelvic osteotomies." Orthopaedic Proceedings. Vol. 98. No. SUPP_5. The British Editorial Society of Bone & Joint Surgery, 2016.
----------------------------------------------------------------------
@inproceedings{grupp2016pelvis,
  title={Pelvis surface estimation from partial {CT} for computer-aided pelvic osteotomies},
  author={Grupp, R and Otake, Y and Murphy, R and Parvizi, J and Armand, M and Taylor, R},
  booktitle={Orthopaedic Proceedings},
  volume={98},
  number={SUPP\_5},
  pages={55--55},
  year={2016},
  organization={The British Editorial Society of Bone \& Joint Surgery}
}
```
```
Grupp, Robert B., et al. "Smooth extrapolation of unknown anatomy via statistical shape models." Medical Imaging 2015: Image-Guided Procedures, Robotic Interventions, and Modeling. Vol. 9415. International Society for Optics and Photonics, 2015.
----------------------------------------------------------------------
@inproceedings{grupp2015smooth,
  title={Smooth extrapolation of unknown anatomy via statistical shape models},
  author={Grupp, Robert B and Chiang, H and Otake, Yoshito and Murphy, Ryan J and Gordon, Chad R and Armand, Mehran and Taylor, Russell H},
  booktitle={Medical Imaging 2015: Image-Guided Procedures, Robotic Interventions, and Modeling},
  volume={9415},
  pages={941524},
  year={2015},
  organization={International Society for Optics and Photonics}
}
```
