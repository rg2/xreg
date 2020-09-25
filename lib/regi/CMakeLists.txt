# MIT License
#
# Copyright (c) 2020 Robert Grupp
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

file(GLOB_RECURSE XREG_CUR_LIB_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(xreg_regi OBJECT ${XREG_CUR_LIB_HEADERS}
                             xregPairedPointRegi3D3D.cpp
                             xregICP3D3D.cpp
                             xregSE3OptVars.cpp
                             sim_metrics_2d/xregImgSimMetric2D.cpp
                             sim_metrics_2d/xregImgSimMetric2DCombine.cpp
                             sim_metrics_2d/xregImgSimMetric2DPatchCommon.cpp
                             sim_metrics_2d/xregImgSimMetric2DCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DSSDCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DNCCCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradImgCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradNCCCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradDiffCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradOrientCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DBoundaryEdgesCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DPatchNCCCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DPatchGradNCCCPU.cpp
                             sim_metrics_2d/xregImgSimMetric2DOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DSSDOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DNCCOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradImgOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DGradNCCOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DPatchNCCOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DPatchGradNCCOCL.cpp
                             sim_metrics_2d/xregImgSimMetric2DProgOpts.cpp
                             pnp_solvers/xregLandmark2D3DRegi.cpp
                             pnp_solvers/xregLandmark2D3DRegiReprojDist.cpp
                             pnp_solvers/xregLandmark2D3DRegiReprojDistCMAES.cpp
                             pnp_solvers/xregPOSIT.cpp
                             pnp_solvers/xregP3PCArm.cpp
                             pnp_solvers/xregRANSACPnP.cpp
                             pnp_solvers/xregPnPUtils.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFn.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnCombo.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnGlobalPelvis.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnLandReproj.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnPelvisAP.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnSE3Mag.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnSE3EulerDecomp.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnRelPose.cpp
                             penalty_fns_2d_3d/xregRegi2D3DPenaltyFnDebugH5.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegi.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiDebug.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiCMAES.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiDiffEvo.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiPSO.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiExhaustive.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiHillClimb.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiNLOptInterface.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiBOBYQA.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiCRS.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiDIRECT.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiDESCH.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiDISRES.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiNEWUOA.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiNelderMead.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiPRAXIS.cpp
                             interfaces_2d_3d/xregIntensity2D3DRegiSbplx.cpp
                             interfaces_2d_3d/xregMultiObjMultiLevel2D3DRegi.cpp
                             interfaces_2d_3d/xregMultiObjMultiLevel2D3DRegiDebug.cpp)

target_compile_definitions(xreg_regi PRIVATE VIENNACL_WITH_OPENCL)

