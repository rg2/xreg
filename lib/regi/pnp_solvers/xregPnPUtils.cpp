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

#include "xregPnPUtils.h"

#include "xregPOSIT.h"
#include "xregLandmark2D3DRegiReprojDistCMAES.h"

xreg::FrameTransform 
xreg::PnPPOSITAndReprojCMAES(const CameraModel& cam,
                             const LandMap3& pts_3d,
                             const LandMap2& inds_2d)
{
  POSIT posit;
  
  posit.set_cam(cam);

  posit.set_inds_2d_and_world_pts_3d(inds_2d, pts_3d);

  posit.run();

  auto cmaes_regi = MakeLand2D3DRegiReprojCMAESStandardParams();

  cmaes_regi->set_cam(cam);
  cmaes_regi->set_inds_2d(posit.inds_2d());
  cmaes_regi->set_world_pts_3d(posit.world_pts_3d());

  cmaes_regi->set_init_cam_to_world(posit.regi_cam_to_world());
  
  cmaes_regi->run();

  return cmaes_regi->regi_cam_to_world();
}

xreg::FrameTransform xreg::PnPReprojCMAES(const CameraModel& cam,
                                          const LandMap3& pts_3d,
                                          const LandMap2& inds_2d,
                                          const FrameTransform init_cam_to_world)
{
  auto cmaes_regi = MakeLand2D3DRegiReprojCMAESStandardParams();

  cmaes_regi->set_cam(cam);
  cmaes_regi->set_inds_2d_and_world_pts_3d(inds_2d, pts_3d);

  cmaes_regi->set_init_cam_to_world(init_cam_to_world);
  
  cmaes_regi->run();

  return cmaes_regi->regi_cam_to_world();
}

