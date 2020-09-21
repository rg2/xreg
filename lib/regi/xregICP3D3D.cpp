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

#include "xregICP3D3D.h"

#include "xregPointCloudUtils.h"
#include "xregMesh.h"
#include "xregPairedPointRegi3D3D.h"
#include "xregBasicStats.h"
#include "xregStringUtils.h"

void xreg::PointToSurRegiICP::init()
{
  dout() << "ICP init..." << std::endl;

  xregASSERT(pts);
  xregASSERT(sur);

  if (!kd_tree)
  {
    dout() << "  Creating KD-Tree for target surface..." << std::endl;

    // The KD Tree for the surface has not been created - create it now
    kd_tree_tris = CreateTrisForKDTree(*sur);

    kd_tree.reset(new KDTree(kd_tree_tris.begin(), kd_tree_tris.end()));
    
    dout() << "    KD-Tree created." << std::endl;
  }
 
  dout() << "Allocating additional working buffers..." << std::endl;

  const size_type num_pts = pts->size();

  match_dists.resize(num_pts);
  match_pts.resize(num_pts);
  reg_pts.resize(num_pts);

  src_inliers.reserve(num_pts);
  match_inliers.reserve(num_pts);
  
  dout() << "ICP init complete!" << std::endl;
}

xreg::FrameTransform
xreg::PointToSurRegiICP::run()
{
  init();

  const size_type num_pts = pts->size();

  CoordScalar prev_mean_dist = -1.0;
  CoordScalar cur_mean_dist  = -1.0;
  CoordScalar ratio          =  2.0;

  const double xform_scale = allow_similarity ? -1.0 : 1.0;

  const bool filter_outliers = std_dev_outlier_coeff > 1.0e-8; 

  FrameTransform cur_xform = init_pts_to_sur_xform;

  dout() << "Starting Point to Surface ICP: Num Pts " << num_pts
         << "\n   Surface Num Verts " << sur->vertices.size()
         << ", Num Tri Faces " << sur->faces.size() << std::endl;
  dout() << "  Initial Xform:\n" << cur_xform.matrix() << std::endl;
  dout() << "    Stop Thresh: " << stop_ratio << std::endl;
  dout() << "  Compute Scale: " << BoolToYesNo(allow_similarity) << std::endl;
  dout() << "   Outlier Det.: " << BoolToYesNo(filter_outliers) << std::endl;

  size_type iter = 0;

  bool should_stop = max_its == 0;

  if (start_of_processing_callback_fn)
  {
    start_of_processing_callback_fn(this);
  }

  while (!should_stop)
  {
    dout() << "  Iteration " << iter << std::endl;
    dout() << "    Xform:\n" << cur_xform.matrix() << std::endl;

    if (start_of_iteration_callback_fn)
    {
      start_of_iteration_callback_fn(this, iter, cur_xform);
    }

    // Find closest point correspondences

    ApplyTransform(cur_xform, *pts, &reg_pts);

    kd_tree->find_closest_points(reg_pts, &match_pts, &match_dists);

    // Calculate the termination criteria
    cur_mean_dist = SampleMean(match_dists);

    dout() << "    Mean Dist: " << cur_mean_dist << std::endl;

    if ((max_its >= 0) && ((iter + 1) == static_cast<size_type>(max_its)))
    {
      dout() << "    termination criteria met: maximum number of iterations reached" << std::endl;
      should_stop = true;
    }
    else if (cur_mean_dist < 1.0e-12)
    {
      dout() << "    termination criteria met: mean distance below very small threshold" << std::endl;
      should_stop = true;
    }
    else if (prev_mean_dist >= 0.0)
    {
      // This is at least the second iteration, we can compute the mean distance
      // ratio

      ratio = cur_mean_dist / prev_mean_dist;
      should_stop = (stop_ratio <= ratio) && (ratio <= 1.0);

      dout() << "    Delta Mean Dist. Ratio: " << ratio << std::endl;
      if (should_stop)
      {
        dout() << "    termination criteria met: mean distance has stopped decreasing" << std::endl;
      }
    }

    // TODO: save some debug information

    if (!should_stop)
    {
      if (filter_outliers)
      {
        src_inliers.clear();
        match_inliers.clear();

        // Identify outlier points that have a transformed distance from the surface
        // greater than a number of standard deviations from the mean distance
        const double dist_thresh = cur_mean_dist + (std_dev_outlier_coeff * SampleStdDev(match_dists, cur_mean_dist));

        for (size_type i = 0; i < num_pts; ++i)
        {
          if (match_dists[i] <= dist_thresh)
          {
            src_inliers.push_back(pts->operator[](i));
            match_inliers.push_back(match_pts[i]);
          }
        }

        cur_xform = PairedPointRegi3D3D(src_inliers, match_inliers, xform_scale);
      }
      else
      {
        cur_xform = PairedPointRegi3D3D(*pts, match_pts, xform_scale);
      }

      prev_mean_dist = cur_mean_dist;
    }

    if (end_of_iteration_callback_fn)
    {
      end_of_iteration_callback_fn(this, iter, cur_xform);
    }

    ++iter;
  }
  
  if (end_of_processing_callback_fn)
  {
    end_of_processing_callback_fn(this);
  }

  return cur_xform;
}

