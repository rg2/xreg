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

#include "xregFitPlane.h"

#include "xregExceptionUtils.h"
#include "xregSampleUtils.h"
#include "xregAssert.h"
#include "xregCMAESInterface.h"

std::tuple<xreg::Plane3,xreg::Pt3>
xreg::FitPlaneToPoints(const Pt3List& pts)
{
  const size_type num_pts = pts.size();

  Pt3 mean_pt = Pt3::Zero();

  for (size_type i = 0; i < num_pts; ++i)
  {
    mean_pt += pts[i];
  }

  mean_pt /= num_pts;

  MatMxN data_mat(3,num_pts);

  for (size_type i = 0; i < num_pts; ++i)
  {
    data_mat.col(i) = pts[i] - mean_pt;
  }

  Eigen::JacobiSVD<MatMxN> svd(data_mat, Eigen::ComputeFullU);

  Plane3 plane;

  plane.normal = svd.matrixU().col(2);
  plane.scalar = plane.normal.dot(mean_pt);

  return std::make_tuple(plane, mean_pt);
}

xreg::Plane3 xreg::FitPlaneToPoints(const Pt3& x1, const Pt3& x2, const Pt3& x3,
                                    const bool checked)
{
  Pt3 n = (x2 - x1).cross(x3 - x1);
  
  const CoordScalar n_norm = n.norm();
  
  if (checked && (n_norm < 1.0e-8))
  {
    xregThrow("Points are colinear!");
  }

  n /= n_norm;

  return Plane3{n, n.dot(x1)};
}

std::tuple<xreg::Plane3,xreg::Pt3,xreg::Pt3List>
xreg::FitPlaneToPointsRANSAC(const Pt3List& pts,
                             const double consensus_ratio_min_thresh,
                             const size_type max_num_its)
{
  const size_type num_pts = pts.size();
  xregASSERT(num_pts > 4);

  const size_type consensus_num_pts_min = std::lround(num_pts * consensus_ratio_min_thresh);

  std::mt19937 rng_eng;
  SeedRNGEngWithRandDev(&rng_eng);

  std::uniform_int_distribution<size_type> uni_dist(0, num_pts - 1);

  auto sample_inlier_inds = [&rng_eng,&uni_dist]()
  {
    std::array<size_type,3> inds;
    inds[0] = uni_dist(rng_eng);
    
    inds[1] = uni_dist(rng_eng);
    while (inds[1] == inds[0])
    {
      inds[1] = uni_dist(rng_eng);
    }
    
    inds[2] = uni_dist(rng_eng);
    while ((inds[2] == inds[0]) || (inds[2] == inds[1]))
    {
      inds[2] = uni_dist(rng_eng);
    }

    return inds;
  };

  std::vector<CoordScalar> all_dists(num_pts);

  std::vector<size_type> poss_inlier_inds;
  poss_inlier_inds.reserve(num_pts);

  Pt3List cur_model_pts;
  cur_model_pts.reserve(num_pts);

  Pt3List best_model_pts;
  best_model_pts.reserve(num_pts);

  std::array<size_type,3> cur_inliers;
  auto is_not_inlier = [&cur_inliers] (const size_type i)
  {
    return (i != cur_inliers[0]) && (i != cur_inliers[1]) && (i != cur_inliers[2]);
  };

  Plane3 best_plane = { Pt3::Zero(), 0 };
  CoordScalar best_avg_dist = std::numeric_limits<CoordScalar>::max();
  Pt3         best_pt_on_plane;

  bool at_least_one_consensus_found = false;

  bool should_stop = false;

  size_type iter = 0;

  while (!should_stop)
  {
    cur_inliers = sample_inlier_inds();

    auto cur_plane = FitPlaneToPoints(pts[cur_inliers[0]], pts[cur_inliers[1]],
                                      pts[cur_inliers[2]]);

    if (cur_plane.normal.norm() > 1.0e-8)
    {
      // compute the standard deviation of non-inlier distances to the current plane
      // we'll ignore points more than some number of standard deviations away from
      // the mean

      CoordScalar sum = 0;

      for (size_type i = 0; i < num_pts; ++i)
      {
        if (is_not_inlier(i))
        {
          all_dists[i] = DistToPlane(pts[i], cur_plane);
          
          sum += all_dists[i];
        }
      }

      const CoordScalar mean = sum / (num_pts - 3);
    
      sum = 0; 
      
      for (size_type i = 0; i < num_pts; ++i)
      {
        if (is_not_inlier(i))
        {
          const CoordScalar a = all_dists[i] - mean;
          
          sum += a * a;
        }
      }

      const CoordScalar std_dev = std::sqrt(sum / (num_pts - 4));
      
      const CoordScalar poss_inlier_thresh = mean + (1 * std_dev);

      // threshold to get the possible inliers
      poss_inlier_inds.clear(); 
      for (size_type i = 0; i < num_pts; ++i)
      {
        if (is_not_inlier(i) && (all_dists[i] < poss_inlier_thresh))
        {
          poss_inlier_inds.push_back(i);
        }
      }
      
      // see if we have a sufficient number of possible inliers
      if (poss_inlier_inds.size() >= consensus_num_pts_min)
      {
        at_least_one_consensus_found = true;

        // we have a consensus, now fit a plane using the consensus set union
        // the current inliers

        poss_inlier_inds.push_back(cur_inliers[0]);
        poss_inlier_inds.push_back(cur_inliers[1]);
        poss_inlier_inds.push_back(cur_inliers[2]);
      
        cur_model_pts.clear();

        for (const auto& i : poss_inlier_inds)
        {
          cur_model_pts.push_back(pts[i]);
        }

        Pt3 pt_on_plane;        
        std::tie(cur_plane,pt_on_plane) = FitPlaneToPoints(cur_model_pts);
        
        // compute the average error of this new model
        sum = 0;
        for (const auto& p : cur_model_pts)
        {
          sum += DistToPlane(p, cur_plane);
        }
        sum /= cur_model_pts.size();
        
        if (sum < best_avg_dist)
        {
          // the current model is a better bit to inliers/consensus than
          // the previous best model, update the best
          best_plane = cur_plane;
          best_pt_on_plane = pt_on_plane;
          best_avg_dist = sum;
        
          best_model_pts = cur_model_pts;

          if (sum < 1.0e-3)
          {
            should_stop = true;
          }
        }
      }
    }
    
    ++iter;
    if (!should_stop && (iter == max_num_its))
    {
      should_stop = true;
    }
  }

  return std::make_tuple(best_plane, best_pt_on_plane, best_model_pts);
}

namespace
{

using namespace xreg;

class PlaneFitMAPCMAES : public CmaesOptimizer
{
public:
  PlaneFitMAPCMAES(const Pt3List& pts, const Plane3& p0)
    : CmaesOptimizer(3,50), pts_(pts), p0_(p0)
  {
    this->obj_fn_exec_type_ = CmaesOptimizer::kPARALLEL_OBJ_FN_EVAL;
    
    set_sigma(Pt3({0.3,0.3,2}));
  }

protected:
  double obj_fn(const Pt& plane_params) override
  {
    const auto p = PlaneSphericalToNormalScalar(plane_params[0], plane_params[1], plane_params[2]);
    
    double f = 0;

    for (const auto& x : pts_)
    {
      const double tmp = p.normal.dot(x) - p.scalar;

      f += tmp * tmp;
    }

    f /= (2 * pts_.size());
    
    f += -p0_.normal.dot(p.normal) * 2;

    return f;
  }

private:

  const Pt3List& pts_;

  const Plane3 p0_;
};

}  // un-named

std::tuple<xreg::Plane3,xreg::Pt3>
xreg::FitPlaneToPointsMAP(const Pt3List& pts, const Plane3& p0)
{
  const size_type num_pts = pts.size();
  xregASSERT(num_pts > 4);

  constexpr bool use_mle_as_init = true;

  Plane3 init_plane;
  Pt3 pt_on_plane;
  std::tie(init_plane,pt_on_plane) = FitPlaneToPoints(pts);
 
  CoordScalar init_theta;
  CoordScalar init_phi;
  CoordScalar init_r;

  if (use_mle_as_init)
  {
    std::tie(init_theta,init_phi,init_r) = PlaneNormalScalarToSpherical(init_plane);
  }
  else
  {
    std::tie(init_theta,init_phi,init_r) = PlaneNormalScalarToSpherical(p0);
  }

  PlaneFitMAPCMAES map_fit(pts, p0);

  map_fit.set_init_guess(Pt3({ init_theta, init_phi, init_r }));
  
  map_fit.run();

  const auto fit_params = map_fit.solution();
  
  const auto fit_plane = PlaneSphericalToNormalScalar(fit_params[0], fit_params[1], fit_params[2]);

  return std::make_tuple(fit_plane, FindClosestOnPlane(pt_on_plane, fit_plane));
}
