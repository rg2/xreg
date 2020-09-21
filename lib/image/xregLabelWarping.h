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

#ifndef XREGLABELWARPING_H_
#define XREGLABELWARPING_H_

#include <random>

#include <boost/container/flat_set.hpp>

#include <itkInterpolateImageFunction.h>

#include "xregCommon.h"

namespace xreg
{

/// \brief Determine if a transformation applied to a set of labels will result
///        in a collision with a set of other labels
///
/// delta_xform_inv is the inverse warp that would be used to "pull" the moving
/// labels to their destination locations.
bool CheckCollisionBetweenLabelSets(const itk::Image<unsigned char,3>* labels,
                                    const boost::container::flat_set<unsigned char>& mov_labels,
                                    const boost::container::flat_set<unsigned char>& fixed_labels,
                                    const FrameTransform& delta_xform_inv);

/// \brief Sample random rigid transformations that will warp some set of labels,
///        but check to make sure the warping will not cause a collision with
///        another set of labels.
struct SampleValidLabelWarpsFn
{
  using LabelScalar = unsigned char;
  using LabelImage  = itk::Image<LabelScalar,3>;

  using LabelSet = boost::container::flat_set<LabelScalar>;

  const LabelImage* labels;
  
  LabelSet mov_labels;
  LabelSet fixed_labels; 

  FrameTransform inter_to_vol_xform;
  
  size_type num_xforms;

  bool check_for_collision;

  // These are the output poses/warps
  FrameTransformList delta_inter_xforms;

  size_type num_samples;

  // this will seed the RNG engine
  SampleValidLabelWarpsFn();

  // perform the sampling
  void operator()();

  double rejection_prob() const;

  using RNGEng = std::mt19937;

  RNGEng rng_eng;

  // Collecting the individual rotations parameters just to look at
  // their statistics later. Can be saved to CSV
  std::vector<CoordScalarList> xform_vals;
  
  using NormalDist  = std::normal_distribution<CoordScalar>;
  using LogNormDist = std::lognormal_distribution<CoordScalar>;
  using UniDist     = std::uniform_real_distribution<CoordScalar>;
  
  // This base class does not perform any randomness with each method returning 0
  struct SampleRepoParams
  {
    virtual void setup(RNGEng*);

    virtual CoordScalar rot_x();
    
    virtual CoordScalar rot_y();

    virtual CoordScalar rot_z();

    virtual CoordScalar trans_x();
    
    virtual CoordScalar trans_y();

    virtual CoordScalar trans_z();
  };

  struct SampleRepoParamsAllNormal : SampleRepoParams
  {
    Pt3 rot_mean;
    Pt3 rot_std_dev;
    
    Pt3 trans_mean;
    Pt3 trans_std_dev;
   
    RNGEng* rng_eng;

    NormalDist rot_x_dist;
    NormalDist rot_y_dist;
    NormalDist rot_z_dist;

    NormalDist trans_x_dist;
    NormalDist trans_y_dist;
    NormalDist trans_z_dist;

    void setup(RNGEng* rng_eng_arg) override;

    CoordScalar rot_x() override;
    
    CoordScalar rot_y() override;

    CoordScalar rot_z() override;

    CoordScalar trans_x() override;
    
    CoordScalar trans_y() override;

    CoordScalar trans_z() override;
  };

  struct SampleRepoParamsAllUniform : SampleRepoParams
  {
    Pt3 rot_lower;
    Pt3 rot_upper;
    
    Pt3 trans_lower;
    Pt3 trans_upper;
   
    RNGEng* rng_eng;

    UniDist rot_x_dist;
    UniDist rot_y_dist;
    UniDist rot_z_dist;

    UniDist trans_x_dist;
    UniDist trans_y_dist;
    UniDist trans_z_dist;

    void setup(RNGEng* rng_eng_arg) override;

    CoordScalar rot_x() override;
    
    CoordScalar rot_y() override;

    CoordScalar rot_z() override;

    CoordScalar trans_x() override;
    
    CoordScalar trans_y() override;

    CoordScalar trans_z() override;
  };
  
  struct SampleRepoParamsLogNormalMixFrag : SampleRepoParams
  {
    Pt3 rot_m;
    Pt3 rot_s;

    Pt3 trans_mean;
    Pt3 trans_std_dev;

    RNGEng* rng_eng;

    LogNormDist rot_x_dist;
    LogNormDist rot_y_dist;
    LogNormDist rot_z_dist;
    
    NormalDist trans_x_dist;
    NormalDist trans_y_dist;
    NormalDist trans_z_dist;

    bool negate_rot_x;
    bool negate_rot_y;
    bool negate_rot_z;

    void setup(RNGEng* rng_eng_arg) override;

    CoordScalar rot_x() override;
    
    CoordScalar rot_y() override;

    CoordScalar rot_z() override;

    CoordScalar trans_x() override;
    
    CoordScalar trans_y() override;

    CoordScalar trans_z() override;
  };

  struct SampleRepoParamsLogNormalMixFemur : SampleRepoParams
  {
    Pt3 rot_m;
    Pt3 rot_s;

    RNGEng* rng_eng;

    LogNormDist rot_x_dist;
    NormalDist rot_y_dist;
    NormalDist rot_z_dist;

    bool negate_rot_x;

    void setup(RNGEng* rng_eng_arg) override;

    CoordScalar rot_x() override;
    
    CoordScalar rot_y() override;

    CoordScalar rot_z() override;
  };

  std::unique_ptr<SampleRepoParams> param_sampler;
};

/// \brief Warps the voxels of an image that match a certain label.
itk::Image<float,3>::Pointer
WarpImageLimitedToLabel(const itk::Image<unsigned char,3>* labels,
                        const unsigned char label_to_warp,
                        const itk::Image<float,3>* img_to_warp,
                        const FrameTransform& warp_xform_inv,
                        const itk::InterpolateImageFunction<itk::Image<float,3>>* interp_fn,
                        const float other_val);

}  // xreg

#endif

