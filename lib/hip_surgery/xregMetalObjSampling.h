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

#ifndef XREGMETALOBJSAMPLING_H_
#define XREGMETALOBJSAMPLING_H_

#include <random>

#include "xregMetalObjs.h"
#include "xregObjWithOStream.h"
#include "xregMesh.h"
#include "xregKDTree.h"

namespace xreg
{

struct CreateRandScrew
{
  std::mt19937 rng_eng;

  CreateRandScrew();

  std::tuple<NaiveScrewModel,FrameTransform> operator()(const Pt3& start_pt, const Pt3& stop_pt);
};

struct CreateRandKWire : public ObjWithOStream
{
  std::mt19937 rng_eng;

  CreateRandKWire();

  std::tuple<NaiveKWireModel,FrameTransform> operator()(const Pt3& iliac_entry_pt, const Pt3& stop_pt);
};

struct PAOSampleScrewWireInsertionPts : public ObjWithOStream
{
  using LabelScalar = unsigned char;
  using LabelVol    = itk::Image<LabelScalar,3>;
  using LabelVolPtr = LabelVol::Pointer;
  using ITKIndex    = LabelVol::IndexType;
  using KDTree      = KDTreeNode<KDTreeTri>;

  LabelVolPtr insert_labels;

  LabelVolPtr cut_seg;

  FrameTransform app_to_vol;

  Pt3 femur_pt_wrt_vol;
  
  // probability of inserting two objects, the remaining
  // probaility is the chance of inserting three objects
  double prob_two_objs = 0.5;

  void init();

  void run(const FrameTransform& frag_xform);

  // These are populated after calling init()
  
  LabelVolPtr insert_labels_trim;
    
  std::vector<ITKIndex> insertion_indices;
    
  TriMesh frag_mesh;
    
  TriMesh insertion_mesh;
    
  std::vector<KDTreeTri> kd_tris_insertion;
  
  std::unique_ptr<KDTree> kdt_insertion;
  
  std::mt19937 rng_eng;

  // These are populated after calling run()
  
  Pt3List obj_starts;
  Pt3List obj_ends;
};

}  // xreg

#endif

