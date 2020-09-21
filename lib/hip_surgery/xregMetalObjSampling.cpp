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

#include "xregMetalObjSampling.h"

#include <itkImageRegionIteratorWithIndex.h>

#include "xregAssert.h"
#include "xregSampleUtils.h"
#include "xregHipSegUtils.h"
#include "xregITKBasicImageUtils.h"
#include "xregVTKMeshUtils.h"

xreg::CreateRandScrew::CreateRandScrew()
{
  SeedRNGEngWithRandDev(&rng_eng);
}

std::tuple<xreg::NaiveScrewModel,xreg::FrameTransform>
xreg::CreateRandScrew::operator()(const Pt3& start_pt, const Pt3& stop_pt)
{
  Pt3 dir_vec = stop_pt - start_pt;
  
  const CoordScalar len = dir_vec.norm();

  dir_vec /= len;

  std::uniform_real_distribution<CoordScalar> add_len_dist(2, 4);
  
  std::uniform_real_distribution<CoordScalar> cap_height_dist(3.5,4.5);
  std::uniform_real_distribution<CoordScalar> cap_rad_dist(3.5,4.5);

  std::uniform_real_distribution<CoordScalar> body_rad_dist(1.8,2.2);

  std::uniform_real_distribution<CoordScalar> tip_height_dist(2,4);

  NaiveScrewModel s;

  s.cap.radius = cap_height_dist(rng_eng);
  s.cap.height = cap_rad_dist(rng_eng);

  s.body.radius = body_rad_dist(rng_eng);
  s.body.height = len + add_len_dist(rng_eng);

  s.tip.radius = s.body.radius;
  s.tip.height = tip_height_dist(rng_eng);

  FrameTransform screw_to_world_xform = FrameTransform::Identity();

  // choose somewhat arbitrary X axis orthogonal to the direction vector
  Pt3 x_vec = Pt3::Zero();

  std::vector<size_type> non_zero_comps;
  for (size_type i = 0; i < 3; ++i)
  {
    if (std::abs(dir_vec(i)) > 1.0e-6)
    {
      non_zero_comps.push_back(i);
    }
  }
  
  xregASSERT(!non_zero_comps.empty());

  if (non_zero_comps.size() >= 2)
  {
    const size_type i1 = non_zero_comps[0];
    const size_type i2 = non_zero_comps[1];
    
    x_vec(i1) = -dir_vec(i2);
    x_vec(i2) = dir_vec(i1);

    x_vec /= x_vec.norm();
  }
  else
  {
    xregASSERT(non_zero_comps.size() == 1);

    x_vec((non_zero_comps.front() + 1) % 3) = 1;
  }

  xregASSERT(std::abs(x_vec.norm() - 1) < 1.0e-6);
  xregASSERT(std::abs(x_vec.dot(dir_vec)) < 1.0e-6); 

  screw_to_world_xform.matrix().block(0,0,3,1) = x_vec;
  screw_to_world_xform.matrix().block(0,1,3,1) = dir_vec;
  screw_to_world_xform.matrix().block(0,2,3,1) = x_vec.cross(dir_vec);
  screw_to_world_xform.matrix().block(0,3,3,1) = start_pt + (CoordScalar(0.5) * len * dir_vec);

  return std::make_tuple(s, screw_to_world_xform);
}

xreg::CreateRandKWire::CreateRandKWire()
{
  SeedRNGEngWithRandDev(&rng_eng);
}
  
std::tuple<xreg::NaiveKWireModel,xreg::FrameTransform>
xreg::CreateRandKWire::operator()(const Pt3& iliac_entry_pt, const Pt3& stop_pt)
{
  this->dout() << "creating random k-wire..." << std::endl;

  Pt3 dir_vec = stop_pt - iliac_entry_pt;
  
  const CoordScalar insertion_len = dir_vec.norm();

  this->dout() << "insertion len: " << insertion_len << std::endl;

  dir_vec /= insertion_len;

  // [7.5", 10.5"]
  std::uniform_real_distribution<CoordScalar> len_dist(190.5, 266.7);
  
  std::uniform_real_distribution<CoordScalar> body_rad_dist(0.5,1.5);

  std::uniform_real_distribution<CoordScalar> tip_height_dist(4,6);

  NaiveKWireModel s;

  s.body.radius = body_rad_dist(rng_eng);
  s.body.height = len_dist(rng_eng);

  this->dout() << "body radius: " << s.body.radius
               << "\nbody height: " << s.body.height << std::endl;

  s.tip.radius = s.body.radius;
  s.tip.height = tip_height_dist(rng_eng);

  this->dout() << "tip height: " << s.tip.height << std::endl;

  FrameTransform wire_to_world_xform = FrameTransform::Identity();

  // choose somewhat arbitrary X axis orthogonal to the direction vector
  Pt3 x_vec = Pt3::Zero();

  std::vector<size_type> non_zero_comps;
  for (size_type i = 0; i < 3; ++i)
  {
    if (std::abs(dir_vec(i)) > 1.0e-6)
    {
      non_zero_comps.push_back(i);
    }
  }
  
  xregASSERT(!non_zero_comps.empty());

  if (non_zero_comps.size() >= 2)
  {
    const size_type i1 = non_zero_comps[0];
    const size_type i2 = non_zero_comps[1];
    
    x_vec(i1) = -dir_vec(i2);
    x_vec(i2) = dir_vec(i1);

    x_vec /= x_vec.norm();
  }
  else
  {
    xregASSERT(non_zero_comps.size() == 1);

    x_vec((non_zero_comps.front() + 1) % 3) = 1;
  }

  xregASSERT(std::abs(x_vec.norm() - 1) < 1.0e-6);
  xregASSERT(std::abs(x_vec.dot(dir_vec)) < 1.0e-6); 

  const Pt3 wire_start = iliac_entry_pt - ((s.body.height - insertion_len) * dir_vec);

  wire_to_world_xform.matrix().block(0,0,3,1) = x_vec;
  wire_to_world_xform.matrix().block(0,1,3,1) = dir_vec;
  wire_to_world_xform.matrix().block(0,2,3,1) = x_vec.cross(dir_vec);
  wire_to_world_xform.matrix().block(0,3,3,1) = (stop_pt + wire_start) / CoordScalar(2);

  return std::make_tuple(s, wire_to_world_xform);
}
  
void xreg::PAOSampleScrewWireInsertionPts::init()
{
  xregASSERT(ImagesHaveSameCoords(insert_labels.GetPointer(), cut_seg.GetPointer()));

  this->dout() << "guess labels..." << std::endl;
  
  LabelScalar pelvis_label;
  LabelScalar femur_label;
  LabelScalar frag_label;
  LabelScalar cut_label;

  std::tie(pelvis_label,femur_label,frag_label,cut_label) =
              GuessPelvisFemurPAOFragLabels(cut_seg.GetPointer(), femur_pt_wrt_vol);

  this->dout() << "Labels:\n"
               << "        Pelvis: " << static_cast<int>(pelvis_label) << '\n'
               << "          Frag: " << static_cast<int>(frag_label)   << '\n'
               << "           Cut: " << static_cast<int>(cut_label)    << '\n'
               << "         Femur: " << static_cast<int>(femur_label)  << std::endl;

  this->dout() << "trimming insertion point label map..." << std::endl;

  insert_labels_trim = ITKImageDeepCopy(insert_labels.GetPointer());

  insertion_indices.clear();

  itk::ImageRegionIteratorWithIndex<LabelVol> insert_labels_it(insert_labels_trim,
                                    insert_labels->GetLargestPossibleRegion());

  while (!insert_labels_it.IsAtEnd())
  {
    auto& cur_insert_label = insert_labels_it.Value();

    // only look at voxels that are possible candidates
    if (cur_insert_label)
    {
      const auto itk_idx = insert_labels_it.GetIndex();
    
      const LabelScalar cut_seg_val = cut_seg->GetPixel(itk_idx);
   
      // if the current label is actually part of a fragment or cut, discard it 
      if ((cur_insert_label != frag_label) && (cur_insert_label != cut_label))
      {
        // Make sure that there is at least one adjacent non-bone pixel
        
        std::array<long,3> idx_offs = { -1, 0, 1 };

        bool found_non_bone = false;

        for (long x_off : idx_offs)
        {
          for (long y_off : idx_offs)
          {
            for (long z_off : idx_offs)
            {
              if (x_off || y_off || z_off)
              {
                ITKIndex neighbor_idx = itk_idx;
                neighbor_idx[0] += x_off;
                neighbor_idx[1] += y_off;
                neighbor_idx[2] += z_off;
                
                if (!cut_seg->GetPixel(neighbor_idx))
                {
                  found_non_bone = true;
                }
              }

              if (found_non_bone)
              {
                break;
              }
            }  // for z_off

            if (found_non_bone)
            {
              break;
            }
          }  // for y_off

          if (found_non_bone)
          {
            break;
          }
        }  // for x_off
        
        if (found_non_bone)
        {
          cur_insert_label = 1;

          insertion_indices.push_back(itk_idx);
        }
        else
        {
          cur_insert_label = 0;
        }
      }
      else
      {
        cur_insert_label = 0;
      }
    }

    ++insert_labels_it;
  }

  xregASSERT(insertion_indices.size() >= 3);

  this->dout() << "creating fragment surface mesh..." << std::endl;
  
  VTKCreateMesh create_mesh;
  create_mesh.labels = { static_cast<double>(frag_label) };
  
  frag_mesh = create_mesh(cut_seg.GetPointer());

  this->dout() << "creating insertion surface mesh..." << std::endl;

  create_mesh.labels = { 1.0 };

  insertion_mesh = create_mesh(insert_labels_trim.GetPointer());

  // create a KD-Tree for fast closest point lookups
  this->dout() << "creating KD-Tree for closest point lookups on screw insertion mesh..." << std::endl;
  kd_tris_insertion = CreateTrisForKDTree(insertion_mesh);

  kdt_insertion.reset(new KDTree(kd_tris_insertion.begin(), kd_tris_insertion.end()));

  SeedRNGEngWithRandDev(&rng_eng);
}

void xreg::PAOSampleScrewWireInsertionPts::run(const FrameTransform& frag_xform)
{
  const FrameTransform frag_xform_wrt_vol = app_to_vol * frag_xform * app_to_vol.inverse();
  
  const Pt3 xform_femur_pt_wrt_vol = frag_xform_wrt_vol * femur_pt_wrt_vol;
 
  TriMesh frag_mesh_xform = frag_mesh;

  frag_mesh_xform.transform(frag_xform_wrt_vol);

  this->dout() << "creating fragment surface triangles for screw direction intersection..." << std::endl;

  const size_type num_tris = frag_mesh_xform.faces.size();

  std::vector<Tri3ForRay3Intersect> tris;
  tris.resize(num_tris);

  for (size_type tri_idx = 0; tri_idx < num_tris; ++tri_idx)
  {
    auto& cur_tri = tris[tri_idx];

    auto& cur_face = frag_mesh_xform.faces[tri_idx];

    cur_tri.verts[0] = &frag_mesh_xform.vertices[cur_face[0]];
    cur_tri.verts[1] = &frag_mesh_xform.vertices[cur_face[1]];
    cur_tri.verts[2] = &frag_mesh_xform.vertices[cur_face[2]];
  
    cur_tri.init();
  }

  std::vector<KDTreeTri> kd_tris_frag;
  
  this->dout() << "creating KD-Tree for closest point lookups on transformed fragment mesh..." << std::endl;
  kd_tris_frag = CreateTrisForKDTree(frag_mesh_xform);

  KDTree kdt_frag(kd_tris_frag.begin(), kd_tris_frag.end());

  std::uniform_int_distribution<size_type> insertion_indices_dist(0, insertion_indices.size() - 1);

  // randomly choose 2 or 3 screws/K-wires
  const size_type num_objs = (std::uniform_real_distribution<double>(0,1)(rng_eng) < prob_two_objs) ? 2 : 3;
  this->dout() << num_objs << " objects (screws/K-wires/etc)..." << std::endl;

  obj_starts.clear();
  obj_starts.reserve(num_objs);
  
  obj_ends.clear();
  obj_ends.reserve(num_objs);

  CoordScalar cur_obj_min_sep_thresh = 20;
  size_type   num_obj_start_rejects  = 0;

  for (size_type obj_idx = 0; obj_idx < num_objs;)
  {
    this->dout() << "chosing insertion point: " << obj_idx << std::endl;

    const ITKIndex insert_idx = insertion_indices[insertion_indices_dist(rng_eng)];
    
    itk::Point<CoordScalar,3> itk_pt;
    insert_labels_trim->TransformIndexToPhysicalPoint(insert_idx, itk_pt);

    Pt3 obj_start = { itk_pt[0], itk_pt[1], itk_pt[2] };
  
    const KDTreeTri* closest_tri = nullptr;
    
    obj_start = std::get<0>(kdt_insertion->find_closest_point(obj_start, &closest_tri));

    // make sure the candidate is sufficiently far away from the previously
    // chosen start points
    bool start_is_good = true;

    for (const Pt3& prev_pt : obj_starts)
    {
      if ((prev_pt - obj_start).norm() < cur_obj_min_sep_thresh)
      {
        start_is_good = false;
        
        ++num_obj_start_rejects;

        if (num_obj_start_rejects >= 100000)
        {
          // we're having trouble sampling a starting point, make the minimum
          // separating distance threshold less strict

          num_obj_start_rejects = 0;
          cur_obj_min_sep_thresh /= 1.5;
        }
        
        break;
      }
    }

    if (start_is_good)
    {
      Pt3 insert_normal = (*closest_tri->verts[1] - *closest_tri->verts[0]).cross(
                             *closest_tri->verts[2] - *closest_tri->verts[0]);
      insert_normal /= insert_normal.norm();

      this->dout() << "finding object end for object: " << obj_idx << std::endl;

      Pt3 obj_start_to_fh = xform_femur_pt_wrt_vol - obj_start;
      obj_start_to_fh /= obj_start_to_fh.norm();

      const Pt3 closest_pt_on_frag = std::get<0>(kdt_frag.find_closest_point(obj_start));
      
      Pt3 obj_start_to_closest_frag_dir = closest_pt_on_frag - obj_start;
      obj_start_to_closest_frag_dir /= obj_start_to_closest_frag_dir.norm();

      Pt3 init_obj_dir_vec = (obj_start_to_closest_frag_dir + insert_normal) / 2; 
      init_obj_dir_vec /= init_obj_dir_vec.norm();

      CoordScalar dist_to_inter = -1;

      SurIntersectInfo inter_info;

      CoordScalar obj_start_to_fh_frac = 0.5;
      
      bool end_is_good = false;

      for (size_type inter_trial_idx = 0; inter_trial_idx < 6;
           obj_start_to_fh_frac += 0.1, ++inter_trial_idx)
      {
        this->dout() << obj_start_to_fh_frac << std::endl;

        Ray3 r;
        r.pt  = obj_start;
        r.dir = (obj_start_to_fh_frac * obj_start_to_fh) +
                ((1 - obj_start_to_fh_frac) * init_obj_dir_vec); 
        r.dir /= r.dir.norm();

        std::tie(dist_to_inter, inter_info) = ExhaustiveRayTrisIntersect(r, tris);
        
        const CoordScalar obj_len = (obj_start - inter_info.inter_pt).norm();

        // terminate this loop on intersection and reasonable object length
        if ((dist_to_inter > -1.0e-6) &&
            (39.99999 < obj_len) && (obj_len < 110.00001))
        {
          end_is_good = true;
          break;
        }
      }
     
      if (end_is_good)
      {
        obj_starts.push_back(obj_start);
        obj_ends.push_back(inter_info.inter_pt);
        ++obj_idx;
      }
    }  // end if start_is_good
  }  // end for screw_idx
}
