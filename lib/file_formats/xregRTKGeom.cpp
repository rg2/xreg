/*
 * MIT License
 *
 * Copyright (c) 2021 Robert Grupp
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

#include "xregRTKGeom.h"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "xregAssert.h"
#include "xregStringUtils.h"

xreg::RTKGeomInfo xreg::ReadRTKGeomInfoFromXMLFile(const std::string& xml_path)
{
  using boost::property_tree::ptree;

  RTKGeomInfo geom_info;

  ptree pt;
  read_xml(xml_path, pt);

  auto& geom_children = pt.get_child("RTKThreeDCircularGeometry");

  geom_info.src_to_iso_center_dist_mm = geom_children.get<CoordScalar>(
                                                "SourceToIsocenterDistance");
  
  geom_info.src_to_det_dist_mm = geom_children.get<CoordScalar>(
                                                "SourceToDetectorDistance");

  for (auto it = geom_children.begin(); it != geom_children.end(); ++it)
  {
    if (it->first == "Projection")
    {
      RTKGeomInfo::ProjInfo tmp_proj_info;
      
      auto& proj_node = it->second;

      tmp_proj_info.gantry_ang_deg = proj_node.get<CoordScalar>("GantryAngle");

      try
      {
        tmp_proj_info.proj_off_x = proj_node.get<CoordScalar>("ProjectionOffsetX");
      }
      catch (const boost::property_tree::ptree_bad_path&)
      {
        tmp_proj_info.proj_off_x = 0;
      }

      try
      {
        tmp_proj_info.proj_off_y = proj_node.get<CoordScalar>("ProjectionOffsetY");
      }
      catch (const boost::property_tree::ptree_bad_path&)
      {
        tmp_proj_info.proj_off_y = 0;
      }

      const auto mat_vals = StringCast<CoordScalar>(
                              StringSplit(proj_node.get<std::string>("Matrix"), " \t\n\r"));
     
      xregASSERT(mat_vals.size() == 12);

      int i = 0;
      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 4; ++c, ++i)
        {
          tmp_proj_info.cam_mat(r,c) = mat_vals[i];
        }
      }

      geom_info.proj_infos.push_back(tmp_proj_info);
    }
  }

  return geom_info;
}

