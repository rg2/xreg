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

#include "xregMultiObjMultiLevel2D3DRegiDebug.h"

#include "xregHDF5.h"
#include "xregH5ProjDataIO.h"
#include "xregHUToLinAtt.h"
#include "xregITKIOUtils.h"
#include "xregITKLabelUtils.h"

xreg::size_type xreg::DebugRegiResultsMultiLevel::total_num_projs_per_view() const
{
  size_type num_projs = 0;

  const size_type num_levels = regi_results.size();

  for (size_type level_idx = 0; level_idx < num_levels; ++level_idx)
  {
    const size_type num_regis = regi_results[level_idx].size();

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      num_projs += regi_results[level_idx][regi_idx]->iter_vars[0].size() + 2; // +2 for initial and final
    }
  }

  return num_projs;
}

namespace  // un-named
{

using namespace xreg;

struct WriteDebugVolInfo : boost::static_visitor<void>
{
  H5::Group* g;

  void operator()(const DebugRegiResultsMultiLevel::VolPathInfo& vol_path_info)
  {
    SetStringAttr("vol-entry-type", "path-info", g);

    WriteStringH5("vol-path", vol_path_info.vol_path, g);
  
    if (!vol_path_info.label_vol_path.empty())
    {
      WriteStringH5("label-vol-path", vol_path_info.label_vol_path, g);

      xregASSERT(!vol_path_info.labels_used.empty());
      WriteVectorH5("labels-used", vol_path_info.labels_used, g);
    }

    WriteSingleScalarH5("paths-on-disk", vol_path_info.paths_on_disk, g);
  }

  void operator()(const RayCaster::VolPtr& vol)
  {
    SetStringAttr("vol-entry-type", "vol-data", g);
    WriteImageH5(vol.GetPointer(), g);
  }
};

struct WriteDebugProjDataInfo : boost::static_visitor<void>
{
  H5::Group* g;

  void operator()(const ProjDataF32List& pd)
  {
    SetStringAttr("projs-entry-type", "proj-data", g);
    WriteProjDataH5(pd, g);
  }

  void operator()(const DebugRegiResultsMultiLevel::ProjDataPathInfo& pd_path_info)
  {
    SetStringAttr("projs-entry-type", "path-info", g);
    
    WriteStringH5("path", pd_path_info.path, g);

    if (!pd_path_info.projs_used.empty())
    {
      WriteVectorH5("projs-used", pd_path_info.projs_used, g);
    }

    WriteSingleScalarH5("paths-on-disk", pd_path_info.paths_on_disk, g);
  }
};

}  // un-named

void xreg::WriteMultiLevel2D3DRegiDebugH5(const DebugRegiResultsMultiLevel& results, H5::Group* h5)
{
  if (!results.vols.empty())
  {
    H5::Group vols_g = h5->createGroup("vols");

    const size_type num_vol_entries = results.vols.size();

    WriteSingleScalarH5("num-entries", num_vol_entries, &vols_g);
  
    for (size_type vol_idx = 0; vol_idx < num_vol_entries; ++vol_idx)
    {
      H5::Group cur_vol_g = vols_g.createGroup(fmt::format("{:03d}", vol_idx));
    
      WriteDebugVolInfo write_debug_vol_info;
      write_debug_vol_info.g = &cur_vol_g;
      
      boost::apply_visitor(write_debug_vol_info, results.vols[vol_idx]);
    }
  }

  {
    H5::Group pd_g = h5->createGroup("fixed-projs");

    WriteDebugProjDataInfo write_pd_info;
    write_pd_info.g = &pd_g;

    boost::apply_visitor(write_pd_info, results.fixed_projs);
  }

  WriteSingleScalarH5("multi-level-time", results.multi_level_run_time_secs, h5);
  
  if (results.tot_time_secs)
  {
    WriteSingleScalarH5("total-time", *results.tot_time_secs, h5);
  }
  
  if (results.pre_proc_time_secs)
  {
    WriteSingleScalarH5("pre-proc-time", *results.pre_proc_time_secs, h5);
  }

  const size_type num_levels = results.multi_res_levels.size();
  xregASSERT(num_levels == results.regi_names.size());
  xregASSERT(num_levels == results.regi_results.size());

  WriteSingleScalarH5("num-levels", num_levels, h5);

  H5::Group levels_g = h5->createGroup("multi-res-levels");

  for (size_type level_idx = 0; level_idx < num_levels; ++level_idx)
  {
    H5::Group lvl_g = levels_g.createGroup(fmt::format("{:03d}", level_idx));
  
    WriteSingleScalarH5("ds-factor", results.multi_res_levels[level_idx], &lvl_g);

    const size_type num_regis_this_level = results.regi_results[level_idx].size();
    xregASSERT(results.regi_names[level_idx].size() == num_regis_this_level);
    
    WriteSingleScalarH5("num-regi", num_regis_this_level, &lvl_g); 
    
    for (size_type regi_idx = 0; regi_idx < num_regis_this_level; ++regi_idx)
    {
      H5::Group regi_g = lvl_g.createGroup(fmt::format("regi-{:03d}", regi_idx));

      WriteStringH5("name", results.regi_names[level_idx][regi_idx], &regi_g, false);

      WriteSingleRegiDebugResultsH5(*results.regi_results[level_idx][regi_idx], &regi_g);
    }
  }

  if (results.proj_pre_proc_info)
  {
    H5::Group pre_proc_g = h5->createGroup("proj-pre-proc-info");
    
    const auto& ppd = *results.proj_pre_proc_info;

    WriteSingleScalarH5("crop-width", ppd.crop_width, &pre_proc_g);
    
    WriteSingleScalarH5("auto-mask", ppd.auto_mask, &pre_proc_g);
    
    WriteSingleScalarH5("invert-mask", ppd.invert_mask, &pre_proc_g);
    
    if (ppd.auto_mask)
    {
      WriteSingleScalarH5("auto-mask-thresh", ppd.auto_mask_thresh, &pre_proc_g);
      WriteSingleScalarH5("auto-mask-level", ppd.auto_mask_level, &pre_proc_g);
      WriteSingleScalarH5("allow-iterative-thresh", ppd.allow_iterative_thresh, &pre_proc_g);
    }
    
    WriteSingleScalarH5("no-log-remap", ppd.no_log_remap, &pre_proc_g);
  }
}

void xreg::WriteMultiLevel2D3DRegiDebugToDisk(const DebugRegiResultsMultiLevel& results, const std::string& path)
{
  H5::H5File h5(path, H5F_ACC_TRUNC);

  WriteMultiLevel2D3DRegiDebugH5(results, &h5);

  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();
}

xreg::DebugRegiResultsMultiLevel xreg::ReadMultiLevel2D3DRegiDebugH5(const H5::Group& h5)
{
  DebugRegiResultsMultiLevel results;
 
  if (ObjectInGroupH5("vols", h5))
  {
    const H5::Group vols_g = h5.openGroup("vols");
  
    const size_type num_vol_entries = ReadSingleScalarH5ULong("num-entries", vols_g);
    
    results.vols.reserve(num_vol_entries);
    
    for (size_type vol_idx = 0; vol_idx < num_vol_entries; ++vol_idx)
    {
      const H5::Group cur_vol_g = vols_g.openGroup(fmt::format("{:03d}", vol_idx));
      
      const std::string entry_type_str = GetStringAttr("vol-entry-type", cur_vol_g);

      if (entry_type_str == "path-info")
      {
        DebugRegiResultsMultiLevel::VolPathInfo vol_path_info;
        
        vol_path_info.vol_path = ReadStringH5("vol-path", cur_vol_g);
        
        if (ObjectInGroupH5("label-vol-path", cur_vol_g))
        {
          vol_path_info.label_vol_path = ReadStringH5("label-vol-path", cur_vol_g);

          vol_path_info.labels_used = ReadVectorH5SizeType("labels-used", cur_vol_g);
        }

        vol_path_info.paths_on_disk = ReadSingleScalarH5Bool("paths-on-disk", cur_vol_g);

        results.vols.push_back(vol_path_info);
      }
      else if (entry_type_str == "vol-data")
      {
        results.vols.push_back(ReadITKImageH5Float3D(cur_vol_g));
      }
      else
      {
        xregThrow("Unsupported regi debug vol entry type: %s", entry_type_str.c_str());
      }
    }
  }
  
  {
    const H5::Group pd_g = h5.openGroup("fixed-projs");

    const std::string entry_type_str = GetStringAttr("projs-entry-type", pd_g);
    
    if (entry_type_str == "path-info")
    {
      DebugRegiResultsMultiLevel::ProjDataPathInfo pd_path_info;

      pd_path_info.path = ReadStringH5("path", pd_g);

      if (ObjectInGroupH5("projs-used", pd_g))
      {
        pd_path_info.projs_used = ReadVectorH5SizeType("projs-used", pd_g);
      }

      pd_path_info.paths_on_disk = ReadSingleScalarH5Bool("paths-on-disk", pd_g);

      results.fixed_projs = pd_path_info;
    }
    else if (entry_type_str == "proj-data")
    {
      results.fixed_projs = ReadProjDataH5F32(pd_g);
    }
    else
    {
      xregThrow("Unsupported proj data entry type: %s", entry_type_str.c_str());
    }
  }
 
  if (ObjectInGroupH5("total-time", h5))
  {
    results.tot_time_secs = ReadSingleScalarH5Double("total-time", h5);
  }
  
  const size_type num_levels = ReadSingleScalarH5ULong("num-levels", h5);

  results.multi_res_levels.assign(num_levels, 0);
  results.regi_results.resize(num_levels);
  results.regi_names.resize(num_levels);

  H5::Group levels_g = h5.openGroup("multi-res-levels");

  for (size_type level_idx = 0; level_idx < num_levels; ++level_idx)
  {
    H5::Group lvl_g = levels_g.openGroup(fmt::format("{:03d}", level_idx));
  
    results.multi_res_levels[level_idx] = ReadSingleScalarH5Double("ds-factor", lvl_g);
  
    auto& cur_level_results = results.regi_results[level_idx];
    auto& cur_level_names   = results.regi_names[level_idx];

    const size_type num_regis = ReadSingleScalarH5ULong("num-regi", lvl_g);

    cur_level_results.resize(num_regis);
    cur_level_names.resize(num_regis);

    for (size_type regi_idx = 0; regi_idx < num_regis; ++regi_idx)
    {
      H5::Group regi_g = lvl_g.openGroup(fmt::format("regi-{:03d}", regi_idx));
      
      cur_level_names[regi_idx] = ReadStringH5("name", regi_g);

      auto regi_results = std::make_shared<SingleRegiDebugResults>();
      
      *regi_results = ReadSingleRegiDebugResultsH5(regi_g);

      cur_level_results[regi_idx] = regi_results;
    }
  }
  
  if (ObjectInGroupH5("proj-pre-proc-info", h5))
  {
    H5::Group pre_proc_g = h5.openGroup("proj-pre-proc-info");
   
    ProjPreProcParams ppd;

    ppd.crop_width = ReadSingleScalarH5ULong("crop-width", pre_proc_g);

    ppd.auto_mask = ReadSingleScalarH5Bool("auto-mask", pre_proc_g);

    ppd.invert_mask = ReadSingleScalarH5Bool("invert-mask", pre_proc_g);

    if (ppd.auto_mask)
    {
      ppd.auto_mask_thresh = ReadSingleScalarH5Double("auto-mask-thresh", pre_proc_g);
      ppd.auto_mask_level  = ReadSingleScalarH5Double("auto-mask-level", pre_proc_g);
      
      ppd.allow_iterative_thresh = ReadSingleScalarH5Bool("allow-iterative-thresh", pre_proc_g);
    }

    ppd.no_log_remap = ReadSingleScalarH5Bool("no-log-remap", pre_proc_g);

    results.proj_pre_proc_info = ppd;
  }

  return results;
}

xreg::DebugRegiResultsMultiLevel xreg::ReadMultiLevel2D3DRegiDebugFromDisk(const std::string& path)
{
  return ReadMultiLevel2D3DRegiDebugH5(H5::H5File(path, H5F_ACC_RDONLY));
}

namespace  // un-named
{

using namespace xreg;

struct ReadVolDataFromDebug
  : boost::static_visitor<std::tuple<RayCaster::VolList,RayCaster::VolList>>
{
  using Vol         = RayCaster::Vol;
  using VolPtr      = RayCaster::VolPtr;
  using VolList     = RayCaster::VolList;
  using VolListPair = std::tuple<VolList,VolList>;

  const H5::Group* h5;
  
  bool hu_to_lin_att;

  VolListPair operator()(const VolPtr& vol) const
  {
    VolList hu_vols = { vol };
    VolList att_vols;

    if (hu_to_lin_att)
    {
      att_vols = { HUToLinAtt(hu_vols[0].GetPointer()) };
    }

    return std::make_tuple(hu_vols, att_vols);
  }

  VolListPair operator()(const DebugRegiResultsMultiLevel::VolPathInfo& vol_path_info) const
  {
    using LabelScalar = unsigned char;
    using LabelVol    = itk::Image<LabelScalar,3>;
    using LabelVolPtr = LabelVol::Pointer;
    using LabelList   = std::vector<LabelScalar>;

    VolList hu_vols;
    VolList att_vols;

    VolPtr single_vol;
    LabelVolPtr label_vol;
    
    LabelList labels_to_use;
    for (const auto& l : vol_path_info.labels_used)
    {
      labels_to_use.push_back(static_cast<unsigned char>(l));
    }

    const bool has_seg = !vol_path_info.label_vol_path.empty();

    if (vol_path_info.paths_on_disk)
    {
      single_vol = ReadITKImageFromDisk<Vol>(vol_path_info.vol_path);

      if (has_seg)
      {
        label_vol = ReadITKImageFromDisk<LabelVol>(vol_path_info.label_vol_path);
      }
    }
    else if (h5)
    {
      single_vol = ReadITKImageH5Float3D(h5->openGroup(vol_path_info.vol_path));
      
      if (has_seg)
      {
        label_vol = ReadITKImageH5UChar3D(h5->openGroup(vol_path_info.label_vol_path));
      }
    }
    else
    {
      xregThrow("HDF5 object must be provided when paths are not on disk!");
    }

    VolPtr single_att_vol;

    if (hu_to_lin_att)
    {
      single_att_vol = HUToLinAtt(single_vol.GetPointer());
    }

    if (has_seg)
    {
      xregASSERT(!labels_to_use.empty());

      hu_vols = MakeVolListFromVolAndLabels(single_vol.GetPointer(),
                                            label_vol.GetPointer(),
                                            labels_to_use,
                                            -1000);
      
      if (hu_to_lin_att)
      {
        att_vols = MakeVolListFromVolAndLabels(single_att_vol.GetPointer(),
                                               label_vol.GetPointer(),
                                               labels_to_use,
                                               0);
      }
    }
    else
    {
      hu_vols = { single_vol };

      if (hu_to_lin_att)
      {
        att_vols = { single_att_vol };
      }
    }

    return std::make_tuple(hu_vols, att_vols);
  }
};

struct ReadProjDataFromDebug : boost::static_visitor<ProjDataF32List>
{
  const H5::Group* h5;

  ProjDataF32List operator()(const ProjDataF32List& pd)
  {
    return pd;
  }

  ProjDataF32List operator()(const DebugRegiResultsMultiLevel::ProjDataPathInfo& pd_path_info)
  {
    ProjDataF32List pd;

    if (pd_path_info.paths_on_disk)
    {
      pd = ReadProjDataH5F32FromDisk(pd_path_info.path);
    }
    else if (h5)
    {
      pd = ReadProjDataH5F32(h5->openGroup(pd_path_info.path));
    }
    else
    {
      xregThrow("HDF5 object must be provided when paths are not on disk!");
    }

    if (!pd_path_info.projs_used.empty())
    {
      ProjDataF32List new_pd;
      new_pd.reserve(pd_path_info.projs_used.size());
      
      for (auto& proj_idx : pd_path_info.projs_used)
      {
        new_pd.push_back(pd[proj_idx]);
      }

      new_pd.swap(pd);
    }

    return pd;
  }
};

}  // un-named

std::tuple<xreg::RayCaster::VolList,
           xreg::RayCaster::VolList>
xreg::VolDataFromDebug(const DebugRegiResultsMultiLevel& results,
                       const bool do_hu_to_lin_att,
                       const H5::Group* h5)
{
  ReadVolDataFromDebug vol_data_visitor;
  vol_data_visitor.h5 = h5;
  vol_data_visitor.hu_to_lin_att = do_hu_to_lin_att;

  RayCaster::VolList hu_vols;
  RayCaster::VolList att_vols;

  for (const auto& vol_info : results.vols)
  {
    const auto cur_vols = boost::apply_visitor(vol_data_visitor, vol_info);
    
    const auto& cur_hu_vols = std::get<0>(cur_vols);
    
    hu_vols.insert(hu_vols.end(), cur_hu_vols.begin(), cur_hu_vols.end());

    if (do_hu_to_lin_att)
    {
      const auto& cur_att_vols = std::get<1>(cur_vols);
      
      att_vols.insert(att_vols.end(), cur_att_vols.begin(), cur_att_vols.end());
    }
  }

  return std::make_tuple(hu_vols, att_vols);
}

xreg::ProjDataF32List xreg::ProjDataFromDebug(const DebugRegiResultsMultiLevel& results,
                                              const H5::Group* h5)
{
  ReadProjDataFromDebug r;
  r.h5 = h5;

  return boost::apply_visitor(r, results.fixed_projs);
}

