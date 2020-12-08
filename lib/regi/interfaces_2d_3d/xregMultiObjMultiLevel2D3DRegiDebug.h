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

#ifndef XREGMULTIOBJMULTILEVEL2D3DREGIDEBUG_H_
#define XREGMULTIOBJMULTILEVEL2D3DREGIDEBUG_H_

#include <boost/variant.hpp>

#include "xregProjData.h"
#include "xregProjPreProc.h"
#include "xregIntensity2D3DRegiDebug.h"

namespace xreg
{

struct DebugRegiResultsMultiLevel
{
  using ScalarList     = SingleRegiDebugResults::ScalarList;
  using ListOfStrLists = std::vector<StrList>;
  using IndexList      = SingleRegiDebugResults::IndexList;
  
  using SingleRegiDebugResultsPtr = std::shared_ptr<SingleRegiDebugResults>;
  using SingleLevelRegiResults    = std::vector<SingleRegiDebugResultsPtr>;
  using MultiLevelRegiResults     = std::vector<SingleLevelRegiResults>;

  struct VolPathInfo
  {
    std::string vol_path;

    /// \brief Path to label volume used to create sub-volumes, if this is empty,
    ///        then the entire volume is assumed to have been used.
    std::string label_vol_path;

    IndexList labels_used;

    bool paths_on_disk = true;
  };

  // List of the volumes used. A variant type is used to allow the user to
  // either provide the volumes directly, in which case they will be serialized
  // into the debug file, or to provide paths on disk in order to avoid duplicating
  // storage of the volumes.
  std::vector<boost::variant<RayCaster::VolPtr,VolPathInfo>> vols;

  struct ProjDataPathInfo
  {
    std::string path;

    IndexList projs_used;  // empty --> use all projs

    bool paths_on_disk = true;
  };

  // see above note about using variant - just for proj data this time
  boost::variant<ProjDataF32List,ProjDataPathInfo> fixed_projs;

  boost::optional<double> pre_proc_time_secs;
  
  double multi_level_run_time_secs;

  // this can include I/O
  boost::optional<double> tot_time_secs;

  ScalarList multi_res_levels;

  /// regi_results[i][j] is the per iteration debug information at the ith
  /// multi-resolution level for the jth registration run.
  /// e.g. for PAO fragment tracking, j = 0 for pelvis, j = 1 for femur,
  /// j = 2 for fragment, j = 3 for all and j = 3 is optional.
  MultiLevelRegiResults regi_results;

  /// regi_names[i][j] is the name of the jth registration run at the ith
  /// multi-resolution level.
  ListOfStrLists regi_names;

  boost::optional<ProjPreProcParams> proj_pre_proc_info;

  /// \brief Computes the total number of projections (per view) that would
  ///        correspond to each registration's initial and final poses, and each
  ///        iteration within the registration.
  size_type total_num_projs_per_view() const;
};

void WriteMultiLevel2D3DRegiDebugH5(const DebugRegiResultsMultiLevel& results, H5::Group* h5);

void WriteMultiLevel2D3DRegiDebugToDisk(const DebugRegiResultsMultiLevel& results, const std::string& path);

DebugRegiResultsMultiLevel ReadMultiLevel2D3DRegiDebugH5(const H5::Group& h5);

DebugRegiResultsMultiLevel ReadMultiLevel2D3DRegiDebugFromDisk(const std::string& path);

std::tuple<xreg::RayCaster::VolList,
           xreg::RayCaster::VolList>
VolDataFromDebug(const DebugRegiResultsMultiLevel& results,
                 const bool do_hu_to_lin_att,
                 const H5::Group* h5 = nullptr);

ProjDataF32List ProjDataFromDebug(const DebugRegiResultsMultiLevel& results,
                                  const H5::Group* h5 = nullptr);

}  // xreg

#endif

