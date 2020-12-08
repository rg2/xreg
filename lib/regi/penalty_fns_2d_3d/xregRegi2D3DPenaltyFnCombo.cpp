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

#include "xregRegi2D3DPenaltyFnCombo.h"

#include "xregHDF5.h"
#include "xregRegi2D3DPenaltyFnDebugH5.h"

void xreg::Regi2D3DPenaltyFnCombo::setup()
{
  Regi2D3DPenaltyFn::setup();
 
  for (auto& p : pen_fns)
  {
    p->set_save_debug_info(this->save_debug_info_);
    p->set_compute_probs(this->compute_probs_);
    
    p->setup();
  }

  if (this->save_debug_info_)
  {
    debug_info_ = std::make_shared<ComboDebugInfo>();
    
    debug_info_->pen_fns_debug_infos.reserve(pen_fns.size());
    
    for (auto& p : pen_fns)
    {
      debug_info_->pen_fns_debug_infos.push_back(p->debug_info());
    }
  }
  else
  {
    debug_info_ = nullptr;
  }
}

void xreg::Regi2D3DPenaltyFnCombo::compute(
             const ListOfFrameTransformLists& cams_wrt_objs,
             const size_type num_projs,
             const CamList& cams,
             const CamAssocList& cam_assocs,
             const std::vector<bool>& intermediate_frames_wrt_vol,
             const FrameTransformList& intermediate_frames,
             const FrameTransformList& regi_xform_guesses,
             const ListOfFrameTransformLists* xforms_from_opt)
{
  this->reg_vals_.assign(num_projs, 0);

  if (this->compute_probs_)
  {
    this->log_probs_.assign(num_projs, 0);
  }

  for (auto& p : pen_fns)
  {
    p->compute(cams_wrt_objs, num_projs, cams, cam_assocs,
               intermediate_frames_wrt_vol,
               intermediate_frames,
               regi_xform_guesses,
               xforms_from_opt);

    const auto& other_reg_vals = p->reg_vals();

    for (size_type i = 0; i < num_projs; ++i)
    {
      this->reg_vals_[i] += other_reg_vals[i];
    }
    
    if (this->compute_probs_)
    {
      const auto& other_log_probs = p->log_probs();

      for (size_type i = 0; i < num_projs; ++i)
      {
        this->log_probs_[i] += other_log_probs[i];
      }
    }
  }
}
  
std::shared_ptr<xreg::Regi2D3DPenaltyFn::DebugInfo>
xreg::Regi2D3DPenaltyFnCombo::debug_info()
{
  return debug_info_;
}
    
void xreg::Regi2D3DPenaltyFnCombo::ComboDebugInfo::write(H5::Group* h5)
{
  WriteStringH5("pen-fn-name", "combo", h5);

  const size_type num_pen_fns = pen_fns_debug_infos.size();

  WriteSingleScalarH5("num-pen-fns", num_pen_fns, h5);

  for (size_type i = 0; i < num_pen_fns; ++i)
  {
    auto& cur_pen_fn = pen_fns_debug_infos[i];

    if (cur_pen_fn)
    {
      H5::Group g = h5->createGroup(fmt::format("pen-fn-{:03d}", i));
      cur_pen_fn->write(&g);
    }
  }
}

void xreg::Regi2D3DPenaltyFnCombo::ComboDebugInfo::read(const H5::Group& h5)
{
  const size_type num_pens = ReadSingleScalarH5ULong("num-pen-fns", h5);
  
  pen_fns_debug_infos.assign(num_pens, nullptr);

  for (size_type i = 0; i < num_pens; ++i)
  {
    const std::string cur_g_name = fmt::format("pen-fn-{:03d}", i);

    if (ObjectInGroupH5(cur_g_name, h5))
    {
      H5::Group g = h5.openGroup(cur_g_name);

      pen_fns_debug_infos[i] = MakePenFnDebugObjH5(g);
      pen_fns_debug_infos[i]->read(g);
    }
  }
}
