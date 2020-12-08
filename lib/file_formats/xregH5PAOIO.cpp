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

#include "xregH5PAOIO.h"

#include "xregHDF5.h"

std::tuple<xreg::PAOCutPlanes,boost::optional<xreg::PAOCutDispInfo>,boost::optional<xreg::PAOCutSlabs>>
xreg::ReadPAOCutPlanesH5(const H5::Group& h5)
{
  PAOCutPlanes cut_defs;

  boost::optional<PAOCutDispInfo> disp_info_ret;
  boost::optional<PAOCutSlabs> slabs_ret;

  cut_defs.ilium.normal = ReadMatrixH5CoordScalar("ilium-normal", h5);
  cut_defs.ilium.scalar = ReadSingleScalarH5CoordScalar("ilium-scalar", h5);

  cut_defs.ischium.normal = ReadMatrixH5CoordScalar("ischium-normal", h5);
  cut_defs.ischium.scalar = ReadSingleScalarH5CoordScalar("ischium-scalar", h5);

  cut_defs.pubis.normal = ReadMatrixH5CoordScalar("pubis-normal", h5);
  cut_defs.pubis.scalar = ReadSingleScalarH5CoordScalar("pubis-scalar", h5);

  cut_defs.post.normal = ReadMatrixH5CoordScalar("post-normal", h5);
  cut_defs.post.scalar = ReadSingleScalarH5CoordScalar("post-scalar", h5);
    
  cut_defs.mid_sagittal.normal = ReadMatrixH5CoordScalar("mid-sagittal-normal", h5);
  cut_defs.mid_sagittal.scalar = ReadSingleScalarH5CoordScalar("mid-sagittal-scalar", h5);

  cut_defs.lateral.normal = ReadMatrixH5CoordScalar("lateral-normal", h5);
  cut_defs.lateral.scalar = ReadSingleScalarH5CoordScalar("lateral-scalar", h5);
 
  try
  {
    HideH5ExceptionPrints he;
    
    PAOCutDispInfo disp_info;
    
    disp_info.orig_ilium_cut_plane_mid_pt   = ReadMatrixH5CoordScalar("ilium-cut-mid-pt",   h5);
    disp_info.orig_ischium_cut_plane_mid_pt = ReadMatrixH5CoordScalar("ischium-cut-mid-pt", h5);
    disp_info.orig_post_cut_plane_mid_pt    = ReadMatrixH5CoordScalar("post-cut-mid-pt",    h5);
    disp_info.orig_pubis_cut_plane_mid_pt   = ReadMatrixH5CoordScalar("pubis-cut-mid-pt",   h5);
    disp_info.midsagittal_pt                = ReadMatrixH5CoordScalar("mid-sagittal-pt",    h5);
    disp_info.lateral_pt                    = ReadMatrixH5CoordScalar("lateral-pt",         h5);

    disp_info.ilium_cut_plane_width   = ReadSingleScalarH5CoordScalar("ilium-plane-width",        h5);
    disp_info.ischium_cut_plane_width = ReadSingleScalarH5CoordScalar("ischium-plane-width",      h5);
    disp_info.pubis_cut_plane_width   = ReadSingleScalarH5CoordScalar("pubis-plane-width",        h5);
    disp_info.post_cut_plane_width    = ReadSingleScalarH5CoordScalar("post-plane-width",         h5);
    disp_info.midsagittal_plane_width = ReadSingleScalarH5CoordScalar("mid-sagittal-plane-width", h5);
    disp_info.lateral_plane_width     = ReadSingleScalarH5CoordScalar("lateral-plane-width",      h5);

    disp_info_ret = disp_info;
  }
  catch (const H5::Exception&)
  {
    // error reading display info, do not populate the return structure
  }

  try
  {
    HideH5ExceptionPrints he;

    PAOCutSlabs slabs;

    slabs.ilium_cut.fll_pt   = ReadMatrixH5CoordScalar("ilium-slab-fll", h5);
    slabs.ilium_cut.bur_pt   = ReadMatrixH5CoordScalar("ilium-slab-bur", h5);
    slabs.ilium_cut_to_world = ReadAffineTransform4x4H5("ilium-slab-to-world", h5);
    
    slabs.ischium_cut.fll_pt   = ReadMatrixH5CoordScalar("ischium-slab-fll", h5);
    slabs.ischium_cut.bur_pt   = ReadMatrixH5CoordScalar("ischium-slab-bur", h5);
    slabs.ischium_cut_to_world = ReadAffineTransform4x4H5("ischium-slab-to-world", h5);
    
    slabs.post_cut.fll_pt   = ReadMatrixH5CoordScalar("post-slab-fll", h5);
    slabs.post_cut.bur_pt   = ReadMatrixH5CoordScalar("post-slab-bur", h5);
    slabs.post_cut_to_world = ReadAffineTransform4x4H5("post-slab-to-world", h5);
    
    slabs.pubis_cut.fll_pt   = ReadMatrixH5CoordScalar("pubis-slab-fll", h5);
    slabs.pubis_cut.bur_pt   = ReadMatrixH5CoordScalar("pubis-slab-bur", h5);
    slabs.pubis_cut_to_world = ReadAffineTransform4x4H5("pubis-slab-to-world", h5);
    
    slabs_ret = slabs;
  }
  catch (const H5::Exception&)
  {
    // error reading slab info, do not populate the return structure
  }
  
  return std::make_tuple(cut_defs, disp_info_ret, slabs_ret);
}

std::tuple<xreg::PAOCutPlanes,boost::optional<xreg::PAOCutDispInfo>,boost::optional<xreg::PAOCutSlabs>>
xreg::ReadPAOCutPlanesH5File(const std::string& path)
{
  return ReadPAOCutPlanesH5(H5::H5File(path, H5F_ACC_RDONLY));
}

void xreg::WritePAOCutPlanesH5(const PAOCutPlanes& cut_defs,
                               H5::Group* h5,
                               const PAOCutDispInfo* cut_disp,
                               const PAOCutSlabs* cut_slabs,
                               const bool compress)
{
  SetStringAttr("xreg-type", "pao-cuts", h5);

  WriteMatrixH5("ilium-normal",       cut_defs.ilium.normal, h5);
  WriteSingleScalarH5("ilium-scalar", cut_defs.ilium.scalar, h5);
  
  WriteMatrixH5("ischium-normal",       cut_defs.ischium.normal, h5);
  WriteSingleScalarH5("ischium-scalar", cut_defs.ischium.scalar, h5);
  
  WriteMatrixH5("pubis-normal",       cut_defs.pubis.normal, h5);
  WriteSingleScalarH5("pubis-scalar", cut_defs.pubis.scalar, h5);
  
  WriteMatrixH5("post-normal",       cut_defs.post.normal, h5);
  WriteSingleScalarH5("post-scalar", cut_defs.post.scalar, h5);
  
  WriteMatrixH5("mid-sagittal-normal",       cut_defs.mid_sagittal.normal, h5);
  WriteSingleScalarH5("mid-sagittal-scalar", cut_defs.mid_sagittal.scalar, h5);
  
  WriteMatrixH5("lateral-normal",       cut_defs.lateral.normal, h5);
  WriteSingleScalarH5("lateral-scalar", cut_defs.lateral.scalar, h5);

  if (cut_disp)
  {
    WriteMatrixH5("ilium-cut-mid-pt",   cut_disp->orig_ilium_cut_plane_mid_pt,   h5);
    WriteMatrixH5("ischium-cut-mid-pt", cut_disp->orig_ischium_cut_plane_mid_pt, h5);
    WriteMatrixH5("post-cut-mid-pt",    cut_disp->orig_post_cut_plane_mid_pt,    h5);
    WriteMatrixH5("pubis-cut-mid-pt",   cut_disp->orig_pubis_cut_plane_mid_pt,   h5);
    WriteMatrixH5("mid-sagittal-pt",    cut_disp->midsagittal_pt,                h5);
    WriteMatrixH5("lateral-pt",         cut_disp->lateral_pt,                    h5);
  
    WriteSingleScalarH5("ilium-plane-width",        cut_disp->ilium_cut_plane_width,   h5);
    WriteSingleScalarH5("ischium-plane-width",      cut_disp->ischium_cut_plane_width, h5);
    WriteSingleScalarH5("pubis-plane-width",        cut_disp->pubis_cut_plane_width,   h5);
    WriteSingleScalarH5("post-plane-width",         cut_disp->post_cut_plane_width,    h5);
    WriteSingleScalarH5("mid-sagittal-plane-width", cut_disp->midsagittal_plane_width, h5);
    WriteSingleScalarH5("lateral-plane-width",      cut_disp->lateral_plane_width,     h5);
  }

  if (cut_slabs)
  {
    WriteMatrixH5("ilium-slab-fll", cut_slabs->ilium_cut.fll_pt, h5);
    WriteMatrixH5("ilium-slab-bur", cut_slabs->ilium_cut.bur_pt, h5);
    WriteAffineTransform4x4("ilium-slab-to-world", cut_slabs->ilium_cut_to_world, h5);
    
    WriteMatrixH5("ischium-slab-fll", cut_slabs->ischium_cut.fll_pt, h5);
    WriteMatrixH5("ischium-slab-bur", cut_slabs->ischium_cut.bur_pt, h5);
    WriteAffineTransform4x4("ischium-slab-to-world", cut_slabs->ischium_cut_to_world, h5);
    
    WriteMatrixH5("post-slab-fll", cut_slabs->post_cut.fll_pt, h5);
    WriteMatrixH5("post-slab-bur", cut_slabs->post_cut.bur_pt, h5);
    WriteAffineTransform4x4("post-slab-to-world", cut_slabs->post_cut_to_world, h5);
    
    WriteMatrixH5("pubis-slab-fll", cut_slabs->pubis_cut.fll_pt, h5);
    WriteMatrixH5("pubis-slab-bur", cut_slabs->pubis_cut.bur_pt, h5);
    WriteAffineTransform4x4("pubis-slab-to-world", cut_slabs->pubis_cut_to_world, h5);
  }
}

void xreg::WritePAOCutPlanesH5File(const PAOCutPlanes& cut_defs,
                                   const std::string& path,
                                   const PAOCutDispInfo* cut_disp,
                                   const PAOCutSlabs* cut_slabs,
                                   const bool compress)
{
  H5::H5File h5(path, H5F_ACC_TRUNC);

  WritePAOCutPlanesH5(cut_defs, &h5, cut_disp, cut_slabs, compress);

  h5.flush(H5F_SCOPE_GLOBAL);
  h5.close();
}

