/*
 * MIT License
 *
 * Copyright (c) 2020-2021 Robert Grupp
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

#include <numeric> // std::iota

#include "xregProgOptUtils.h"
#include "xregStringUtils.h"
#include "xregH5ProjDataIO.h"
#include "xregCSVUtils.h"
#include "xregITKResampleUtils.h"
#include "xregITKIOUtils.h"
#include "xregITKOpenCVUtils.h"
#include "xregLocalContrastNorm.h"
#include "xregOpenCVUtils.h"

using namespace xreg;

template <class tScalar>
void ProcessAndSave(itk::Image<tScalar,2>* img,
                    const std::string& path,
                    const boost::optional<ProjDataRotToPatUp>& rot_to_pat_up,
                    const double ds_factor,
                    const bool discard_geom,
                    std::ostream& vout)
{
  using Img    = itk::Image<tScalar,2>;
  using ImgPtr = typename Img::Pointer;

  Img* img_to_save = img;

  ImgPtr tmp_img;

  if (rot_to_pat_up)
  {
    vout << "    rotating to patient up..." << std::endl;

    ModifyForPatUp(img_to_save, *rot_to_pat_up);
  }

  if (std::abs(ds_factor - 1.0) > 0.001)
  {
    vout << "    downsampling..." << std::endl;
    tmp_img = DownsampleImage(img_to_save, ds_factor);
    img_to_save = tmp_img.GetPointer();
  }

  if (discard_geom)
  {
    const std::array<double,2> spacing = { 1.0, 1.0 };
    const std::array<double,2> origin = { 0.0, 0.0 };
    typename Img::DirectionType dir_mat;
    dir_mat.SetIdentity();

    img_to_save->SetSpacing(spacing.data());
    img_to_save->SetOrigin(origin.data());
    img_to_save->SetDirection(dir_mat);
  }
  
  vout << "    writing image..." << std::endl;
  WriteITKImageToDisk(img_to_save, path);
}

int main(int argc, char* argv[])
{
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;
  constexpr int kEXIT_VAL_NO_PROJS = 2;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Extract images from projection data format into individual NIFTI (.nii/.nii.gz) "
              "files which may be input to other software (e.g. for loading into 3D Slicer to "
              "annotate landmarks). "
              "Projection indices may be specified as a combination of comma delimited "
              "values and ranges - e.g. \"0,2,5-7\" will extract indices 0,2,5,6,7. "
              "All projections are extracted when no proj. indices are supplied.");
  po.set_arg_usage("<Proj. Data File> <Output NIFTI prefix> [<proj. indices to extract>]");
  po.set_min_num_pos_args(2);

  po.add("no-pat-rot-up", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-pat-rot-up",
         "Ignore any flags for rotating the image to achieve patient \"up\" orientation. ")
    << false;

  po.add("projs", 'p', ProgOpts::kSTORE_STRING, "projs",
         "Comma delimited list of zero-based projection indices and ranges to put "
         "in the tiled output image. \"\" --> use all projections.")
    << "";

  po.add("ds-factor", 'd', ProgOpts::kSTORE_DOUBLE, "ds-factor",
         "Downsampling factor applied to projection data")
    << 1.0;
  
  po.add("no-geom", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "no-geom",
         "Discard any geometric metadata (e.g. pixel spacings, origin, orientation) when writing "
         "NIFTI files. The output file will have pixel spacings equal to one, identity orientation "
         "and zero origin.")
    << false;
  
  po.add("grad-mag", 'g', ProgOpts::kSTORE_TRUE, "grad-mag",
         "Prior to remap and any other LCN, smooth the image and replace it with the "
         "gradient magnitude.")
    << false;
  
  po.add("lcn", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "lcn",
         "Local contrast normalization (LCN) method to apply on each projection. "
         "\"none\" --> no LCN. "
         "\"norm\" --> Standard Normal LCN. "
         "\"jarrett\" --> Method of Jarrett, et al. "
         "Windows of 5x5 pixels are used for all methods.")
    << "none";

  try
  {
    po.parse(argc, argv);
  }
  catch (const ProgOpts::Exception& e)
  {
    std::cerr << "Error parsing command line arguments: " << e.what() << std::endl;
    po.print_usage(std::cerr);
    return kEXIT_VAL_BAD_USE;
  }

  if (po.help_set())
  {
    po.print_usage(std::cout);
    po.print_help(std::cout);
    return kEXIT_VAL_SUCCESS;
  }

  std::ostream& vout = po.vout();
  
  const std::string proj_inds_str = po.get("projs");

  const bool ignore_pat_rot_up = po.get("no-pat-rot-up");

  const double proj_ds_factor = po.get("ds-factor");

  const bool discard_geom = po.get("no-geom");
  
  const bool do_grad_mag = po.get("grad-mag");
  
  const std::string lcn_str = ToLowerCase(po.get("lcn").as_string());

  bool do_std_norm_lcn = false;
  bool do_jarrett_lcn   = false;

  if (lcn_str == "norm")
  {
    do_std_norm_lcn = true;
  }
  else if (lcn_str == "jarrett")
  {
    do_jarrett_lcn = true;
  }

  const bool adjust_proj_intens = do_grad_mag ||
                                  do_std_norm_lcn ||
                                  do_jarrett_lcn;

  const std::string proj_data_path = po.pos_args()[0];
  const std::string nii_prefix     = po.pos_args()[1];

  vout << "creating proj data reader obj..." << std::endl;
  DeferredProjReader pd_reader(proj_data_path);

  const auto scalar_type = pd_reader.scalar_type_on_disk();
  
  if (scalar_type == kPROJ_DATA_TYPE_FLOAT32)
  {
    vout << "  pixels are float32..." << std::endl;
  }
  else if (scalar_type == kPROJ_DATA_TYPE_UINT16)
  {
    vout << "  pixels are uint16..." << std::endl;
  }
  else if (scalar_type == kPROJ_DATA_TYPE_UINT8)
  {
    vout << "  pixels are uint8..." << std::endl;
  }

  vout << "reading metadata..." << std::endl;
  const auto pd_metas = pd_reader.proj_data_F32();

  const size_type num_src_projs = pd_metas.size();
  vout << "number of cameras/projections in file: " << num_src_projs << std::endl;

  std::vector<long> projs_to_use;
  
  if (proj_inds_str.empty())
  {
    projs_to_use.resize(num_src_projs);
    std::iota(projs_to_use.begin(), projs_to_use.end(), 0);
  }
  else
  {
    projs_to_use = ParseCSVRangeOfInts(proj_inds_str);
  }

  const size_type num_projs = projs_to_use.size();
  vout << "number of images to extract: " << num_projs << std::endl;

  if (num_projs)
  {
    for (size_type proj_idx = 0; proj_idx < num_projs; ++proj_idx)
    {
      vout << "  extracting image #" << (proj_idx + 1) << std::endl;

      const size_type src_proj_idx = static_cast<size_type>(projs_to_use[proj_idx]);
      xregASSERT(src_proj_idx < num_src_projs);

      const std::string dst_path = fmt::format("{}_{:03d}.nii.gz", nii_prefix, src_proj_idx);

      vout << "    original index: " << src_proj_idx << '\n'
           << "       output path: " << dst_path << std::endl;

      const auto& cur_meta = pd_metas[src_proj_idx];

      boost::optional<ProjDataRotToPatUp> rot_to_pat_up;

      if (!ignore_pat_rot_up)
      {
        rot_to_pat_up = cur_meta.rot_to_pat_up;
      }

      // always read the images in as float when adjusting pixel intensities
      if (adjust_proj_intens || (scalar_type == kPROJ_DATA_TYPE_FLOAT32))
      {
        auto img = pd_reader.read_proj_F32(src_proj_idx);

        if (adjust_proj_intens)
        {
          cv::Mat img_ocv = ShallowCopyItkToOpenCV(img.GetPointer());
            
          if (do_grad_mag)
          {
            vout << "      grad. mag. calc. ..." << std::endl;
            cv::Mat smooth_img(img_ocv.rows, img_ocv.cols, img_ocv.type());
            cv::Mat tmp_img(img_ocv.rows, img_ocv.cols, img_ocv.type());
            cv::Mat grad_img(img_ocv.rows, img_ocv.cols, img_ocv.type());
            
            SmoothAndGradMag(img_ocv, smooth_img, grad_img, tmp_img, 5);
            grad_img.copyTo(img_ocv);
          }

          if (do_std_norm_lcn)
          {
            vout << "      performing LCN: std. norm. ..." << std::endl; 
            LocalContrastNormStdNorm(img_ocv, 5, 5, 0.0f).copyTo(img_ocv);
          }
          else if (do_jarrett_lcn)
          {
            vout << "      performing LCN: jarrett..." << std::endl;
            LocalContrastNormJarrett(img_ocv, 5, 5, 0.0f).copyTo(img_ocv);
          }
        }

        ProcessAndSave(img.GetPointer(), dst_path, rot_to_pat_up, proj_ds_factor, discard_geom, vout);
      }
      else if (scalar_type == kPROJ_DATA_TYPE_UINT16)
      {
        auto img = pd_reader.read_proj_U16(src_proj_idx);
        
        ProcessAndSave(img.GetPointer(), dst_path, rot_to_pat_up, proj_ds_factor, discard_geom, vout);
      }
      else if (scalar_type == kPROJ_DATA_TYPE_UINT8)
      {
        auto img = pd_reader.read_proj_U8(src_proj_idx);
        
        ProcessAndSave(img.GetPointer(), dst_path, rot_to_pat_up, proj_ds_factor, discard_geom, vout);
      }
    }
  }
  else
  {
    std::cerr << "ERROR: NO PROJECTIONS IN FILE!" << std::endl;
    return kEXIT_VAL_NO_PROJS;
  }

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}
