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

#include <fmt/format.h>

#include "xregFilesystemUtils.h"
#include "xregProgOptUtils.h"
#include "xregReadProjDataFromDICOM.h"
#include "xregITKIOUtils.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
    
  constexpr int kEXIT_VAL_SUCCESS  = 0;
  constexpr int kEXIT_VAL_BAD_USE  = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Given a directory of DICOM files, writes remapped versions (for display) of each "
              "file to an output directory. If the output directory does not exist, it is created. "
              "For DICOM files with more than one frame, each frame is saved as a separate remapped "
              "file with a \"_<frame index>\" string appended before the extension (For example "
              "multi_frame_dicom.dcm --> { multi_frame_dicom_0.png, ..., multi_frame_dicom_N.png }).");
  po.set_arg_usage("<Input Directory> <Output Directory>");
  po.set_min_num_pos_args(2);

  po.add("no-proc", 'n', ProgOpts::kSTORE_TRUE, "no-proc",
         "Do not perform any pre-processing to the image pixels - e.g. do NOT flip "
         "or rotate the image using the DICOM FOV Rotation or FOV Horizontal Flip fields.")
    << false;

  po.add("ds", 'd', ProgOpts::kSTORE_DOUBLE, "ds",
         "Downsample factor in each 2D dimension. "
         "0.25 --> 2x downsampling in each dimension. "
         "1 --> no downsampling. "
         "2 --> 2x upsampling in each dimension.")
    << 1.0;

  po.add("ext", 'e', ProgOpts::kSTORE_STRING, "ext",
         "Extension (and file format) to use for remapped images.")
    << "png";
  
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

  ReadProjDataFromDICOMParams read_dcm_params;
  
  read_dcm_params.vout = &vout;

  read_dcm_params.no_proc = po.get("no-proc");

  const double ds_factor = po.get("ds");

  const bool do_ds = std::abs(1.0 - ds_factor) > 0.001;

  const std::string out_ext = po.get("ext");

  const std::string& src_dcm_dir_path   = po.pos_args()[0];
  const std::string& dst_remap_dir_path = po.pos_args()[1];

  Path dst_dir_path_obj(dst_remap_dir_path);

  if (!dst_dir_path_obj.exists())
  {
    vout << "creating output directory..." << std::endl;
    MakeDirRecursive(dst_remap_dir_path);
  }
  else if (!dst_dir_path_obj.is_dir())
  {
    std::cerr << "ERROR: Destination directory is NOT a directory!" << std::endl;
    return kEXIT_VAL_BAD_USE;
  }

  PathList dcm_paths;
  Path(src_dcm_dir_path).get_dir_contents(&dcm_paths);
  vout << "number of entries in source dir: " << dcm_paths.size() << std::endl;

  using PDList = decltype(ReadProjDataFromDICOMF32(""));

  for (const auto& cur_path : dcm_paths)
  {
    PDList pd;

    try
    {
      pd = ReadProjDataFromDICOMF32(cur_path.string(), read_dcm_params);
    }
    catch (...)
    {
      vout << "error reading... assuming is not dicom..." << std::endl;
    }

    if (!pd.empty())
    {
      if (do_ds)
      {
        pd = DownsampleProjData(pd, ds_factor);
      }

      const std::string src_filename_wo_ext = std::get<0>(cur_path.filename().split_ext());
     
      const size_type num_frames = pd.size();

      if (num_frames == 1)
      {
        WriteITKImageToDisk(ITKImageRemap8bpp(pd[0].img.GetPointer()).GetPointer(),
                            fmt::format("{}/{}.{}", dst_remap_dir_path, src_filename_wo_ext, out_ext));
      }
      else
      {
        std::stringstream ss;

        ss << "{}/{}_{:0"
           << static_cast<size_type>(std::log10(num_frames) + 1)
           << "d}.{}";

        const std::string fmt_str = ss.str();

        for (size_type frame_idx = 0; frame_idx < num_frames; ++frame_idx)
        {
          WriteITKImageToDisk(ITKImageRemap8bpp(pd[frame_idx].img.GetPointer()).GetPointer(),
                              fmt::format(fmt_str,
                                          dst_remap_dir_path, src_filename_wo_ext, frame_idx, out_ext));
        }
      }
    }
  }

  return kEXIT_VAL_SUCCESS;
}

