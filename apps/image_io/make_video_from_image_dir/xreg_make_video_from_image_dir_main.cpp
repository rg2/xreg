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

#include "xregCommon.h"
#include "xregProgOptUtils.h"
#include "xregWriteVideo.h"

int main(int argc, char* argv[])
{
  using namespace xreg;
  
  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;

  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Create a video from a directory of image files, which are used as frames.");
  po.set_arg_usage("<Directory of Images> <Output Movie File> [<Image Extension 1> [<Image Extension 2> ... [<Image Extension N>]]]");
  po.set_min_num_pos_args(2);

  po.add("sort", 's', ProgOpts::kSTORE_TRUE, "sort", "Sort frame order by lexographic file path order.")
    << false;

  po.add("fps", 'f', ProgOpts::kSTORE_DOUBLE, "fps", "The frame rate of the output movie.")
    << 1.0;

  po.add("len", 'l', ProgOpts::kSTORE_DOUBLE, "len",
         "The length of the output movie in seconds; this argument will "
         "take precedent over the fps argument.");

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
  
  const size_type num_cmd_args = po.pos_args().size();

  const size_type num_exts_on_cmd_line = num_cmd_args - 2;

  const std::string src_dir = po.pos_args()[0];
  const std::string dst_mov = po.pos_args()[1];

  double fps_or_len = po.get("fps");

  const bool sort_paths = po.get("sort");

  const bool video_len_passed = po.has("len");

  double video_len_secs = video_len_passed ? po.get("len").as_double() : 0.0;

  vout << num_exts_on_cmd_line << " extensions passed on the command line..." << std::endl;

  std::vector<std::string> img_exts;

  if (num_exts_on_cmd_line)
  {
    img_exts.insert(img_exts.end(), po.pos_args().begin() + 2, po.pos_args().end());
  }
  else
  {
    vout << "using default image extension of .png" << std::endl;

    img_exts = { ".png" };
  }

  if (video_len_passed)
  {
    vout << "overriding FPS with video length argument..." << std::endl;
    fps_or_len = po.get("len");
  }

  vout << "processing directory contents and creating video..." << std::endl;
  WriteDirOfImagesToVideo(dst_mov, src_dir, sort_paths, img_exts, fps_or_len,
                          !video_len_passed);

  return kEXIT_VAL_SUCCESS;
}

