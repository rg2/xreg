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

#include <fmt/printf.h>

#include "xregMeshIO.h"
#include "xregProgOptUtils.h"
#include "xregVTKBasicUtils.h"
#include "xregVTKMeshUtils.h"
#include "xregFilesystemUtils.h"
#include "xregAssert.h"
#include "xregVTK3DPlotter.h"

int main(int argc, char* argv[])
{
  using namespace xreg;

  constexpr int kEXIT_VAL_SUCCESS = 0;
  constexpr int kEXIT_VAL_BAD_USE = 1;
  
  ProgOpts po;

  xregPROG_OPTS_SET_COMPILE_DATE(po);

  po.set_help("Shows an interactive 3D visualization of a mesh via VTK.");
  po.set_arg_usage("<mesh file path>");
  po.set_min_num_pos_args(1);

  po.add("bg-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "bg-color",
         "The background color to be displayed; RGB, each component should be in [0,1].")
    .nvals(3)
    << 0.7 << 0.8 << 1.0;

  po.add("mesh-color", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "mesh-color",
         "The mesh color to use; ignored when a set of scalar values per vertex (e.g. distance heat map) is provided; RGB, each component should be in [0,1]")
    .nvals(3)
    << 0.65 << 0.65 << 0.65;

  po.add("mesh-color-gray", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "mesh-color-gray",
         "Use a gray mesh color of [0.65, 0.65, 0.65]") << false;
  po.add("mesh-color-yellow", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE,
         "mesh-color-yellow", "Use a yellow mesh color of [1.0, 0.84, 0.41]") << false;
  po.add("mesh-color-bone", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "mesh-color-bone",
         "Use a bone mesh color of [1, 0.9922, 0.8980]")
    << false;

  po.add("win-size", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "win-size",
         "The initial size of the rendering window in pixels; width followed by height")
    .nvals(2)
    << 800 << 800;
  po.add("full-screen", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_TRUE, "full-screen",
         "Display in full-screen mode; overrides any specified window dimensions.")
    << false;

  po.add("az", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "az",
         "Set the azimuth angle for display (degrees)") << 0.0;
  po.add("el", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "el",
         "Set the elevation angle for display (degrees)") << 0.0;

  po.add("vert-scalars", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "vert-scalars",
         "Path to a file describing scalar values for each vertex; this is typically used "
         "to highlight surface distances from another shape via heat map")
    << "";
  po.add("scalars-title", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_STRING, "scalars-title",
         "The title to be displayed for the scalar data color bar") << "Distance";
  po.add("scalars-num-ticks", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_INT32, "scalars-num-ticks",
         "The number of numeric values to display in the scalars color bar")
    << ProgOpts::int32(4);
  po.add("scalar-min", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "scalar-min",
         "Override the minimum value to be used for scalar color coding.");
  po.add("scalar-max", ProgOpts::kNO_SHORT_FLAG, ProgOpts::kSTORE_DOUBLE, "scalar-max",
         "Override the maximum value to be used for scalar color coding.");

  po.set_help_epilogue(VTK3DPlotter::KeyUsageStr());

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

  ProgOpts::DoubleList bg_color;
  po.get_vals("bg-color", &bg_color);

  ProgOpts::DoubleList mesh_color;
  po.get_vals("mesh-color", &mesh_color);

  if (po.val_defaulted("mesh-color"))
  {
    // A color was not specified via the mesh-color flag, if any other convenience
    // flags were used to set the color use it

    const bool do_mesh_color_gray   = po.get("mesh-color-gray");
    const bool do_mesh_color_yellow = po.get("mesh-color-yellow");
    const bool do_mesh_color_bone   = po.get("mesh-color-bone");

    // Only support passing one flag
    xregASSERT((int(do_mesh_color_gray) + int(do_mesh_color_yellow) +
                int(do_mesh_color_bone)) < 2);

    if (do_mesh_color_gray)
    {
      mesh_color[0] = 0.65;
      mesh_color[1] = 0.65;
      mesh_color[2] = 0.65;
    }
    else if (do_mesh_color_yellow)
    {
      mesh_color[0] = 1.0;
      mesh_color[1] = 0.84;
      mesh_color[2] = 0.41;
    }
    else if (do_mesh_color_bone)
    {
      mesh_color[0] = 1.0;
      mesh_color[1] = 0.9922;
      mesh_color[2] = 0.8980;
    }
  }

  ProgOpts::Int32List win_size;
  po.get_vals("win-size", &win_size);

  const bool full_screen = po.get("full-screen");

  const double az_deg = po.get("az");
  const double el_deg = po.get("el");
  
  const bool use_custom_az_el = !po.val_defaulted("az") || !po.val_defaulted("el");

  const std::string scalars_path = po.get("vert-scalars").as_string();
  const std::string scalars_title = po.get("scalars-title").as_string();
  const ProgOpts::int32 scalars_ticks = po.get("scalars-num-ticks");

  const bool   do_scalar_override_min = po.has("scalar-min");
  const double scalar_override_min    = do_scalar_override_min ? po.get("scalar-min").as_double() : 0.0;
  const bool   do_scalar_override_max = po.has("scalar-max");
  const double scalar_override_max    = do_scalar_override_min ? po.get("scalar-max").as_double() : 0.0;

  vout << "Settings:"
       << "\n    BG Color: [" << bg_color[0] << ',' << bg_color[1] << ',' << bg_color[2] << ']'
       << "\n  Mesh Color: [" << mesh_color[0] << ',' << mesh_color[1] << ',' << mesh_color[2] << ']'
       << "\n   Win. Size: [" << win_size[0] << ',' << win_size[1] << ']'
       << "\n Full Screen: " << full_screen
       << "\n     Azimuth: " << az_deg
       << "\n   Elevation: " << el_deg
       << "\n  Scalars Params: "
       << "\n            Path: " << scalars_path
       << "\n           Title: " << scalars_title
       << "\n    Scalar Ticks: " << scalars_ticks
       << "\n Scalar Override Min: " << (do_scalar_override_min ? fmt::sprintf("%.4f", scalar_override_min) : "NO")
       << "\n Scalar Override Max: " << (do_scalar_override_max ? fmt::sprintf("%.4f", scalar_override_max) : "NO")
       << std::endl;

  const std::string input_mesh_path = po.pos_args()[0];

  vout << "reading mesh: " << input_mesh_path << std::endl;
  const auto mesh = ReadMeshFromDisk(input_mesh_path);

  CoordScalarList scalars;
  if (!scalars_path.empty())
  {
    vout << "Reading scalars..." << std::endl;
    scalars = FileToCoordScalars(scalars_path);
    xregASSERT(scalars.size() == mesh.vertices.size());
  }

  vout << "creating VTK plotter..." << std::endl;
  VTK3DPlotter plot;
  
  plot.set_size(win_size[0], win_size[1]);

  plot.set_title(Path(input_mesh_path).filename().string());
  
  plot.set_show_axes(false);

  if (scalars.empty())
  {
    plot.add_mesh(mesh, mesh_color[0], mesh_color[1], mesh_color[2]);
  }
  else
  {
    AnyParamMap scalar_params = { { "title", scalars_title },
                                  { "ticks", int(scalars_ticks) } };

    if (do_scalar_override_min)
    {
      scalar_params.emplace("min", CoordScalar(scalar_override_min));
    }
    
    if (do_scalar_override_max)
    {
      scalar_params.emplace("max", CoordScalar(scalar_override_max));
    }

    plot.add_mesh_w_scalar_map(mesh, scalars, scalar_params);
  }
 
  plot.set_bg_color(bg_color[0], bg_color[1], bg_color[2]);

  plot.set_full_screen(full_screen);

  if (use_custom_az_el)
  {
    plot.set_custom_az_el(az_deg, el_deg);
  }
  
  plot.show();

  vout << "exiting..." << std::endl;
  return kEXIT_VAL_SUCCESS;
}
