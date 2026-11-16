# xreg Python Bindings

Python bindings for [xReg](https://github.com/rg2/xreg), built with
[nanobind](https://github.com/wjakob/nanobind).

The bindings currently cover what is needed to set up and run intensity-based 2D/3D registrations
with `MultiLevelMultiObjRegi`: multiple objects, multiple views, multiple resolution levels, the
optimizers and regularizers used by the hip surgery tools and by the
[IPCAI 2020 experiments](https://github.com/rg2/Regi2D3D-IPCAI2020), and the surrounding utilities
(projection pre-processing, PnP initialization, DRRs, debug output, and replaying a registration
into movies). The Python API mirrors the C++
API: classes keep their C++ names, free functions use `snake_case`, and plain structs gain keyword
constructors. The C++ documentation and tools are therefore the best reference for *how* to set up
a registration. In a few places short strings may replace small helper objects.

The API may still change as more of xReg is wrapped.

## Requirements

* Python 3.12 or newer. The extension module targets the Python stable ABI (`abi3`), so a single
  binary works with Python 3.12 and all later versions.
* numpy at runtime. [SimpleITK](https://simpleitk.org) and/or [ITK Python](https://itk.org) are
  optional, and are the recommended way to read and write images.
* For building: CMake 3.26 or newer, a C++17 compiler, nanobind 3.1 or newer, and the xReg
  dependencies (ITK, VTK, OpenCV, Boost), e.g. as built by `example_build_script`.

## Building

The extension statically links the xreg library and its dependencies.

### Development build (CMake)

Create a Python environment containing the build and test requirements:

```bash
python3 -m venv build-py/venv
build-py/venv/bin/pip install nanobind numpy pytest SimpleITK
```

Configure xreg with `XREG_BUILD_PYTHON=ON`, pointing CMake at that Python interpreter:

```bash
cmake -S . -B build-py/dev -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DXREG_BUILD_PYTHON=ON -DXREG_BUILD_APPS=OFF \
      -DPython_EXECUTABLE=$PWD/build-py/venv/bin/python \
      -DITK_DIR=... -DVTK_DIR=... -DOpenCV_DIR=... -DBoost_INCLUDE_DIR=...
ninja -C build-py/dev
```

This creates an importable package at `build-py/dev/python/xreg`:

```bash
PYTHONPATH=build-py/dev/python build-py/venv/bin/python -c "import xreg; print(xreg.__version__, xreg.available_backends())"
PYTHONPATH=build-py/dev/python build-py/venv/bin/python -m pytest python/tests
```

nanobind is located using the Python interpreter (`python -m nanobind --cmake_dir`); set
`nanobind_DIR` to use a specific copy instead.

CMake options:

| Option | Default | |
|---|---|---|
| `XREG_BUILD_PYTHON` | `OFF` | Build the bindings. Enables position independent code for all xreg targets. |
| `XREG_PY_STABLE_ABI` | `ON` | Target the stable ABI. Configuration fails if this is not possible, rather than silently producing a version specific module. |
| `XREG_PY_BUILD_STUBS` | `ON` (`OFF` when cross compiling) | Generate type stubs (`_xreg.pyi`) by importing the built module. |

### Wheels / pip

`pyproject.toml` at the repository root uses
[scikit-build-core](https://scikit-build-core.readthedocs.io). It builds xreg and the extension
module, and produces a wheel tagged `cp312-abi3-<platform>`:

```bash
CMAKE_ARGS="-DITK_DIR=... -DVTK_DIR=... -DOpenCV_DIR=... -DBoost_INCLUDE_DIR=..." pip wheel . -w dist-py
# or
CMAKE_ARGS="..." pip install .
```

Intermediate build files are stored in `build-py/<wheel tag>`, which allows incremental rebuilds.

### Tests

The tests in `python/tests` only use small synthetic data. Tests which use a compute backend are
run once for each backend available on the system, using the default device of each backend. Set
`XREG_PY_TEST_BACKENDS` to choose the backends and devices instead, e.g.:

```bash
XREG_PY_TEST_BACKENDS="cpu,metal:intel,ocl:uhd" python -m pytest python/tests
```

### Platform notes

Only macOS on Intel has been built and tested so far. The bindings contain no platform specific
code and are intended to also work on macOS/Apple silicon, Linux (x86-64 and arm64), and Windows
(x86-64). Things to be aware of when building wheels elsewhere:

* **Linux**: static dependencies (ITK, VTK, OpenCV) must be compiled with position independent
  code (`-DCMAKE_POSITION_INDEPENDENT_CODE=ON`), since they are linked into a shared module.
* **Linux and Windows**: xreg builds its bundled OpenCL ICD loader as a shared library by default.
  It must be bundled into the wheel (e.g. by `auditwheel`/`delvewheel`), or a system OpenCL used
  (`XREG_USE_SYSTEM_OPENCL=ON`).
* **macOS**: the wheel's platform tag follows the deployment target that the dependencies were
  built with; set `MACOSX_DEPLOYMENT_TARGET` consistently for the dependencies and the wheel to
  support older systems. The Metal backend is only compiled on macOS.
* Free-threaded Python builds cannot use the Python 3.12 stable ABI; build with
  `-DXREG_PY_STABLE_ABI=OFF` for those.

## Overview

```python
import numpy as np
import SimpleITK as sitk
import xreg

backend = 'metal'  # or 'ocl', 'cpu', '' (default), 'ocl:amd', an xreg.Backend object, etc.

# Images read with SimpleITK (or ITK Python) may be passed directly to xreg
vol = xreg.hu_to_lin_att(sitk.ReadImage('ct.nii.gz'))

# Projection data (camera model, image, landmarks), pre-processed for registration
pp = xreg.ProjPreProc()
pp.input_projs = xreg.read_proj_data('projs.h5', proj_idxs=[0])
pp.run()

regi = xreg.MultiLevelMultiObjRegi()
regi.vols = [vol]
regi.fixed_proj_data = pp.output_projs
regi.init_cam_to_vols = [np.eye(4)]   # 4x4 camera to volume poses
regi.ref_frames = [xreg.CamAlignRefFrameWithCurPose(vol_idx=0,
                                                    cam_extrins=pp.output_projs[0].cam.extrins,
                                                    center_of_rot_wrt_vol=xreg.vol_center_as_phys_pt(vol))]

bobyqa = xreg.Intensity2D3DRegiBOBYQA()
bobyqa.set_opt_vars('se3')
bobyqa.set_bounds([2.5 * xreg.DEG2RAD] * 3 + [5, 5, 10])

regi.levels = [
    xreg.Level(ds_factor=0.25,
               ray_caster=xreg.make_ray_caster('line_int', backend),
               sim_metrics=[xreg.make_sim_metric('patch_grad_ncc', backend, patch_radius=10, patch_stride=1,
                                                 smooth_img_before_sobel_kernel_radius=5)],
               fixed_imgs_to_use=[0],
               regis=[xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                      init_mov_vol_poses=['prev:0'], regi=bobyqa)])
]

regi.run()
print(regi.cur_cam_to_vols[0])
```

### What is bound

| Area | Python |
|---|---|
| Images | `Image3DF32`, `Image3DU8`, `Image2DF32`, `Image2DU8`, `from_numpy`, `from_sitk`, `to_sitk`, `from_itk`, `to_itk`, `to_xreg_image`, `read_image`, `write_image` |
| Volumes | `hu_to_lin_att`, `make_vol_list_from_vol_and_labels`, `vol_center_as_phys_pt` |
| Cameras and projections | `CameraModel`, `ProjData`, `ProjDataRotToPatUp`, `ProjPreProc`, `ProjPreProcParams`, `downsample_proj_data` |
| File I/O | `read_proj_data`, `write_proj_data`, `read_landmarks`, `read_transform`, `write_transform` |
| Backends | `Backend`, `get_backend`, `available_backends`, `available_devices` |
| Ray casting | `RayCaster`, `make_ray_caster`, `compute_drrs`, `Proj3DLabelsTo2D`, `RAY_CAST_MAX_DEPTH` |
| Similarity metrics | `ImgSimMetric2D`, `make_sim_metric` |
| Regularization | `NormalDist1D`, `FoldNormDist`, `LogNormDist`, `NullDist`, `Regi2D3DPenaltyFnSE3EulerDecomp`, `...SE3Mag`, `...GlobalPelvis`, `...LandReproj`, `...SE3RelPose`, `...Combo` |
| Optimization variables | `SE3OptVarsLieAlg`, `SE3OptVarsEuler`, `SE3OptVarsTransOnly`, `SO3OptVarsLieAlg`, `SO3OptVarsEuler`, `make_opt_vars` |
| Optimizers | `Intensity2D3DRegiCMAES`, `...BOBYQA` (and the other NLopt methods), `...DiffEvo`, `...PSO`, `...HillClimb`, `...Exhaustive` |
| Multi-level registration | `MultiLevelMultiObjRegi`, `Level`, `SingleRegi`, `StaticRefFrame`, `CamAlignRefFrameWithCurPose`, `DynVolRefFrame`, `InitPoseId`, `InitPosePrevPoseEst`, `InitPosePrevPoseEstTransOnly`, `InitPosePrevPoseEstRotOnly`, `make_init_pose` |
| Debug output | `DebugRegiResultsMultiLevel`, `VolPathInfo`, `ProjDataPathInfo` |
| Replay movies | `Regi2D3DReplay` (the implementation of `xreg-regi2d3d-replay`) |
| Hip anatomy and PnP | `anterior_pelvic_plane_from_landmarks`, `create_ap_view_of_app`, `guess_pelvis_femur_pao_frag_labels`, `pnp_posit_and_reproj_cmaes`, `pnp_reproj_cmaes` |

`help(xreg.<name>)` shows the signature and documentation of each item, and type stubs are
installed with the package.

### Replaying a registration

`Regi2D3DReplay` turns the debug information collected by a registration into the movies made by
the `xreg-regi2d3d-replay` tool; it is the same code, and the options have the tool's defaults.

```python
regi.set_save_debug_info(True)
regi.debug_info.vols = [vol_hu]                 # volumes in HU (or VolPathInfo objects)
regi.debug_info.fixed_projs = pp.input_projs    # projections prior to pre-processing (or a ProjDataPathInfo)
regi.debug_info.proj_pre_proc_info = pp.params
regi.debug_info.regi_names = [['BOBYQA']]       # one name per registration of each level
regi.run()

replay = xreg.Regi2D3DReplay(backend, proj_ds_factor=0.5)
replay.run(regi.debug_info)                     # or the path of a file written by regi.write_debug()
replay.write_videos('mov.mp4', 'edges.mp4', fps=10)

frames = replay.edge_frames                     # list of (rows, cols, 3) uint8 RGB numpy arrays
```

* There is one frame for the initial pose, each iteration, and the final pose of every
  registration. `drr_frames` are DRRs at those poses, `edge_frames` are the fixed images overlaid
  with edges of the DRRs, and all views are tiled into each frame. `frame_titles` has the text
  that is burned into the movies.
* The volumes in the debug information should be in Hounsfield units: they are converted to
  linear attenuation for the DRRs and thresholded (`boundary_edge_hu_thresh`) for the object
  boundaries.
* Movies are encoded with `ffmpeg` when it is found on the system path. Otherwise AVFoundation is
  used on macOS and OpenCV elsewhere.
* Paths stored in debug information (`VolPathInfo`, `ProjDataPathInfo`) are used as-is, so
  relative paths are resolved against the current working directory when replaying.

### Conventions

* **Images**: the image classes wrap ITK images of xreg's own ITK build, and are shared (not
  copied) when passed between xreg objects.
  * *Into xreg*: `xreg.from_sitk()`, `xreg.from_itk()` and `xreg.from_numpy()` copy the pixel data
    once, casting to `float32` (or `uint8` for label maps and masks, with range checking).
    SimpleITK and ITK Python images may also be passed directly wherever an xreg image is
    expected, which performs the same conversion. Conversion goes through numpy and the image
    geometry, as the ITK inside of the SimpleITK and ITK Python packages is not binary compatible
    with xreg's ITK.
  * *Out of xreg*: `img.numpy()` and `numpy.asarray(img)` return an array that shares memory with
    the image (and keeps it alive); `xreg.to_sitk()` and `xreg.to_itk()` copy.
  * Numpy arrays use SimpleITK/ITK Python index ordering: an array of shape `(nz, ny, nx)` is an
    image of size `(nx, ny, nz)`. Spacing, origin, direction and size use ITK ordering (x first).
* **Poses, points and matrices**: rigid transforms are 4x4 arrays, points are length 2 or 3
  arrays, and landmark sets are `dict`s of name to point. Numpy arrays of any floating point dtype
  and (nested) lists/tuples are accepted; `float32` numpy arrays are returned. Registration poses
  map camera coordinates to volume coordinates (`cam_to_vol`), as in C++.
* **Values are copied, objects are shared**: arrays, lists and dicts (e.g. `fixed_proj_data`,
  `init_cam_to_vols`, `sim_metrics`, `landmarks`, `cam.extrins`) are copied when read from or
  assigned to a field, so assign a whole new value rather than modifying one in place. The objects
  stored in them (images, ray casters, similarity metrics, optimizers, penalty functions) are
  shared. `MultiLevelMultiObjRegi.levels` and `Level.regis` are the exception: they are list-like
  objects that hold references, so `regi.levels[0].ds_factor = 0.5` and `regi.levels.append(...)`
  work as expected.
* **Strings for small objects** (the corresponding objects are always accepted as well):
  * Backends: `"cpu"`, `"ocl"`, `"metal"`, `"<backend>:<device ID substring>"`, or `""` for the
    default. Backends created from the same string share one GPU context/queue. Backend selection
    uses the same code as the `--backend`, `--ocl-id` and `--metal-id` flags of the xReg tools.
  * Initial and static poses: `"identity"`, `"prev:<vol idx>"`, `"prev_trans:<vol idx>"`,
    `"prev_rot:<vol idx>"`.
  * Reference frames: `"identity"`.
  * Optimization variables: `"se3"` (Lie algebra), `"se3_euler"`, `"so3"`, `"so3_euler"`,
    `"trans"`, `"trans_x"`, `"trans_y"`, `"trans_z"`, `"rot_x"`, `"rot_y"`, `"rot_z"`.
  * Ray casters: `"line_int"`, `"depth"`. Similarity metrics: `"ssd"`, `"ncc"`, `"grad_ncc"`,
    `"patch_ncc"`, `"patch_grad_ncc"`.
* **Resources**: with `dealloc_resources = True` (the default, as in C++), `run()` releases each
  level's ray caster, similarity metrics and optimizer objects as the registration proceeds, and
  the corresponding `Level`/`SingleRegi` fields are empty/`None` afterwards. Create new objects to
  run again.
* **Errors**: a failed xReg assertion raises `RuntimeError` instead of terminating the
  interpreter, as do errors from ITK, OpenCV and HDF5 (e.g. a file that cannot be read). `run()` checks the setup (sizes, indices, missing objects) before starting and
  raises `ValueError` or `IndexError`.
* **Threads**: the GIL is released while registrations, replays, DRR computations, PnP solvers,
  projection pre-processing and image/projection file I/O run.
* **Output**: `set_verbose(True)` prints status to the process's standard output, which notebooks
  do not capture.

### Not (yet) supported

* Python callbacks during a registration (e.g. `fns_to_call_right_before_regi_run`).
* xReg functionality outside of 2D/3D intensity-based registration (meshes, ICP, PAO planning,
  DICOM conversion, visualization, etc.).
* Pre-built wheels and wheel building in CI.
