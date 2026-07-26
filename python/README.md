# xreg Python bindings (`pyxreg`)

Native [pybind11](https://github.com/pybind/pybind11) bindings that expose a
first slice of xreg to Python, with data crossing the boundary as NumPy arrays
and Eigen matrices (no subprocess/CLI round trips).

This is the initial increment described in the project roadmap
("Python bindings, conda integration"). It covers the foundation
(type marshaling) plus the simplest, self-contained library functions — the
ones that back the thin command-line tools. The heavier registration pipelines
are planned follow-ups (see [Roadmap](#roadmap)).

## What is exposed today

| Submodule | Functions | Backs CLI tool(s) |
|-----------|-----------|-------------------|
| `xreg.transforms` | `read_itk_affine_transform`, `read_slicer_affine_transform`, `write_itk_affine_transform`, `rot_ang_trans_mag`, `frame_diff_rot_ang_trans_mag`, `se3_inv`, `se3_exp`, `se3_log` | `print_rigid_transform`, `xform_fcsv` |
| `xreg.landmarks` | `read_pts`, `read_name_pt_map`, `is_supported_pts_file`, `is_supported_name_pt_map_file` | `print_lands`, `xform_fcsv` |
| `xreg.image_io` | `read_volume`, `write_volume`, `read_dicom_volume` | `convert_sta_raw_to_itk`, `crop_vol` |
| `xreg.mesh` | `read_mesh`, `write_mesh` | `create_mesh`, `show_mesh` |
| `xreg.proj_data` | `read_proj_data_f32`, `read_cam_models`, `num_projs`, `CameraModel`, `Projection` | `convert_*_to_proj_data`, `extract_nii_from_proj_data` |

### Data types across the boundary

- **Transforms / poses** — 4x4 `float32` NumPy arrays (from Eigen `Mat4x4`).
- **Points / landmarks** — `(3,)` `float32` arrays; name→point maps become dicts.
- **Volumes** — `xreg.Volume`: `pixels` is a `(nz, ny, nx)` `float32` array
  (ITK buffer order, x fastest), with `spacing`, `origin` (`(3,)`) and
  `direction` (`3x3`) physical metadata preserved on round trips.
- **Meshes** — `xreg.Mesh`: `vertices` `(N, 3)` `float32`, `faces` `(M, 3)`
  `uint64`, optional per-face `normals`.
- **Projections** — `xreg.proj_data.Projection`: `pixels` `(rows, cols)`
  `float32`, a `CameraModel`, and a `landmarks` dict.

## Building

The bindings compile against the existing `xreg` library target, so they need
xreg's full dependency stack (ITK, VTK, OpenCV, Eigen, Boost, TBB, fmt, NLopt,
ViennaCL, OpenCL) — see the top-level `README.md`. Additionally you need
`pybind11` and Python development headers.

```bash
# from a build directory
cmake -G Ninja \
      -DXREG_BUILD_PYTHON=ON \
      /path/to/xreg
ninja xreg_python
```

`pybind11` is located via `find_package(pybind11 CONFIG)`; if it is not
installed, CMake fetches a pinned release (disable with
`-DXREG_PYBIND11_FETCH=OFF`). The simplest way to provide it is:

```bash
pip install pybind11
cmake ... -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
```

The build produces a `pyxreg` extension module (e.g. `pyxreg.cpython-*.so`).

## Using it

Put the compiled `pyxreg` module and the pure-Python `xreg` package (this
directory's `xreg/`) on `PYTHONPATH`:

```python
import xreg

vol = xreg.image_io.read_volume("ct.nii.gz")
print(vol.shape, vol.spacing)          # (nz, ny, nx), (sx, sy, sz)

T = xreg.transforms.read_itk_affine_transform("pose.h5")
ang_rad, trans_mm = xreg.transforms.rot_ang_trans_mag(T)

projs = xreg.proj_data.read_proj_data_f32("example.h5")
print(len(projs), projs[0].cam.focal_len, projs[0].pixels.shape)
```

You can also `import pyxreg` directly; the `xreg` package is a thin re-export.

## Testing

```bash
pip install pytest numpy
PYTHONPATH=/path/to/build:/path/to/xreg/python pytest xreg/python/tests
```

The tests are Python-only round trips and do not require the CLI executables.
If `XREG_BIN_DIR` points at the directory holding the built `xreg-*` tools, the
optional cross-check tests comparing the Python API against the CLI tools also
run.

## Roadmap

Planned follow-up increments (see the repository plan):

1. **Surface ICP** (`sur_regi`) — the simplest registration binding, no OpenCL.
2. **Multi-object / multi-level 2D/3D registration** — a thin C++ facade over
   `xreg::MultiLevelMultiObjRegi` that takes NumPy volumes/projections plus a
   level/step configuration and returns estimated poses, keeping the ITK / Eigen
   / OpenCL object-graph complexity on the C++ side.
