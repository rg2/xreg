# MIT License
#
# Copyright (c) 2026 Robert Grupp
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Poses, camera models, projection data, pre-processing, and file I/O."""

import numpy as np
import pytest

import xreg

from conftest import make_cam, rot_x, trans


def test_frame_transform_conversion():
    x = rot_x(0.3) @ trans([1, 2, 3])

    rf = xreg.StaticRefFrame(x, maps_vol_to_ref=False)

    out = rf.ref_frame
    assert out.shape == (4, 4)
    assert out.dtype == np.float32
    assert np.allclose(out, x, atol=1e-6)
    assert not rf.maps_vol_to_ref

    # lists of poses and float32 input
    ml = xreg.MultiLevelMultiObjRegi()
    ml.init_cam_to_vols = [x.astype(np.float32), np.eye(4)]
    assert len(ml.init_cam_to_vols) == 2
    assert np.allclose(ml.init_cam_to_vols[0], x, atol=1e-6)

    with pytest.raises(TypeError):
        ml.init_cam_to_vols = [np.eye(3)]


def test_camera_model():
    cam = make_cam()

    assert cam.num_det_rows == 64
    assert cam.num_det_cols == 64
    assert cam.focal_len == pytest.approx(1000)

    # The principal ray projects to the center of the detector, (n - 1) / 2 in pixel indices
    ind = cam.phys_pt_to_ind_pt(np.array([0, 0, -500.0]))
    assert np.allclose(ind[:2], [31.5, 31.5], atol=1e-3)

    assert cam == make_cam()
    assert cam != make_cam(32, 32)

    assert np.allclose(cam.extrins, np.eye(4))


def test_proj_data():
    cam = make_cam(4, 5, 1.0)

    img = xreg.from_numpy(np.ones((4, 5), np.float32))

    pd = xreg.ProjData(cam, img, {'a': np.array([1.0, 2.0])}, rot_to_pat_up=90)

    assert pd.cam == cam
    assert pd.img == img
    assert pd.img.shape == (4, 5)
    assert np.allclose(pd.landmarks['a'], [1, 2])
    assert pd.rot_to_pat_up == xreg.ProjDataRotToPatUp.NINETY
    assert pd.rot_to_pat_up == 90

    pd.rot_to_pat_up = xreg.ProjDataRotToPatUp.ZERO
    assert pd.rot_to_pat_up == xreg.ProjDataRotToPatUp.ZERO

    pd.rot_to_pat_up = None
    assert pd.rot_to_pat_up is None

    with pytest.raises(ValueError):
        pd.rot_to_pat_up = 45

    pd.img = None
    assert pd.img is None

    empty = xreg.ProjData()
    assert empty.img is None
    assert len(empty.landmarks) == 0

    # the image type is checked
    with pytest.raises(TypeError):
        pd.img = xreg.from_numpy(np.ones((4, 5), np.uint8))


def test_proj_data_from_sitk_image():
    sitk = pytest.importorskip('SimpleITK')

    src = sitk.GetImageFromArray(np.arange(20, dtype=np.uint16).reshape(4, 5))

    pd = xreg.ProjData(make_cam(4, 5, 1.0), src)

    assert isinstance(pd.img, xreg.Image2DF32)
    assert np.array_equal(pd.img.numpy(), np.arange(20).reshape(4, 5))


def test_proj_pre_proc():
    rng = np.random.default_rng(0)

    arr = rng.uniform(100, 1000, (40, 50)).astype(np.float32)

    cam = make_cam(40, 50, 1.0)

    pp = xreg.ProjPreProc()

    assert pp.params.crop_width == 50

    # params is a reference to the member of the pre-processing object
    pp.params.crop_width = 5
    assert pp.params.crop_width == 5

    pp.input_projs = [xreg.ProjData(cam, xreg.from_numpy(arr), {'a': [20.0, 10.0]})]
    pp.run()

    assert len(pp.output_projs) == 1

    out = pp.output_projs[0]

    # cropped on each side
    assert out.img.shape == (30, 40)
    assert out.cam.num_det_rows == 30
    assert out.cam.num_det_cols == 40
    assert np.allclose(out.landmarks['a'], [15, 5])

    # log remapping
    assert not np.allclose(out.img.numpy(), arr[5:-5, 5:-5])

    # the input is not modified
    assert np.array_equal(pp.input_projs[0].img.numpy(), arr)

    pp.params = xreg.ProjPreProcParams(crop_width=5, no_log_remap=True)
    pp()

    assert np.allclose(pp.output_projs[0].img.numpy(), arr[5:-5, 5:-5])

    pp.input_projs = [xreg.ProjData(cam)]
    with pytest.raises(ValueError):
        pp.run()


def test_downsample_proj_data():
    pd = xreg.ProjData(make_cam(64, 48, 0.5), xreg.from_numpy(np.ones((64, 48), np.float32)),
                       {'a': [20.0, 10.0]})

    ds = xreg.downsample_proj_data(pd, 0.5)

    assert ds.img.shape == (32, 24)
    assert ds.cam.num_det_rows == 32
    assert ds.cam.det_row_spacing == pytest.approx(1.0)

    ds_list = xreg.downsample_proj_data([pd, pd], 0.25)
    assert [p.img.shape for p in ds_list] == [(16, 12)] * 2


def test_proj_data_file_io(tmp_path):
    cam = make_cam(8, 10, 1.5)

    projs = [xreg.ProjData(cam, xreg.from_numpy(np.full((8, 10), i, np.float32)),
                           {'FH-l': [1.0 + i, 2.0]}, rot_to_pat_up=180)
             for i in range(3)]

    path = str(tmp_path / 'pd.h5')

    xreg.write_proj_data(projs, path)

    all_projs = xreg.read_proj_data(path)

    assert len(all_projs) == 3

    for i, pd in enumerate(all_projs):
        assert np.all(pd.img.numpy() == i)
        assert pd.cam == cam
        assert np.allclose(pd.landmarks['FH-l'], [1.0 + i, 2.0])
        assert pd.rot_to_pat_up == 180

    some_projs = xreg.read_proj_data(path, proj_idxs=[2, 0])
    assert [float(pd.img.numpy()[0, 0]) for pd in some_projs] == [2, 0]

    no_pix = xreg.read_proj_data(path, read_pixels=False)
    assert all(pd.img is None for pd in no_pix)

    with pytest.raises(IndexError):
        xreg.read_proj_data(path, proj_idxs=[3])


def test_transform_and_landmark_file_io(tmp_path):
    x = rot_x(0.2) @ trans([4, 5, 6])

    path = str(tmp_path / 'pose.h5')

    xreg.write_transform(path, x)

    assert np.allclose(xreg.read_transform(path), x, atol=1e-5)

    fcsv_path = tmp_path / 'lands.fcsv'
    fcsv_path.write_text('# Markups fiducial file version = 4.10\n'
                         '# CoordinateSystem = 0\n'
                         '# columns = id,x,y,z,ow,ox,oy,oz,vis,sel,lock,label,desc,associatedNodeID\n'
                         'f_1,1,2,3,0,0,0,1,1,1,0,FH-l,,\n'
                         'f_2,-4,5,6,0,0,0,1,1,1,0,FH-r,,\n')

    # the file is in RAS and landmarks are converted to LPS by default
    lands = xreg.read_landmarks(str(fcsv_path))

    assert set(lands.keys()) == {'FH-l', 'FH-r'}
    assert np.allclose(lands['FH-l'], [-1, -2, 3])
    assert np.allclose(lands['FH-r'], [4, -5, 6])

    lands_ras = xreg.read_landmarks(str(fcsv_path), output_in_lps=False)
    assert np.allclose(lands_ras['FH-l'], [1, 2, 3])
