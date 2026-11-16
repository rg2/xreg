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

"""Backends, factories, and setting up registration objects (nothing is run)."""

import numpy as np
import pytest

import xreg

from conftest import backends_overridden, make_cam


def test_version():
    assert isinstance(xreg.__version__, str) and xreg.__version__

    assert xreg.DEG2RAD == pytest.approx(np.pi / 180)


def test_stable_abi():
    import importlib.machinery
    import os
    import sys

    fname = os.path.basename(xreg._xreg.__file__)

    if sys.platform == 'win32':
        assert fname == '_xreg.pyd'
    else:
        assert '.abi3.' in fname

    assert any(fname.endswith(s) for s in importlib.machinery.EXTENSION_SUFFIXES)


def test_backends():
    backends = xreg.available_backends()

    assert 'cpu' in backends

    b = xreg.Backend('cpu')
    assert b.kind == 'cpu'
    assert b.device_name == 'cpu'

    # backends created from strings are cached
    assert xreg.get_backend('cpu') is xreg.get_backend('cpu')

    with pytest.raises(ValueError):
        xreg.Backend('not-a-backend')

    with pytest.raises(ValueError):
        xreg.Backend('cpu', 'some-device')

    assert xreg.available_devices('cpu') == ['cpu']

    with pytest.raises(ValueError):
        xreg.available_devices('not-a-backend')

    if not backends_overridden():
        # the default backend is the first one listed
        assert xreg.Backend().kind == backends[0]
        assert xreg.get_backend('').kind == backends[0]


def test_backend_devices(backend):
    kind = backend.split(':')[0]

    b = xreg.get_backend(backend)

    assert b.kind == kind
    assert b.device_name
    assert kind in repr(b)

    devs = xreg.available_devices(kind)
    assert len(devs) > 0

    if kind != 'cpu':
        with pytest.raises(ValueError):
            xreg.Backend(kind, 'this-device-does-not-exist')

        if not backends_overridden():
            # each device may be selected using its full ID string
            for d in devs:
                assert xreg.Backend(kind, d).kind == kind

            assert xreg.get_backend('{}:{}'.format(kind, devs[0])).kind == kind


def test_factories(backend):
    rc = xreg.make_ray_caster('line_int', backend)
    assert isinstance(rc, xreg.RayCaster)

    rc.ray_step_size = 0.5
    assert rc.ray_step_size == pytest.approx(0.5)

    rc.set_interp_method('linear')

    with pytest.raises(ValueError):
        rc.set_interp_method('cubic')

    assert isinstance(xreg.make_ray_caster('depth', xreg.get_backend(backend)), xreg.RayCaster)

    with pytest.raises(ValueError):
        xreg.make_ray_caster('not-a-ray-caster', backend)

    sm = xreg.make_sim_metric('patch_grad_ncc', backend, patch_radius=5, patch_stride=1,
                              smooth_img_before_sobel_kernel_radius=5)
    assert sm.is_patch_based and sm.is_grad_based

    sm.set_wgt_img(xreg.from_numpy(np.ones((8, 8), np.float32)))
    sm.set_wgt_img(None)

    for kind in ['ssd', 'ncc', 'grad_ncc', 'patch_ncc', 'patch-grad-ncc']:
        assert isinstance(xreg.make_sim_metric(kind, backend), xreg.ImgSimMetric2D)

    sm = xreg.make_sim_metric('ncc', backend)
    assert not sm.is_patch_based

    with pytest.raises(TypeError):
        sm.set_patch_radius(3)

    with pytest.raises(TypeError):
        xreg.make_sim_metric('ncc', backend, patch_radius=3)

    with pytest.raises(ValueError):
        xreg.make_sim_metric('not-a-metric', backend)


def test_string_parsing():
    p = xreg.make_init_pose('prev:2')
    assert isinstance(p, xreg.InitPosePrevPoseEst)
    assert p.vol_idx == 2

    assert isinstance(xreg.make_init_pose('identity'), xreg.InitPoseId)
    assert isinstance(xreg.make_init_pose('prev_rot:1'), xreg.InitPosePrevPoseEstRotOnly)
    assert isinstance(xreg.make_init_pose('prev_trans:0'), xreg.InitPosePrevPoseEstTransOnly)

    for bad in ['prev', 'prev:', 'prev:x', 'prev:-1', 'blah']:
        with pytest.raises(ValueError):
            xreg.make_init_pose(bad)

    assert xreg.make_opt_vars('se3').num_params == 6
    assert xreg.make_opt_vars('so3').num_params == 3
    assert xreg.make_opt_vars('trans').num_params == 3
    assert isinstance(xreg.make_opt_vars('SE3'), xreg.SE3OptVarsLieAlg)

    with pytest.raises(ValueError):
        xreg.make_opt_vars('se4')

    sr = xreg.SingleRegi(mov_vols=[0, 1], init_mov_vol_poses=['prev:0', xreg.InitPosePrevPoseEst(1)])
    assert [p.vol_idx for p in sr.init_mov_vol_poses] == [0, 1]

    sr.static_vol_poses = ['prev:3', 'identity']
    assert sr.static_vol_poses[0].vol_idx == 3

    ml = xreg.MultiLevelMultiObjRegi()
    ml.ref_frames = ['identity', xreg.StaticRefFrame(np.eye(4))]
    assert len(ml.ref_frames) == 2
    assert isinstance(ml.ref_frames[0], xreg.StaticRefFrame)

    with pytest.raises(ValueError):
        ml.ref_frames = ['not-a-ref-frame']


def test_opt_vars():
    se3 = xreg.SE3OptVarsLieAlg()

    assert np.allclose(se3(np.zeros(6)), np.eye(4))

    x = se3(np.array([0, 0, 0, 1, 2, 3.0]))
    assert np.allclose(x[:3, 3], [1, 2, 3])
    assert np.allclose(x[:3, :3], np.eye(3))

    so3 = xreg.SO3OptVarsLieAlg()
    r = so3(np.array([0, 0, np.pi / 2]))
    assert np.allclose(r[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-6)


def test_dists_and_penalty_fns():
    n = xreg.NormalDist1D(0, 2)
    assert n.density(0) > n.density(1) > n.density(2)
    assert n.log_density(0) == pytest.approx(np.log(n.density(0)), rel=1e-5)

    assert xreg.NullDist(1).density(5) == pytest.approx(1)

    f = xreg.FoldNormDist(10 * xreg.DEG2RAD, 10 * xreg.DEG2RAD)
    assert f.dim == 1

    pen = xreg.Regi2D3DPenaltyFnSE3EulerDecomp(rot_x_pdf=n, trans_z_pdf=xreg.NormalDist1D(0, 100))
    assert pen.rot_x_pdf is n
    assert pen.rot_y_pdf is None

    pen.rot_y_pdf = xreg.NormalDist1D(0, 1)
    assert isinstance(pen.rot_y_pdf, xreg.NormalDist1D)

    mag = xreg.Regi2D3DPenaltyFnSE3Mag(rot_pdfs_per_obj=[f], trans_pdfs_per_obj=[xreg.NullDist(1)])
    assert len(mag.rot_pdfs_per_obj) == 1
    assert isinstance(mag.trans_pdfs_per_obj[0], xreg.NullDist)

    gp = xreg.Regi2D3DPenaltyFnGlobalPelvis()
    gp.vol_idx = 0
    gp.pat_is_up = True
    gp.left_fh_wrt_vol = [1, 2, 3]
    assert np.allclose(gp.left_fh_wrt_vol, [1, 2, 3])

    lr = xreg.Regi2D3DPenaltyFnLandReproj()
    lr.set_lands({'a': [1, 2, 3], 'b': [4, 5, 6], 'c': [7, 8, 9]},
                 {'a': [10, 20], 'c': [30, 40], 'd': [0, 0]})
    assert lr.num_lands == 2

    combo = xreg.Regi2D3DPenaltyFnCombo()
    combo.pen_fns = [pen, mag]
    assert len(combo.pen_fns) == 2

    # penalty functions may be attached to any optimizer
    regi = xreg.Intensity2D3DRegiCMAES()
    regi.set_penalty_fn(pen)
    assert regi.penalty_fn is pen
    regi.set_img_sim_penalty_coefs(0.9, 0.1)

    regi.set_penalty_fn(None)
    assert regi.penalty_fn is None


def test_optimizers():
    for name in ['CMAES', 'BOBYQA', 'NelderMead', 'Sbplx', 'NEWUOA', 'PRAXIS', 'CRS', 'DIRECT',
                 'DIRECTL', 'DIRECTRandL', 'DESCH', 'DISRES', 'DiffEvo', 'PSO', 'HillClimb',
                 'Exhaustive']:
        regi = getattr(xreg, 'Intensity2D3DRegi' + name)()

        assert isinstance(regi, xreg.Intensity2D3DRegi)

        regi.set_opt_vars('se3')
        regi.set_opt_vars(xreg.SO3OptVarsLieAlg())

    cmaes = xreg.Intensity2D3DRegiCMAES()
    cmaes.set_opt_x_tol(0.01)
    cmaes.set_opt_obj_fn_tol(0.01)
    cmaes.set_pop_size(100)
    cmaes.set_sigma([15 * xreg.DEG2RAD] * 3 + [50, 50, 100])
    cmaes.set_sigma(np.ones(6))

    bobyqa = xreg.Intensity2D3DRegiBOBYQA()
    assert isinstance(bobyqa, xreg.Intensity2D3DRegiNLOptInterface)
    bobyqa.set_bounds([2.5 * xreg.DEG2RAD] * 3 + [5, 5, 10])

    de = xreg.Intensity2D3DRegiDiffEvo()
    de.set_box_constraint([60 * xreg.DEG2RAD, 40 * xreg.DEG2RAD, 10 * xreg.DEG2RAD, 200, 200, 250])
    de.set_max_num_iters(400)
    de.set_pop_size(1000)

    pso = xreg.Intensity2D3DRegiPSO()
    pso.set_num_particles(100)
    pso.set_max_num_iters(50)
    pso.set_box_constraint([7.5 * xreg.DEG2RAD, 10 * xreg.DEG2RAD, 10 * xreg.DEG2RAD, 20, 20, 25])

    ex = xreg.Intensity2D3DRegiExhaustive()
    ex.set_opt_vars('se3')
    ex.set_cam_wrt_vols([(-5 * xreg.DEG2RAD, 5 * xreg.DEG2RAD, 1 * xreg.DEG2RAD),
                         (0, 0, 0), (0, 0, 0), (-10, 10, 2), (0, 0, 0), (0, 0, 0)], 64)
    assert ex.tot_num_xforms == 11 * 11

    ex.set_cam_wrt_vols([[np.eye(4), np.eye(4), np.eye(4)]])
    assert ex.tot_num_xforms == 3


def test_ref_frames():
    cam = make_cam()

    rf = xreg.CamAlignRefFrameWithCurPose(vol_idx=1, cam_extrins=cam.extrins,
                                          center_of_rot_wrt_vol=[1, 2, 3])
    assert rf.vol_idx == 1
    assert np.allclose(rf.center_of_rot_wrt_vol, [1, 2, 3])
    assert isinstance(rf, xreg.RefFrame)

    dyn = xreg.DynVolRefFrame(dyn_vol_idx=2)
    assert dyn.dyn_vol_idx == 2
    assert np.allclose(dyn.inter_to_vol, np.eye(4))
    assert not dyn.standard

    s = xreg.MultiLevelMultiObjRegi.make_static_ref_frame(np.eye(4), True)
    assert s.maps_vol_to_ref and s.standard

    assert isinstance(xreg.MultiLevelMultiObjRegi.make_id_ref_frame(), xreg.StaticRefFrame)


def test_levels_reference_semantics():
    ml = xreg.MultiLevelMultiObjRegi()

    ml.levels = [xreg.Level(ds_factor=0.25)]
    ml.levels.append(xreg.Level(ds_factor=0.5))

    assert len(ml.levels) == 2

    ml.levels[0].ds_factor = 0.125
    ml.levels[1].regis.append(xreg.SingleRegi(mov_vols=[0]))
    ml.levels[1].regis[0].mov_vols = [1]

    assert ml.levels[0].ds_factor == pytest.approx(0.125)
    assert ml.levels[1].regis[0].mov_vols == [1]

    # objects are shared, not copied
    sm = xreg.make_sim_metric('ncc', 'cpu')
    ml.levels[0].sim_metrics = [sm]
    assert ml.levels[0].sim_metrics[0] is sm

    regi = xreg.Intensity2D3DRegiBOBYQA()
    ml.levels[1].regis[0].regi = regi
    assert ml.levels[1].regis[0].regi is regi


def test_masks_and_names():
    ml = xreg.MultiLevelMultiObjRegi()

    mask = xreg.from_numpy(np.ones((4, 4), np.uint8))

    ml.masks_2d = [None, mask]
    assert ml.masks_2d[0] is None
    assert ml.masks_2d[1] == mask

    ml.vol_names = ['Pelvis', 'Femur']
    assert ml.vol_names == ['Pelvis', 'Femur']

    with pytest.raises(TypeError):
        ml.masks_2d = [xreg.from_numpy(np.ones((4, 4), np.float32))]


def test_assert_raises_exception():
    cmaes = xreg.Intensity2D3DRegiCMAES()
    cmaes.set_pop_size(10)

    # one coefficient per population member is required, this triggers an xregASSERT,
    # which should raise a Python exception instead of terminating the process
    with pytest.raises(RuntimeError):
        cmaes.set_img_sim_penalty_coefs([0.9], [0.1])


def test_run_validation():
    ml = xreg.MultiLevelMultiObjRegi()

    with pytest.raises(ValueError):
        ml.run()

    ml.vols = [xreg.from_numpy(np.zeros((4, 4, 4), np.float32))]
    ml.fixed_proj_data = [xreg.ProjData(make_cam(4, 4, 1.0), xreg.from_numpy(np.zeros((4, 4), np.float32)))]
    ml.init_cam_to_vols = [np.eye(4)]
    ml.levels = [xreg.Level(ds_factor=1.0, ray_caster=xreg.make_ray_caster('line_int', 'cpu'),
                            sim_metrics=[], fixed_imgs_to_use=[0])]

    # missing similarity metric
    with pytest.raises(ValueError):
        ml.run()

    ml.levels[0].sim_metrics = [xreg.make_sim_metric('ncc', 'cpu')]
    ml.levels[0].regis = [xreg.SingleRegi(mov_vols=[3], init_mov_vol_poses=['prev:0'],
                                          regi=xreg.Intensity2D3DRegiBOBYQA())]

    # out of range volume index
    with pytest.raises(IndexError):
        ml.run()

    ml.levels[0].regis[0].mov_vols = [0]
    ml.levels[0].regis[0].ref_frames = [0]

    # reference frame index is out of range (no reference frames were set)
    with pytest.raises(IndexError):
        ml.run()

    ml.vols = [None]
    with pytest.raises(ValueError):
        ml.run()
