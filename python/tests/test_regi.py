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

"""DRRs and registrations run on small synthetic data, using each available backend."""

import numpy as np
import pytest

import xreg

from conftest import make_cam, make_synthetic_vol, pts_about, reproj_err, rot_x, rot_y, rot_z, trans


def make_gt_pose(vol, rx_deg=10.0, ry_deg=-5.0, offset=(0.0, 0.0, 0.0)):
    """A camera world to volume pose which places the volume half-way between the X-ray source
    and the detector (with an optional offset in camera coordinates)."""
    center = xreg.vol_center_as_phys_pt(vol)

    return (trans(center) @ rot_x(np.radians(rx_deg)) @ rot_y(np.radians(ry_deg)) @
            trans([0, 0, 500]) @ trans(-np.asarray(offset)))


def perturb(vol, cam_to_vol, rot_deg=(3.0, -3.0, 0.0), t=(3.0, -2.0, 5.0)):
    """Perturbs a pose with a rotation about the volume center and a translation."""
    center = xreg.vol_center_as_phys_pt(vol)

    r = rot_x(np.radians(rot_deg[0])) @ rot_y(np.radians(rot_deg[1])) @ rot_z(np.radians(rot_deg[2]))

    return trans(center) @ r @ trans(-center) @ trans(t) @ cam_to_vol


def make_second_view(cam, rot_deg=30.0):
    """A camera rotated about the point half-way between the source and the detector of the
    first camera, the world frame is the frame of the first camera."""
    extrins = trans([0, 0, -500]) @ rot_y(np.radians(rot_deg)) @ trans([0, 0, 500])

    cam2 = xreg.CameraModel()
    cam2.setup(cam.intrins, extrins, cam.num_det_rows, cam.num_det_cols,
               cam.det_row_spacing, cam.det_col_spacing)

    return cam2


def make_level(ds_factor, backend, regis, num_views=1, sim_metric='ncc', **sm_kwargs):
    return xreg.Level(ds_factor=ds_factor,
                      ray_caster=xreg.make_ray_caster('line_int', backend),
                      sim_metrics=[xreg.make_sim_metric(sim_metric, backend, **sm_kwargs)
                                   for _ in range(num_views)],
                      fixed_imgs_to_use=list(range(num_views)),
                      regis=regis)


def make_bobyqa(bounds, opt_vars='se3'):
    bobyqa = xreg.Intensity2D3DRegiBOBYQA()
    bobyqa.set_opt_vars(opt_vars)
    bobyqa.set_opt_x_tol(1e-4)
    bobyqa.set_opt_obj_fn_tol(1e-5)
    bobyqa.set_bounds(bounds)
    return bobyqa


def is_rigid(x):
    x = np.asarray(x, dtype=np.float64)
    return (np.allclose(x[:3, :3] @ x[:3, :3].T, np.eye(3), atol=1e-3) and
            np.allclose(x[3], [0, 0, 0, 1]) and np.all(np.isfinite(x)))


def test_compute_drrs(backend):
    vol = make_synthetic_vol()

    cam = make_cam(96, 96, 2.0, 1000.0)

    gt_cam_to_vol = make_gt_pose(vol)

    drrs = xreg.compute_drrs([vol], [cam, make_second_view(cam)], [gt_cam_to_vol], backend=backend)

    assert len(drrs) == 2

    for drr in drrs:
        assert isinstance(drr, xreg.Image2DF32)
        assert drr.shape == (96, 96)
        assert drr.numpy().max() > 0
        assert np.all(np.isfinite(drr.numpy()))

    # the views differ
    assert not np.allclose(drrs[0].numpy(), drrs[1].numpy())

    # the volume is projected near the center of the detector
    arr = drrs[0].numpy()
    rows, cols = np.nonzero(arr > 0.5 * arr.max())
    assert 30 < rows.mean() < 66
    assert 30 < cols.mean() < 66

    # compare to the CPU implementation, which uses a different sampling/interpolation
    # scheme along each ray, so the images are similar, but not identical
    ref = xreg.compute_drrs([vol], [cam], [gt_cam_to_vol], backend='cpu')[0].numpy()
    assert np.corrcoef(arr.ravel(), ref.ravel())[0, 1] > 0.99
    assert abs(arr.sum() - ref.sum()) < 0.15 * ref.sum()

    # two objects: line integrals are summed
    two = xreg.compute_drrs([vol, vol], [cam], [gt_cam_to_vol, gt_cam_to_vol], backend=backend)[0].numpy()
    assert np.allclose(two, 2 * arr, rtol=1e-3, atol=1e-3 * arr.max())

    with pytest.raises(ValueError):
        xreg.compute_drrs([vol], [cam], [], backend=backend)

    with pytest.raises(ValueError):
        xreg.compute_drrs([None], [cam], [gt_cam_to_vol], backend=backend)


def test_single_view_regi(backend, tmp_path):
    vol = make_synthetic_vol()

    # the volume (64 mm wide) projects to roughly 64 pixels at the center of the detector
    cam = make_cam(96, 96, 2.0, 1000.0)

    center = xreg.vol_center_as_phys_pt(vol)

    gt_cam_to_vol = make_gt_pose(vol)

    drr = xreg.compute_drrs([vol], [cam], [gt_cam_to_vol], backend=backend)[0]

    init_cam_to_vol = perturb(vol, gt_cam_to_vol)

    fixed_pd = xreg.ProjData(cam, drr)

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vol_names = ['Synthetic']
    ml.vols = [vol]
    ml.fixed_proj_data = [fixed_pd]
    ml.init_cam_to_vols = [init_cam_to_vol]
    ml.ref_frames = [xreg.CamAlignRefFrameWithCurPose(vol_idx=0, cam_extrins=cam.extrins,
                                                      center_of_rot_wrt_vol=center)]

    cmaes = xreg.Intensity2D3DRegiCMAES()
    cmaes.set_opt_vars('se3')
    cmaes.set_opt_x_tol(0.01)
    cmaes.set_opt_obj_fn_tol(0.01)
    cmaes.set_pop_size(20)
    cmaes.set_sigma([np.radians(3)] * 3 + [3, 3, 6])

    pen_fn = xreg.Regi2D3DPenaltyFnSE3EulerDecomp(
        rot_x_pdf=xreg.NormalDist1D(0, np.radians(10)), rot_y_pdf=xreg.NormalDist1D(0, np.radians(10)),
        rot_z_pdf=xreg.NormalDist1D(0, np.radians(10)), trans_x_pdf=xreg.NormalDist1D(0, 20),
        trans_y_pdf=xreg.NormalDist1D(0, 20), trans_z_pdf=xreg.NormalDist1D(0, 100))

    cmaes.set_penalty_fn(pen_fn)
    cmaes.set_img_sim_penalty_coefs(0.9, 0.1)

    ml.levels = [make_level(0.5, backend, [xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                                           init_mov_vol_poses=['prev:0'], regi=cmaes)],
                            sim_metric='patch_grad_ncc', patch_radius=5, patch_stride=1,
                            smooth_img_before_sobel_kernel_radius=1),
                 make_level(1.0, backend, [xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                                           init_mov_vol_poses=['prev:0'],
                                                           regi=make_bobyqa([np.radians(5)] * 3 + [10] * 3))])]

    ml.set_save_debug_info(True)

    assert ml.debug_info is not None

    ml.debug_info.vols = [vol]
    ml.debug_info.fixed_projs = [fixed_pd]
    ml.debug_info.regi_names = [['CMA-ES'], ['BOBYQA']]
    ml.debug_info.proj_pre_proc_info = xreg.ProjPreProcParams(crop_width=0, no_log_remap=True)

    ml.run()

    assert len(ml.cur_cam_to_vols) == 1
    assert is_rigid(ml.cur_cam_to_vols[0])

    # single-view registration constrains the in-plane components of the pose well, so
    # compare the re-projection errors of points spread throughout the volume
    pts = pts_about(center)

    init_err = reproj_err(cam, init_cam_to_vol, gt_cam_to_vol, pts)
    regi_err = reproj_err(cam, ml.cur_cam_to_vols[0], gt_cam_to_vol, pts)

    assert init_err > 2
    assert regi_err < 0.25 * init_err

    # the inputs are not modified
    assert np.allclose(ml.init_cam_to_vols[0], init_cam_to_vol, atol=1e-4)

    # resources are released by default after running
    assert ml.levels[0].ray_caster is None
    assert ml.levels[0].sim_metrics == []
    assert ml.levels[0].regis[0].regi is None

    # debug information
    assert len(ml.debug_info.multi_res_levels) == 2
    assert ml.debug_info.multi_level_run_time_secs > 0

    debug_path = str(tmp_path / 'debug.h5')

    ml.write_debug(debug_path)

    dbg = xreg.DebugRegiResultsMultiLevel.read(debug_path)

    assert dbg.regi_names == [['CMA-ES'], ['BOBYQA']]
    assert np.allclose(dbg.multi_res_levels, [0.5, 1.0])
    assert len(dbg.vols) == 1
    assert np.array_equal(dbg.vols[0].numpy(), vol.numpy())
    assert len(dbg.fixed_projs) == 1
    assert dbg.proj_pre_proc_info.no_log_remap


def test_keep_resources():
    vol = make_synthetic_vol()

    cam = make_cam(96, 96, 2.0, 1000.0)

    gt_cam_to_vol = make_gt_pose(vol)

    drr = xreg.compute_drrs([vol], [cam], [gt_cam_to_vol], backend='cpu')[0]

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vols = [vol]
    ml.fixed_proj_data = [xreg.ProjData(cam, drr)]
    ml.init_cam_to_vols = [perturb(vol, gt_cam_to_vol, rot_deg=(0, 0, 0), t=(2, -2, 0))]
    ml.ref_frames = ['identity']
    ml.dealloc_resources = False

    regi = make_bobyqa([5] * 3, opt_vars='trans')

    ml.levels = [make_level(0.5, 'cpu', [xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                                         init_mov_vol_poses=['prev:0'], regi=regi)])]

    ml.run()

    # the objects are still referenced by the level, and the result of the single
    # registration is available
    assert ml.levels[0].ray_caster is not None
    assert len(ml.levels[0].sim_metrics) == 1
    assert ml.levels[0].regis[0].regi is regi
    assert np.allclose(regi.regi_xform(0), ml.cur_cam_to_vols[0], atol=1e-4)

    assert ml.debug_info is None

    with pytest.raises(ValueError):
        ml.write_debug('should_not_be_written.h5')


def test_multi_obj_multi_view_regi(backend):
    vol_a = make_synthetic_vol(seed=42)
    vol_b = make_synthetic_vol(seed=7, offset=(100, 0, 0))

    cams = [make_cam(96, 96, 2.0, 1000.0)]
    cams.append(make_second_view(cams[0]))

    # the objects are side-by-side in the views
    gt_a = make_gt_pose(vol_a, offset=(-35, 0, 0))
    gt_b = make_gt_pose(vol_b, rx_deg=-8, ry_deg=12, offset=(35, 0, 0))

    drrs = xreg.compute_drrs([vol_a, vol_b], cams, [gt_a, gt_b], backend=backend)

    init_a = perturb(vol_a, gt_a, rot_deg=(2, -2, 1), t=(2, -2, 2))
    init_b = perturb(vol_b, gt_b, rot_deg=(-2, 2, 1), t=(-2, 2, -2))

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vol_names = ['A', 'B']
    ml.vols = [vol_a, vol_b]
    ml.fixed_proj_data = [xreg.ProjData(c, d) for c, d in zip(cams, drrs)]
    ml.init_cam_to_vols = [init_a, init_b]

    center_a = xreg.vol_center_as_phys_pt(vol_a)
    center_b = xreg.vol_center_as_phys_pt(vol_b)

    # each object is optimized in a frame located at its center
    ml.ref_frames = [xreg.StaticRefFrame(trans(-center_a), maps_vol_to_ref=True),
                     xreg.StaticRefFrame(trans(-center_b), maps_vol_to_ref=True)]

    bounds = [np.radians(5)] * 3 + [8] * 3

    lvl_1 = make_level(0.5, backend, num_views=2, regis=[
        # object A, while B is not projected
        xreg.SingleRegi(mov_vols=[0], ref_frames=[0], init_mov_vol_poses=['prev:0'],
                        regi=make_bobyqa(bounds)),
        # object B, with A projected in the background at its current pose estimate
        xreg.SingleRegi(mov_vols=[1], ref_frames=[1], static_vols=[0],
                        init_mov_vol_poses=['prev:1'], static_vol_poses=['prev:0'],
                        regi=make_bobyqa(bounds))])

    lvl_2 = make_level(1.0, backend, num_views=2, regis=[
        # both objects simultaneously
        xreg.SingleRegi(mov_vols=[0, 1], ref_frames=[0, 1],
                        init_mov_vol_poses=['prev:0', xreg.InitPosePrevPoseEst(1)],
                        regi=make_bobyqa(bounds * 2))])

    ml.levels = [lvl_1, lvl_2]

    ml.run()

    assert len(ml.cur_cam_to_vols) == 2

    for vol, gt, init, est in [(vol_a, gt_a, init_a, ml.cur_cam_to_vols[0]),
                               (vol_b, gt_b, init_b, ml.cur_cam_to_vols[1])]:
        assert is_rigid(est)

        pts = pts_about(xreg.vol_center_as_phys_pt(vol))

        for cam in cams:
            init_err = reproj_err(cam, init, gt, pts)
            regi_err = reproj_err(cam, est, gt, pts)

            assert init_err > 1
            assert regi_err < 0.35 * init_err


def test_global_search_optimizers(backend):
    vol = make_synthetic_vol()

    cam = make_cam(96, 96, 2.0, 1000.0)

    center = xreg.vol_center_as_phys_pt(vol)

    gt_cam_to_vol = make_gt_pose(vol)

    drr = xreg.compute_drrs([vol], [cam], [gt_cam_to_vol], backend=backend)[0]

    # an in-plane translation in the camera frame, which lies on the grid searched below
    init_cam_to_vol = gt_cam_to_vol @ trans([6, -4, 0])

    def make_regi(optimizer):
        ml = xreg.MultiLevelMultiObjRegi()
        ml.vols = [vol]
        ml.fixed_proj_data = [xreg.ProjData(cam, drr)]
        ml.init_cam_to_vols = [init_cam_to_vol]
        ml.ref_frames = [xreg.CamAlignRefFrameWithCurPose(vol_idx=0, cam_extrins=cam.extrins,
                                                          center_of_rot_wrt_vol=center)]
        ml.levels = [make_level(0.5, backend, [xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                                               init_mov_vol_poses=['prev:0'],
                                                               regi=optimizer)])]
        return ml

    pts = pts_about(center)

    init_err = reproj_err(cam, init_cam_to_vol, gt_cam_to_vol, pts)
    assert init_err > 2

    # exhaustive/grid search
    ex = xreg.Intensity2D3DRegiExhaustive()
    ex.set_opt_vars('se3')
    ex.set_cam_wrt_vols([(0, 0, 0), (0, 0, 0), (0, 0, 0), (-8, 8, 2), (-8, 8, 2), (0, 0, 0)], 32)

    assert ex.tot_num_xforms == 81

    ml = make_regi(ex)
    ml.run()

    assert reproj_err(cam, ml.cur_cam_to_vols[0], gt_cam_to_vol, pts) < 0.1 * init_err

    # differential evolution, with a regularizer
    de = xreg.Intensity2D3DRegiDiffEvo()
    de.set_opt_vars('trans')
    de.set_box_constraint([10, 10, 10])
    de.set_max_num_iters(15)
    de.set_pop_size(20)

    pen_fn = xreg.Regi2D3DPenaltyFnSE3Mag(rot_pdfs_per_obj=[xreg.NullDist(1)],
                                          trans_pdfs_per_obj=[xreg.FoldNormDist(10, 10)])
    de.set_penalty_fn(pen_fn)
    de.set_img_sim_penalty_coefs(0.9, 0.1)

    ml = make_regi(de)
    ml.run()

    assert is_rigid(ml.cur_cam_to_vols[0])

    # particle swarm
    pso = xreg.Intensity2D3DRegiPSO()
    pso.set_opt_vars('trans')
    pso.set_num_particles(20)
    pso.set_max_num_iters(15)
    pso.set_box_constraint([10, 10, 10])

    ml = make_regi(pso)
    ml.run()

    assert is_rigid(ml.cur_cam_to_vols[0])


def test_regi_with_mask_and_patch_weights(backend):
    vol = make_synthetic_vol()

    cam = make_cam(96, 96, 2.0, 1000.0)

    gt_cam_to_vol = make_gt_pose(vol)

    drr = xreg.compute_drrs([vol], [cam], [gt_cam_to_vol], backend=backend)[0]

    init_cam_to_vol = gt_cam_to_vol @ trans([3, -2, 0])

    # only use the central portion of the image
    mask = np.zeros((96, 96), np.uint8)
    mask[8:88, 8:88] = 1

    wgts = np.zeros((96, 96), np.float32)
    wgts[16:80, 16:80] = 1

    sm = xreg.make_sim_metric('patch_ncc', backend, patch_radius=4, patch_stride=2)
    sm.set_wgt_img(xreg.from_numpy(wgts))

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vols = [vol]
    ml.fixed_proj_data = [xreg.ProjData(cam, drr)]
    ml.masks_2d = [xreg.from_numpy(mask)]
    ml.init_cam_to_vols = [init_cam_to_vol]
    ml.ref_frames = ['identity']
    ml.levels = [xreg.Level(ds_factor=1.0, ray_caster=xreg.make_ray_caster('line_int', backend),
                            sim_metrics=[sm], fixed_imgs_to_use=[0],
                            regis=[xreg.SingleRegi(mov_vols=[0], ref_frames=[0],
                                                   init_mov_vol_poses=['prev:0'],
                                                   regi=make_bobyqa([8] * 3, opt_vars='trans'))])]

    ml.run()

    pts = pts_about(xreg.vol_center_as_phys_pt(vol))

    init_err = reproj_err(cam, init_cam_to_vol, gt_cam_to_vol, pts)
    regi_err = reproj_err(cam, ml.cur_cam_to_vols[0], gt_cam_to_vol, pts)

    assert regi_err < 0.35 * init_err


def test_pnp():
    cam = make_cam(256, 256, 1.0, 1000.0)

    rng = np.random.default_rng(5)

    lands_3d = {'L{}'.format(i): rng.uniform(-40, 40, 3) for i in range(8)}

    gt_cam_to_vol = rot_x(np.radians(20)) @ rot_z(np.radians(-15)) @ trans([2, -4, 500])

    def proj(p):
        world_pt = (np.linalg.inv(gt_cam_to_vol) @ np.append(p, 1))[:3]
        return np.asarray(cam.phys_pt_to_ind_pt(world_pt))[:2]

    lands_2d = {k: proj(v) for k, v in lands_3d.items()}

    # all landmarks are visible
    for v in lands_2d.values():
        assert np.all(v > 0) and np.all(v < 255)

    est = xreg.pnp_posit_and_reproj_cmaes(cam, lands_3d, lands_2d)

    pts = list(lands_3d.values())

    assert reproj_err(cam, est, gt_cam_to_vol, pts) < 1.0

    # refinement starting from a perturbed pose, using a subset of the landmarks in 2D
    init = gt_cam_to_vol @ trans([5, -5, 10])

    est = xreg.pnp_reproj_cmaes(cam, lands_3d, {k: lands_2d[k] for k in list(lands_2d)[:5]}, init)

    assert reproj_err(cam, est, gt_cam_to_vol, pts) < 0.5 * reproj_err(cam, init, gt_cam_to_vol, pts)


def test_anterior_pelvic_plane():
    # Landmarks in an LPS volume frame: left is +X, posterior is +Y, superior is +Z
    lands = {'ASIS-l': [120.0, -80, 100], 'ASIS-r': [-120.0, -80, 100],
             'IPS-l': [30.0, -85, 0], 'IPS-r': [-30.0, -85, 0],
             'FH-l': [90.0, -20, 20], 'FH-r': [-90.0, -20, 20]}

    app_to_vol = xreg.anterior_pelvic_plane_from_landmarks(lands)

    assert app_to_vol.shape == (4, 4)
    assert is_rigid(app_to_vol)

    # the origin may be placed at a femoral head
    app_to_vol_fh = xreg.anterior_pelvic_plane_from_landmarks(lands, 'left_fh')

    assert np.allclose(app_to_vol_fh[:3, 3], lands['FH-l'], atol=1e-4)
    assert np.allclose(app_to_vol_fh[:3, :3], app_to_vol[:3, :3], atol=1e-5)

    with pytest.raises(ValueError):
        xreg.anterior_pelvic_plane_from_landmarks(lands, 'not-an-origin')

    cam = make_cam(256, 256, 1.0, 1000.0)

    cam_to_app = xreg.create_ap_view_of_app(cam, 0.8, True, True)

    assert is_rigid(cam_to_app)

    # the APP origin projects to the center of the detector
    cam_to_vol = app_to_vol @ cam_to_app

    world_pt = (np.linalg.inv(cam_to_vol) @ np.append(app_to_vol[:3, 3], 1))[:3]

    assert np.allclose(np.asarray(cam.phys_pt_to_ind_pt(world_pt))[:2], [127.5, 127.5], atol=1.5)

    # a reference point may be aligned to a specific pixel
    ref_pt_wrt_app = np.array([10.0, 20, 0])
    ref_idx = np.array([100.0, 150])

    cam_to_app = xreg.create_ap_view_of_app(cam, 0.8, True, True, ref_pt_wrt_app, ref_idx)

    world_pt = (np.linalg.inv(cam_to_app) @ np.append(ref_pt_wrt_app, 1))[:3]

    assert np.allclose(np.asarray(cam.phys_pt_to_ind_pt(world_pt))[:2], ref_idx, atol=1.0)

    with pytest.raises(ValueError):
        xreg.create_ap_view_of_app(cam, 0.8, True, True, ref_pt_wrt_app)


def test_proj_3d_labels_to_2d(backend):
    # two boxes, one in front of the other in the left half of the image
    lbl = np.zeros((32, 32, 32), np.uint8)
    lbl[8:24, 8:24, 4:14] = 1
    lbl[8:24, 8:24, 18:28] = 2

    label_vol = xreg.from_numpy(lbl, spacing=[2.0] * 3)

    cam = make_cam(96, 96, 2.0, 1000.0)

    center = xreg.vol_center_as_phys_pt(label_vol)

    cam_to_vol = trans(center) @ trans([0, 0, 500])

    def single_chan_fn(depths):
        assert len(depths) == 3

        seg = np.zeros(depths[1].shape, np.uint8)
        seg[depths[2] < xreg.RAY_CAST_MAX_DEPTH] = 2
        seg[depths[1] < xreg.RAY_CAST_MAX_DEPTH] = 1

        return seg

    lp = xreg.Proj3DLabelsTo2D()
    lp.depth_ray_caster = xreg.make_ray_caster('depth', backend)
    lp.label_vol = label_vol
    lp.labels_to_proj = [1, 2]
    lp.cam = cam
    lp.set_single_chan_seg_fn(single_chan_fn)

    lp.init()

    lp.obj_poses = [cam_to_vol, cam_to_vol]

    lp.run()

    chans = lp.seg_channels

    assert len(chans) == 3
    assert all(c.shape == (96, 96) and c.dtype == np.float32 for c in chans)

    # each pixel is assigned to the background or shared among the labels intersected
    assert np.allclose(sum(chans), 1)

    for c in chans:
        assert 0 < c.sum() < c.size

    seg = lp.single_chan_seg.numpy()

    assert seg.shape == (96, 96)
    assert set(np.unique(seg)) == {0, 1, 2}

    assert np.array_equal(seg == 0, chans[0] == 1)
    assert np.array_equal(seg == 1, chans[1] > 0)

    lp.obj_poses = [cam_to_vol]
    with pytest.raises(ValueError):
        lp.run()
