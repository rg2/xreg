"""Replaying registrations into movie frames and movies."""

import os

import numpy as np
import pytest

import xreg

from conftest import make_bobyqa, make_cam, make_gt_pose, make_level, make_second_view, make_synthetic_vol, trans


def make_hu_vol():
    """A synthetic volume in (approximately) Hounsfield units: air with a few dense objects."""
    att = make_synthetic_vol()

    hu = -1000 + 1500 * (att.numpy() / 0.02)

    return xreg.from_numpy(hu, att.spacing, att.origin, att.direction)


def run_small_regi(backend, num_views=1):
    """Runs a short translation only registration while collecting debug information."""
    vol_hu = make_hu_vol()
    vol = xreg.hu_to_lin_att(vol_hu)

    cams = [make_cam(96, 96, 2.0, 1000.0)]
    if num_views > 1:
        cams.append(make_second_view(cams[0]))

    gt_cam_to_vol = make_gt_pose(vol)

    drrs = xreg.compute_drrs([vol], cams, [gt_cam_to_vol], backend=backend)

    fixed_pds = [xreg.ProjData(c, d) for c, d in zip(cams, drrs)]

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vol_names = ['Synthetic']
    ml.vols = [vol]
    ml.fixed_proj_data = fixed_pds
    ml.init_cam_to_vols = [gt_cam_to_vol @ trans([4, -3, 0])]
    ml.ref_frames = ['identity']

    def make_regi():
        return xreg.SingleRegi(mov_vols=[0], ref_frames=[0], init_mov_vol_poses=['prev:0'],
                               regi=make_bobyqa([8] * 3, opt_vars='trans'))

    ml.levels = [make_level(0.5, backend, [make_regi()], num_views=num_views),
                 make_level(1.0, backend, [make_regi()], num_views=num_views)]

    ml.set_save_debug_info(True)

    # the volumes stored for replay are in HU, as they are thresholded to find object boundaries
    ml.debug_info.vols = [vol_hu]
    ml.debug_info.fixed_projs = fixed_pds
    ml.debug_info.regi_names = [['Coarse'], ['Fine']]

    ml.run()

    return ml


def is_gray(frame, border=12):
    """True when the frame has no color, excluding the colored border drawn about each tile."""
    f = frame[border:-border, border:-border]

    return np.array_equal(f[..., 0], f[..., 1]) and np.array_equal(f[..., 1], f[..., 2])


def test_replay_frames(backend, tmp_path):
    ml = run_small_regi(backend)

    dbg = ml.debug_info

    num_frames = dbg.total_num_projs_per_view()

    # an initial and a final frame for each of the two registrations, plus the iterations
    assert num_frames > 4

    replay = xreg.Regi2D3DReplay(backend)

    assert replay.backend is xreg.get_backend(backend)
    assert replay.num_frames == 0
    assert replay.drr_frames == []

    # the debug information that is still in memory is replayed
    replay.run(dbg)

    assert replay.num_frames == num_frames

    drr_frames = replay.drr_frames
    edge_frames = replay.edge_frames
    titles = replay.frame_titles

    assert len(drr_frames) == len(edge_frames) == len(titles) == num_frames

    for f in drr_frames + edge_frames:
        assert f.shape == (96, 96, 3)
        assert f.dtype == np.uint8

    # DRRs are gray, and the edges are overlaid in color on the fixed image
    assert all(is_gray(f) for f in drr_frames)
    assert not any(is_gray(f) for f in edge_frames)

    for f in drr_frames:
        assert f.max() > 100

    # the frames show the object moving from the initial to the final pose estimate
    assert not np.array_equal(drr_frames[0], drr_frames[-1])
    assert not np.array_equal(edge_frames[0], edge_frames[-1])

    assert titles[0].startswith('Level 0 (0.500), Coarse, 000')
    assert titles[-1].startswith('Level 1 (1.000), Fine, ')
    assert sum('Coarse' in t for t in titles) + sum('Fine' in t for t in titles) == num_frames

    # replaying the debug file written to disk produces the same frames
    debug_path = str(tmp_path / 'debug.h5')

    ml.write_debug(debug_path)

    replay_from_file = xreg.Regi2D3DReplay(backend)
    replay_from_file.run(debug_path)

    assert replay_from_file.frame_titles == titles
    assert all(np.array_equal(a, b) for a, b in zip(replay_from_file.drr_frames, drr_frames))
    assert all(np.array_equal(a, b) for a, b in zip(replay_from_file.edge_frames, edge_frames))

    # running again replaces the frames
    replay.run(dbg)
    assert replay.num_frames == num_frames


def test_replay_options(backend):
    ml = run_small_regi(backend)

    dbg = ml.debug_info

    defaults = xreg.Regi2D3DReplay(backend)

    # the defaults of the xreg-regi2d3d-replay tool
    assert defaults.compute_boundary_edges and defaults.compute_canny_edges
    assert defaults.boundary_edge_hu_thresh == 150
    assert (defaults.smooth_kernel_width, defaults.canny_lo_thresh, defaults.canny_hi_thresh) == (3, 0, 48)
    assert defaults.edge_dilate_width == 3
    assert defaults.proj_ds_factor == 1
    assert defaults.max_tile_cols == 7
    assert defaults.use_pat_rot_up and not defaults.flip_rows and not defaults.flip_cols
    assert not defaults.show_lands and not defaults.do_preproc

    defaults.run(dbg)
    ref_edges = defaults.edge_frames

    # no edge overlays: the frames are the (gray) fixed image
    no_edges = xreg.Regi2D3DReplay(backend, compute_boundary_edges=False, compute_canny_edges=False)
    no_edges.run(dbg)

    frames = no_edges.edge_frames
    assert len(frames) == len(ref_edges)
    assert all(is_gray(f) for f in frames)
    assert all(np.array_equal(f, frames[0]) for f in frames)

    # only one type of edge
    for kwargs in [{'compute_boundary_edges': False}, {'compute_canny_edges': False}]:
        r = xreg.Regi2D3DReplay(backend, **kwargs)
        r.run(dbg)

        frames = r.edge_frames
        assert len(frames) == len(ref_edges)
        assert not any(is_gray(f) for f in frames)

    # options may also be set using attributes: downsampled and flipped frames
    r = xreg.Regi2D3DReplay(backend)
    r.proj_ds_factor = 0.5
    r.run(dbg)

    ds_drrs = r.drr_frames
    assert all(f.shape == (48, 48, 3) for f in ds_drrs)

    r.flip_rows = True
    r.flip_cols = True
    r.run(dbg)

    assert all(np.array_equal(f, g[::-1, ::-1]) for f, g in zip(r.drr_frames, ds_drrs))

    # the backend may be changed
    r.backend = 'cpu'
    assert r.backend.kind == 'cpu'

    with pytest.raises(ValueError):
        r.backend = 'not-a-backend'

    with pytest.raises(TypeError):
        xreg.Regi2D3DReplay(backend, not_an_option=1)


def test_replay_multi_view(backend):
    ml = run_small_regi(backend, num_views=2)

    replay = xreg.Regi2D3DReplay(backend)
    replay.run(ml.debug_info)

    assert replay.num_frames == ml.debug_info.total_num_projs_per_view()

    # the views are tiled side-by-side
    frames = replay.drr_frames
    assert all(f.shape == (96, 192, 3) for f in frames)
    assert not np.array_equal(frames[0][:, :96], frames[0][:, 96:])

    # one column per row of the tiling
    replay.max_tile_cols = 1
    replay.run(ml.debug_info)

    assert all(f.shape == (192, 96, 3) for f in replay.edge_frames)


def test_replay_videos(tmp_path):
    ml = run_small_regi('cpu')

    replay = xreg.Regi2D3DReplay('cpu')

    drr_path = str(tmp_path / 'mov.mp4')
    edges_path = str(tmp_path / 'edges.mp4')

    # frames are required
    with pytest.raises(ValueError):
        replay.write_videos(drr_path, edges_path)

    replay.run(ml.debug_info)

    drr_frames = replay.drr_frames

    replay.write_videos(drr_path, edges_path, fps=10)

    assert os.path.getsize(drr_path) > 0
    assert os.path.getsize(edges_path) > 0

    # the titles are only burned into the movies, the frames are not modified
    assert all(np.array_equal(a, b) for a, b in zip(replay.drr_frames, drr_frames))

    # only one movie, with a length specified instead of a frame rate
    only_edges_path = str(tmp_path / 'only_edges.mp4')

    replay.write_videos(edges_path=only_edges_path, video_len=2.0)

    assert os.path.getsize(only_edges_path) > 0
    assert sorted(os.listdir(tmp_path)) == ['edges.mp4', 'mov.mp4', 'only_edges.mp4']

    with pytest.raises(ValueError):
        replay.write_videos()

    with pytest.raises(ValueError):
        replay.write_videos(drr_path, fps=0)

    # debug images of the frames
    replay.debug_img_prefix = str(tmp_path / 'frame')
    replay.run(ml.debug_info)
    replay.write_videos(drr_path, fps=10)

    pngs = [f for f in os.listdir(tmp_path) if f.endswith('.png')]

    # for each frame: the single view and the tiling, for both the DRRs and the edge overlays
    assert len(pngs) == 4 * replay.num_frames


def test_replay_errors(tmp_path):
    replay = xreg.Regi2D3DReplay('cpu')

    with pytest.raises(RuntimeError):
        replay.run(str(tmp_path / 'does_not_exist.h5'))

    not_h5_path = tmp_path / 'not_hdf5.h5'
    not_h5_path.write_text('this is not a HDF5 file')

    with pytest.raises(RuntimeError):
        replay.run(str(not_h5_path))

    with pytest.raises(RuntimeError):
        xreg.DebugRegiResultsMultiLevel.read(str(not_h5_path))

    with pytest.raises(TypeError):
        replay.run(None)

    assert replay.num_frames == 0
