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

"""Shared helpers and fixtures for the tests of the xreg Python bindings.

All tests use small synthetic data and do not require any files on disk.

Run with a build tree:
  PYTHONPATH=<build dir>/python python -m pytest python/tests
or after installing the package (pip install .):
  python -m pytest python/tests

Tests which take a "backend" argument are run once for each compute backend available on
the system, using the default device of each backend. The XREG_PY_TEST_BACKENDS environment
variable overrides this with a comma separated list of backend strings, which may include a
device, e.g.:
  XREG_PY_TEST_BACKENDS="cpu,metal:intel,ocl:uhd"
"""

import os

import numpy as np
import pytest

import xreg


def backends_to_test():
    """Backend strings (e.g. "cpu", "metal", "ocl:uhd") used by the tests."""
    s = os.environ.get('XREG_PY_TEST_BACKENDS', '')

    names = [b.strip() for b in s.split(',') if b.strip()]

    return names if names else xreg.available_backends()


def backends_overridden():
    """True when the user specified the backends (and devices) that may be used."""
    return bool(os.environ.get('XREG_PY_TEST_BACKENDS', '').strip())


def rot_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    x = np.eye(4)
    x[1:3, 1:3] = [[c, -s], [s, c]]
    return x


def rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    x = np.eye(4)
    x[0, 0], x[0, 2], x[2, 0], x[2, 2] = c, s, -s, c
    return x


def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    x = np.eye(4)
    x[0:2, 0:2] = [[c, -s], [s, c]]
    return x


def trans(t):
    x = np.eye(4)
    x[:3, 3] = t
    return x


def make_cam(num_rows=64, num_cols=64, spacing=2.0, focal_len=1000.0):
    cam = xreg.CameraModel()
    cam.setup(focal_len, num_rows, num_cols, spacing, spacing)
    return cam


def make_synthetic_vol(seed=42, offset=(0.0, 0.0, 0.0)):
    """A small volume of a few rods and blobs (linear attenuation-like values)."""
    n = 32
    spacing = 2.0

    z, y, x = np.meshgrid(*([np.arange(n) * spacing] * 3), indexing='ij')

    rng = np.random.default_rng(seed)

    arr = np.zeros((n, n, n), np.float32)

    # blobs of varying sizes
    for _ in range(6):
        c = rng.uniform(16, 48, size=3)
        r = rng.uniform(3, 6)
        arr += np.exp(-((x - c[0]) ** 2 + (y - c[1]) ** 2 + (z - c[2]) ** 2) / (2 * r * r)).astype(np.float32)

    # a few rods along each axis, which are sensitive to rotations
    arr[8:24, 10:13, 20:23] += 1
    arr[14:17, 6:26, 8:11] += 1
    arr[22:25, 18:21, 4:28] += 1

    return xreg.from_numpy(arr * 0.02, spacing=[spacing] * 3, origin=offset)


def reproj_err(cam, cam_to_vol, ref_cam_to_vol, pts_wrt_vol):
    """Mean 2D distance (pixels) between projections of volume points using two poses.

    The poses map the camera world frame to the volume.
    """
    def proj(p, x):
        world_pt = (np.linalg.inv(p) @ np.append(x, 1))[:3]
        return np.asarray(cam.phys_pt_to_ind_pt(world_pt))[:2]

    return np.mean([np.linalg.norm(proj(cam_to_vol, x) - proj(ref_cam_to_vol, x)) for x in pts_wrt_vol])


def pts_about(center, r=20.0):
    """Points spread about a center, used for evaluating re-projection errors."""
    offs = [(-r, -r, -r), (r, -r, 0), (0, r, r), (-r, 0, r), (0, 0, 0)]
    return [np.asarray(center) + np.array(o) for o in offs]


def pytest_generate_tests(metafunc):
    # Tests with a "backend" argument are run once for each backend to test
    if 'backend' in metafunc.fixturenames:
        metafunc.parametrize('backend', backends_to_test())
