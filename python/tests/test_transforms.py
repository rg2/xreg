# MIT License
#
# Copyright (c) 2022 Robert Grupp
#
# See the repository LICENSE file for the full MIT License text.

"""Round-trip and correctness tests for xreg.transforms."""

import math

import numpy as np
import pytest


def _rigid(axis, angle_rad, translation):
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    C = 1.0 - c
    R = np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T


def test_affine_transform_roundtrip(xreg_mod, tmp_path):
    T = _rigid([0.2, 1.0, -0.3], math.radians(17.0), [3.0, -5.0, 8.0])

    path = str(tmp_path / "pose.h5")
    xreg_mod.transforms.write_itk_affine_transform(path, T)
    T2 = xreg_mod.transforms.read_itk_affine_transform(path)

    assert T2.shape == (4, 4)
    np.testing.assert_allclose(T2, T, atol=1e-5)


def test_rot_ang_trans_mag(xreg_mod):
    angle = math.radians(30.0)
    trans = [1.0, 2.0, 2.0]  # magnitude 3
    T = _rigid([0.0, 0.0, 1.0], angle, trans)

    ang, mag = xreg_mod.transforms.rot_ang_trans_mag(T)
    assert ang == pytest.approx(angle, abs=1e-4)
    assert mag == pytest.approx(3.0, abs=1e-4)


def test_se3_exp_produces_valid_rigid(xreg_mod):
    x = np.array([0.05, -0.1, 0.2, 3.0, -2.0, 1.0], dtype=np.float32)
    T = xreg_mod.transforms.se3_exp(x)
    assert T.shape == (4, 4)

    # Rotation block is orthonormal and the bottom row is [0, 0, 0, 1].
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-4)
    np.testing.assert_allclose(T[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-5)

    # se3_inv is a true inverse.
    Tinv = xreg_mod.transforms.se3_inv(T)
    np.testing.assert_allclose(Tinv @ T, np.eye(4), atol=1e-4)


def test_se3_log_is_lie_algebra_element(xreg_mod):
    x = np.array([0.05, -0.1, 0.2, 3.0, -2.0, 1.0], dtype=np.float32)
    T = xreg_mod.transforms.se3_exp(x)
    logT = xreg_mod.transforms.se3_log(T)

    # The upper-left 3x3 block of an se(3) element is skew-symmetric.
    W = logT[:3, :3]
    np.testing.assert_allclose(W, -W.T, atol=1e-4)


def test_frame_diff_zero_for_equal(xreg_mod):
    T = _rigid([1.0, 0.0, 0.0], math.radians(10.0), [1.0, 1.0, 1.0])
    ang, mag = xreg_mod.transforms.frame_diff_rot_ang_trans_mag(T, T)
    assert ang == pytest.approx(0.0, abs=1e-5)
    assert mag == pytest.approx(0.0, abs=1e-5)
