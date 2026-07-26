# MIT License
#
# Copyright (c) 2022 Robert Grupp
#
# See the repository LICENSE file for the full MIT License text.

"""Round-trip tests for xreg.image_io (3D volume read/write)."""

import numpy as np
import pytest


@pytest.mark.parametrize("ext", [".nii.gz", ".mha", ".nrrd"])
def test_volume_roundtrip(xreg_mod, tmp_path, ext):
    rng = np.random.default_rng(0)
    # shape is (nz, ny, nx)
    pixels = rng.standard_normal((5, 7, 9)).astype(np.float32)

    vol = xreg_mod.Volume()
    vol.pixels = pixels
    vol.spacing = np.array([0.5, 0.75, 1.25], dtype=np.float64)
    vol.origin = np.array([-10.0, 20.0, 3.0], dtype=np.float64)
    vol.direction = np.eye(3, dtype=np.float64)

    path = str(tmp_path / ("vol" + ext))
    xreg_mod.image_io.write_volume(vol, path)
    vol2 = xreg_mod.image_io.read_volume(path)

    assert vol2.shape == (5, 7, 9)
    np.testing.assert_allclose(vol2.pixels, pixels, atol=1e-5)
    np.testing.assert_allclose(vol2.spacing, vol.spacing, atol=1e-6)
    np.testing.assert_allclose(vol2.origin, vol.origin, atol=1e-6)
    np.testing.assert_allclose(vol2.direction, vol.direction, atol=1e-6)


def test_volume_shape_orientation(xreg_mod, tmp_path):
    # A distinctive value at (z=1, y=2, x=3) must survive a round trip in place,
    # verifying axis ordering is preserved (not transposed).
    pixels = np.zeros((4, 5, 6), dtype=np.float32)
    pixels[1, 2, 3] = 42.0

    vol = xreg_mod.Volume()
    vol.pixels = pixels
    vol.spacing = np.ones(3, dtype=np.float64)
    vol.origin = np.zeros(3, dtype=np.float64)
    vol.direction = np.eye(3, dtype=np.float64)

    path = str(tmp_path / "vol.nii.gz")
    xreg_mod.image_io.write_volume(vol, path)
    vol2 = xreg_mod.image_io.read_volume(path)

    assert vol2.pixels[1, 2, 3] == pytest.approx(42.0)
    assert np.count_nonzero(vol2.pixels) == 1
