# MIT License
#
# Copyright (c) 2022 Robert Grupp
#
# See the repository LICENSE file for the full MIT License text.

"""Tests for xreg.landmarks (FCSV / Slicer Markups reading)."""

import numpy as np
import pytest

FCSV = """# Markups fiducial file version = 4.11
# CoordinateSystem = RAS
# columns = id,x,y,z,ow,ox,oy,oz,vis,sel,lock,label,desc,associatedNodeID
vtkMRMLMarkupsFiducialNode_0,10.0,20.0,30.0,0,0,0,1,1,1,0,P1,,
vtkMRMLMarkupsFiducialNode_1,-5.0,15.0,25.0,0,0,0,1,1,1,0,P2,,
"""


@pytest.fixture()
def fcsv_path(tmp_path):
    p = tmp_path / "lands.fcsv"
    p.write_text(FCSV)
    return str(p)


def test_is_supported(xreg_mod, fcsv_path):
    assert xreg_mod.landmarks.is_supported_pts_file(fcsv_path)
    assert xreg_mod.landmarks.is_supported_name_pt_map_file(fcsv_path)


def test_read_pts_count(xreg_mod, fcsv_path):
    pts = xreg_mod.landmarks.read_pts(fcsv_path)
    assert len(pts) == 2
    assert all(np.asarray(p).shape == (3,) for p in pts)


def test_lps_vs_ras_sign_flip(xreg_mod, fcsv_path):
    # FCSV stores points in RAS; converting to LPS negates the x and y axes and
    # leaves z unchanged. Assert the relationship between the two reads directly
    # so the test does not depend on point ordering conventions.
    pm_ras = xreg_mod.landmarks.read_name_pt_map(fcsv_path, output_in_lps=False)
    pm_lps = xreg_mod.landmarks.read_name_pt_map(fcsv_path, output_in_lps=True)

    assert set(pm_ras.keys()) == {"P1", "P2"}
    assert set(pm_lps.keys()) == {"P1", "P2"}

    for name in pm_ras:
        ras = np.asarray(pm_ras[name])
        lps = np.asarray(pm_lps[name])
        np.testing.assert_allclose(lps[0], -ras[0], atol=1e-5)
        np.testing.assert_allclose(lps[1], -ras[1], atol=1e-5)
        np.testing.assert_allclose(lps[2], ras[2], atol=1e-5)
