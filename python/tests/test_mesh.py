# MIT License
#
# Copyright (c) 2022 Robert Grupp
#
# See the repository LICENSE file for the full MIT License text.

"""Round-trip tests for xreg.mesh (surface mesh read/write)."""

import numpy as np
import pytest


def _tetrahedron():
    vertices = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    faces = np.array(
        [
            [0, 1, 2],
            [0, 1, 3],
            [0, 2, 3],
            [1, 2, 3],
        ],
        dtype=np.uint64,
    )
    return vertices, faces


@pytest.mark.parametrize("ext", [".ply", ".stl", ".obj"])
def test_mesh_roundtrip(xreg_mod, tmp_path, ext):
    vertices, faces = _tetrahedron()

    m = xreg_mod.Mesh()
    m.vertices = vertices
    m.faces = faces

    path = str(tmp_path / ("mesh" + ext))
    xreg_mod.mesh.write_mesh(m, path)
    m2 = xreg_mod.mesh.read_mesh(path)

    assert m2.vertices.shape == (4, 3)
    assert m2.faces.shape == (4, 3)
    # Vertex/face counts must be preserved; exact coordinates for well-known
    # formats (PLY) should match closely.
    if ext == ".ply":
        np.testing.assert_allclose(np.sort(m2.vertices, axis=0),
                                   np.sort(vertices, axis=0), atol=1e-5)
