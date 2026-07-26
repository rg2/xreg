# MIT License
#
# Copyright (c) 2022 Robert Grupp
#
# See the repository LICENSE file for the full MIT License text.

"""Shared pytest fixtures for the pyxreg binding tests.

The tests are Python-only round trips by default (they do not require the CLI
executables to be built). If ``XREG_BIN_DIR`` points at a directory containing
the built ``xreg-*`` executables, the cross-check tests that compare the Python
API against the CLI tools will additionally run.
"""

import os
import shutil

import pytest

pyxreg = pytest.importorskip(
    "pyxreg",
    reason="the compiled pyxreg extension module is not importable; "
    "build with -DXREG_BUILD_PYTHON=ON and put it on PYTHONPATH",
)


@pytest.fixture(scope="session")
def xreg_mod():
    return pyxreg


@pytest.fixture(scope="session")
def xreg_bin_dir():
    """Directory holding the built xreg-* executables, or None."""
    return os.environ.get("XREG_BIN_DIR")


def xreg_exe(bin_dir, name):
    """Resolve an xreg-* executable path, or None if unavailable."""
    if not bin_dir:
        return None
    candidate = os.path.join(bin_dir, name)
    if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
        return candidate
    return shutil.which(name)
