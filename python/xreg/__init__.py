# MIT License
#
# Copyright (c) 2022 Robert Grupp
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

"""Pythonic front-end for the compiled ``pyxreg`` extension module.

This package re-exports the compiled bindings so that callers can write
``import xreg`` and reach the feature submodules directly::

    import xreg
    vol = xreg.image_io.read_volume("ct.nii.gz")
    T   = xreg.transforms.read_itk_affine_transform("pose.h5")

The compiled extension (``pyxreg``) must be importable (on ``PYTHONPATH`` or
installed alongside this package). Build it with ``-DXREG_BUILD_PYTHON=ON``.
"""

try:
    import pyxreg as _pyxreg
except ImportError as exc:  # pragma: no cover - environment dependent
    raise ImportError(
        "The compiled 'pyxreg' extension module could not be imported. "
        "Build xreg with -DXREG_BUILD_PYTHON=ON and ensure the resulting "
        "module is on PYTHONPATH."
    ) from exc

# Value types
Volume = _pyxreg.Volume
Mesh = _pyxreg.Mesh

# Feature submodules
transforms = _pyxreg.transforms
landmarks = _pyxreg.landmarks
image_io = _pyxreg.image_io
mesh = _pyxreg.mesh
proj_data = _pyxreg.proj_data

__all__ = [
    "Volume",
    "Mesh",
    "transforms",
    "landmarks",
    "image_io",
    "mesh",
    "proj_data",
]
