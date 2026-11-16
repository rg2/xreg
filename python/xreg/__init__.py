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

"""Python bindings for xReg: 2D/3D registration and related utilities.

The primary interface is MultiLevelMultiObjRegi, which mirrors the C++
xreg::MultiLevelMultiObjRegi struct. See python/README.md for an overview.
"""

from ._xreg import *  # noqa: F401,F403
from ._xreg import __version__, DEG2RAD, RAD2DEG, RAY_CAST_MAX_DEPTH  # noqa: F401

from ._image_interop import (  # noqa: F401
    from_numpy, from_sitk, to_sitk, from_itk, to_itk, to_xreg_image)

# Shorter aliases for the nested classes of MultiLevelMultiObjRegi
Level                        = MultiLevelMultiObjRegi.Level  # noqa: F405
SingleRegi                   = MultiLevelMultiObjRegi.Level.SingleRegi  # noqa: F405
LevelList                    = MultiLevelMultiObjRegi.LevelList  # noqa: F405
SingleRegiList               = MultiLevelMultiObjRegi.Level.SingleRegiList  # noqa: F405
RefFrame                     = MultiLevelMultiObjRegi.RefFrame  # noqa: F405
StaticRefFrame               = MultiLevelMultiObjRegi.StaticRefFrame  # noqa: F405
CamAlignRefFrameWithCurPose  = MultiLevelMultiObjRegi.CamAlignRefFrameWithCurPose  # noqa: F405
DynVolRefFrame               = MultiLevelMultiObjRegi.DynVolRefFrame  # noqa: F405
InitPose                     = SingleRegi.InitPose
InitPoseId                   = SingleRegi.InitPoseId
InitPosePrevPoseEst          = SingleRegi.InitPosePrevPoseEst
InitPosePrevPoseEstTransOnly = SingleRegi.InitPosePrevPoseEstTransOnly
InitPosePrevPoseEstRotOnly   = SingleRegi.InitPosePrevPoseEstRotOnly
VolPathInfo                  = DebugRegiResultsMultiLevel.VolPathInfo  # noqa: F405
ProjDataPathInfo             = DebugRegiResultsMultiLevel.ProjDataPathInfo  # noqa: F405
