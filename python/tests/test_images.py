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

"""Images: numpy, SimpleITK, and ITK Python interoperability."""

import numpy as np
import pytest

import xreg


@pytest.mark.parametrize('cls,shape,dtype', [(xreg.Image3DF32, (4, 5, 6), np.float32),
                                             (xreg.Image3DU8, (4, 5, 6), np.uint8),
                                             (xreg.Image2DF32, (5, 6), np.float32),
                                             (xreg.Image2DU8, (5, 6), np.uint8)])
def test_numpy_round_trip(cls, shape, dtype):
    rng = np.random.default_rng(0)

    arr = (rng.random(shape) * 100).astype(dtype)

    nd = len(shape)

    spacing = np.arange(1, nd + 1) * 0.5
    origin = np.arange(nd) * -10.0
    direction = np.eye(nd)[::-1].copy()

    img = xreg.from_numpy(arr, spacing, origin, direction)

    assert isinstance(img, cls)
    assert img.shape == shape
    assert img.size == shape[::-1]
    assert img.ndim == nd
    assert np.dtype(img.dtype) == dtype

    assert np.array_equal(img.numpy(), arr)
    assert img.numpy().dtype == dtype

    assert np.allclose(img.spacing, spacing)
    assert np.allclose(img.origin, origin)
    assert np.allclose(img.direction, direction)

    img_copy = img.copy()
    img_copy.spacing = np.ones(nd) * 3
    assert np.allclose(img.spacing, spacing)
    assert np.allclose(img_copy.spacing, 3)

    assert img == img
    assert img != img_copy


def test_from_numpy_copies_and_numpy_is_a_view():
    arr = np.zeros((2, 3, 4), np.float32)

    img = xreg.from_numpy(arr)

    # the input is copied
    arr[0, 0, 0] = 5
    assert img.numpy()[0, 0, 0] == 0

    # numpy() and np.asarray() share memory with the image
    view = img.numpy()
    view[1, 2, 3] = 7
    assert np.asarray(img)[1, 2, 3] == 7
    assert np.shares_memory(view, np.asarray(img))

    # np.array() copies
    cpy = np.array(img)
    cpy[1, 2, 3] = 1
    assert img.numpy()[1, 2, 3] == 7

    assert np.asarray(img, dtype=np.float64).dtype == np.float64


def test_view_keeps_image_alive():
    view = xreg.from_numpy(np.full((8, 8, 8), 3, np.float32)).numpy()

    # the image object is gone, but the pixel buffer must remain valid
    import gc
    gc.collect()

    assert np.all(view == 3)


def test_from_numpy_converts_dtype():
    arr = np.arange(24, dtype=np.int16).reshape(2, 3, 4)

    img = xreg.from_numpy(arr)
    assert isinstance(img, xreg.Image3DF32)
    assert np.array_equal(img.numpy(), arr.astype(np.float32))

    lbl = xreg.from_numpy(arr, dtype=np.uint8)
    assert isinstance(lbl, xreg.Image3DU8)
    assert np.array_equal(lbl.numpy(), arr)

    assert isinstance(xreg.from_numpy(arr > 3), xreg.Image3DU8)

    # labels are never silently wrapped or truncated
    with pytest.raises(ValueError):
        xreg.from_numpy(arr + 300, dtype=np.uint8)

    with pytest.raises(ValueError):
        xreg.from_numpy(arr * 0.5, dtype=np.uint8)

    with pytest.raises(ValueError):
        xreg.from_numpy(np.zeros((2, 2, 2, 2)))

    with pytest.raises(ValueError):
        xreg.from_numpy(np.zeros((2, 2, 2)), spacing=(1, 0, 1))


def test_index_order_and_physical_points():
    # a single bright voxel at numpy index (z=1, y=2, x=3) -> ITK index (3, 2, 1)
    arr = np.zeros((4, 5, 6), np.float32)
    arr[1, 2, 3] = 1

    spacing = (0.5, 0.75, 2.0)
    origin = (-1.0, 2.0, 3.0)
    direction = np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]])

    img = xreg.from_numpy(arr, spacing, origin, direction)

    expected = np.asarray(origin) + direction @ (np.asarray(spacing) * [3, 2, 1])

    assert np.allclose(img.index_to_physical_point([3, 2, 1]), expected)
    assert np.allclose(img.physical_point_to_index(expected), [3, 2, 1])

    # center of the volume in physical coordinates
    c = xreg.vol_center_as_phys_pt(img)
    assert np.allclose(c, img.index_to_physical_point(np.array(img.size) / 2.0), atol=1e-5)


def make_sitk_image():
    sitk = pytest.importorskip('SimpleITK')

    arr = np.random.default_rng(1).random((4, 5, 6)).astype(np.float32)

    src = sitk.GetImageFromArray(arr)
    src.SetSpacing((0.5, 0.75, 2.0))
    src.SetOrigin((-1.0, 2.0, 3.0))
    src.SetDirection((0, 1, 0, 1, 0, 0, 0, 0, 1))

    return sitk, arr, src


def test_sitk_round_trip():
    sitk, arr, src = make_sitk_image()

    img = xreg.from_sitk(src)

    assert isinstance(img, xreg.Image3DF32)
    assert img.size == src.GetSize()
    assert np.allclose(img.spacing, src.GetSpacing())
    assert np.allclose(img.origin, src.GetOrigin())
    assert np.allclose(np.asarray(img.direction).flatten(), src.GetDirection())

    # both libraries agree on where voxels are located in space
    for idx in [(0, 0, 0), (5, 4, 3), (2, 1, 3)]:
        assert np.allclose(img.index_to_physical_point(idx), src.TransformIndexToPhysicalPoint(idx))
        assert img.numpy()[idx[::-1]] == src.GetPixel(*idx)

    dst = xreg.to_sitk(img)

    assert np.array_equal(sitk.GetArrayFromImage(dst), arr)
    assert np.allclose(dst.GetSpacing(), src.GetSpacing())
    assert np.allclose(dst.GetOrigin(), src.GetOrigin())
    assert np.allclose(dst.GetDirection(), src.GetDirection())

    # integer label maps
    lbl = xreg.from_sitk(sitk.Cast(src > 0.5, sitk.sitkUInt16), dtype=np.uint8)
    assert isinstance(lbl, xreg.Image3DU8)
    assert np.array_equal(lbl.numpy(), (arr > 0.5).astype(np.uint8))

    # 2D images
    img_2d = xreg.from_sitk(src[:, :, 0])
    assert isinstance(img_2d, xreg.Image2DF32)
    assert np.array_equal(img_2d.numpy(), arr[0])


def test_sitk_implicit_conversion():
    sitk, arr, src = make_sitk_image()

    # a SimpleITK image may be passed directly where an xreg image is expected
    assert np.allclose(xreg.vol_center_as_phys_pt(src),
                       xreg.vol_center_as_phys_pt(xreg.from_sitk(src)))

    # int16 CT-like volume is cast to float32
    ct = sitk.Cast(src * 1000 - 500, sitk.sitkInt16)

    att = xreg.hu_to_lin_att(ct)
    assert isinstance(att, xreg.Image3DF32)
    assert att.size == ct.GetSize()
    assert np.allclose(att.spacing, ct.GetSpacing())

    # explicit construction
    assert np.array_equal(xreg.Image3DF32(src).numpy(), arr)

    # lists of images, and conversion to the label type
    lbl = sitk.Cast(src > 0.5, sitk.sitkUInt8)

    vols = xreg.make_vol_list_from_vol_and_labels(src, lbl, [1])
    assert len(vols) == 1
    assert isinstance(vols[0], xreg.Image3DF32)

    ml = xreg.MultiLevelMultiObjRegi()
    ml.vols = [src, xreg.from_sitk(src)]
    assert all(isinstance(v, xreg.Image3DF32) for v in ml.vols)

    # dimension mismatch
    with pytest.raises(TypeError):
        xreg.vol_center_as_phys_pt(src[:, :, 0])

    # labels outside of the uint8 range are rejected
    with pytest.raises((TypeError, ValueError)):
        xreg.make_vol_list_from_vol_and_labels(src, sitk.Cast(src * 1000, sitk.sitkInt16), [1])

    # arbitrary objects are not converted
    with pytest.raises(TypeError):
        xreg.vol_center_as_phys_pt(arr)


def test_itk_round_trip():
    itk = pytest.importorskip('itk')

    arr = np.random.default_rng(2).random((4, 5, 6)).astype(np.float32)

    src = itk.image_from_array(arr)
    src.SetSpacing((0.5, 0.75, 2.0))
    src.SetOrigin((-1.0, 2.0, 3.0))
    src.SetDirection(itk.matrix_from_array(np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, 1]])))

    img = xreg.from_itk(src)

    assert isinstance(img, xreg.Image3DF32)
    assert img.size == tuple(src.GetLargestPossibleRegion().GetSize())
    assert np.allclose(img.spacing, np.asarray(src.GetSpacing()))
    assert np.allclose(img.origin, np.asarray(src.GetOrigin()))
    assert np.allclose(img.direction, itk.array_from_matrix(src.GetDirection()))
    assert np.array_equal(img.numpy(), arr)

    for idx in [(0, 0, 0), (5, 4, 3), (2, 1, 3)]:
        assert np.allclose(img.index_to_physical_point(idx),
                           np.asarray(src.TransformIndexToPhysicalPoint(idx)))

    dst = xreg.to_itk(img)

    assert np.array_equal(itk.array_from_image(dst), arr)
    assert np.allclose(np.asarray(dst.GetSpacing()), np.asarray(src.GetSpacing()))
    assert np.allclose(np.asarray(dst.GetOrigin()), np.asarray(src.GetOrigin()))
    assert np.allclose(itk.array_from_matrix(dst.GetDirection()), itk.array_from_matrix(src.GetDirection()))

    # implicit conversion
    assert np.allclose(xreg.vol_center_as_phys_pt(src), xreg.vol_center_as_phys_pt(img))


def test_hu_to_lin_att():
    vol = xreg.from_numpy(np.zeros((10, 20, 30), dtype=np.float32), spacing=(1, 2, 3))

    assert np.allclose(xreg.vol_center_as_phys_pt(vol), [15, 20, 15])

    att = xreg.hu_to_lin_att(vol)
    assert att.shape == vol.shape
    assert np.all(att.numpy() > 0)

    # the input is not modified
    assert np.all(vol.numpy() == 0)

    with pytest.raises(ValueError):
        xreg.hu_to_lin_att(None)


def test_make_vol_list_from_vol_and_labels():
    arr = np.arange(8 * 8 * 8, dtype=np.float32).reshape(8, 8, 8) + 1

    lbl = np.zeros((8, 8, 8), np.uint8)
    lbl[1:3, 2:5, 3:7] = 1
    lbl[5:8, 0:2, 0:2] = 2

    vols = xreg.make_vol_list_from_vol_and_labels(xreg.from_numpy(arr), xreg.from_numpy(lbl), [1, 2], 0)

    assert len(vols) == 2

    # each volume is tightly cropped about its label
    assert vols[0].shape == (2, 3, 4)
    assert vols[1].shape == (3, 2, 2)

    assert np.array_equal(vols[0].numpy(), arr[1:3, 2:5, 3:7])
    assert np.allclose(vols[0].origin, [3, 2, 1])


def test_image_file_io(tmp_path):
    arr = np.random.default_rng(3).random((4, 5, 6)).astype(np.float32)

    img = xreg.from_numpy(arr, spacing=(0.5, 0.75, 2.0), origin=(1, 2, 3))

    path = str(tmp_path / 'vol.nii.gz')

    xreg.write_image(img, path)

    img2 = xreg.read_image(path)

    assert isinstance(img2, xreg.Image3DF32)
    assert np.array_equal(img2.numpy(), arr)
    assert np.allclose(img2.spacing, img.spacing)
    assert np.allclose(img2.origin, img.origin)

    lbl = xreg.from_numpy((arr > 0.5).astype(np.uint8))

    lbl_path = str(tmp_path / 'lbl.nii.gz')
    xreg.write_image(lbl, lbl_path)

    lbl2 = xreg.read_image(lbl_path, dtype='uint8')
    assert isinstance(lbl2, xreg.Image3DU8)
    assert np.array_equal(lbl2.numpy(), lbl.numpy())

    with pytest.raises(ValueError):
        xreg.read_image(path, dtype='int16')

    with pytest.raises(RuntimeError):
        xreg.read_image(str(tmp_path / 'does_not_exist.nii.gz'))
