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

"""Conversion of images between xreg and numpy, SimpleITK, and ITK Python.

xreg images are converted by copying pixel data through numpy arrays along
with the spacing, origin, and direction metadata. No binary compatibility
between xreg's ITK and the ITK used by SimpleITK or ITK Python is required,
and neither package is imported unless one of its images is converted.

Numpy arrays use the SimpleITK/ITK Python index ordering, e.g. an array of
shape (nz, ny, nx) corresponds to an image of size (nx, ny, nz). The spacing,
origin, and direction use ITK ordering (x first).
"""

import numpy as np

from . import _xreg

_IMAGE_TYPES = {
    (3, np.dtype(np.float32)): _xreg.Image3DF32,
    (3, np.dtype(np.uint8)):   _xreg.Image3DU8,
    (2, np.dtype(np.float32)): _xreg.Image2DF32,
    (2, np.dtype(np.uint8)):   _xreg.Image2DU8,
}

_XREG_IMAGE_CLASSES = tuple(_IMAGE_TYPES.values())


def _xreg_image_type(ndim, dtype):
    dtype = np.dtype(dtype)

    if ndim not in (2, 3):
        raise ValueError('only 2D and 3D images are supported, got {}D'.format(ndim))

    if dtype not in (np.dtype(np.float32), np.dtype(np.uint8)):
        raise ValueError('unsupported pixel type: {} (supported: float32, uint8)'.format(dtype))

    return _IMAGE_TYPES[(ndim, dtype)]


def _cast_pixels(arr, dtype):
    """Returns a C-contiguous array of the requested dtype (float32 or uint8).

    Conversions to uint8 are checked, so that label values are never silently
    wrapped or truncated.
    """
    arr = np.asarray(arr)
    dtype = np.dtype(dtype)

    if (dtype == np.uint8) and (arr.dtype != np.uint8) and (arr.dtype != np.bool_):
        if arr.size and ((arr.min() < 0) or (arr.max() > 255)):
            raise ValueError('pixel values are outside of the range of uint8')

        if (arr.dtype.kind == 'f') and not np.array_equal(arr, np.rint(arr)):
            raise ValueError('non-integer pixel values may not be converted to uint8')

    return np.ascontiguousarray(arr, dtype=dtype)


def from_numpy(arr, spacing=None, origin=None, direction=None, dtype=None):
    """Creates an xreg image from a 2D or 3D numpy array (numpy/SimpleITK index order).

    Pixel data is copied. The spacing, origin, and direction are specified in
    ITK order (x first) and default to unit spacing, zero origin, and identity
    direction.

    dtype: pixel type of the xreg image (np.float32 or np.uint8). When None,
    uint8 and bool arrays become uint8 and all other arrays become float32.
    """
    arr = np.asarray(arr)

    if dtype is None:
        dtype = np.uint8 if arr.dtype in (np.uint8, np.bool_) else np.float32

    img_type = _xreg_image_type(arr.ndim, dtype)

    nd = arr.ndim

    if spacing is not None:
        spacing = np.asarray(spacing, dtype=np.float64).reshape(nd)
    if origin is not None:
        origin = np.asarray(origin, dtype=np.float64).reshape(nd)
    if direction is not None:
        direction = np.asarray(direction, dtype=np.float64).reshape(nd, nd)

    return img_type.from_numpy(_cast_pixels(arr, dtype), spacing, origin, direction)


def _sitk_array_and_geom(img):
    import SimpleITK as sitk

    if img.GetNumberOfComponentsPerPixel() != 1:
        raise ValueError('only scalar images are supported')

    nd = img.GetDimension()

    return (sitk.GetArrayViewFromImage(img),
            np.asarray(img.GetSpacing(), dtype=np.float64),
            np.asarray(img.GetOrigin(), dtype=np.float64),
            np.asarray(img.GetDirection(), dtype=np.float64).reshape(nd, nd))


def _itk_array_and_geom(img):
    import itk

    if img.GetNumberOfComponentsPerPixel() != 1:
        raise ValueError('only scalar images are supported')

    return (itk.array_view_from_image(img),
            np.asarray(img.GetSpacing(), dtype=np.float64),
            np.asarray(img.GetOrigin(), dtype=np.float64),
            np.asarray(itk.array_from_matrix(img.GetDirection()), dtype=np.float64))


def _foreign_array_and_geom(img):
    mod = type(img).__module__

    if mod.startswith('SimpleITK'):
        return _sitk_array_and_geom(img)
    elif mod.startswith('itk'):
        return _itk_array_and_geom(img)

    raise TypeError('unsupported image type: {}'.format(type(img)))


def _foreign_to_array_and_geom(img, ndim, dtype):
    """Used by the extension module to implicitly convert SimpleITK/ITK images."""
    arr, spacing, origin, direction = _foreign_array_and_geom(img)

    if arr.ndim != ndim:
        raise ValueError('expected a {}D image, got a {}D image'.format(ndim, arr.ndim))

    return (_cast_pixels(arr, dtype), spacing, origin, direction)


def from_sitk(img, dtype=None):
    """Creates an xreg image from a SimpleITK image (pixel data is copied).

    dtype: pixel type of the xreg image (np.float32 or np.uint8). When None,
    uint8 images remain uint8 and all other images are converted to float32
    (e.g. use dtype=np.uint8 for label maps stored with other integer types).
    """
    arr, spacing, origin, direction = _sitk_array_and_geom(img)

    return from_numpy(arr, spacing, origin, direction, dtype=dtype)


def from_itk(img, dtype=None):
    """Creates an xreg image from an ITK Python image (pixel data is copied).

    dtype: see from_sitk().
    """
    arr, spacing, origin, direction = _itk_array_and_geom(img)

    return from_numpy(arr, spacing, origin, direction, dtype=dtype)


def to_sitk(img):
    """Creates a SimpleITK image from an xreg image (pixel data is copied)."""
    import SimpleITK as sitk

    if not isinstance(img, _XREG_IMAGE_CLASSES):
        raise TypeError('expected an xreg image, got {}'.format(type(img)))

    out = sitk.GetImageFromArray(img.numpy())
    out.SetSpacing([float(s) for s in img.spacing])
    out.SetOrigin([float(o) for o in img.origin])
    out.SetDirection([float(d) for d in np.asarray(img.direction).flatten()])

    return out


def to_itk(img):
    """Creates an ITK Python image from an xreg image (pixel data is copied)."""
    import itk

    if not isinstance(img, _XREG_IMAGE_CLASSES):
        raise TypeError('expected an xreg image, got {}'.format(type(img)))

    out = itk.image_from_array(np.array(img))
    out.SetSpacing([float(s) for s in img.spacing])
    out.SetOrigin([float(o) for o in img.origin])
    out.SetDirection(itk.matrix_from_array(np.ascontiguousarray(img.direction, dtype=np.float64)))

    return out


def to_xreg_image(img, dtype=None):
    """Converts a SimpleITK image, ITK Python image, or xreg image to an xreg image.

    xreg images are returned as-is (not copied) when no dtype conversion is required.
    """
    if isinstance(img, _XREG_IMAGE_CLASSES):
        if (dtype is None) or (np.dtype(dtype) == np.dtype(img.dtype)):
            return img

        return from_numpy(img.numpy(), img.spacing, img.origin, img.direction, dtype=dtype)

    arr, spacing, origin, direction = _foreign_array_and_geom(img)

    return from_numpy(arr, spacing, origin, direction, dtype=dtype)
