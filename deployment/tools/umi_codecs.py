"""Register the image codecs used by UMI zarr datasets.

The UMI SLAM pipeline compresses RGB arrays with the
``imagecodecs_jpegxl`` numcodecs codec from ``diffusion_policy``.  zarr and
numcodecs do not ship that codec, so this module registers a compatible
reader/writer when ``imagecodecs`` is available.
"""

from __future__ import annotations

import numpy as np

_REGISTERED = False


def _squeeze_image_buffer(buffer: np.ndarray) -> np.ndarray:
    array = np.asarray(buffer)
    if array.ndim > 3:
        return array.reshape((-1,) + array.shape[-3:])
    return array


def _register(codec_class) -> None:
    import numcodecs
    from numcodecs.registry import register_codec

    registry = getattr(numcodecs.registry, "codec_registry", {})
    if codec_class.codec_id not in registry:
        register_codec(codec_class)


def register_umi_codecs() -> None:
    """Register the UMI image codecs, once per process."""
    global _REGISTERED
    if _REGISTERED:
        return

    try:
        from numcodecs.abc import Codec
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise RuntimeError(
            "UMI zarr support requires zarr and numcodecs. "
            "Install them in the converter environment."
        ) from exc

    try:
        import imagecodecs
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise RuntimeError(
            "UMI datasets commonly use the imagecodecs_jpegxl zarr codec. "
            "Install imagecodecs in the converter environment."
        ) from exc

    class JpegXl(Codec):
        codec_id = "imagecodecs_jpegxl"

        def __init__(
            self,
            level=None,
            effort=None,
            distance=None,
            lossless=None,
            decodingspeed=None,
            photometric=None,
            planar=None,
            usecontainer=None,
            index=None,
            keeporientation=None,
            numthreads=None,
            **kwargs,
        ):
            self.level = level
            self.effort = effort
            self.distance = distance
            self.lossless = lossless
            self.decodingspeed = decodingspeed
            self.photometric = photometric
            self.planar = planar
            self.usecontainer = usecontainer
            self.index = index
            self.keeporientation = keeporientation
            self.numthreads = numthreads
            self.extra = kwargs

        def encode(self, buffer):
            if not getattr(imagecodecs, "JPEGXL", False):
                raise RuntimeError(
                    "This imagecodecs build has no JPEG XL support; "
                    "cannot decode a UMI imagecodecs_jpegxl array."
                )
            return imagecodecs.jpegxl_encode(
                _squeeze_image_buffer(buffer),
                level=self.level,
                effort=self.effort,
                distance=self.distance,
                lossless=self.lossless,
                decodingspeed=self.decodingspeed,
                photometric=self.photometric,
                planar=self.planar,
                usecontainer=self.usecontainer,
                numthreads=self.numthreads,
            )

        def decode(self, buffer, out=None):
            return imagecodecs.jpegxl_decode(
                buffer,
                index=self.index,
                keeporientation=self.keeporientation,
                numthreads=self.numthreads,
                out=out,
            )

        def get_config(self):
            return {
                "id": self.codec_id,
                "level": self.level,
                "effort": self.effort,
                "distance": self.distance,
                "lossless": self.lossless,
                "decodingspeed": self.decodingspeed,
                "photometric": self.photometric,
                "planar": self.planar,
                "usecontainer": self.usecontainer,
                "index": self.index,
                "keeporientation": self.keeporientation,
                "numthreads": self.numthreads,
            }

    class Jpeg2k(Codec):
        codec_id = "imagecodecs_jpeg2k"

        def __init__(self, level=None, **kwargs):
            self.level = level
            self.extra = kwargs

        def encode(self, buffer):
            return imagecodecs.jpeg2k_encode(
                _squeeze_image_buffer(buffer), level=self.level)

        def decode(self, buffer, out=None):
            return imagecodecs.jpeg2k_decode(buffer, out=out)

        def get_config(self):
            return {"id": self.codec_id, "level": self.level}

    class Jpeg2kLossless(Jpeg2k):
        pass

    Jpeg2kLossless.codec_id = "imagecodecs_jpeg2k_lossless"

    for codec_class in (JpegXl, Jpeg2k, Jpeg2kLossless):
        _register(codec_class)

    _REGISTERED = True
