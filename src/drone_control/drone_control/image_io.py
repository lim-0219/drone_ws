"""
sensor_msgs/Image <-> PIL conversions without OpenCV or numpy.

python3-opencv on this machine is built against numpy 1.x and fails to import
next to the numpy 2.x in ~/.local, and cv_bridge drags it in, so the ROS nodes
never touch either of them.  The stitcher has its own virtualenv for OpenCV.
"""

from __future__ import annotations

from typing import Optional

from builtin_interfaces.msg import Time
from PIL import Image as PilImage
from sensor_msgs.msg import Image

# How the PIL "raw" decoder should read each ROS image encoding:
# (PIL mode, raw mode, bytes per pixel).
_IMAGE_ENCODINGS = {
    'rgb8': ('RGB', 'RGB', 3),
    'bgr8': ('RGB', 'BGR', 3),
    'rgba8': ('RGBA', 'RGBA', 4),
    'bgra8': ('RGBA', 'BGRA', 4),
    'mono8': ('L', 'L', 1),
}


def image_to_pil(image: Image) -> PilImage.Image:
    """Convert a ``sensor_msgs/Image`` to a PIL image (row stride honoured)."""
    try:
        mode, raw_mode, bytes_per_pixel = _IMAGE_ENCODINGS[image.encoding.lower()]
    except KeyError:
        raise ValueError(f'unsupported image encoding "{image.encoding}"') from None
    stride = int(image.step) or image.width * bytes_per_pixel
    return PilImage.frombytes(
        mode, (image.width, image.height), bytes(image.data), 'raw', raw_mode, stride, 1)


def pil_to_image(pil: PilImage.Image, frame_id: str = '', stamp: Optional[Time] = None) -> Image:
    """Convert a PIL image to an rgb8 ``sensor_msgs/Image``."""
    rgb = pil.convert('RGB')
    msg = Image()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width, msg.height = rgb.size
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = rgb.width * 3
    msg.data = rgb.tobytes()
    return msg


def png_to_image(path: str, frame_id: str = '', stamp: Optional[Time] = None) -> Image:
    """Load an image file from disk as an rgb8 ``sensor_msgs/Image``."""
    with PilImage.open(path) as pil:
        return pil_to_image(pil, frame_id, stamp)
