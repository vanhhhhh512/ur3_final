"""RGB segmentation and ray-plane geometry, independent of Gazebo state."""
import collections
import dataclasses as dc
import cv2
import numpy as np
from .contracts import Observation, Rejected, configuration


@dc.dataclass(frozen=True)
class Blob:
    name: str
    pixel: tuple
    xy: tuple
    area: float
    confidence: float


class PixelGeometry:
    def __init__(self, calibration, detector_settings):
        info = calibration['image']
        self.resolution = (info['width_px'], info['height_px'])
        self.frame = info['frame_id']
        self.intrinsics = np.array(info['k'], dtype=float)
        if self.intrinsics.shape != (9,) or not np.isfinite(self.intrinsics).all():
            raise Rejected('Invalid camera intrinsics')
        if any(info['distortion_coefficients']):
            raise Rejected('Rectified RGB is required for homography')
        pairs = calibration['calibration']
        pixels = {p['id']: (p['u'], p['v']) for p in pairs['image_points_px']}
        real = {p['id']: (p['x'], p['y']) for p in pairs['world_points_xy_m']}
        if len(pixels) < 4 or pixels.keys() != real.keys():
            raise Rejected('At least four corresponding calibration landmarks are required')
        rows = []
        for name, (u, v) in pixels.items():
            x, y = real[name]
            rows.extend(([u, v, 1, 0, 0, 0, -x*u, -x*v, -x],
                         [0, 0, 0, u, v, 1, -y*u, -y*v, -y]))
        design = np.array(rows, dtype=float)
        if np.linalg.matrix_rank(design) < 8:
            raise Rejected('Degenerate calibration landmarks')
        self.projection = np.linalg.svd(design)[2][-1].reshape((3, 3))
        if abs(np.linalg.det(self.projection)) < 1e-15:
            raise Rejected('Singular camera mapping')
        self.hull = cv2.convexHull(np.array(list(pixels.values()), dtype=np.float32))
        self.camera_xy = np.array(detector_settings['camera_center_world_xyz_m'][:2])
        camera_z = detector_settings['camera_center_world_xyz_m'][2]
        self.ray_scale = (camera_z - detector_settings['cube_top_z_m']) / (camera_z - calibration['plane']['z_m'])
        if not 0 < self.ray_scale < 1:
            raise Rejected('Cube top must lie between camera and support plane')
        for name, pixel in pixels.items():
            if np.linalg.norm(self.plane_point(pixel) - real[name]) > 0.002:
                raise Rejected('Calibration landmark residual exceeds 2 mm')

    def plane_point(self, pixel):
        p = np.asarray(pixel, dtype=float)
        if not np.isfinite(p).all() or cv2.pointPolygonTest(self.hull, tuple(map(float, p)), True) < -0.001:
            raise Rejected('Pixel lies outside calibrated tabletop')
        ray = self.projection @ np.r_[p, 1]
        if abs(ray[2]) < 1e-12:
            raise Rejected('Camera mapping has no finite intersection')
        return ray[:2] / ray[2]

    def block_point(self, pixel):
        return tuple(self.camera_xy + self.ray_scale * (self.plane_point(pixel) - self.camera_xy))

    def validate_info(self, message):
        if (message.width, message.height) != self.resolution or message.header.frame_id != self.frame:
            raise Rejected('Live CameraInfo differs from calibrated sensor')
        if not np.allclose(message.k, self.intrinsics, atol=0.001, rtol=0) or any(abs(x) > 1e-9 for x in message.d):
            raise Rejected('Live camera intrinsics/distortion are incompatible')


class ColourTracker:
    def __init__(self, settings, geometry):
        self.spec = settings
        self.geometry = geometry
        self.kernel = np.ones((settings['blob']['morphology_kernel_px'],) * 2, dtype=np.uint8)

    def segment(self, rgb):
        width, height = self.geometry.resolution
        if rgb.shape != (height, width, 3) or rgb.dtype != np.uint8:
            raise Rejected('RGB frame shape or pixel encoding differs from calibration')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        found = {}
        limits = self.spec['blob']
        for name, description in self.spec['colors'].items():
            mask = np.zeros((height, width), dtype=np.uint8)
            for h0, h1, s0, s1, v0, v1 in description['hsv_ranges']:
                mask |= cv2.inRange(hsv, np.array([h0, s0, v0]), np.array([h1, s1, v1]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            candidates = []
            for boundary in contours:
                area = cv2.contourArea(boundary)
                _, (w, h), _ = cv2.minAreaRect(boundary)
                if not limits['min_area_px'] <= area <= limits['max_area_px']:
                    continue
                if min(w, h) <= 0 or not limits['min_aspect_ratio'] <= w / h <= limits['max_aspect_ratio']:
                    continue
                fill = area / (w * h)
                if fill < limits['min_fill_ratio']:
                    continue
                moments = cv2.moments(boundary)
                centre = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
                try:
                    mapped = self.geometry.block_point(centre)
                except Rejected:
                    continue
                candidates.append(Blob(name, centre, mapped, area, min(1., fill / limits['min_fill_ratio'])))
            if len(candidates) != 1:
                raise Rejected(f'RGB needs one unambiguous {name}, found {len(candidates)}')
            found[name] = candidates[0]
        return found


class StableWindow:
    def __init__(self, layout, count=8, tolerance=0.005):
        self.layout, self.tolerance = layout, tolerance
        self.samples = collections.deque(maxlen=count)
        self.minimum = count

    def append(self, stamp, blobs):
        if self.samples and stamp <= self.samples[-1][0]:
            raise Rejected('Camera timestamps are duplicated or out of order')
        if set(blobs) != set(self.layout.blocks):
            self.samples.clear()
            raise Rejected('Missing block resets stability window')
        self.samples.append((stamp, blobs))

    def latest(self, now, newer_than=-1):
        if len(self.samples) < self.minimum or self.samples[-1][0] <= newer_than:
            raise Rejected('Awaiting a complete new stability window')
        for item in self.layout.blocks:
            coordinates = [row[1][item].xy for row in self.samples]
            if max(np.linalg.norm(np.subtract(a, b)) for a in coordinates for b in coordinates) > self.tolerance:
                raise Rejected('Block position has not stabilized')
        stamp, latest = self.samples[-1]
        result = Observation.assemble(self.layout, stamp, {k: b.xy for k, b in latest.items()})
        result.assert_recent(now)
        return result


def camera_pipeline(folder, layout):
    spec = configuration(folder / 'perception.yaml')
    projector = PixelGeometry(configuration(folder / 'camera_calibration.yaml'), spec)
    return ColourTracker(spec, projector), StableWindow(layout, spec['stability']['frames'], spec['stability']['max_xy_spread_m'])
