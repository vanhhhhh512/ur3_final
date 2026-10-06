"""ROS event boundary: sensor reception, deadlines and checked service/action calls."""
import copy
import json
import math
import subprocess
import time
import numpy as np
import rclpy as ros
from rclpy.node import Node as RosNode
from rclpy.action import ActionClient as RosAction
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time as RosTime
from geometry_msgs.msg import Pose, PoseStamped
from .contracts import Rejected
from .optics import camera_pipeline


ARM_AXES = tuple(x + '_joint' for x in ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3'))
FINGERS = tuple(x + '_finger_joint' for x in ('left', 'right'))


def latched_policy():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quat_matrix(values):
    vector = np.array(values, dtype=float)
    norm = np.dot(vector, vector)
    if not np.isfinite(vector).all() or norm < 1e-12:
        raise Rejected('Quaternion must have finite nonzero norm')
    x, y, z, w = vector / math.sqrt(norm)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_at(xyz, quaternion=(0., 0., 0., 1.)):
    msg = Pose()
    for axis, value in zip('xyz', xyz):
        setattr(msg.position, axis, float(value))
    for axis, value in zip('xyzw', quaternion):
        setattr(msg.orientation, axis, float(value))
    return msg


class EventHub(RosNode):
    def __init__(self, layout, config_folder, name='bai03_observer'):
        super().__init__(name, parameter_overrides=[ros.parameter.Parameter('use_sim_time', value=True)])
        self.layout = layout
        self.tracker, self.window = camera_pipeline(config_folder, layout)
        self.decode_image = CvBridge()
        self.last_frame, self.last_blobs = None, None
        self.camera_fault, self.info_ok = None, False
        self.joints, self.joint_received = None, float('-inf')
        self.joint_serial = 0
        self.joint_states = {name: [None, 0] for name in layout.blocks}
        self.transforms = Buffer()
        self.listener = TransformListener(self.transforms, self)
        self.resources = [self.create_subscription(Image, '/camera/image_raw', self._image, qos_profile_sensor_data),
                          self.create_subscription(CameraInfo, '/camera/camera_info', self._info, qos_profile_sensor_data),
                          self.create_subscription(JointState, '/joint_states', self._feedback, qos_profile_sensor_data)]
        for item in layout.blocks:
            self.resources.append(self.create_subscription(String, f'/m7/grasp_cache/{item}/state',
                                  lambda msg, key=item: self._joint_event(key, msg.data), latched_policy()))

    def _feedback(self, sample):
        if len(sample.name) != len(sample.position) or len(set(sample.name)) != len(sample.name):
            return
        if not set(ARM_AXES + FINGERS).issubset(sample.name) or not all(math.isfinite(x) for x in sample.position):
            return
        self.joints, self.joint_received = copy.deepcopy(sample), time.monotonic()
        self.joint_serial += 1

    def _joint_event(self, item, state):
        if state in ('attached', 'detached'):
            self.joint_states[item] = [state, self.joint_states[item][1] + 1]

    def _info(self, message):
        try:
            self.tracker.geometry.validate_info(message)
            self.info_ok = True
        except Rejected as error:
            self.info_ok, self.camera_fault = False, str(error)
            self.window.samples.clear()

    def _image(self, message):
        if not self.info_ok:
            return
        try:
            if message.header.frame_id != self.tracker.geometry.frame:
                raise Rejected('RGB frame_id does not match CameraInfo')
            rgb = self.decode_image.imgmsg_to_cv2(message, desired_encoding='rgb8')
            blobs = self.tracker.segment(rgb)
            self.window.append(stamp_seconds(message.header.stamp), blobs)
            self.last_frame, self.last_blobs = rgb, blobs
            self.camera_fault = None
        except Exception as error:
            self.camera_fault = str(error)
            self.window.samples.clear()

    def spin_until(self, predicate, limit, description):
        end = time.monotonic() + limit
        while ros.ok() and time.monotonic() < end:
            if predicate():
                return
            ros.spin_once(self, timeout_sec=min(0.05, max(0., end-time.monotonic())))
        raise Rejected('Timed out: ' + description)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def observe(self, newer_than=-1., timeout=12.):
        result = []
        def ready():
            try:
                result[:] = [self.window.latest(self.now(), newer_than)]
                return self.info_ok
            except Rejected:
                return False
        self.spin_until(ready, timeout, 'stable calibrated RGB: ' + str(self.camera_fault))
        return result[0]

    def measured(self):
        if self.joints is None or time.monotonic() - self.joint_received > 1.:
            raise Rejected('Fresh complete joint feedback is required')
        return copy.deepcopy(self.joints)

    def frame_transform(self, target, source):
        self.spin_until(lambda: self.transforms.can_transform(target, source, RosTime()), 3., 'TF ' + source + '→' + target)
        return self.transforms.lookup_transform(target, source, RosTime())

    def convert_pose(self, pose, source, target):
        if source == target:
            return copy.deepcopy(pose)
        from tf2_geometry_msgs import do_transform_pose
        return do_transform_pose(pose, self.frame_transform(target, source))

    def service(self, message_type, address, request, timeout=20.):
        client = self.create_client(message_type, address)
        try:
            if not client.wait_for_service(timeout_sec=min(3., timeout)):
                raise Rejected('Service unavailable: ' + address)
            pending = client.call_async(request)
            try:
                self.spin_until(pending.done, timeout, address)
            except Rejected:
                pending.cancel()
                raise
            response = pending.result()
            if response is None:
                raise Rejected('Empty service response: ' + address)
            return response
        finally:
            self.destroy_client(client)

    def action(self, definition, address, goal, timeout=45.):
        channel = RosAction(self, definition, address)
        handle = None
        try:
            if not channel.wait_for_server(timeout_sec=3.):
                raise Rejected('Action unavailable: ' + address)
            submitted = channel.send_goal_async(goal)
            self.spin_until(submitted.done, 5., address + ' acceptance')
            handle = submitted.result()
            if handle is None or not handle.accepted:
                raise Rejected('Action rejected: ' + address)
            completed = handle.get_result_async()
            self.spin_until(completed.done, timeout, address + ' completion')
            envelope = completed.result()
            if envelope is None or envelope.status != 4:
                detail = '' if envelope is None else str(getattr(envelope.result, 'error_string', ''))
                raise Rejected(f'Action did not succeed: {address} {detail}')
            return envelope.result
        except BaseException:
            if handle is not None:
                pending = handle.cancel_goal_async()
                try:
                    self.spin_until(pending.done, 2., 'cancel interrupted action')
                except Rejected:
                    pass
            raise
        finally:
            channel.destroy()


def gazebo_samples(timeout=3.):
    """Read current dynamic model positions from Gazebo without changing state."""
    try:
        process = subprocess.run(['ign', 'topic', '-e', '-t', '/world/empty/pose/info', '--json-output', '-n', '1'],
                                 timeout=timeout, capture_output=True, text=True, check=False)
        if process.returncode != 0:
            raise Rejected('Gazebo observation command failed')
        payload = json.JSONDecoder().raw_decode(process.stdout.lstrip())[0]
        observed = {}
        for row in payload.get('pose', []):
            name = row.get('name')
            if not name:
                continue
            position = row.get('position', {})
            xyz = tuple(float(position.get(axis, 0.)) for axis in 'xyz')
            if all(math.isfinite(value) for value in xyz):
                observed[name] = xyz
        return observed
    except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
        raise Rejected('No usable read-only Gazebo pose snapshot') from error


def gazebo_sample(item, timeout=3.):
    """A read-only proximity/oracle query; never used as a planning target."""
    observed = gazebo_samples(timeout)
    if item not in observed:
        raise Rejected('Gazebo proximity sample does not uniquely identify ' + item)
    return observed[item]
