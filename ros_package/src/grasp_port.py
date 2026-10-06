"""Feedback-gated finger commands and acknowledged Gazebo joint transitions."""
import math
import time
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from .contracts import Rejected
from .ros_port import FINGERS, gazebo_sample


class GraspPort:
    OPEN, CLOSED = .0375, .0235

    def __init__(self, hub):
        self.hub = hub
        self.signals = {(item, state): hub.create_publisher(Empty, f'/m7/grasp/{item}/{command}', 10)
                        for item in hub.layout.blocks for state, command in (('attached', 'attach'), ('detached', 'detach'))}

    def state(self, item):
        return self.hub.joint_states[item][0]

    def require_empty(self):
        if any(self.state(item) != 'detached' for item in self.hub.layout.blocks):
            raise Rejected('Grasp states must confirm every block detached')

    def baseline(self):
        def connected():
            return all(port.get_subscription_count() > 0 for port in self.signals.values()) and all(self.state(item) is not None for item in self.hub.layout.blocks)
        self.hub.spin_until(connected, 30., 'Gazebo bridge and latched joint states')
        for item in self.hub.layout.blocks:
            if self.state(item) != 'detached':
                self.transition(item, 'detached')
        self.require_empty()

    def transition(self, item, desired):
        previous = self.hub.joint_states[item][1]
        publisher = self.signals[item, desired]
        next_emit = [0.]
        def confirmed():
            if time.monotonic() >= next_emit[0]:
                publisher.publish(Empty())
                next_emit[0] = time.monotonic() + .5
            state, serial = self.hub.joint_states[item]
            return serial > previous and state == desired
        self.hub.spin_until(confirmed, 8., item + ' ' + desired + ' acknowledgment')

    def finger_target(self, desired):
        if not 0 <= desired <= .04:
            raise Rejected('Finger target exceeds physical stroke')
        feedback = self.hub.measured()
        measured = dict(zip(feedback.name, feedback.position))
        initial = JointTrajectoryPoint(positions=[measured[name] for name in FINGERS], velocities=[0., 0.])
        finish = JointTrajectoryPoint(positions=[float(desired)] * 2, velocities=[0., 0.], time_from_start=DurationMsg(sec=2))
        route = JointTrajectory(joint_names=list(FINGERS), points=[initial, finish])
        serial = self.hub.joint_serial
        outcome = self.hub.action(FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory',
                                  FollowJointTrajectory.Goal(trajectory=route), timeout=12.)
        if outcome.error_code != 0:
            raise Rejected(f'Finger controller error {outcome.error_code}: {outcome.error_string}')
        self.hub.spin_until(lambda: self.hub.joint_serial > serial, 2., 'fresh finger feedback')
        actual = dict(zip(self.hub.measured().name, self.hub.measured().position))
        if any(abs(actual[name] - desired) > .003 for name in FINGERS):
            raise Rejected('Finger endpoint disagrees with controller success')

    def secure(self, item):
        self.require_empty()
        joints = self.hub.measured()
        positions = dict(zip(joints.name, joints.position))
        if any(abs(positions[name] - self.CLOSED) > .003 for name in FINGERS):
            raise Rejected('Grasp requires measured closed fingers')
        xyz = gazebo_sample(item)
        tcp = self.hub.frame_transform('world', 'gripper_tcp').transform.translation
        xy_gap = math.dist(xyz[:2], (tcp.x, tcp.y))
        z_gap = abs(xyz[2] - tcp.z)
        print(f'[PROXIMITY] {item} xy={xy_gap:.6f}m z={z_gap:.6f}m', flush=True)
        if xy_gap > .015 or z_gap > .020:
            raise Rejected('Physical object is outside the grasp envelope')
        self.transition(item, 'attached')
        self.require_held(item)

    def require_held(self, item):
        if self.state(item) != 'attached' or any(self.state(other) != 'detached' for other in self.hub.layout.blocks if other != item):
            raise Rejected('Gazebo must confirm exactly one attached object')

    def release(self, item):
        self.require_held(item)
        self.transition(item, 'detached')
        self.require_empty()
