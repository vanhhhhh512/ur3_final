"""MoveIt-generated motion only; task plans never contain joint targets."""
import copy
import math
from geometry_msgs.msg import PoseStamped
from moveit_msgs import msg as mm
from moveit_msgs import srv as ms
from moveit_msgs import action as ma
from controller_manager_msgs.srv import ListControllers
from builtin_interfaces.msg import Duration as DurationMsg
from .contracts import Rejected
from .ros_port import ARM_AXES, FINGERS, pose_at, quat_matrix, stamp_seconds


class ArmPort:
    def __init__(self, hub, scene, settings):
        self.hub, self.scene, self.settings = hub, scene, settings
        if settings['planning_tip'] != 'tool0' or settings['application_tip'] != 'gripper_tcp':
            raise Rejected('This adapter requires tool0 planning and gripper_tcp application frames')
        if not 0 < settings['tool0_to_tcp_z'] < .5:
            raise Rejected('TCP offset is outside the supported physical range')
        quat_matrix(settings['manipulation']['orientation_xyzw'])
        if set(settings['home']) != set(ARM_AXES):
            raise Rejected('Home posture must define exactly six arm joints')
        if not 0 < settings['velocity_scaling'] <= 1 or not 0 < settings['acceleration_scaling'] <= 1:
            raise Rejected('Invalid MoveIt scaling factors')

    def check_controllers(self):
        state = self.hub.service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
        active = {row.name for row in state.controller if row.state == 'active'}
        if not {'joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller'}.issubset(active):
            raise Rejected('Arm, state and gripper controllers must all be active')
        self.hub.measured()

    def target_pose(self, xyz):
        q = tuple(self.settings['manipulation']['orientation_xyzw'])
        desired = self.hub.convert_pose(pose_at(xyz, q), 'world', self.settings['planning_frame'])
        # Offset uses the transformed orientation, including any mounting rotation.
        orientation = desired.orientation
        shifted = quat_matrix(tuple(getattr(orientation, axis) for axis in 'xyzw')) @ [0., 0., self.settings['tool0_to_tcp_z']]
        for axis, offset in zip('xyz', shifted):
            setattr(desired.position, axis, getattr(desired.position, axis) - float(offset))
        result = PoseStamped(pose=desired)
        result.header.frame_id = self.settings['planning_frame']
        result.header.stamp = self.hub.get_clock().now().to_msg()
        return result

    def with_axes(self, state, values):
        result = copy.deepcopy(state)
        if set(values) != set(ARM_AXES) or not all(math.isfinite(v) and abs(v) <= math.tau for v in values.values()):
            raise Rejected('Invalid six-axis planning seed')
        index = dict(zip(result.joint_state.name, range(len(result.joint_state.name))))
        if not set(ARM_AXES + FINGERS).issubset(index):
            raise Rejected('Planning state lacks arm or gripper feedback')
        positions = list(result.joint_state.position)
        for name, value in values.items():
            positions[index[name]] = float(value)
        result.joint_state.position, result.joint_state.velocity, result.joint_state.effort = positions, [], []
        return result

    def trajectory_ok(self, trajectory):
        data = trajectory.joint_trajectory
        if set(data.joint_names) != set(ARM_AXES) or len(data.joint_names) != 6 or not data.points:
            raise Rejected('MoveIt returned an incomplete arm trajectory')
        previous = -1.
        for point in data.points:
            timing = stamp_seconds(point.time_from_start)
            values = [*point.positions, *point.velocities, *point.accelerations]
            if len(point.positions) != 6 or not all(math.isfinite(v) for v in values) or timing <= previous:
                raise Rejected('MoveIt trajectory contains invalid samples or timing')
            previous = timing
        return trajectory

    def joint_plan(self, values, start=None, alternate=None):
        start = start if start is not None else self.scene.state()
        self.with_axes(start, values)
        goal = ma.MoveGroup.Goal()
        request = goal.request
        request.group_name = self.settings['planning_group']
        request.start_state = copy.deepcopy(start)
        request.allowed_planning_time = float(self.settings['planning_time'])
        request.num_planning_attempts = int(self.settings['planning_attempts'])
        request.max_velocity_scaling_factor = float(self.settings['velocity_scaling'])
        request.max_acceleration_scaling_factor = float(self.settings['acceleration_scaling'])
        request.goal_constraints = [mm.Constraints(joint_constraints=[mm.JointConstraint(
            joint_name=name, position=float(values[name]), tolerance_above=.001, tolerance_below=.001, weight=1.) for name in ARM_AXES])]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        if alternate is not None:
            goal.planning_options.planning_scene_diff = copy.deepcopy(alternate)
        result = self.hub.action(ma.MoveGroup, '/move_action', goal)
        if result.error_code.val != 1:
            raise Rejected('Collision-aware MoveIt planning failed: ' + str(result.error_code.val))
        return self.trajectory_ok(result.planned_trajectory)

    def pose_plan(self, xyz, start=None, alternate=None):
        start = start if start is not None else self.scene.state()
        origin = dict(zip(start.joint_state.name, start.joint_state.position))
        variants = [dict((name, origin[name]) for name in ARM_AXES), dict(self.settings['home'])]
        for axis in ('elbow_joint', 'shoulder_pan_joint', 'wrist_1_joint'):
            variant = dict(variants[0])
            variant[axis] = max(-math.tau, min(math.tau, variant[axis] + math.pi))
            variants.append(variant)
        candidates = {}
        for values in variants:
            query = ms.GetPositionIK.Request()
            task = query.ik_request
            task.group_name = self.settings['planning_group']
            task.robot_state = self.with_axes(start, values)
            if alternate is not None:
                task.robot_state.attached_collision_objects = copy.deepcopy(alternate.robot_state.attached_collision_objects)
            task.ik_link_name, task.avoid_collisions = self.settings['planning_tip'], True
            task.pose_stamped, task.timeout = self.target_pose(xyz), DurationMsg(sec=2)
            solution = self.hub.service(ms.GetPositionIK, '/compute_ik', query)
            if solution.error_code.val != 1:
                continue
            names = dict(zip(solution.solution.joint_state.name, solution.solution.joint_state.position))
            if not set(ARM_AXES).issubset(names):
                continue
            corrected = {name: names[name] + math.tau * round((origin[name] - names[name]) / math.tau) for name in ARM_AXES}
            if any(not math.isfinite(v) or abs(v) > math.tau for v in corrected.values()):
                continue
            candidates[tuple(round(corrected[n], 5) for n in ARM_AXES)] = corrected
        ranked = sorted(candidates.values(), key=lambda row: sum((row[n] - origin[n]) ** 2 for n in ARM_AXES))
        for values in ranked:
            try:
                return self.joint_plan(values, start, alternate)
            except Rejected:
                continue
        raise Rejected('No complete collision-checked TCP plan')

    def execute(self, trajectory):
        self.trajectory_ok(trajectory)
        sequence = self.hub.joint_serial
        outcome = self.hub.action(ma.ExecuteTrajectory, '/execute_trajectory', ma.ExecuteTrajectory.Goal(trajectory=trajectory), timeout=90.)
        if outcome.error_code.val != 1:
            raise Rejected('MoveIt execution reported failure')
        self.hub.spin_until(lambda: self.hub.joint_serial > sequence, 2., 'post-motion joint feedback')
        self.hub.measured()

    def home(self):
        self.execute(self.joint_plan(self.settings['home']))

    def travel(self, xyz):
        self.execute(self.pose_plan(xyz))

    def straight(self, xyz):
        pose = self.target_pose(xyz)
        query = ms.GetCartesianPath.Request(header=pose.header, start_state=self.scene.state(),
            group_name=self.settings['planning_group'], link_name=self.settings['planning_tip'],
            waypoints=[pose.pose], max_step=.005, jump_threshold=0., avoid_collisions=True,
            max_velocity_scaling_factor=float(self.settings['velocity_scaling']),
            max_acceleration_scaling_factor=float(self.settings['acceleration_scaling']))
        result = self.hub.service(ms.GetCartesianPath, '/compute_cartesian_path', query)
        if result.error_code.val != 1 or result.fraction < 1.:
            raise Rejected('Cartesian path is incomplete or colliding')
        route = copy.deepcopy(self.trajectory_ok(result.solution))
        for point in route.joint_trajectory.points:
            nanos = round(stamp_seconds(point.time_from_start) * 2e9)
            point.time_from_start = DurationMsg(sec=nanos // 1000000000, nanosec=nanos % 1000000000)
            point.velocities = [v / 2 for v in point.velocities]
            point.accelerations = [v / 4 for v in point.accelerations]
        self.execute(route)

    def inspect_slot(self, item, candidate):
        alternate = self.scene.imagined_carry(item)
        state = self.with_axes(self.scene.state(), self.settings['home'])
        state.attached_collision_objects = copy.deepcopy(alternate.robot_state.attached_collision_objects)
        try:
            for height in (candidate.height + self.settings['manipulation']['approach_clearance'], candidate.height):
                plan = self.pose_plan((*candidate.xy, height), state, alternate)
                data = plan.joint_trajectory
                state = self.with_axes(state, dict(zip(data.joint_names, data.points[-1].positions)))
            return True
        except Rejected:
            return False
