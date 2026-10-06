"""Small scene services: observed-joint relay and fixture/zone visualization."""
import sys
from functools import partial
from pathlib import Path
import rclpy as ros
from rclpy.node import Node as RosNode
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from .contracts import Layout
from .ros_port import gazebo_samples, latched_policy, pose_at


PACKAGE = 'ur3_perception_llm_control'


def scene_path(node):
    default = str(Path(get_package_share_directory(PACKAGE)) / 'config' / 'scene.yaml')
    return node.declare_parameter('scene_config', default).value


class JointRelay(RosNode):
    def __init__(self):
        super().__init__('bai03_joint_relay')
        scene = Layout.read(scene_path(self))
        self.outputs = {}
        self.inputs = []
        for item in scene.blocks:
            output = self.create_publisher(String, '/m7/grasp_cache/' + item + '/state', latched_policy())
            self.outputs[item] = output
            self.inputs.append(self.create_subscription(String, '/m7/grasp/' + item + '/state', partial(self.forward, item), 10))

    def forward(self, item, message):
        if message.data in {'detached', 'attached'}:
            self.outputs[item].publish(String(data=message.data))


class Fixtures(RosNode):
    def __init__(self):
        super().__init__('bai03_fixtures')
        self.layout = Layout.read(scene_path(self))
        self.markers = self.create_publisher(MarkerArray, '/workcell/zone_markers', latched_policy())
        self.sent = False
        self.timer = self.create_timer(1., self.refresh)

    def refresh(self):
        zones = []
        for index, (name, row) in enumerate(self.layout.data['zones'].items()):
            marker = Marker(ns='placement_regions', id=index, type=Marker.CUBE, action=Marker.ADD)
            marker.header.frame_id = 'world'
            marker.pose = pose_at(tuple(row['pose'][axis] for axis in 'xyz'))
            for axis in 'xyz':
                setattr(marker.scale, axis, float(row['size'][axis]))
            for channel in 'rgba':
                setattr(marker.color, channel, float(row['color'][channel]))
            zones.append(marker)
            label = Marker(ns='placement_region_labels', id=index, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            label.header.frame_id = 'world'
            label.pose = pose_at((row['pose']['x'], row['pose']['y'], row['pose']['z'] + .018))
            label.scale.z = .014
            label.color.r = label.color.g = label.color.b = .08
            label.color.a = 1.
            label.text = name.replace('_', ' ').title()
            zones.append(label)
        self.markers.publish(MarkerArray(markers=zones))
        if self.sent:
            return
        if not hasattr(self, 'client'):
            self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not self.client.service_is_ready():
            return
        try:
            observed = gazebo_samples(timeout=1.5)
        except Exception as error:
            self.get_logger().debug(f'Waiting for read-only Gazebo object poses: {error}')
            return
        if not set(self.layout.blocks).issubset(observed):
            self.get_logger().debug('Waiting for all dynamic cube poses from Gazebo')
            return
        patch = PlanningScene(is_diff=True)
        patch.robot_state.is_diff = True
        rows = [(self.layout.data[key], tuple(self.layout.data[key]['pose'][axis] for axis in 'xyz'))
                for key in ('table', 'pedestal')]
        rows.extend((self.layout.data['objects'][item], observed[item]) for item in self.layout.blocks)
        for row, xyz in rows:
            box = CollisionObject(id=row['name'], operation=CollisionObject.ADD, pose=pose_at(xyz))
            box.header.frame_id = 'world'
            for dimensions, offset in self.layout.collision_parts(row['name']):
                box.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions)))
                box.primitive_poses.append(pose_at(offset))
            patch.world.collision_objects.append(box)
            patch.object_colors.append(ObjectColor(id=row['name'], color=ColorRGBA(**row['color'])))
        if not hasattr(self, 'pending'):
            self.pending = self.client.call_async(ApplyPlanningScene.Request(scene=patch))
        elif self.pending.done():
            if self.pending.result() is not None and self.pending.result().success:
                self.sent = True
            else:
                del self.pending


def run_service(kind, args=None):
    ros.init(args=args)
    process = JointRelay() if kind == 'relay' else Fixtures()
    try:
        ros.spin(process)
    except KeyboardInterrupt:
        pass
    finally:
        process.destroy_node()
        if ros.ok():
            ros.shutdown()


def relay_main():
    run_service('relay')


def fixtures_main():
    run_service('fixtures')
