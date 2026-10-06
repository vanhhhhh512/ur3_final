"""Transactional MoveIt scene updates sourced from observed RGB geometry."""
import copy
import math
from moveit_msgs import msg as mm
from moveit_msgs import srv as ms
from shape_msgs.msg import SolidPrimitive
from .contracts import Rejected
from .ros_port import pose_at, quat_matrix


class ScenePort:
    def __init__(self, hub):
        self.hub, self.layout = hub, hub.layout
        self.attachment_frame = 'gripper_tcp'
        self.contact_links = ['gripper_base', 'left_finger_link', 'right_finger_link']

    def fetch(self):
        query = ms.GetPlanningScene.Request()
        query.components.components = 2 | 4 | 8 | 16 | 64 | 128
        return self.hub.service(ms.GetPlanningScene, '/get_planning_scene', query).scene

    def state(self):
        result = copy.deepcopy(self.fetch().robot_state)
        result.joint_state = self.hub.measured()
        result.is_diff = False
        return result

    def box(self, name, xyz, frame='world'):
        result = mm.CollisionObject(id=name, pose=pose_at(xyz), operation=mm.CollisionObject.ADD)
        result.header.frame_id = frame
        for dimensions, offset in self.layout.collision_parts(name):
            shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, dimensions)))
            result.primitives.append(shape)
            result.primitive_poses.append(pose_at(offset))
        return result

    def removal(self, name):
        deleted = mm.CollisionObject(id=name, operation=mm.CollisionObject.REMOVE)
        deleted.header.frame_id = 'world'
        return deleted

    def patch(self, boxes=(), attachments=(), matrix=None, colors=()):
        update = mm.PlanningScene(is_diff=True)
        update.robot_state.is_diff = True
        update.world.collision_objects = list(boxes)
        update.robot_state.attached_collision_objects = list(attachments)
        update.object_colors = list(colors)
        if matrix is not None:
            update.allowed_collision_matrix = matrix
        request = ms.ApplyPlanningScene.Request(scene=update)
        if not self.hub.service(ms.ApplyPlanningScene, '/apply_planning_scene', request).success:
            raise Rejected('MoveIt scene update was rejected')

    def sync(self, snapshot):
        snapshot.assert_recent(self.hub.now())
        before = self.fetch()
        if before.robot_state.attached_collision_objects:
            raise Rejected('Cannot replace observed WORLD state while an object is attached')
        fixtures = [self.layout.data[key] for key in ('pedestal', 'table')]
        boxes = [self.box(row['name'], tuple(row['pose'][axis] for axis in 'xyz')) for row in fixtures]
        boxes += [self.box(item, (*snapshot.xy[item], self.layout.centre_height(item))) for item in self.layout.blocks]
        rows = [*fixtures, *(self.layout.data['objects'][item] for item in self.layout.blocks)]
        colors = [mm.ObjectColor(id=row['name'], color=self._color(row['color'])) for row in rows]
        self.patch(boxes=boxes, colors=colors)
        after = self.fetch()
        by_name = {box.id: box for box in after.world.collision_objects}
        required = set(self.layout.blocks) | {row['name'] for row in fixtures}
        if not required.issubset(by_name) or after.robot_state.attached_collision_objects:
            raise Rejected('MoveIt did not acknowledge the complete observed scene')
        for item in self.layout.blocks:
            p = by_name[item].pose.position
            actual = (p.x, p.y, p.z)
            expected = (*snapshot.xy[item], self.layout.centre_height(item))
            if math.dist(actual, expected) > 0.001:
                raise Rejected('MoveIt geometry differs from camera for ' + item)

    @staticmethod
    def _color(rgba):
        from std_msgs.msg import ColorRGBA
        return ColorRGBA(r=float(rgba['r']), g=float(rgba['g']), b=float(rgba['b']), a=float(rgba['a']))

    def contact(self, item, enabled):
        self._allow_pairs(item, self.contact_links, enabled)

    def support_contact(self, item, enabled):
        self._allow_pairs(item, [self.layout.data['table']['name']], enabled)

    def _allow_pairs(self, item, links, enabled):
        matrix = copy.deepcopy(self.fetch().allowed_collision_matrix)
        if len(matrix.entry_values) != len(matrix.entry_names) or any(len(row.enabled) != len(matrix.entry_names) for row in matrix.entry_values):
            raise Rejected('Malformed allowed-collision matrix')
        for name in [item, *links]:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(mm.AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
        position = {name: i for i, name in enumerate(matrix.entry_names)}
        for link in links:
            a, b = position[item], position[link]
            matrix.entry_values[a].enabled[b] = enabled
            matrix.entry_values[b].enabled[a] = enabled
        self.patch(matrix=matrix)

    def release_tcp(self, item, centre, orientation):
        matches = [box for box in self.fetch().robot_state.attached_collision_objects if box.object.id == item]
        if len(matches) != 1 or matches[0].link_name != self.attachment_frame:
            raise Rejected('Release requires a unique gripper attachment transform')
        carried = matches[0].object
        if carried.header.frame_id not in ('', self.attachment_frame):
            raise Rejected('Attachment geometry is not expressed in the gripper frame')
        offset = tuple(getattr(carried.pose.position, axis) for axis in 'xyz')
        if not all(math.isfinite(v) for v in offset) or math.sqrt(sum(v*v for v in offset)) > .03:
            raise Rejected('Attachment offset exceeds the grasp envelope')
        world_offset = quat_matrix(orientation) @ offset
        return tuple(float(centre[i] - world_offset[i]) for i in range(3))

    def attach(self, item, world_pose):
        carried = self.box(item, (0, 0, 0), self.attachment_frame)
        carried.pose = self.hub.convert_pose(world_pose, 'world', self.attachment_frame)
        attachment = mm.AttachedCollisionObject(link_name=self.attachment_frame, touch_links=self.contact_links, object=carried)
        # MoveIt moves a WORLD object into ATTACHED while processing this ADD.
        # A WORLD REMOVE in the same diff runs afterwards and fails because
        # that ownership transfer has already removed the object.
        self.patch(attachments=[attachment])
        self.assert_ownership(item, attached=True)

    def release(self, item, xyz):
        detached = mm.AttachedCollisionObject(link_name=self.attachment_frame, object=self.removal(item))
        self.patch(boxes=[self.box(item, xyz)], attachments=[detached])
        self.assert_ownership(item, attached=False)

    def assert_ownership(self, item, attached):
        state = self.fetch()
        present = item in {box.id for box in state.world.collision_objects}
        carried = item in {box.object.id for box in state.robot_state.attached_collision_objects}
        if (present, carried) != (not attached, attached):
            raise Rejected('WORLD/ATTACHED ownership mismatch for ' + item)

    def imagined_carry(self, item):
        alternate = mm.PlanningScene(is_diff=True)
        alternate.robot_state.is_diff = True
        alternate.robot_state.attached_collision_objects = [mm.AttachedCollisionObject(
            link_name=self.attachment_frame, touch_links=self.contact_links,
            object=self.box(item, (0, 0, 0), self.attachment_frame))]
        return alternate
