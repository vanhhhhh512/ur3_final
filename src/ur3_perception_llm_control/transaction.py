"""One command is a verified transaction; observations, not predictions, commit it."""
import dataclasses as dc
import math
from .contracts import Goal, Operation, Rejected
from .ros_port import FINGERS, pose_at
from .vacancy import TableSearch


@dc.dataclass
class Journal:
    command: str
    plan: object = None
    completed: int = 0
    state: str = 'pending'
    error: str | None = None
    events: list = dc.field(default_factory=list)

    def record(self, event, **fields):
        self.events.append({'event': event, **fields})

    def document(self):
        return {'command': self.command, 'status': self.state, 'completed_steps': self.completed,
                'plan': self.plan, 'error': self.error, 'events': self.events}


class WorkcellSession:
    def __init__(self, hub, scene, arm, grasp, model, margins, logger=None):
        self.hub, self.scene, self.arm, self.grasp, self.model = hub, scene, arm, grasp, model
        self.layout = hub.layout
        self.search = TableSearch(self.layout, margins)
        self.hand = None
        self.observed = None
        self.slot = None
        self.poisoned = False
        self.logger = logger
        self.target_zone = None

    def _log(self, line):
        if self.logger is not None:
            self.logger(line)

    def _camera_report(self, snapshot, items=None, zones=None):
        for item in items or ('red_cube', 'blue_cube', 'yellow_cube', 'green_cube', 'purple_cube'):
            if item not in snapshot.xy:
                continue
            x, y = snapshot.xy[item]
            z = self.layout.centre_height(item)
            self._log(f'[DETECTION] {item} detected at x={x:.3f}, y={y:.3f}, z={z:.3f} m')
        for zone in zones or self.layout.areas:
            occupant = snapshot.occupants[zone]
            label = zone.replace('_', ' ').title()
            self._log(f'[DETECTION] {label}: ' + ('EMPTY' if occupant is None else f'OCCUPIED by {occupant}'))

    def _new_view(self, after=-1.):
        observed = self.hub.observe(newer_than=after)
        observed.assert_recent(self.hub.now())
        self.scene.sync(observed)
        self.observed = observed
        return observed

    def _idle_check(self):
        self.grasp.require_empty()
        if self.scene.fetch().robot_state.attached_collision_objects:
            raise Rejected('MoveIt still contains a carried object')
        feedback = self.hub.measured()
        positions = dict(zip(feedback.name, feedback.position))
        if any(abs(positions[name] - self.grasp.OPEN) > .003 for name in FINGERS):
            raise Rejected('Idle gripper must be physically open')

    def perform(self, command):
        record = Journal(command)
        planner_called = False
        try:
            if self.poisoned:
                raise Rejected('Session is latched in failure; restart after reviewing actual state')
            goal = Goal.from_text(command)
            self.arm.check_controllers()
            self._idle_check()
            before = self._new_view()
            self._log('[CAMERA] Camera initialized successfully')
            self._log('[CAMERA] Detecting cubes and zones...')
            self._camera_report(before)
            self._log(f'[PLANNER] Requested task: Move {goal.item} to {goal.destination.replace("_", " ").title()}')
            self.target_zone = goal.destination
            occupant = before.occupants[goal.destination]
            if occupant is not None:
                self._log(f'[PLANNER] Target {goal.destination.replace("_", " ").title()} is occupied by {occupant}')
            else:
                self._log(f'[PLANNER] Target {goal.destination.replace("_", " ").title()} is empty')
            planner_called = True
            plan, raw = self.model.propose(command, goal, before)
            record.plan = [instruction.payload() for instruction in plan.steps]
            record.record('llm_response', text=raw)
            record.record('validated', requested_item=goal.item, destination=goal.destination)
            for attempt in getattr(self.model, 'last_attempts', [])[:-1]:
                self._log('[PLANNER] Rejected plan: ' + attempt['error'] + '; requesting a corrected plan')
            self._log(f'[PLANNER] Camera state verified; accepted {len(plan.steps)} validated skills')
            latest = self._new_view(after=before.stamp)
            if not before.agrees(latest):
                raise Rejected('Scene changed while waiting for language-model planning')
            blocker = latest.occupants[goal.destination]
            if any(step.operation == Operation.STAGE for step in plan.steps):
                self.slot = self.search.choose(blocker, latest, lambda candidate: self.arm.inspect_slot(blocker, candidate))
                record.record('temporary_reserved', xy=list(self.slot.xy), clearance_m=self.slot.clearance)
                self._log(f'[PLANNER] Target {goal.destination.replace("_", " ").title()} is occupied')
                self._log('[PLANNER] Searching for a temporary placement location...')
                self._log(f'[PLANNER] Temporary location selected: x={self.slot.xy[0]:.3f}, y={self.slot.xy[1]:.3f}, z={self.slot.height:.3f} m')
            # Nothing above this boundary executes a trajectory or changes a Gazebo joint.
            record.state = 'executing'
            for instruction in plan.steps:
                self._step(instruction)
                record.completed += 1
                record.record('skill_complete', **instruction.payload())
                if instruction.operation == Operation.STAGE:
                    self._log(f'[ACTION] {instruction.item} successfully relocated')
                elif instruction.operation == Operation.DEPOSIT:
                    label = instruction.destination.replace('_', ' ').title()
                    self._log(f'[ACTION] {instruction.item} successfully placed in {label}')
                elif instruction.operation == Operation.PARK:
                    self._log('[ACTION] Robot returned to HOME')
            final = self._new_view(after=self.observed.stamp)
            self._idle_check()
            if final.locations != plan.predicted_locations:
                raise Rejected('Final camera state disagrees with validated goal state')
            measured = self.hub.measured()
            axes = dict(zip(measured.name, measured.position))
            home_error = max(abs(axes[name] - angle) for name, angle in self.arm.settings['home'].items())
            if home_error > .02:
                raise Rejected('Home feedback exceeds the final tolerance')
            record.record('verified', camera=final.context(), home_error_rad=home_error)
            record.state = 'success'
            self._log('[SYSTEM] Task completed successfully')
        except (Rejected, ValueError, KeyError, TypeError) as error:
            record.error = str(error)
            self._log('[ERROR] ' + str(error))
            if record.state == 'executing':
                self.poisoned = True
            record.state = 'failed'
        finally:
            for attempt in (getattr(self.model, 'last_attempts', []) if planner_called else []):
                record.record('planner_attempt', **attempt)
            self.slot = None
        return record

    def _step(self, instruction):
        op = instruction.operation
        if op == Operation.TAKE:
            self._log(f'[ACTION] Picking {instruction.item}...')
            if self.hand is not None:
                raise Rejected('Executor hand is already occupied')
            previous = self.observed
            observed = self._new_view(after=previous.stamp)
            if not previous.agrees(observed):
                raise Rejected('Observed scene changed after reservation or preceding step')
            centre = (*observed.xy[instruction.item], self.layout.centre_height(instruction.item))
            self.grasp.require_empty()
            self.grasp.finger_target(self.grasp.OPEN)
            approach = self.arm.settings['manipulation']['approach_clearance']
            self.arm.travel((*centre[:2], centre[2] + approach))
            self.scene.contact(instruction.item, True)
            self.arm.straight(centre)
            self.grasp.finger_target(self.grasp.CLOSED)
            self.grasp.secure(instruction.item)
            self.scene.attach(instruction.item, pose_at(centre))
            self.hand = instruction.item
            self.grasp.require_held(self.hand)
            self.arm.straight((*centre[:2], centre[2] + self.arm.settings['manipulation']['retreat_clearance']))
            self.grasp.require_held(self.hand)
            self.arm.home()
        elif op in (Operation.STAGE, Operation.DEPOSIT):
            if op == Operation.STAGE:
                self._log(f'[ACTION] Moving {instruction.item} to the temporary location...')
            else:
                label = instruction.destination.replace('_', ' ').title()
                self._log(f'[ACTION] Moving {instruction.item} into {label}...')
            if self.hand != instruction.item:
                raise Rejected('Executor cannot place an object it is not holding')
            if op == Operation.STAGE:
                if self.slot is None:
                    raise Rejected('No validated temporary reservation')
                xyz = (*self.slot.xy, self.slot.height)
                expected = 'table'
            else:
                area = self.layout.areas[instruction.destination]
                xyz = (*area.centre, self.layout.centre_height(instruction.item))
                expected = instruction.destination
            self._release_at(instruction.item, xyz, expected)
        else:
            self._log('[ACTION] Returning robot to HOME...')
            if self.hand is not None:
                raise Rejected('Cannot park with a held object')
            self.grasp.require_empty()
            self.arm.home()

    def _release_at(self, item, centre, expected):
        self.grasp.require_held(item)
        self.scene.assert_ownership(item, attached=True)
        clearance = self.arm.settings['manipulation']['approach_clearance']
        tcp = self.scene.release_tcp(item, centre, self.arm.settings['manipulation']['orientation_xyzw'])
        self.arm.travel((*tcp[:2], tcp[2] + clearance))
        self.grasp.require_held(item)
        self.scene.support_contact(item, True)
        self.arm.straight(tcp)
        self.grasp.require_held(item)
        self.scene.release(item, centre)
        self.grasp.release(item)
        released = self.hub.now()
        self.grasp.finger_target(self.grasp.OPEN)
        self.hand = None
        self.arm.straight((*tcp[:2], tcp[2] + self.arm.settings['manipulation']['retreat_clearance']))
        self.scene.support_contact(item, False)
        self.scene.contact(item, False)
        self.arm.home()
        self.hub.spin_until(lambda: self.hub.now() - released >= 2., 15., 'two simulation seconds of release settling')
        if expected == 'table' and self.target_zone is not None:
            self._log(f'[CAMERA] Rechecking {self.target_zone.replace("_", " ").title()} after moving the blocker...')
        observed = self._new_view(after=released)
        self._camera_report(observed, items=[item], zones=[self.target_zone] if expected == 'table' and self.target_zone else [expected])
        if observed.locations[item] != expected or math.dist(observed.xy[item], centre[:2]) > .005:
            raise Rejected('Released object did not settle within 5 mm at its destination')
        self.grasp.require_empty()
        self.scene.assert_ownership(item, attached=False)
