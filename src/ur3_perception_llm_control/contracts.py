"""Domain contracts; no ROS messages, model pose reader, or actuator imports."""
import dataclasses as dc
import enum
import math
import re
import types
from pathlib import Path
import yaml


class Rejected(RuntimeError):
    """A failed precondition prevents further task progress."""


def frozen(values):
    return types.MappingProxyType(dict(values))


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise Rejected(f'Expected finite numeric value, received {value!r}')
    return float(value)


def configuration(filename):
    payload = yaml.safe_load(Path(filename).read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise Rejected(f'Configuration root is not a mapping: {filename}')
    return payload


@dc.dataclass(frozen=True)
class Rectangle:
    centre: tuple
    extent: tuple

    def includes(self, point, inset=0.0):
        return all(abs(point[k] - self.centre[k]) <= self.extent[k] / 2 - inset for k in range(2))

    def overlaps_box(self, point, dimensions, padding=0.0):
        return all(abs(point[k] - self.centre[k]) < (dimensions[k] + self.extent[k]) / 2 + padding for k in range(2))


@dc.dataclass(frozen=True)
class Layout:
    data: object
    blocks: tuple
    areas: object
    support: Rectangle
    surface_z: float
    dimensions: object

    @classmethod
    def read(cls, filename):
        scene = configuration(filename)
        objects = scene.get('objects', {})
        required = {f'{c}_cube' for c in ('red', 'yellow', 'blue', 'green', 'purple')}
        if set(objects) != required or set(scene.get('zones', {})) != {'zone_a', 'zone_b', 'zone_c'}:
            raise Rejected('Workcell requires exactly five named blocks and three zones')
        for component in (scene['table'], scene['pedestal'], *objects.values(), *scene['zones'].values()):
            tuple(finite_number(component['pose'][axis]) for axis in 'xyz')
            size = tuple(finite_number(component['size'][axis]) for axis in 'xyz')
            if min(size) <= 0 or any(abs(component['pose'].get(axis, 0)) > 1e-9 for axis in ('roll', 'pitch', 'yaw')):
                raise Rejected('Only positive-size axis-aligned workcell boxes are supported')
        table = scene['table']
        support = Rectangle(tuple(table['pose'][k] for k in 'xy'), tuple(table['size'][k] for k in 'xy'))
        zones = {key: Rectangle(tuple(row['pose'][k] for k in 'xy'), tuple(row['size'][k] for k in 'xy')) for key, row in scene['zones'].items()}
        sizes = {key: tuple(row['size'][k] for k in 'xyz') for key, row in objects.items()}
        return cls(scene, tuple(sorted(objects)), frozen(zones), support,
                   table['pose']['z'] + table['size']['z'] / 2, frozen(sizes))

    def centre_height(self, item):
        return self.surface_z + self.dimensions[item][2] / 2

    def collision_parts(self, name):
        """Return local-frame BOX parts for the planning-scene fixture geometry."""
        if name in self.dimensions:
            return [(self.dimensions[name], (0.0, 0.0, 0.0))]
        row = next(self.data[key] for key in ('table', 'pedestal') if self.data[key]['name'] == name)
        size = tuple(float(row['size'][axis]) for axis in 'xyz')
        if name != self.data['table']['name']:
            return [(size, (0.0, 0.0, 0.0))]
        thickness = min(0.04, size[2] * 0.15)
        leg_height = size[2] - thickness
        parts = [((size[0], size[1], thickness), (0.0, 0.0, size[2] / 2 - thickness / 2))]
        leg_x = size[0] / 2 - 0.0175 - 0.012
        leg_y = size[1] / 2 - 0.0175 - 0.012
        for sign_x in (-1, 1):
            for sign_y in (-1, 1):
                parts.append(((0.035, 0.035, leg_height),
                              (sign_x * leg_x, sign_y * leg_y, -thickness / 2)))
        return parts

    def locate(self, item, xy):
        if not self.support.includes(xy, max(self.dimensions[item][:2]) / 2):
            raise Rejected(f'{item} lies outside the supported table')
        zones = [key for key, area in self.areas.items() if area.includes(xy)]
        if len(zones) > 1:
            raise Rejected('Overlapping zones make occupancy ambiguous')
        return zones[0] if zones else 'table'


@dc.dataclass(frozen=True)
class Observation:
    stamp: float
    xy: object
    locations: object
    occupants: object

    @classmethod
    def assemble(cls, layout, stamp, centres):
        if set(centres) != set(layout.blocks):
            raise Rejected('Incomplete RGB observation; no spawn-pose fallback')
        points = {k: tuple(finite_number(x) for x in centres[k]) for k in layout.blocks}
        if any(len(v) != 2 for v in points.values()):
            raise Rejected('RGB centres must be XY pairs')
        locations = {k: layout.locate(k, p) for k, p in points.items()}
        occupancy = dict.fromkeys(layout.areas)
        for item, location in locations.items():
            if location != 'table':
                if occupancy[location] is not None:
                    raise Rejected(f'Multiple blocks occupy {location}')
                occupancy[location] = item
        return cls(finite_number(stamp), frozen(points), frozen(locations), frozen(occupancy))

    def assert_recent(self, sim_now, maximum_age=1.0):
        age = finite_number(sim_now) - self.stamp
        if not -0.05 <= age <= maximum_age:
            raise Rejected(f'RGB age {age:.3f}s is outside the freshness window')

    def agrees(self, later, displacement=0.005):
        return self.locations == later.locations and all(math.dist(self.xy[k], later.xy[k]) <= displacement for k in self.xy)

    def context(self):
        return {'blocks': {k: {'xy_m': list(self.xy[k]), 'location': self.locations[k]} for k in self.xy},
                'zones': dict(self.occupants), 'observed_at_sim_s': self.stamp}


@dc.dataclass(frozen=True)
class Goal:
    item: str
    destination: str

    @classmethod
    def from_text(cls, text):
        pattern = r'\s*(?:put|place|move)\s+(?:the\s+)?(red|yellow|blue|green|purple)[ _]+cube\s+(?:in|into|to|at)\s+zone[ _]+([abc])(?:\s+and\s+(?:return|go)\s+home)?[.!]?\s*'
        match = re.fullmatch(pattern, text, flags=re.IGNORECASE)
        if not match:
            raise Rejected('Supported request: Put the <colour> cube in Zone <A/B/C> [and return home].')
        return cls(match[1].lower() + '_cube', 'zone_' + match[2].lower())


class Operation(str, enum.Enum):
    TAKE = 'take'
    STAGE = 'stage'
    DEPOSIT = 'deposit'
    PARK = 'park'


@dc.dataclass(frozen=True)
class Instruction:
    operation: Operation
    item: str | None = None
    destination: str | None = None

    @classmethod
    def decode(cls, row, layout):
        if not isinstance(row, dict):
            raise Rejected('Each instruction must be an object')
        try:
            action = Operation(row.get('op'))
        except (ValueError, TypeError):
            raise Rejected('Unknown skill or low-level control request') from None
        keys = {'op'} | ({'item'} if action != Operation.PARK else set()) | ({'zone'} if action == Operation.DEPOSIT else set())
        if set(row) != keys or row.get('item', layout.blocks[0]) not in layout.blocks:
            raise Rejected('Instruction fields or object identifier are invalid')
        if action == Operation.DEPOSIT and row['zone'] not in layout.areas:
            raise Rejected('Unknown destination zone')
        return cls(action, row.get('item'), row.get('zone'))

    def payload(self):
        row = {'op': self.operation.value}
        if self.item is not None:
            row['item'] = self.item
        if self.destination is not None:
            row['zone'] = self.destination
        return row


@dc.dataclass(frozen=True)
class VerifiedPlan:
    steps: tuple
    predicted_locations: object
    source: Observation
    goal: Goal

    @classmethod
    def check(cls, document, observation, goal, layout):
        if not isinstance(document, dict) or set(document) != {'steps'} or not isinstance(document['steps'], list):
            raise Rejected('Plan must contain only a steps array')
        if not 1 <= len(document['steps']) <= 12:
            raise Rejected('Plan length exceeds the public-skill budget')
        sequence = tuple(Instruction.decode(row, layout) for row in document['steps'])
        if sequence[-1].operation != Operation.PARK:
            raise Rejected('Every transaction must end with park')
        positions = dict(observation.locations)
        held = None
        blocker = observation.occupants[goal.destination]
        staged = False
        for index, step in enumerate(sequence):
            if step.operation == Operation.TAKE:
                if held is not None:
                    raise Rejected('A second take cannot occur while holding a block')
                if step.item == goal.item and blocker not in (None, goal.item) and not staged:
                    raise Rejected('The destination blocker must be staged before taking the requested block')
                if step.item not in {goal.item, blocker}:
                    raise Rejected('Plan moves an unrelated object')
                held = step.item
                positions[held] = 'held'
            elif step.operation == Operation.STAGE:
                if held != step.item or step.item != blocker or blocker == goal.item:
                    raise Rejected('Only the actual destination blocker can be staged')
                positions[held] = 'table'
                staged, held = True, None
            elif step.operation == Operation.DEPOSIT:
                if held != step.item or step.item != goal.item or step.destination != goal.destination:
                    raise Rejected('Deposit must satisfy the requested object and zone')
                if any(k != held and location == step.destination for k, location in positions.items()):
                    raise Rejected('Destination remains occupied')
                positions[held], held = step.destination, None
            elif held is not None or index != len(sequence) - 1:
                raise Rejected('Park is only permitted at the end with an empty hand')
        if held is not None or positions[goal.item] != goal.destination:
            raise Rejected('Plan does not reach the requested final state')
        return cls(sequence, frozen(positions), observation, goal)
