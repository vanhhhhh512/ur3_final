"""Rank free table positions from measured occupancy, not spawn coordinates."""
import dataclasses as dc
import math
import numpy as np
from .contracts import Rectangle, Rejected


@dc.dataclass(frozen=True)
class Slot:
    xy: tuple
    height: float
    clearance: float


class TableSearch:
    def __init__(self, layout, margins):
        self.layout, self.rules = layout, margins
        if any(value <= 0 or not math.isfinite(value) for value in margins.values()):
            raise Rejected('Temporary-position margins must be positive finite distances')

    def candidates(self, item, observation):
        extent = self.layout.dimensions[item]
        inset = max(extent[:2]) / 2 + self.rules['table_edge_clearance']
        low = np.subtract(self.layout.support.centre, np.array(self.layout.support.extent) / 2 - inset)
        high = np.add(self.layout.support.centre, np.array(self.layout.support.extent) / 2 - inset)
        coords = [np.arange(low[k], high[k] + 1e-9, self.rules['grid_spacing']) for k in range(2)]
        obstacles = [(Rectangle(observation.xy[name], self.layout.dimensions[name][:2]), self.rules['cube_clearance']) for name in observation.xy if name != item]
        obstacles.extend((area, self.rules['zone_clearance']) for area in self.layout.areas.values())
        base = self.layout.data['pedestal']
        obstacles.append((Rectangle(tuple(base['pose'][k] for k in 'xy'), tuple(base['size'][k] for k in 'xy')), self.rules['fixed_obstacle_clearance']))
        choices = []
        for x in coords[0]:
            for y in coords[1]:
                point = (float(x), float(y))
                if any(box.overlaps_box(point, extent, padding) for box, padding in obstacles):
                    continue
                gap = min(math.hypot(*(max(0, abs(point[k] - box.centre[k]) - (extent[k] + box.extent[k]) / 2) for k in range(2))) for box, _ in obstacles)
                choices.append(Slot(point, self.layout.centre_height(item), gap))
        return sorted(choices, key=lambda candidate: (math.dist(candidate.xy, observation.xy[item]), -candidate.clearance))

    def choose(self, item, observation, feasible):
        for proposal in self.candidates(item, observation):
            if feasible(proposal):
                return proposal
        raise Rejected('No free collision-checkable temporary placement exists')
