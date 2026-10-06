"""Deterministic student-ID assignment and camera-verified arrangement task."""
from dataclasses import dataclass
import re
import unicodedata
from types import MappingProxyType

from .contracts import Rejected, configuration


ZONES = ('zone_a', 'zone_b', 'zone_c')
ASSIGNED_OBJECTS = {'red_cube', 'blue_cube', 'yellow_cube'}


@dataclass(frozen=True)
class StudentAssignment:
    last_two: int
    remainder: int
    zone_to_item: object


def load_assignment(filename, student_id=None):
    """Resolve the configured assignment using the final two ID digits modulo six."""
    config = configuration(filename)
    identity = str(student_id if student_id is not None else config.get('student_id', '')).strip()
    digits = ''.join(character for character in identity if character.isdigit())
    if len(digits) < 2:
        raise Rejected('Student ID must contain at least two digits')
    last_two = int(digits[-2:])
    remainder = last_two % 6
    table = config.get('assignment_by_last_two_mod_6')
    row = table.get(remainder) if isinstance(table, dict) else None
    if not isinstance(row, dict) or set(row) != set(ZONES):
        raise Rejected(f'Student assignment configuration is incomplete for remainder {remainder}')
    assignment = {str(zone): str(item) for zone, item in row.items()}
    if set(assignment.values()) != ASSIGNED_OBJECTS:
        raise Rejected(f'Student assignment {remainder} must assign red, blue and yellow cubes once each')
    return StudentAssignment(last_two, remainder, MappingProxyType(assignment))


def is_student_arrangement(text):
    """Recognize the explicit whole-workcell student-ID task in English or Vietnamese."""
    normalized = unicodedata.normalize('NFKD', str(text).casefold())
    normalized = ''.join(char for char in normalized if not unicodedata.combining(char))
    normalized = re.sub(r'[^a-z0-9]+', ' ', normalized).strip()
    patterns = (
        r'(?:arrange|sort|organize|tidy) (?:all )?(?:the )?(?:objects|cubes|blocks|everything) '
        r'according to (?:my )?(?:student id|student number|id)',
        r'(?:sap xep|phan loai|bo tri) (?:tat ca )?(?:cac )?(?:vat|do vat|khoi|cube|cubes|objects|moi thu|tat ca) '
        r'(?:theo|dua theo) (?:ma so sinh vien|ma sinh vien|mssv|student id)',
        r'(?:sap xep|phan loai|bo tri) (?:theo|dua theo) (?:ma so sinh vien|ma sinh vien|mssv|student id)',
    )
    return any(re.fullmatch(pattern, normalized) for pattern in patterns)


def perform_student_arrangement(session, command, assignment):
    """Arrange the three assigned cubes through the ordinary blocker-safe transaction."""
    completed = []
    session._log(f'[PLANNER] Student assignment selected (P={assignment.remainder})')
    for zone in ZONES:
        session._log(f'[PLANNER] Assignment: {zone} -> {assignment.zone_to_item[zone]}')

    try:
        view = session._new_view()
        session._log('[CAMERA] Camera initialized successfully')
        session._log('[CAMERA] Detecting cubes and zones...')
        session._camera_report(view)
        for zone in ZONES:
            item = assignment.zone_to_item[zone]
            if view.occupants[zone] == item:
                session._log(f'[PLANNER] {zone.replace("_", " ").title()} already contains {item}; skipping movement')
                completed.append({'zone': zone, 'item': item, 'status': 'already_satisfied'})
                continue

            request = f'Put the {item.replace("_cube", "")} cube in {zone.replace("_", " ").title()}.'
            session._log(f'[PLANNER] Applying assignment: {item} to {zone.replace("_", " ").title()}')
            result = session.perform(request)
            completed.append({'zone': zone, 'item': item, 'transaction': result.document()})
            if result.state != 'success':
                return {'command': command, 'status': 'failed', 'assignment_remainder': assignment.remainder,
                        'assignment': dict(assignment.zone_to_item), 'completed_assignments': completed,
                        'error': result.error}
            view = session.observed

        final = session._new_view(after=session.observed.stamp)
        session._camera_report(final)
        wrong = {zone: {'expected': item, 'observed': final.occupants[zone]}
                 for zone, item in assignment.zone_to_item.items() if final.occupants[zone] != item}
        if wrong:
            raise Rejected('Final camera state does not match the student assignment: ' + repr(wrong))

        session._idle_check()
        home = session.arm.settings['home']
        measured = session.hub.measured()
        axes = dict(zip(measured.name, measured.position))
        home_error = max(abs(axes[name] - angle) for name, angle in home.items())
        if home_error > .02:
            session._log('[ACTION] Returning robot to HOME...')
            session.arm.home()
            measured = session.hub.measured()
            axes = dict(zip(measured.name, measured.position))
            home_error = max(abs(axes[name] - angle) for name, angle in home.items())
        if home_error > .02:
            raise Rejected(f'Home feedback exceeds the final tolerance ({home_error:.4f} rad)')
        session._log('[SYSTEM] All student-assigned zones verified by camera')
        session._log('[SYSTEM] Task completed successfully')
        return {'command': command, 'status': 'success', 'assignment_remainder': assignment.remainder,
                'assignment': dict(assignment.zone_to_item), 'completed_assignments': completed,
                'verified_camera': final.context(), 'home_error_rad': home_error}
    except (Rejected, KeyError, TypeError, ValueError) as error:
        session._log('[ERROR] ' + str(error))
        return {'command': command, 'status': 'failed', 'assignment_remainder': assignment.remainder,
                'assignment': dict(assignment.zone_to_item), 'completed_assignments': completed,
                'error': str(error)}
