"""One CLI for read-only inspection, plan preview, and explicit skill execution."""
import argparse
import json
import sys
import time
from pathlib import Path
import rclpy as ros
from ament_index_python.packages import get_package_share_directory
from .contracts import Goal, Layout, Rejected, configuration
from .language import SkillModel
from .ros_port import EventHub
from .scene_port import ScenePort
from .arm_port import ArmPort
from .grasp_port import GraspPort
from .transaction import WorkcellSession
from .student_task import is_student_arrangement, load_assignment, perform_student_arrangement


PACKAGE_ID = 'llm_va'


def arguments(argv):
    argv = sys.argv[1:] if argv is None else list(argv)
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    flags = argparse.ArgumentParser(description='Camera-derived block manipulation with validated symbolic skills')
    flags.add_argument('--scene', type=Path)
    flags.add_argument('--config', type=Path)
    mode = flags.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='Enable verified task execution; otherwise no motion is sent')
    mode.add_argument('--plan-only', action='store_true', help='Request and validate a skill plan without execution')
    flags.add_argument('--command', help='A single supported placement request; execute without it opens an interactive prompt')
    flags.add_argument('--output', type=Path, help='Write diagnostic or task JSON')
    flags.add_argument('--student-id', help='Override the student ID from config/student_config.yaml for the arrangement task')
    flags.add_argument('--camera-timeout', type=float, default=12.)
    flags.add_argument('--oracle', action='store_true', help='Compare RGB to read-only Gazebo poses for evaluation')
    return flags.parse_args(argv)


def write_result(document, destination):
    rendered = json.dumps(document, indent=2)
    print(rendered, flush=True)
    if destination is not None:
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(rendered + '\n')


def inspect(hub, scene, arm, oracle=False, timeout=12.):
    snapshot = hub.observe(timeout=timeout)
    arm.check_controllers()
    state = scene.fetch()
    document = {'mode': 'read_only', 'camera': snapshot.context(),
                'image_size': list(hub.tracker.geometry.resolution), 'stable_frames': len(hub.window.samples),
                'grasp_states': {name: state[0] for name, state in hub.joint_states.items()},
                'moveit_world': sorted(box.id for box in state.world.collision_objects),
                'moveit_attached': [box.object.id for box in state.robot_state.attached_collision_objects],
                'joint_positions': dict(zip(hub.measured().name, hub.measured().position))}
    if oracle:
        import math
        from .ros_port import gazebo_sample
        comparisons = {item: math.dist(gazebo_sample(item)[:2], snapshot.xy[item]) * 1000 for item in hub.layout.blocks}
        document['camera_oracle_error_mm'] = comparisons
        document['maximum_camera_error_mm'] = max(comparisons.values())
    return document


def run(argv=None):
    opts = arguments(argv)
    root = opts.config or Path(get_package_share_directory(PACKAGE_ID)) / 'config'
    filename = opts.scene or root / 'scene_m12_blocker.yaml'
    layout = Layout.read(filename)
    if opts.command is not None:
        if not is_student_arrangement(opts.command):
            Goal.from_text(opts.command)
        else:
            load_assignment(root / 'student_config.yaml', opts.student_id)
    ros.init(args=[])
    hub = EventHub(layout, root)
    scene = ScenePort(hub)
    arm = ArmPort(hub, scene, configuration(root / 'robot_motion.yaml'))
    code = 0
    try:
        if not opts.execute:
            if opts.plan_only:
                if not opts.command:
                    raise Rejected('--plan-only requires --command')
                view = hub.observe(timeout=opts.camera_timeout)
                accepted, raw = SkillModel(layout).propose(opts.command, Goal.from_text(opts.command), view)
                later = hub.observe(newer_than=view.stamp, timeout=opts.camera_timeout)
                if not view.agrees(later):
                    raise Rejected('Scene changed during plan preview')
                result = {'mode': 'plan_only', 'validated_steps': [step.payload() for step in accepted.steps], 'llm_response': raw}
            else:
                result = inspect(hub, scene, arm, opts.oracle, opts.camera_timeout)
            write_result(result, opts.output)
        else:
            print('[SYSTEM] Robot sorting system started', flush=True)
            grasp = GraspPort(hub)
            session = WorkcellSession(hub, scene, arm, grasp, SkillModel(layout),
                                      configuration(root / 'temporary_position.yaml'), logger=lambda message: print(message, flush=True))
            started = False
            while True:
                text = opts.command or input('Task (quit to stop): ').strip()
                if text.lower() in {'quit', 'exit'}:
                    break
                try:
                    student_task = is_student_arrangement(text)
                    if student_task:
                        assignment = load_assignment(root / 'student_config.yaml', opts.student_id)
                    else:
                        Goal.from_text(text)
                    if not started:
                        arm.check_controllers()
                        grasp.baseline()
                        started = True
                    task_start_ns = time.time_ns()
                    if student_task:
                        document = perform_student_arrangement(session, text, assignment)
                        result_state = document['status']
                    else:
                        result = session.perform(text)
                        document = result.document()
                        result_state = result.state
                    document['task_start_unix_ns'] = task_start_ns
                    document['task_end_unix_ns'] = time.time_ns()
                    write_result(document, opts.output)
                    print('TASK SUCCESS' if result_state == 'success' else 'TASK FAILED', flush=True)
                    code = int(result_state != 'success')
                    if session.poisoned or opts.command or result_state != 'success':
                        break
                except Rejected as error:
                    write_result({'status': 'rejected_before_execution', 'error': str(error)}, opts.output)
                    code = 1
                    if opts.command:
                        break
    except (Rejected, KeyboardInterrupt, EOFError) as error:
        if isinstance(error, Rejected):
            write_result({'status': 'failed', 'error': str(error)}, opts.output)
            code = 1
    finally:
        hub.destroy_node()
        if ros.ok():
            ros.shutdown()
    return code


def main():
    try:
        raise SystemExit(run())
    except (Rejected, OSError, ValueError) as error:
        print(f'STARTUP FAILED: {error}', file=sys.stderr)
        raise SystemExit(1)
