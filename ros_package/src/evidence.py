"""Independent recording/export tools with explicit time and evidence boundaries."""
import argparse
import bisect
import csv
import json
import math
import sqlite3
import time
from pathlib import Path
from .contracts import Rejected


def export_bag(folder, output, task_start_ns):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .ros_port import ARM_AXES, FINGERS, stamp_seconds
    files = sorted(Path(folder).glob('*.db3'))
    if not files:
        raise Rejected('No SQLite rosbag segments found')
    clocks, fingers, bodies, changes = [], [], [], []
    count = 0
    for segment in files:
        with sqlite3.connect(f'file:{segment}?mode=ro', uri=True) as database:
            topics = {ident: (name, get_message(kind)) for ident, name, kind in database.execute('SELECT id,name,type FROM topics')
                      if name in ('/clock', '/joint_states', '/world/empty/dynamic_pose/info') or name.startswith('/m7/grasp_cache/')}
            for ident, receipt, data in database.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
                count += 1
                if ident not in topics:
                    continue
                name, cls = topics[ident]
                sample = deserialize_message(data, cls)
                if name == '/clock':
                    clocks.append((receipt, stamp_seconds(sample.clock)))
                elif name == '/joint_states':
                    fingers.append((receipt, dict(zip(sample.name, sample.position))))
                elif name.startswith('/m7/grasp_cache/'):
                    changes.append({'receive_ns': receipt, 'object': name.split('/')[3], 'state': sample.data})
                else:
                    for transform in sample.transforms:
                        if transform.child_frame_id.endswith('_cube'):
                            t, q = transform.transform.translation, transform.transform.rotation
                            bodies.append((receipt, transform.child_frame_id, (t.x, t.y, t.z, q.x, q.y, q.z, q.w)))
    clocks.sort(); fingers.sort(); bodies.sort(key=lambda row: row[0]); changes.sort(key=lambda row: row['receive_ns'])
    if not clocks or not fingers or not bodies:
        raise Rejected('Bag lacks clock, joint feedback or real Gazebo cube poses')
    clock_keys, joint_keys = [x[0] for x in clocks], [x[0] for x in fingers]
    def simulation(receipt):
        index = min(len(clocks)-1, max(0, bisect.bisect_right(clock_keys, receipt)-1))
        return clocks[index][1]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    written, dropped, max_gap = 0, 0, 0.
    with (output / 'object_joint.csv').open('w', newline='') as destination:
        writer = csv.writer(destination)
        writer.writerow(['receive_ns', 'sim_s', 'object', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'joint_alignment_ms', *ARM_AXES, *FINGERS])
        for receipt, name, coordinates in bodies:
            slot = bisect.bisect_left(joint_keys, receipt)
            neighbours = [x for x in (slot-1, slot) if 0 <= x < len(joint_keys)]
            chosen = min(neighbours, key=lambda x: abs(joint_keys[x]-receipt))
            delta = abs(joint_keys[chosen]-receipt) / 1e6
            values = fingers[chosen][1]
            if delta > 30 or not set(ARM_AXES + FINGERS).issubset(values):
                dropped += 1
                continue
            max_gap = max(delta, max_gap)
            writer.writerow([receipt, simulation(receipt), name, *coordinates, delta, *(values[n] for n in ARM_AXES + FINGERS)])
            written += 1
    task_events = [dict(row, sim_s=simulation(row['receive_ns'])) for row in changes if row['receive_ns'] >= task_start_ns]
    report = {'bag_messages_scanned': count, 'pose_rows': written, 'alignment_dropped': dropped,
              'maximum_alignment_ms': max_gap, 'task_event_cutoff_ns': task_start_ns,
              'startup_events_excluded': sum(row['receive_ns'] < task_start_ns for row in changes),
              'task_joint_events': task_events,
              'sim_span_s': clocks[-1][1] - clocks[0][1], 'receive_span_s': (clocks[-1][0]-clocks[0][0]) / 1e9}
    (output / 'export_summary.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def record_camera(output, duration):
    import fractions
    import av
    import rclpy as ros
    from ament_index_python.packages import get_package_share_directory
    from .contracts import Layout
    from .ros_port import EventHub
    folder = Path(get_package_share_directory('llm')) / 'config'
    layout = Layout.read(folder / 'scene_m12_blocker.yaml')
    ros.init(args=[])
    hub = EventHub(layout, folder, name='bai03_camera_recorder')
    encoded = av.open(str(output), mode='w')
    width, height = hub.tracker.geometry.resolution
    stream = encoded.add_stream('libvpx', rate=10)
    stream.width, stream.height, stream.pix_fmt = width, height, 'yuv420p'
    stream.options = {'deadline': 'realtime', 'cpu-used': '8'}
    stream.time_base = fractions.Fraction(1, 1000)
    begin, last, frames = time.monotonic(), -1., 0
    timestamps = []
    try:
        while ros.ok() and time.monotonic() - begin < duration:
            ros.spin_once(hub, timeout_sec=.05)
            if not hub.window.samples or hub.window.samples[-1][0] <= last or hub.last_frame is None:
                continue
            last = hub.window.samples[-1][0]
            elapsed = time.monotonic() - begin
            frame = av.VideoFrame.from_ndarray(hub.last_frame, format='rgb24')
            frame.pts, frame.time_base = round(elapsed * 1000), fractions.Fraction(1, 1000)
            for packet in stream.encode(frame):
                encoded.mux(packet)
            timestamps.append({'frame': frames, 'wall_unix_s': time.time(), 'sim_s': last, 'pts_ms': frame.pts})
            frames += 1
    except KeyboardInterrupt:
        pass
    finally:
        for packet in stream.encode():
            encoded.mux(packet)
        encoded.close()
        hub.destroy_node()
        if ros.ok():
            ros.shutdown()
        Path(str(output) + '.json').write_text(json.dumps({'frames': frames, 'wall_duration_s': time.monotonic()-begin, 'timestamps': timestamps}, indent=2))
    if frames == 0:
        raise Rejected('No valid camera frames were recorded')
    return {'frames': frames, 'video': str(output)}


def main():
    parser = argparse.ArgumentParser(description='Read-only Gazebo evidence tools; no actuator calls')
    commands = parser.add_subparsers(dest='mode', required=True)
    bag = commands.add_parser('export')
    bag.add_argument('bag', type=Path)
    bag.add_argument('output', type=Path)
    bag.add_argument('--task-start-ns', type=int, required=True, help='Exclude startup attachment transitions before the actual task')
    camera = commands.add_parser('record-rgb')
    camera.add_argument('output', type=Path)
    camera.add_argument('--duration', type=float, default=120.)
    opts = parser.parse_args()
    if opts.mode == 'export':
        result = export_bag(opts.bag, opts.output, opts.task_start_ns)
    else:
        if not 0 < opts.duration <= 3600:
            parser.error('duration must be within (0,3600] seconds')
        result = record_camera(opts.output, opts.duration)
    print(json.dumps(result, indent=2))
