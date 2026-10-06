"""Launch composition for the official UR simulator and this workcell's adapters."""
import os
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription as Description
from launch.actions import DeclareLaunchArgument as Argument, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Option
from launch_ros.actions import Node as Process
from .contracts import Layout, configuration
from .sdf_factory import spawn_specs


PKG = 'ur3_perception_llm_control'


def package_file(package, *parts):
    return str(Path(share(package)).joinpath(*parts))


def process(package, executable, **options):
    return Process(package=package, executable=executable, output='screen', **options)


def include(package, file, values):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(package_file(package, 'launch', file)), launch_arguments=values.items())


def enabled(context, option):
    return Option(option).perform(context).lower() == 'true'


def moveit_processes(context):
    os.environ['UR3_LLM_SCENE_CONFIG'] = Option('scene_config').perform(context)
    model = Option('ur_type').perform(context)
    mappings = {'name': 'ur', 'ur_type': model, 'sim_ignition': 'false'}
    for argument, filename in {'joint_limit_params': 'joint_limits', 'kinematics_params': 'default_kinematics', 'physical_params': 'physical_parameters', 'visual_params': 'visual_parameters'}.items():
        mappings[argument] = package_file('ur_description', 'config', model, filename + '.yaml')
    for key in ('safety_limits', 'safety_pos_margin', 'safety_k_position'):
        mappings[key] = Option(key).perform(context)
    urdf = xacro.process_file(package_file(PKG, 'urdf', 'mounted_ur.urdf.xacro'), mappings=mappings).toxml()
    semantic = xacro.process_file(package_file(PKG, 'srdf', 'ur_with_gripper.srdf.xacro'), mappings={'name': 'ur', 'prefix': ''}).toxml()
    cfg = lambda filename: configuration(package_file('ur_moveit_config', 'config', filename))
    pipeline = cfg('ompl_planning.yaml')
    pipeline.update({'planning_plugin': 'ompl_interface/OMPLPlanner', 'start_state_max_bounds_error': .1,
                     'request_adapters': ' '.join('default_planner_request_adapters/' + name for name in ('AddTimeOptimalParameterization', 'FixWorkspaceBounds', 'FixStartStateBounds', 'FixStartStateCollision', 'FixStartStatePathConstraints'))})
    controllers = cfg('controllers.yaml')
    for controller in ('scaled_joint_trajectory_controller', 'joint_trajectory_controller'):
        controllers[controller]['default'] = controller == 'joint_trajectory_controller'
    parameters = [{'robot_description': urdf, 'robot_description_semantic': semantic, 'publish_robot_description_semantic': True,
        'robot_description_planning': cfg('joint_limits.yaml'), 'move_group': pipeline,
        'moveit_simple_controller_manager': controllers, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': False, 'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': .5, 'trajectory_execution.allowed_start_tolerance': .01,
        'trajectory_execution.execution_duration_monitoring': False, 'use_sim_time': True,
        **{key: True for key in ('publish_planning_scene', 'publish_geometry_updates', 'publish_state_updates', 'publish_transforms_updates')}},
        package_file('ur_moveit_config', 'config', 'kinematics.yaml')]
    actions = [process('moveit_ros_move_group', 'move_group', parameters=parameters),
               process(PKG, 'planning_scene', parameters=[{'scene_config': Option('scene_config')}])]
    if enabled(context, 'launch_rviz'):
        actions.append(process('rviz2', 'rviz2', parameters=parameters, arguments=['-d', package_file(PKG, 'rviz', 'workcell.rviz')]))
    return actions


def workcell_processes(context):
    filename = Option('scene_config').perform(context)
    layout = Layout.read(filename)
    names_and_specs = spawn_specs(layout)
    queue = []
    for name, serialized, pose in names_and_specs:
        args = ['-string', serialized, '-name', name]
        args.extend(value for key, switch in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), ('-x', '-y', '-z', '-R', '-P', '-Y')) for value in (switch, str(pose.get(key, 0))))
        queue.append(process('ros_gz_sim', 'create', arguments=args, name='create_' + name))
    chain = [RegisterEventHandler(OnProcessExit(target_action=queue[i], on_exit=[queue[i+1]])) for i in range(len(queue)-1)]
    simulation = include('ur_simulation_gz', 'ur_sim_control.launch.py', {
        'ur_type': Option('ur_type'), 'gazebo_gui': Option('gazebo_gui'), 'launch_rviz': 'false',
        'runtime_config_package': PKG, 'controllers_file': 'ur3_controllers.yaml', 'description_package': PKG,
        'description_file': 'mounted_ur.urdf.xacro', 'world_file': package_file(PKG, 'worlds', 'rgb_workcell.sdf')})
    streams = []
    for item in layout.blocks:
        root = '/m7/grasp/' + item
        streams += [root + '/state@std_msgs/msg/String[ignition.msgs.StringMsg',
                    root + '/attach@std_msgs/msg/Empty]ignition.msgs.Empty', root + '/detach@std_msgs/msg/Empty]ignition.msgs.Empty']
    sensor = layout.data['camera']
    image_streams = [sensor['image_topic'] + '@sensor_msgs/msg/Image[ignition.msgs.Image',
                     sensor['camera_info_topic'] + '@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo']
    actions = [simulation, *chain, queue[0],
               TimerAction(period=2., actions=[process('ros_gz_bridge', 'parameter_bridge', name='grasp_bridge', arguments=streams),
                                               process(PKG, 'grasp_state_cache', parameters=[{'scene_config': filename}])]),
               TimerAction(period=3., actions=[process('controller_manager', 'spawner', arguments=['gripper_controller', '-c', '/controller_manager', '--controller-manager-timeout', '30'])]),
               TimerAction(period=5., actions=[process('ros_gz_bridge', 'parameter_bridge', name='rgb_bridge', arguments=image_streams)])]
    if enabled(context, 'launch_moveit'):
        actions.append(include(PKG, 'moveit.launch.py', {'ur_type': Option('ur_type'), 'scene_config': filename,
                                                       'launch_rviz': Option('launch_rviz').perform(context)}))
    if enabled(context, 'start_local_llm'):
        actions.append(OpaqueFunction(function=local_llm_processes))
    if enabled(context, 'execute_demo'):
        actions.append(TimerAction(period=15., actions=[process(PKG, 'assignment3_runtime',
            arguments=['--execute', '--scene', Option('scene_config'), '--command', Option('demo_command'),
                       '--output', Option('task_output')])]))
    return actions


def local_llm_processes(context):
    endpoint = os.environ.get('NINEROUTER_BASE_URL', 'http://127.0.0.1:11434/v1').rstrip('/')
    if endpoint != 'http://127.0.0.1:11434/v1':
        return []
    try:
        with urlopen(endpoint.rsplit('/v1', 1)[0] + '/api/tags', timeout=1.) as response:
            if response.status == 200:
                return []
    except (OSError, URLError):
        pass
    workspace = Path(package_file(PKG)).parents[3]
    executable = workspace / '.local-llm' / 'bin' / 'ollama'
    if not executable.is_file():
        print('[ERROR] Local Ollama is unavailable and the bundled runner was not found', flush=True)
        return []
    environment = {'OLLAMA_HOST': '127.0.0.1:11434', 'OLLAMA_NO_CLOUD': 'true'}
    models = workspace / '.local-llm' / 'models'
    if models.is_dir():
        environment['OLLAMA_MODELS'] = str(models)
    print('[SYSTEM] Starting the local Ollama service', flush=True)
    return [ExecuteProcess(cmd=[str(executable), 'serve'], name='local_ollama', output='screen', additional_env=environment)]


def assemble_launch(mode):
    options = {'ur_type': 'ur3e', 'scene_config': package_file(PKG, 'config', 'scene.yaml'),
               'launch_rviz': 'true', 'safety_limits': 'true', 'safety_pos_margin': '0.15', 'safety_k_position': '20'}
    if mode == 'workcell':
        options.update(gazebo_gui='true', launch_moveit='true', start_local_llm='true', execute_demo='true',
                       demo_command='Put the red cube in Zone B and return home.',
                       task_output='/tmp/ur3_perception_llm_control_last_task.json')
    declared = [Argument(key, default_value=value) for key, value in options.items()]
    if mode == 'workcell':
        declared.extend([Argument('start_local_llm', default_value='true'), Argument('execute_demo', default_value='true'),
                         Argument('demo_command', default_value='Put the red cube in Zone B and return home.'),
                         Argument('task_output', default_value='/tmp/ur3_perception_llm_control_last_task.json')])
    declared.append(SetEnvironmentVariable('UR3_LLM_SCENE_CONFIG', Option('scene_config')))
    declared.append(OpaqueFunction(function=workcell_processes if mode == 'workcell' else moveit_processes))
    return Description(declared)
