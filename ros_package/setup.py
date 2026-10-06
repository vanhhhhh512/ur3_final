"""Build the ROS package from its compact, flattened source tree."""
from pathlib import Path
from shutil import copyfile
import setuptools

identity = 'ur3_perception_llm_control'
package_root = Path(__file__).resolve().parent
assets = package_root / 'assets'
normalized = package_root / 'build' / 'normalized_assets'
groups = ('config', 'launch', 'prompts', 'rviz', 'srdf', 'urdf', 'worlds', 'resource')
normalized_files = {}
for folder in groups:
    normalized_files[folder] = []
    for source in sorted(assets.glob(folder + '__*')):
        target = normalized / folder / source.name[len(folder) + 2:]
        target.parent.mkdir(parents=True, exist_ok=True)
        copyfile(source, target)
        normalized_files[folder].append(target.relative_to(package_root).as_posix())

resources = [('share/ament_index/resource_index/packages', normalized_files['resource']),
             ('share/' + identity, ['package.xml', 'LICENSE', 'NOTICE'])]
for folder in groups[:-1]:
    resources.append(('share/' + identity + '/' + folder, normalized_files[folder]))
commands = {
    'assignment3_runtime': 'console:main',
    'workcell_inspect': 'console:main',
    'grasp_state_cache': 'scene_nodes:relay_main',
    'planning_scene': 'scene_nodes:fixtures_main',
    'zone_markers': 'scene_nodes:fixtures_main',
    'workcell_evidence': 'evidence:main',
}
setuptools.setup(name=identity, version='0.2.0', packages=[identity], package_dir={identity: 'src'}, data_files=resources,
    install_requires=['setuptools'], license='Apache-2.0', zip_safe=False,
    maintainer='Do Viet Anh', maintainer_email='maintainer@example.com',
    description='Immutable camera state and feedback-verified UR3 skill transactions',
    entry_points={'console_scripts': [f'{command} = {identity}.{function}' for command, function in commands.items()]})
