"""Install the independent Python runtime alongside attributed physical assets."""
from pathlib import Path
import setuptools

identity = 'ur3_perception_llm_control'
resources = [('share/ament_index/resource_index/packages', ['assets/resource/' + identity]),
             ('share/' + identity, ['package.xml', 'LICENSE', 'NOTICE'])]
for folder in ('config', 'launch', 'prompts', 'srdf', 'urdf', 'rviz', 'worlds'):
    source = Path('assets') / folder
    resources.append(('share/' + identity + '/' + folder,
                      [str(path) for path in sorted(source.glob('*')) if path.is_file()]))
commands = {
    'assignment3_runtime': 'console:main',
    'workcell_inspect': 'console:main',
    'grasp_state_cache': 'scene_nodes:relay_main',
    'planning_scene': 'scene_nodes:fixtures_main',
    'zone_markers': 'scene_nodes:fixtures_main',
    'workcell_evidence': 'evidence:main',
}
setuptools.setup(name=identity, version='0.2.0', packages=setuptools.find_packages(where='src'), package_dir={'': 'src'}, data_files=resources,
    install_requires=['setuptools'], license='Apache-2.0', zip_safe=False,
    maintainer='Do Viet Anh', maintainer_email='maintainer@example.com',
    description='Immutable camera state and feedback-verified UR3 skill transactions',
    entry_points={'console_scripts': [f'{command} = {identity}.{function}' for command, function in commands.items()]})
