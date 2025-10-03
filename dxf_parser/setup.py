from setuptools import setup

package_name = 'dxf_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/scara.urdf']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/scara_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Parser DXF + IK + planner + fk + rviz helper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dxf_parser_node = dxf_parser.dxf_parser_node:main',
            'ik_node = dxf_parser.ik_node:main',
            'trajectory_planner_node = dxf_parser.trajectory_planner_node:main',
            'fk_node = dxf_parser.fk_node:main',
            'joint_state_relay = dxf_parser.joint_state_relay:main',
            'trajectory_visualizer_node = dxf_parser.trajectory_visualizer_node:main',
        ],
    },
)
