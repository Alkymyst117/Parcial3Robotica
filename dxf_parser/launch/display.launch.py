from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dxf_parser')
    urdf_path = os.path.join(pkg_share, 'urdf', 'scara.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'scara_config.rviz')

    # Leer el contenido del URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Publica robot_description (lee URDF y publica TF a partir de /joint_states)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Nuestro relay que convierte JointTrajectory -> JointState
        Node(
            package='dxf_parser',
            executable='joint_state_relay',
            name='joint_state_relay',
            output='screen'
        ),

        # Visualizador de trayectoria (muestra el recorrido de la herramienta)
        Node(
            package='dxf_parser',
            executable='trajectory_visualizer_node',
            name='trajectory_visualizer_node',
            output='screen'
        ),

        # Lanzar RViz2 con configuración personalizada
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
