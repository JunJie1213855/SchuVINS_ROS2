from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('schurvins'))
    return LaunchDescription([
        DeclareLaunchArgument('calib_file', default_value=str(share / 'config/calib/euroc_stereo.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'config/euroc_stereo.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('cam0_topic', default_value='/cam0/image_raw'),
        DeclareLaunchArgument('cam1_topic', default_value='/cam1/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu0'),
        DeclareLaunchArgument('trajectory_file', default_value=''),
        Node(package='schurvins', executable='svo_node', name='svo', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'calib_file': LaunchConfiguration('calib_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'cam0_topic': LaunchConfiguration('cam0_topic'),
                 'cam1_topic': LaunchConfiguration('cam1_topic'),
                 'imu_topic': LaunchConfiguration('imu_topic'),
                 'trajectory_file': ParameterValue(LaunchConfiguration('trajectory_file'), value_type=str),
             }]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/schurvins.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
