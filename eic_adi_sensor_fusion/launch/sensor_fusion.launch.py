from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory("eic_adi_sensor_fusion"))
    
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(ekf_config, 'eic_bot.yaml')],
    )

    imu_filter_config = os.path.join(
        get_package_share_directory('eic_adi_sensor_fusion'))
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(imu_filter_config, 'imu_filter.yaml')],
        remappings=[("imu/data_raw", "imu")]
    )

    return LaunchDescription([
        ekf_node,
        imu_filter_node,
    ])
