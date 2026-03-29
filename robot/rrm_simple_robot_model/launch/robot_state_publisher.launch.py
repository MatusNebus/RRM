import os

import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _build_nodes(context):
    pkg_share = FindPackageShare('rrm_simple_robot_model').find('rrm_simple_robot_model')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    urdf_file = os.path.join(urdf_dir, f'{robot_name}.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}
    rsp = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    return [rsp]


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='advancedArm',
        description='URDF file name without extension (simpleArm|advancedArm)',
    )

    return launch.LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function=_build_nodes),
    ])