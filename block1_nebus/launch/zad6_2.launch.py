import os

import launch
import launch.actions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def _make_sim_include(robot_name: str):
    rrm_sim_share = FindPackageShare('rrm_sim').find('rrm_sim')
    sim_launch = os.path.join(rrm_sim_share, 'launch', 'rrm_sim.launch.xml')

    return launch.actions.IncludeLaunchDescription(
        XMLLaunchDescriptionSource(sim_launch),
        launch_arguments={'robot_name': robot_name}.items(),
    )


def _build_launch_description(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)

    if robot_name not in ('simpleArm', 'manipulator'):
        raise RuntimeError(
            f"Unsupported robot_name '{robot_name}'. Allowed values are: simpleArm, manipulator."
        )

    nodes = []

    if robot_name == 'simpleArm':
        nodes.append(_make_sim_include('simpleArm'))
        nodes.append(
            launch_ros.actions.Node(
                package='block1_nebus',
                executable='ik_solver_node',
                name='ik_solver_node',
                output='screen',
            )
        )
        nodes.append(
            launch_ros.actions.Node(
                package='block1_nebus',
                executable='motion_manager_node',
                name='motion_manager_node',
                output='screen',
                parameters=[{'max_velocity': 1.0}],
            )
        )
    else:
        nodes.append(_make_sim_include('manipulator'))
        nodes.append(
            launch_ros.actions.Node(
                package='block1_nebus',
                executable='ik_solver_6dof_node',
                name='ik_solver_6dof_node',
                output='screen',
            )
        )
        nodes.append(
            launch_ros.actions.Node(
                package='block1_nebus',
                executable='motion_manager_6dof_node',
                name='motion_manager_6dof_node',
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='simpleArm',
        description='Allowed values: simpleArm, manipulator',
    )

    return launch.LaunchDescription([
        robot_name_arg,
        OpaqueFunction(function=_build_launch_description),
    ])