from launch import LaunchDescription, SomeSubstitutionsType
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import List


def get_shared_file_substitution(package_name: str, *path: List[str]) -> SomeSubstitutionsType:
    """Return a Substitution that resolves to the path to the given shared file."""
    return PathJoinSubstitution([
        FindPackageShare(package=package_name),
        *path,
    ])


def generate_launch_description():

    config = get_shared_file_substitution('joy_teleop', 'config', 'joy_teleop.yaml')

    return LaunchDescription([
        Node(
            package='joy_teleop',
            executable='joy_teleop_node',
            name='joy_teleop_node',
            parameters=[
                config
            ],
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                config,
            ],
            output='screen',
        ),
    ])
