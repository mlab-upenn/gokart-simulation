import os

from typing import Optional, List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext, SomeActionsType, SomeSubstitutionsType, Condition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events.process import ProcessExited
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.conditions.evaluate_condition_expression_impl import evaluate_condition_expression as expr_to_bool
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def get_shared_file(package_name: str, *path: List[str]) -> str:
    """Return the path to the given shared file."""
    return os.path.join(
        get_package_share_directory(package_name),
        *path,
    )


def SharedFile(package_name: str, *path: List[str]) -> SomeSubstitutionsType:
    """Return a Substitution that resolves to the path to the given shared file."""
    return PathJoinSubstitution([
        FindPackageShare(package=package_name),
        *path,
    ])


def generate_launch_description():

    # Constants for paths to different files and folders
    package_name = 'simulator'
    robot_name_in_model = 'gokart'
    path_desc = f'Must be either an absolute path or a relative path within the {package_name} package\'s share dir.'
    default_config = 'config/simulation.yaml'
    default_world = 'worlds/test_world.world'
    default_urdf_model = 'urdf/gokart/main.urdf'
    default_rviz_config = 'rviz/demo.rviz'

    # Pose where we want to spawn the robot
    # (This is the starting line of the Purdue Racetrack)
    spawn_x_val = '56.39758568'
    spawn_y_val = '47.69298874'
    # It is better to spawn the car a bit above the ground and let it fall
    # to avoid weird simulation bugs due to the floating point numeric errors.
    spawn_z_val = '0.17'
    spawn_yaw_val = '-0.57'

    # Launch configuration variables
    config_abs_path = SharedFile(
        package_name, LaunchConfiguration('config')
    )
    world_abs_path = SharedFile(
        package_name, LaunchConfiguration('world')
    )
    urdf_model_abs_path = SharedFile(
        package_name, LaunchConfiguration('urdf_model')
    )
    rviz_config_abs_path = SharedFile(
        package_name, LaunchConfiguration('rviz_config')
    )
    headless = LaunchConfiguration('headless')
    start_gzclient = LaunchConfiguration('start_gzclient')
    start_map_sender = LaunchConfiguration('start_map_sender')
    start_rviz = LaunchConfiguration('start_rviz')
    start_joint_state_publisher_legacy = LaunchConfiguration(
        'start_joint_state_publisher_legacy'
    )
    teleop = LaunchConfiguration('teleop')

    # Declare the launch arguments
    declare_config_la = DeclareLaunchArgument(
        name='config',
        default_value=default_config,
        description=f'Path to the parameters file that will be added to all nodes (except Gazebo ones). {path_desc}',
    )

    declare_world_la = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description=f'Path to the world model file to load. {path_desc}',
    )

    declare_urdf_model_la = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model,
        description=f'Path to the robot URDF file (xacro supported). {path_desc}',
    )

    declare_rviz_config_la = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config,
        description=f'Path to the RViz config file to use. {path_desc}',
    )

    declare_headless_la = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='If true, no GUI tools (gzclient, rviz, teleop) are started.',
    )

    declare_start_gzclient_la = DeclareLaunchArgument(
        name='start_gzclient',
        default_value='true',
        description='Whether to start Gazebo GUI (gzclient).',
    )

    declare_start_map_sender_la = DeclareLaunchArgument(
        name='start_map_sender',
        default_value='true',
        description='Whether to run send_map.py (sends map viz for RViz).',
    )

    declare_start_rviz_la = DeclareLaunchArgument(
        name='start_rviz',
        default_value='False',
        description='Whether to start RViz.',
    )

    declare_teleop_la = DeclareLaunchArgument(
        name='teleop',
        default_value='none',
        description='Start teleop tool. Allowed values: key, joy, none.',
    )

    declare_start_joint_state_publisher_legacy_la = DeclareLaunchArgument(
        name='start_joint_state_publisher_legacy',
        default_value='false',
        description=(
            'Whether to start joint_state_publisher. Not needed anymore.'
            ' simulator_gazebo_plugin publishes joint states by default (at the correct frequency).'
            ' Do NOT this use unless you set simulator_gazebo_plugin publishJointStates to false.'
        ),
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            config_abs_path,
            {
                'robot_description': Command(['xacro ', urdf_model_abs_path]),
            },
        ],
    )

    # Publish the joint states of the robot
    # (needed only in case the simulation does not publish the joint state)
    joint_state_publisher_node = Node(
        condition=IfCondition(start_joint_state_publisher_legacy),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            config_abs_path,
        ],
    )

    # Start Gazebo server
    gazebo_server_ld = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=SharedFile(
                'gazebo_ros', 'launch', 'gzserver.launch.py'
            ),
        ),
        launch_arguments=[
            ('server_required', 'true'),
            ('world', world_abs_path),
            # ('pause', 'true'),
            # ('gdb', 'false'),
            ('extra_gazebo_args', '--verbose'),
        ],
    )

    def start_gzclient_predicate(context: LaunchContext) -> bool:
        start_gzclient_ = expr_to_bool(context, [start_gzclient])
        headless_ = expr_to_bool(context, [headless])
        return start_gzclient_ and not headless_

    # Start Gazebo client
    gazebo_client_ld = IncludeLaunchDescription(
        condition=Condition(predicate=start_gzclient_predicate),
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=SharedFile(
                'gazebo_ros', 'launch', 'gzclient.launch.py'
            ),
        ),
        launch_arguments=[
            # allow user to close the Gazebo GUI without stopping the simulation
            ('gui_required', 'false'),
        ],
    )

    # Start map sender
    map_sender_node = Node(
        condition=IfCondition(start_map_sender),
        package='simulator',
        executable='send_map.py',
        name='map_sender',
        parameters=[
            config_abs_path,
        ],
        output='screen',
    )

    def start_rviz_predicate(context: LaunchContext) -> bool:
        start_rviz_ = expr_to_bool(context, [start_rviz])
        headless_ = expr_to_bool(context, [headless])
        return start_rviz_ and not headless_

    # Start RViz
    delayed_rviz_node = TimerAction(
        condition=Condition(predicate=start_rviz_predicate),
        period=8.0,  # 8 seconds should be enough for the Gazebo to fully start
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_abs_path],
                parameters=[
                    config_abs_path,
                ],
            ),
        ],
    )

    # teleop
    key_teleop_node = Node(
        condition=LaunchConfigurationEquals('teleop', 'key'),
        package='key_teleop',
        executable='key_teleop_node',
        name='key_teleop_node',
        parameters=[
            config_abs_path,
        ],
        output='screen',
    )
    joy_teleop_ld = IncludeLaunchDescription(
        condition=LaunchConfigurationEquals('teleop', 'joy'),
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=SharedFile(
                'joy_teleop', 'launch', 'joy_teleop.launch.py'
            ),
        ),
        launch_arguments=[
            # TODO: allow passing additional config files
            # ('additional_config', 'TODO'),
        ],
    )

    # Add the robot to the simulation
    # using the spawn_entity.py script
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic', 'robot_description',
            '-x', spawn_x_val,
            '-y', spawn_y_val,
            '-z', spawn_z_val,
            '-Y', spawn_yaw_val,
        ],
        parameters=[
            config_abs_path,
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(declare_config_la)
    ld.add_action(declare_world_la)
    ld.add_action(declare_urdf_model_la)
    ld.add_action(declare_rviz_config_la)
    ld.add_action(declare_headless_la)
    ld.add_action(declare_start_gzclient_la)
    ld.add_action(declare_start_map_sender_la)
    ld.add_action(declare_start_rviz_la)
    ld.add_action(declare_start_joint_state_publisher_legacy_la)
    ld.add_action(declare_teleop_la)

    # Add any actions
    ld.add_action(gazebo_server_ld)
    ld.add_action(gazebo_client_ld)
    ld.add_action(map_sender_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(delayed_rviz_node)
    ld.add_action(key_teleop_node)
    ld.add_action(joy_teleop_ld)

    def handle_exit(event: ProcessExited, context: LaunchContext) -> Optional[SomeActionsType]:
        if context.is_shutdown or event.returncode == 0:
            return None
        return [
            LogInfo(
                msg=f'Shutting down because of the process {event.process_name} exited with an error code {event.returncode}'
            ),
            Shutdown(),
        ]

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            # target_action=some_action,
            on_exit=handle_exit,
        ),
    ))

    return ld
