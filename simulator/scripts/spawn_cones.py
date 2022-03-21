#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import random
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point
from gen_racetrack import load_wall, get_earth_radius_at_latitude, convert_points, trajectory_interpolate
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, Parameter, SetParametersResult
from typing import List, Tuple
from ament_index_python import get_package_share_directory
import numpy as np
import tf_transformations


class ConeSpawner(Node):

    def __init__(self):
        super().__init__('cone_spawner')

        self.add_on_set_parameters_callback(self.reconfigure_callback)

        self.declare_parameter(
            name='distance_between_cones',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.00,
                    to_value=float('inf'),
                    step=0.0,
                )],
                description='distance between two consecutive cones [m]',
            ),
        )

        left_wall_gps = load_wall(
            get_package_share_directory('simulator') + '/models/purdue_racetrack/gps_data/purdue_left.csv'
        )
        right_wall_gps = load_wall(
            get_package_share_directory('simulator') + '/models/purdue_racetrack/gps_data/purdue_right.csv'
        )
        base_point_gps_ = [-86.945105, 40.437265, 0.0]

        radius_north_, radius_east_ = get_earth_radius_at_latitude(base_point_gps_[1])

        self.left_wall_xyz = convert_points(
            points_gps=left_wall_gps,
            base_point_gps=base_point_gps_,
            radius_north=radius_north_,
            radius_east=radius_east_,
            num_points=None,
            visualize=False,
        )

        self.right_wall_xyz = convert_points(
            points_gps=right_wall_gps,
            base_point_gps=base_point_gps_,
            radius_north=radius_north_,
            radius_east=radius_east_,
            num_points=None,
            visualize=False,
        )

        self.spawned_cones_id = []

        # self.get_parameter('steering_angle.min').value

        self.gazebo_spawner_cli = self.create_client(SpawnEntity, 'spawn_entity')

        while not self.gazebo_spawner_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, trying again...')
        self.get_logger().info('Connected to the service SpawnEntity')

        with open(get_package_share_directory('simulator') + '/models/cone/model.sdf', 'r') as file:
            self.cone_xml = file.read()

        # Define request and unchanging parameters
        self.spawn_entity_req = SpawnEntity.Request()
        # self.spawn_entity_req.xml = self.cone_xml
        self.spawn_entity_req.reference_frame = 'map'
        self.spawn_entity_req.robot_namespace = 'cones'
        self.spawn_entity_req.name = 'cones'
        self.spawn_entity_req.initial_pose.position.x = 0.0
        self.spawn_entity_req.initial_pose.position.y = 0.0
        self.spawn_entity_req.initial_pose.position.z = 0.0
        self.spawn_entity_req.initial_pose.orientation.x = 0.0
        self.spawn_entity_req.initial_pose.orientation.y = 0.0
        self.spawn_entity_req.initial_pose.orientation.z = 0.0
        self.spawn_entity_req.initial_pose.orientation.w = 0.0

        left_cones_pose = self.get_cones_pose(5.0, self.left_wall_xyz)
        self.spawn_cones(left_cones_pose)

    def get_cones_pose(self, distance_between_cones: float, border_points: np.array) -> List[Tuple[Point, float]]:
        border_length = np.sum(np.sqrt(
            np.sum(np.diff(border_points, axis=0) ** 2, axis=1)
        ))

        number_of_cones = round(border_length / distance_between_cones)

        cones_position = trajectory_interpolate(border_points, int_size=number_of_cones)

        cones_poses = []
        for i in range(len(cones_position)):
            cone_position = Point()
            cone_position.x = cones_position[i][0]
            cone_position.y = cones_position[i][1]
            cone_position.z = 0.0

            yaw = math.atan2(
                cones_position[(i + 1) % len(cones_position)][1] - cones_position[i - 1][1],
                cones_position[(i + 1) % len(cones_position)][0] - cones_position[i - 1][0]
            )

            cones_poses.append((cone_position, yaw))
        return cones_poses

    def spawn_cones(self, cones_pose: List[Tuple[Point, float]]) -> None:

        cones_xml = '''<?xml version="1.0"?>
<?xml-model href="http://sdformat.org/schemas/root.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<sdf version="1.7">
    <model name="test_track"> '''

        for i, (position, yaw) in enumerate(cones_pose):
            cones_xml += f'''       <include>
        <uri>model://cone</uri>
        <pose>{position.x:.3f} {position.y:.3f} {position.z:.3f} 0 0 {yaw}</pose>
        <name>cone_{i}</name>
    </include>'''

        cones_xml += '''    </model>
</sdf>'''

        self.spawn_entity_req.xml = cones_xml
        self.gazebo_spawner_cli.call_async(self.spawn_entity_req)

    def reconfigure_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        """Called when there is a request to set one or multiple parameters
        Called even for the initial parameter declaration (if registered before the declaration).
        Registered using self.add_on_set_parameters_callback(self.reconfigure_callback).
        Called for each parameter separately (i.e. len(parameters) == 1),
        unless multiple parameters are set using set_parameters_atomically (then len(parameters) >= 1).
        Before this callback is called, parameters' values are validated against their specified constraints (if any).
        If type or constraints validation fails, this callback will not be called at all.
        If this callback returns SetParametersResult(successful=False), the values will not be set.
        """

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    cone_spawner = ConeSpawner()

    rclpy.spin(cone_spawner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cone_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
