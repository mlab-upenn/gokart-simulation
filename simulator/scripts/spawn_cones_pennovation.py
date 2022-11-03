#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Point
from gen_racetrack import load_wall, get_earth_radius_at_latitude, convert_points, trajectory_interpolate
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, Parameter, SetParametersResult
from typing import List, Tuple
from ament_index_python import get_package_share_directory
import numpy as np


class ConeSpawner(Node):

    def __init__(self):
        super().__init__('cone_spawner')

        self.declare_parameter(
            name='distance_between_cones',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[FloatingPointRange(
                    from_value=0.00,
                    to_value=float(100),
                    step=0.0,
                )],
                description='distance between two consecutive cones [m]',
            ),
            value=5.0,
        )

        self.declare_parameter(
            name='gnss_data_paths',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='absolute paths to gnss .csv data',
            ),
            value=[
                get_package_share_directory('simulator')
                + '/models/pennovation_track/gps_data/left.csv',
                get_package_share_directory('simulator')
                + '/models/pennovation_track/gps_data/right.csv',
            ],
        )

        self.declare_parameter(
            name='gnss_origin_point',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='array of three numbers [longitude, latitude, elevation], this point will be '
                            'transformed to the origin [0,0,0] in the cartesian coordinate system in Gazebo simulation',
            ),
            value=[-75.19913, 39.94117, 0.0],
        )

        self.add_on_set_parameters_callback(self.reconfigure_callback)

        self.gazebo_spawner_cli = self.create_client(
            srv_type=SpawnEntity,
            srv_name='spawn_entity',
        )
        self.gazebo_deleter_cli = self.create_client(
            srv_type=DeleteEntity,
            srv_name='delete_entity',
        )

        while not self.gazebo_spawner_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn_entity service not available, trying again...')
        self.get_logger().info('connected to the service spawn_entity')
        while not self.gazebo_deleter_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('delete_entity service not available, trying again...')
        self.get_logger().info('connected to the service delete_entity')

        # init spawn request
        self.spawn_entity_req = SpawnEntity.Request()
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

        # init delete request
        self.delete_entity_req = DeleteEntity.Request()
        self.delete_entity_req.name = 'cones'

        # parameters (automatically updated via self.update_parameters())
        self.gnss_data_paths = []
        self.distance_between_cones = 0.0
        self.gnss_origin_point = [0, 0, 0]

        self.update_parameters([
            self.get_parameter('gnss_data_paths'),
            self.get_parameter('distance_between_cones'),
            self.get_parameter('gnss_origin_point'),
        ])

        self.spawn_cones()

    def update_parameters(self, parameters: List[Parameter]) -> None:
        # validate parameters
        for param in parameters:
            if param.name == 'gnss_data_paths':
                self.gnss_data_paths = param.value
            elif param.name == 'distance_between_cones':
                self.distance_between_cones = param.value
            elif param.name == 'gnss_origin_point':
                self.gnss_origin_point = param.value

    def spawn_cones(self):
        self.get_logger().info(
            f'spawning cones with dist = {self.distance_between_cones}'
        )

        self.radius_north_, self.radius_east_ = get_earth_radius_at_latitude(
            latitude=self.gnss_origin_point[1],
        )

        all_cones = []

        for path in self.gnss_data_paths:
            loaded_gnss_points = load_wall(path)

            points_cartesian = convert_points(
                points_gps=loaded_gnss_points,
                base_point_gps=self.gnss_origin_point,
                radius_north=self.radius_north_,
                radius_east=self.radius_east_,
                num_points=None,
                visualize=False,
            )

            cones = self.get_cones_poses(
                self.distance_between_cones,
                points_cartesian
            )

            set_name = path.split(sep='/')[-1].split(sep='.')[0]

            all_cones.append((set_name, cones))

        self.send_spawn_request(all_cones)

        self.get_logger().info(
            f'spawn request with dist = {self.distance_between_cones} finished'
        )

    def get_cones_poses(self, distance_between_cones: float, border_points: np.array) -> List[Tuple[Point, float]]:
        border_length = np.sum(np.sqrt(
            np.sum(np.diff(border_points, axis=0) ** 2, axis=1)
        ))

        number_of_cones = round(border_length / distance_between_cones)

        cones_position = trajectory_interpolate(
            border_points, int_size=number_of_cones
        )

        cones = []

        for i in range(len(cones_position)):
            cone_position = Point()
            cone_position.x = cones_position[i][0]
            cone_position.y = cones_position[i][1]
            cone_position.z = 0.0

            yaw = math.atan2(
                # y
                cones_position[(i + 1) % len(cones_position)][1]
                - cones_position[i - 1][1],
                # x
                cones_position[(i + 1) % len(cones_position)][0]
                - cones_position[i - 1][0]
            )

            cones.append((cone_position, yaw))

        return cones

    def send_spawn_request(self, cones_all: List[Tuple[str, List[Tuple[Point, float]]]]) -> None:

        xml = '<?xml version="1.0"?>' \
              '<?xml-model href="http://sdformat.org/schemas/root.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>' \
              '<sdf version="1.7">' \
              '<model name="cones">'

        for name, cones in cones_all:
            for idx, (position, yaw) in enumerate(cones):
                xml += '<include>' \
                       '<uri>model://cone</uri>' \
                       f'<pose>{position.x:.3f} {position.y:.3f} {position.z:.3f} 0 0 {yaw}</pose>' \
                       f'<name>cone_{name}_{idx}</name>' \
                       '</include>'

        xml += '</model>' \
               '</sdf>'

        self.spawn_entity_req.xml = xml

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

        self.gazebo_deleter_cli.call_async(self.delete_entity_req)
        self.update_parameters(parameters)
        self.spawn_cones()

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    cone_spawner = ConeSpawner()
    try:
        rclpy.spin(cone_spawner)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
