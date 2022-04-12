#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, Parameter, SetParametersResult
from typing import List, Tuple
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA


class MapSender(Node):

    def __init__(self):
        super().__init__('map_sender')

        self.map_pub = self.create_publisher(
            msg_type=Marker,
            topic='viz_map',
            qos_profile=1,
        )

        # see https://github.com/ros2/common_interfaces/blob/foxy/visualization_msgs/msg/Marker.msg
        self.map = Marker()
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map.header.frame_id = 'map'
        self.map.ns = 'map'
        self.map.id = 1
        self.map.type = Marker.MESH_RESOURCE
        self.map.action = Marker.ADD

        self.map.pose = Pose()
        self.map.pose.position = Point()
        self.map.pose.position.x = 0.0
        self.map.pose.position.y = 0.0
        self.map.pose.position.z = 0.0
        self.map.pose.orientation = Quaternion()
        self.map.pose.orientation.x = 0.0
        self.map.pose.orientation.y = 0.0
        self.map.pose.orientation.z = 0.0
        self.map.pose.orientation.w = 1.0

        # 1:1
        self.map.scale = Vector3()
        self.map.scale.x = 1.0
        self.map.scale.y = 1.0
        self.map.scale.z = 1.0

        # gray
        self.map.color = ColorRGBA()
        self.map.color.r = 0.67
        self.map.color.g = 0.67
        self.map.color.b = 0.67
        self.map.color.a = 1.0

        # 0 means forever
        self.map.lifetime = Duration()
        self.map.lifetime.sec = 0
        self.map.lifetime.nanosec = 0

        # see https://answers.ros.org/question/290293/what-exactly-does-frame_locked-mean/
        self.map.frame_locked = True

        self.map.mesh_resource = 'package://simulator/models/purdue_racetrack/meshes/track-svg-python.dae'
        self.map.mesh_use_embedded_materials = False

        # TODO: It seems that we need to publish the map periodically.
        #       Otherwise, when you switch between frames, it disappers.
        self.create_timer(1.0, self.timer_callback)

        pass

    def timer_callback(self):
        self.map.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        # self.get_logger().info('map sent')
        pass


def main(args=None):
    rclpy.init(args=args)
    map_sender = MapSender()
    try:
        rclpy.spin(map_sender)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
