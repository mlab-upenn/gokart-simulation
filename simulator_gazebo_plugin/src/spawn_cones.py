#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from gazebo_msgs.srv import SpawnEntity


class ConeSpawner(Node):

    def __init__(self):
        super().__init__('cone_spawner')

        self.gazebo_spawner_cli = self.create_client(SpawnEntity, 'spawn_entity')

        while not self.gazebo_spawner_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_entity_req = SpawnEntity.Request()
        self.get_logger().info('Connected to the service SpawnEntity')

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        with open("/home/tomas/model_editor_models/cone/model.sdf", "r") as file:
            self.cone_xml = file.read()
        self.cone_idx = 0

    def timer_callback(self):
        self.spawn_entity_req.name = "cone" + str(self.cone_idx)
        self.spawn_entity_req.xml = self.cone_xml
        self.spawn_entity_req.reference_frame = "map"
        self.spawn_entity_req.robot_namespace = "cones"
        self.spawn_entity_req.initial_pose.position.x = random.random() * 20.0 - 10.0
        self.spawn_entity_req.initial_pose.position.y = random.random() * 20.0 - 10.0
        self.spawn_entity_req.initial_pose.position.z = 0.01
        self.spawn_entity_req.initial_pose.orientation.x = 0.0
        self.spawn_entity_req.initial_pose.orientation.y = 0.0
        self.spawn_entity_req.initial_pose.orientation.z = 0.0
        self.spawn_entity_req.initial_pose.orientation.w = 1.0

        self.gazebo_spawner_cli.call_async(self.spawn_entity_req)

        self.cone_idx = self.cone_idx + 1

        print("Spawning...")


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
