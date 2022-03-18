from enum import Enum
import rclpy
from rclpy.node import Node
from simulator_msgs.msg import ControlCommand

from pynput.keyboard import Key, KeyCode, Listener
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from typing import Optional, Union

import threading


class Direction(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4

    @staticmethod
    def from_key(key: Union[Key, KeyCode]) -> Optional['Direction']:
        if key == Key.up or (isinstance(key, KeyCode) and key.char == 'w'):
            return Direction.UP
        elif key == Key.down or (isinstance(key, KeyCode) and key.char == 's'):
            return Direction.DOWN
        if key == Key.left or (isinstance(key, KeyCode) and key.char == 'a'):
            return Direction.LEFT
        elif key == Key.right or (isinstance(key, KeyCode) and key.char == 'd'):
            return Direction.RIGHT


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop')

        self.command_publisher_ = self.create_publisher(
            ControlCommand,
            'control_cmd',
            1,
        )
        self.command_timer_ = self.create_timer(
            1 / 60,  # 60 Hz
            self.publish_command,
        )
        self.msg = ControlCommand()
        self.msg.velocity = 0.0
        self.msg.steering_angle = 0.0

        # TODO: fail when no paramaters values are not defined
        self.declare_parameter(
            name='velocity.step',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='step velocity in m/s',
            ),
        )
        self.declare_parameter(
            name='steering_angle.step',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='step steering angle in radians',
            ),
        )
        self.declare_parameter(
            name='velocity.min',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='minimum velocity in m/s',
            ),
        )
        self.declare_parameter(
            name='velocity.max',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='maximum velocity in m/s',
            ),
        )
        self.declare_parameter(
            name='steering_angle.min',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='minimum angle in radians',
            ),
        )
        self.declare_parameter(
            name='steering_angle.max',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='maximum angle in radians',
            ),
        )

        self.run_keyboard_listener()

    @staticmethod
    def clamp(value, min_value: float, max_value):
        return max(min(value, max_value), min_value)

    def clamp_velocity(self, velocity):
        return TeleopNode.clamp(
            velocity,
            self.get_parameter('velocity.min').value,
            self.get_parameter('velocity.max').value,
        )

    def clamp_steering_angle(self, steering_angle):
        return TeleopNode.clamp(
            steering_angle,
            self.get_parameter('steering_angle.min').value,
            self.get_parameter('steering_angle.max').value,
        )

    def publish_command(self):
        print(self.msg.velocity, self.msg.steering_angle)
        self.command_publisher_.publish(self.msg)

    def on_press(self, key: Union[Key, KeyCode]):
        # print(f'on_press: key of type {key.__class__.__name__} = {key}')
        direction = Direction.from_key(key)
        if direction is Direction.UP:
            self.msg.velocity = self.clamp_velocity(
                self.msg.velocity + self.get_parameter('velocity.step').value
            )
        elif direction is Direction.DOWN:
            self.msg.velocity = self.clamp_velocity(
                self.msg.velocity - self.get_parameter('velocity.step').value
            )
        if direction is Direction.LEFT:
            self.msg.steering_angle = self.clamp_steering_angle(
                self.msg.steering_angle +
                self.get_parameter('steering_angle.step').value
            )
        elif direction is Direction.RIGHT:
            self.msg.steering_angle = self.clamp_steering_angle(
                self.msg.steering_angle -
                self.get_parameter('steering_angle.step').value
            )

    def on_release(self, key):
        # print(f'on_release: key={key}')
        # if key == Key.esc:
        #     # stop listener
        #     return False
        direction = Direction.from_key(key)
        if direction is Direction.UP:
            self.msg.velocity = 0.0
        elif direction is Direction.DOWN:
            self.msg.velocity = 0.0
        if direction is Direction.LEFT:
            self.msg.steering_angle = 0.0
        elif direction is Direction.RIGHT:
            self.msg.steering_angle = 0.0

    def run_keyboard_listener(self):
        # collect events until released
        keyboard_thread = threading.Thread(
            target=self.keyboard_listener,
            args=(),
        )
        keyboard_thread.start()

    def keyboard_listener(self):
        print('listener starting')
        with Listener(
            on_press=self.on_press,
            on_release=self.on_release
        ) as listener:
            print('listener running')
            listener.join()
            print('listener finished')


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    # TODO: handle KeyboardInterrupt
    rclpy.spin(teleop_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
