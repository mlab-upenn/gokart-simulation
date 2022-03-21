import datetime
from enum import IntEnum
import rclpy
from rclpy.node import Node
from simulator_msgs.msg import ControlCommand

from pynput.keyboard import Key, KeyCode, Listener
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from typing import Optional, Union


class Direction(IntEnum):
    UP = 0
    DOWN = 1
    LEFT = 2
    RIGHT = 3

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

        self.stop = False

        self.last_timestamp: int = self.get_clock().now().nanoseconds

        self.msg = ControlCommand()
        self.msg.velocity = 0.0
        self.msg.steering_angle = 0.0

        # TODO: fail when no paramaters values are not defined
        self.declare_parameter(
            name='velocity.acceleration',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='acceleration [m/s^2]',
            ),
        )
        self.declare_parameter(
            name='velocity.min',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='minimum velocity [m/s]',
            ),
        )
        self.declare_parameter(
            name='velocity.max',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='maximum velocity [m/s]',
            ),
        )
        self.declare_parameter(
            name='steering_angle.angular_velocity',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='angular velocity [rad/s]',
            ),
        )
        self.declare_parameter(
            name='steering_angle.min',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='minimum angle [rad]',
            ),
        )
        self.declare_parameter(
            name='steering_angle.max',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='maximum angle [rad]',
            ),
        )

        self.command_publisher_ = self.create_publisher(
            ControlCommand,
            'control_cmd',
            1,
        )
        self.command_timer_ = self.create_timer(
            # TODO: make configurable
            1 / 10,  # 60 Hz
            self.publish_command,
        )

        # TODO: remove this debugging code
        self.ttm = 0
        self.t1 = 0
        self.t2 = 0

        self.keyboard_state = [False for _ in range(4)]
        self.run_keyboard_listener()

    @staticmethod
    def clamp(value, min_value, max_value):
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
        timestamp = self.get_clock().now().nanoseconds
        dt = (timestamp - self.last_timestamp) / 1e9  # seconds
        self.last_timestamp = timestamp

        acceleration = self.get_parameter('velocity.acceleration').value
        if self.keyboard_state[Direction.UP]:
            self.msg.velocity = self.clamp_velocity(
                self.msg.velocity + acceleration * dt
            )
        elif self.keyboard_state[Direction.DOWN]:
            self.msg.velocity = self.clamp_velocity(
                self.msg.velocity - acceleration * dt
            )
        else:
            self.msg.velocity = 0.0

        angular_velocity = (
            self.get_parameter('steering_angle.angular_velocity').value
        )
        if self.keyboard_state[Direction.LEFT]:
            self.msg.steering_angle = self.clamp_steering_angle(
                self.msg.steering_angle + angular_velocity * dt
            )
        elif self.keyboard_state[Direction.RIGHT]:
            self.msg.steering_angle = self.clamp_steering_angle(
                self.msg.steering_angle - angular_velocity * dt
            )
        else:
            self.msg.steering_angle = 0.0

        # TODO: remove this debugging code
        if self.msg.velocity == 0.0:
            self.t1 = datetime.datetime.now()
            self.ttm = 0
        elif self.ttm != 0:
            print(f'ttm: time to max velocity: {self.ttm}')
        elif self.msg.velocity == 8.0:
            self.t2 = datetime.datetime.now()
            self.ttm = self.t2 - self.t1

        print(self.msg.velocity, self.msg.steering_angle)
        print(self.keyboard_state)
        self.command_publisher_.publish(self.msg)

    def on_press(self, key: Union[Key, KeyCode]):
        # print(f'on_press: key of type {key.__class__.__name__} = {key}')
        direction = Direction.from_key(key)
        if direction is not None:
            self.keyboard_state[direction.value] = True
        pass

    def on_release(self, key):
        # print(f'on_release: key={key}')
        # if key == Key.esc:
        #     # stop listener
        #     return False
        direction = Direction.from_key(key)
        if direction is not None:
            self.keyboard_state[direction.value] = False
        pass

    def run_keyboard_listener(self):
        print('keyboard listener starting')
        # non-blocking keyboard listener
        # see https://pynput.readthedocs.io/en/latest/keyboard.html#monitoring-the-keyboard
        self.keyboard_listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.keyboard_listener.start()

    def stop_keyboard_listener(self):
        self.keyboard_listener.stop()
        pass


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        # actually, we do not need to explicitly stop it
        teleop_node.stop_keyboard_listener()
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
