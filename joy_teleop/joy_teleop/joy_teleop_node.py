import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from simulator_msgs.msg import ControlCommand

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop')

        self.stop = False

        self.last_timestamp: int = self.get_clock().now().nanoseconds

        self.msg = ControlCommand()
        self.msg.velocity = 0.0
        self.msg.steering_angle = 0.0

        # TODO: fail when no paramaters values are not defined
        self.declare_parameter(
            name='axes.forward',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='id of the joystick axis for forward',
            ),
        )
        self.declare_parameter(
            name='axes.reverse',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='id of the joystick axis for reverse',
            ),
        )
        self.declare_parameter(
            name='axes.steering',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='id of the joystick axis for steering',
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
            msg_type=ControlCommand,
            topic='control_cmd',
            qos_profile=1,
        )
        self.command_timer_ = self.create_timer(
            # TODO: make configurable
            1 / 10,  # 60 Hz
            self.publish_command,
        )
        self.joy_subscription_ = self.create_subscription(
            msg_type=Joy,
            topic='joy',
            callback=self.joy_cb,
            qos_profile=1,
        )

        pass

    def publish_command(self):
        timestamp = self.get_clock().now().nanoseconds
        dt = (timestamp - self.last_timestamp) / 1e9  # seconds
        self.last_timestamp = timestamp

        print(self.msg.velocity, self.msg.steering_angle)
        self.command_publisher_.publish(self.msg)

    def joy_cb(self, msg: Joy):
        forward_axis = self.get_parameter('axes.forward').value
        reverse_axis = self.get_parameter('axes.reverse').value
        steering_axis = self.get_parameter('axes.steering').value

        velocity_min = self.get_parameter('velocity.min').value
        velocity_max = self.get_parameter('velocity.max').value
        steering_angle_max = self.get_parameter('steering_angle.max').value

        forward_value = msg.axes[forward_axis]
        reverse_value = msg.axes[reverse_axis]
        steering_value = msg.axes[steering_axis]

        forward = ((forward_value - 1.0) / (-2.0)) * velocity_max
        reverse = ((reverse_value - 1.0) / (-2.0)) * velocity_min
        self.msg.velocity = forward + reverse
        # TODO: correct steering mapping
        self.msg.steering_angle = steering_value * steering_angle_max

        # self.get_logger().info(
        #     f'F={forward_value} R={reverse_value} S={steering_axis}'
        # )
        # self.get_logger().info(
        #     f'f={forward} r={reverse} v={self.msg.velocity} s={self.msg.steering_angle}'
        # )

        pass


def main(args=None):
    rclpy.init(args=args)
    teleop_node = JoyTeleopNode()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
