import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Axes
AXIS_LINEAR_X  = 1
AXIS_LINEAR_Y  = 0
AXIS_ANGULAR_Z = 3

# Buttons
BUTTON_ENABLE    = 5
BUTTON_HEIGHT_UP = 6
BUTTON_HEIGHT_DN = 7

# Height is in mm, sent directly — there is no normalized equivalent
HEIGHT_STEP = 2.0


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        self._height = 0.0
        self._prev_buttons = []

        # Normalized velocity: axes map directly to [-1, 1], the value is then scaled to the actual
        # velocity range by the hexapod based on the config yaml
        self._cmd_vel_pub = self.create_publisher(Twist, '/hexapod/cmd_vel_norm', 10)
        self._cmd_height_pub = self.create_publisher(Float32, '/hexapod/cmd_height', 10)

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.get_logger().info('JoyTeleopNode started')

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        if len(self._prev_buttons) < len(buttons):
            self._prev_buttons = [0] * len(buttons)

        twist = Twist()
        if buttons[BUTTON_ENABLE]:
            twist.linear.x = axes[AXIS_LINEAR_X]
            twist.linear.y = axes[AXIS_LINEAR_Y]
            twist.angular.z = axes[AXIS_ANGULAR_Z]
        self._cmd_vel_pub.publish(twist)

        up   = buttons[BUTTON_HEIGHT_UP] and not self._prev_buttons[BUTTON_HEIGHT_UP]
        down = buttons[BUTTON_HEIGHT_DN] and not self._prev_buttons[BUTTON_HEIGHT_DN]

        if up:
            self._height = self._height + HEIGHT_STEP
            self._publish_height()
        elif down:
            self._height = self._height - HEIGHT_STEP
            self._publish_height()

        self._prev_buttons = list(buttons)

    def _publish_height(self):
        msg = Float32()
        msg.data = self._height
        self._cmd_height_pub.publish(msg)
        self.get_logger().info(f'cmd_height: {self._height:.1f} mm')


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()