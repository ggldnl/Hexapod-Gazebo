import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# Axes
AXIS_LINEAR_X   = 1
AXIS_LINEAR_Y   = 0
AXIS_ANGULAR_Z  = 3
AXIS_BODY_PITCH = 4    # Right stick vertical, negate below if the tilt feels inverted

# Axis values with a magnitude below this are treated as zero (stick noise and drift)
AXIS_DEADZONE = 0.05

# Buttons (indices depend on the controller, adjust to yours)
BUTTON_MOVE      = 5   # Deadman, hold to send velocity
BUTTON_STAND_UP  = 3   # Stand up (enable)
BUTTON_SIT_DOWN  = 0   # Sit down (shutdown)
BUTTON_HEIGHT_UP = 6
BUTTON_HEIGHT_DN = 7
BUTTON_GAIT      = 2   # Cycle gait (tripod -> wave -> ripple)

# Height is in mm, sent directly, there is no normalized equivalent
HEIGHT_STEP = 2.0

# Body pitch: the right stick maps to +-PITCH_MAX degrees (match safety.pitch_range)
PITCH_MAX = 10.0  # TODO take this from the config instead

# Gait cycle order for BUTTON_GAIT
GAITS = ("tripod", "wave", "ripple")


def _deadzone(value):
    # Treat small axis noise and drift as no input
    return value if abs(value) >= AXIS_DEADZONE else 0.0


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        self._height = 0.0
        self._prev_buttons = []
        self._was_moving = False
        self._last_pitch = 0.0
        self._gait_index = 0

        # Normalized velocity: axes map directly to [-1, 1], the value is then scaled to the actual
        # velocity range by the hexapod based on the config yaml
        self._cmd_vel_pub = self.create_publisher(Twist, '/hexapod/cmd_vel_norm', 10)
        self._cmd_height_pub = self.create_publisher(Float32, '/hexapod/cmd_height', 10)
        self._cmd_pitch_pub = self.create_publisher(Float32, '/hexapod/cmd_pitch', 10)
        self._cmd_gait_pub = self.create_publisher(String, '/hexapod/cmd_gait', 10)
        self._enable_pub = self.create_publisher(Bool, '/hexapod/enable', 10)

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.get_logger().info('JoyTeleopNode started')

    def _joy_cb(self, msg: Joy):
        buttons = msg.buttons
        axes = msg.axes

        if len(self._prev_buttons) < len(buttons):
            self._prev_buttons = [0] * len(buttons)

        # Only assert velocity while the deadman is held, and send a single zero
        # when it is released, so the teleop stays quiet when idle and does not
        # clobber other command sources such as a manual /hexapod/cmd_vel
        moving = bool(buttons[BUTTON_MOVE])
        if moving:
            twist = Twist()
            twist.linear.x = _deadzone(axes[AXIS_LINEAR_X])
            twist.linear.y = _deadzone(axes[AXIS_LINEAR_Y])
            twist.angular.z = _deadzone(axes[AXIS_ANGULAR_Z])
            self._cmd_vel_pub.publish(twist)
        elif self._was_moving:
            self._cmd_vel_pub.publish(Twist())
        self._was_moving = moving

        # Edge-detected buttons (fire once per press)
        stand = buttons[BUTTON_STAND_UP] and not self._prev_buttons[BUTTON_STAND_UP]
        sit   = buttons[BUTTON_SIT_DOWN] and not self._prev_buttons[BUTTON_SIT_DOWN]
        up    = buttons[BUTTON_HEIGHT_UP] and not self._prev_buttons[BUTTON_HEIGHT_UP]
        down  = buttons[BUTTON_HEIGHT_DN] and not self._prev_buttons[BUTTON_HEIGHT_DN]
        gait  = buttons[BUTTON_GAIT] and not self._prev_buttons[BUTTON_GAIT]

        if stand:
            self._publish_enable(True)
        elif sit:
            self._publish_enable(False)

        if gait:
            self._gait_index = (self._gait_index + 1) % len(GAITS)
            self._publish_gait(GAITS[self._gait_index])

        if up:
            self._height = self._height + HEIGHT_STEP
            self._publish_height()
        elif down:
            self._height = self._height - HEIGHT_STEP
            self._publish_height()

        # Body pitch from the right stick, published only when it changes so it stays quiet when centered
        pitch = round(_deadzone(axes[AXIS_BODY_PITCH]) * PITCH_MAX, 1)
        if pitch != self._last_pitch:
            self._last_pitch = pitch
            self._publish_pitch(pitch)

        self._prev_buttons = list(buttons)

    def _publish_height(self):
        msg = Float32()
        msg.data = self._height
        self._cmd_height_pub.publish(msg)
        self.get_logger().info(f'cmd_height: {self._height:.1f} mm')

    def _publish_pitch(self, pitch: float):
        msg = Float32()
        msg.data = pitch
        self._cmd_pitch_pub.publish(msg)

    def _publish_gait(self, gait: str):
        msg = String()
        msg.data = gait
        self._cmd_gait_pub.publish(msg)
        self.get_logger().info(f'gait: {gait}')

    def _publish_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self._enable_pub.publish(msg)
        self.get_logger().info('stand up' if enable else 'sit down')


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