import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist

# Axes
AXIS_LINEAR_X   = 1
AXIS_LINEAR_Y   = 0
AXIS_ANGULAR_Z  = 3
AXIS_BODY_PITCH = 4    # Right stick vertical, negate below if the tilt feels inverted
AXIS_BODY_YAW   = AXIS_ANGULAR_Z   # Same stick axis, the deadman picks which one it drives

# Height on the triggers (L2/R2, B6/B7 in a browser gamepad tester). Linux reports
# them as axes, not buttons: +1.0 released, -1.0 fully pressed, and 0.0 until first touched
AXIS_HEIGHT_DN  = 2    # L2
AXIS_HEIGHT_UP  = 5    # R2
TRIGGER_PRESSED = -0.5  # an axis below this counts as a press

# Axis values with a magnitude below this are treated as zero (stick noise and drift)
AXIS_DEADZONE = 0.05

# Buttons (indices depend on the controller, adjust to yours)
BUTTON_MOVE      = 5   # Deadman, hold to send velocity
BUTTON_STAND_UP  = 3   # Stand up (enable)
BUTTON_SIT_DOWN  = 0   # Sit down (shutdown)
BUTTON_GAIT      = 2   # Cycle gait (tripod -> wave -> ripple)

# Height is in mm, sent directly, there is no normalized equivalent
HEIGHT_STEP = 2.0

# Body-pose limits, [min, max] each, read from the controller node, which publishes
# them as parameters straight from the config (safety.z_range and so on). Height is
# clamped to z_range, and the right stick spans pitch_range / yaw_range
RANGE_PARAMS = ('z_range', 'pitch_range', 'yaw_range')

# Gait cycle order for BUTTON_GAIT
GAITS = ("tripod", "wave", "ripple")


def _deadzone(value):
    # Treat small axis noise and drift as no input
    return value if abs(value) >= AXIS_DEADZONE else 0.0


def _scale(value, limits):
    # Map an axis in [-1, 1] onto a [min, max] range, so 0 stays 0 even when it is asymmetric
    return value * (limits[1] if value > 0 else -limits[0])


class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joy_teleop_node')

        self._height = 0.0
        self._prev_buttons = []
        self._prev_up = False
        self._prev_down = False
        self._was_moving = False
        self._last_pitch = 0.0
        self._last_yaw = 0.0
        self._gait_index = 0
        self._limits = None  # {param: [min, max]}, None until fetched from the controller

        self.declare_parameter('controller_node', 'hexapod_controller_node')
        controller = self.get_parameter('controller_node').get_parameter_value().string_value

        # Normalized velocity: axes map directly to [-1, 1], the value is then scaled to the actual
        # velocity range by the hexapod based on the config yaml
        self._cmd_vel_pub = self.create_publisher(Twist, '/hexapod/cmd_vel_norm', 10)
        self._cmd_height_pub = self.create_publisher(Float32, '/hexapod/cmd_height', 10)
        self._cmd_pitch_pub = self.create_publisher(Float32, '/hexapod/cmd_pitch', 10)
        self._cmd_yaw_pub = self.create_publisher(Float32, '/hexapod/cmd_yaw', 10)
        self._cmd_gait_pub = self.create_publisher(String, '/hexapod/cmd_gait', 10)
        self._enable_pub = self.create_publisher(Bool, '/hexapod/enable', 10)

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        # The controller may start after the teleop (or on another machine), so keep
        # asking until it answers. Walking works meanwhile, only the body pose waits
        self._params_client = self.create_client(GetParameters, f'/{controller}/get_parameters')
        self._params_pending = False
        self._params_timer = self.create_timer(1.0, self._fetch_limits)
        self._fetch_limits()
        self.get_logger().info('JoyTeleopNode started')

    def _fetch_limits(self):
        if self._limits is not None or self._params_pending:
            return
        if not self._params_client.service_is_ready():
            self.get_logger().info(f'waiting for {self._params_client.srv_name} to read the body limits',
                                   throttle_duration_sec=5.0)
            return
        self._params_pending = True
        request = GetParameters.Request(names=list(RANGE_PARAMS))
        self._params_client.call_async(request).add_done_callback(self._on_limits)

    def _on_limits(self, future):
        self._params_pending = False
        values = future.result().values if future.result() else []
        limits = {name: list(v.double_array_value) for name, v in zip(RANGE_PARAMS, values)}
        if len(limits) != len(RANGE_PARAMS) or any(len(v) != 2 for v in limits.values()):
            self.get_logger().warn('controller did not report the body limits, retrying')
            return
        self._limits = limits
        self._params_timer.cancel()
        self.get_logger().info('body limits: ' + ', '.join(f'{k} {v}' for k, v in limits.items()))

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
        gait  = buttons[BUTTON_GAIT] and not self._prev_buttons[BUTTON_GAIT]

        # Edge-detected triggers, same once-per-press behaviour as the buttons
        up_held = axes[AXIS_HEIGHT_UP] < TRIGGER_PRESSED
        down_held = axes[AXIS_HEIGHT_DN] < TRIGGER_PRESSED
        up = up_held and not self._prev_up
        down = down_held and not self._prev_down
        self._prev_up = up_held
        self._prev_down = down_held

        if stand:
            self._publish_enable(True)
        elif sit:
            self._publish_enable(False)

        if gait:
            self._gait_index = (self._gait_index + 1) % len(GAITS)
            self._publish_gait(GAITS[self._gait_index])

        self._prev_buttons = list(buttons)

        # Everything below moves the body pose, which needs the limits from the controller
        if self._limits is None:
            if up or down:
                self.get_logger().warn('body limits not received yet, ignoring the height command')
            return

        z_min, z_max = self._limits['z_range']
        if up:
            self._height = min(self._height + HEIGHT_STEP, z_max)
            self._publish_height()
        elif down:
            self._height = max(self._height - HEIGHT_STEP, z_min)
            self._publish_height()

        # Body pose from the right stick, only while the deadman is released: while it is held
        # the same stick steers the walk, so the pose holds its last value. Published only when
        # it changes so it stays quiet when centered
        if not moving:
            pitch = round(_scale(_deadzone(axes[AXIS_BODY_PITCH]), self._limits['pitch_range']), 1)
            if pitch != self._last_pitch:
                self._last_pitch = pitch
                self._publish_pitch(pitch)

            yaw = round(_scale(_deadzone(axes[AXIS_BODY_YAW]), self._limits['yaw_range']), 1)
            if yaw != self._last_yaw:
                self._last_yaw = yaw
                self._publish_yaw(yaw)

    def _publish_height(self):
        msg = Float32()
        msg.data = self._height
        self._cmd_height_pub.publish(msg)
        self.get_logger().info(f'cmd_height: {self._height:.1f} mm')

    def _publish_pitch(self, pitch: float):
        msg = Float32()
        msg.data = pitch
        self._cmd_pitch_pub.publish(msg)

    def _publish_yaw(self, yaw: float):
        msg = Float32()
        msg.data = yaw
        self._cmd_yaw_pub.publish(msg)

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