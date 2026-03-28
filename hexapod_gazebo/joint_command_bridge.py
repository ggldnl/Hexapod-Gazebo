import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointCommandBridge(Node):

    def __init__(self):
        super().__init__("joint_command_bridge")
        self.declare_parameter("publish_rate", 50.0)

        # Must match the order in forward_command_controller.joints in the YAML
        self._joint_names: list[str] = [
            f'leg_{i}_{joint}'
            for i in range(1, 7)
            for joint in ['coxa', 'femur', 'tibia']
        ]

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._last_commands: dict[str, float] = {j: 0.0 for j in self._joint_names}

        self._pub = self.create_publisher(
            Float64MultiArray, "/forward_command_controller/commands", 10
        )
        self.create_subscription(
            JointState, "/hexapod/joint_values", self._cb, 10
        )
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Bridge ready — {len(self._joint_names)} joints at {rate} Hz"
        )

    def _cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._last_commands:
                self._last_commands[name] = float(pos)
            else:
                self.get_logger().warning(f"Unknown joint received: '{name}'")

    def _publish(self) -> None:
        msg = Float64MultiArray()
        msg.data = [self._last_commands[j] for j in self._joint_names]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():      # guard against double-shutdown
            rclpy.shutdown()