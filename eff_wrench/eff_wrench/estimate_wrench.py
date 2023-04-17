import rclpy
from rclpy import qos
from rclpy.node import Node

from typing import Tuple

from scipy.optimize import minimize

qos_profile = qos.qos_profile_system_default

import optas
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped


class EstimateWrenchNode(Node):

    """ROS node that estimates the wrench at a given link."""

    def __init__(self):
        """Constructor for he EstimateWrenchNode class."""
        super().__init__("estimate_wrench_node")

        # Get parameters
        self.declare_parameter("urdf_file_name")
        urdf_file_name = (
            self.get_parameter("urdf_file_name").get_parameter_value().string_value
        )

        self.declare_parameter("link_name")
        link_name = self.get_parameter("link_name").get_parameter_value().string_value

        # Setup function that computes Jacobian
        robot = optas.RobotModel(urdf_filename=urdf_file_name)
        self.J = robot.get_global_link_geometric_jacobian_function(
            link_name, numpy_output=True
        )
        self.get_logger().info(
            "Expected joint order:" + str(robot.actuated_joint_names)
        )

        # Setup wrench publisher
        self.wrench_pub = self.create_publisher(WrenchStamped, "wrench", qos_profile)

        # Setup joint state subscriber
        self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            qos_profile,
        )

    def publish_wrench(self, w: np.ndarray) -> None:
        """Publish wrench to ROS."""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wrench.force.x = w[0]
        msg.wrench.force.y = w[1]
        msg.wrench.force.z = w[2]
        msg.wrench.torque.x = w[3]
        msg.wrench.torque.y = w[4]
        msg.wrench.torque.z = w[5]
        self.wrench_pub.publish(msg)

    @staticmethod
    def extract(msg: JointState) -> Tuple[np.ndarray]:
        """Extract the robot joint position and external torque from joint state message."""
        return np.array(msg.position), np.array(msg.effort)

    def estimate_wrench(self, q: np.ndarray, tau_ext: np.ndarray) -> np.ndarray:
        J = self.J(q)
        w = np.linalg.pinv(J.T) @ tau_ext
        return w

    def joint_state_callback(self, msg: JointState) -> None:
        q, tau_ext = self.extract(msg)
        w = self.estimate_wrench(q, tau_ext)
        self.publish_wrench(w)


def main(args=None):
    # Start node, and spin
    rclpy.init(args=args)
    node = EstimateWrenchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
