#!/usr/bin/env python3
"""Lockstep PyBullet peer for the Nero ROS controller and its flange trajectory."""

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_srvs.srv import Empty
from agx_arm_msgs.msg import MoveMITMsg
from oscbf_control_msgs.msg import EEState

from oscbf.core.manipulator import load_nero, NERO_READY
from oscbf.examples.nero import make_env, flange_target


class SimNode(Node):
    def __init__(self):
        super().__init__("nero_sim")
        self.tf = np.asarray(load_nero().ee_transform(np.asarray(NERO_READY)))
        self.env = make_env(self.tf[:3, 3])
        self.halted = False
        self.feedback = self.create_publisher(JointState, "feedback/joint_states", 1)
        self.targets = self.create_publisher(EEState, "ee_state", 1)
        self.create_subscription(MoveMITMsg, "control/move_mit", self.step, 1)
        self.create_service(Empty, "emergency_stop", self.stop)
        self.create_timer(0.01, self.publish_state)

    def step(self, msg):
        if self.halted:
            return
        if list(msg.joint_index) != list(range(1, 8)) or len(msg.torque) != 7:
            raise ValueError("Expected all seven Nero joints in model order")
        # Five physics steps per 100 Hz MIT command.
        for _ in range(5):
            self.env.apply_control(np.asarray(msg.torque))
            self.env.step()

    def stop(self, request, response):
        self.halted = True
        self.get_logger().info("Simulation stopped on driver stop request")
        return response

    def publish_state(self):
        stamp = self.get_clock().now().to_msg()
        z = self.env.get_joint_state()
        state = JointState(name=[f"joint{i}" for i in range(1, 8)],
                           position=z[:7].tolist(), velocity=z[7:].tolist())
        state.header.stamp = stamp
        self.feedback.publish(state)
        target = flange_target(self.tf, self.env.t)
        desired = EEState()
        desired.header.stamp = stamp
        desired.header.frame_id = "base_link"
        desired.pose.position = Point(x=target[0], y=target[1], z=target[2])
        x, y, z, w = Rotation.from_matrix(self.tf[:3, :3]).as_quat()
        desired.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        desired.twist.linear = Vector3(x=target[12], y=target[13], z=target[14])
        self.targets.publish(desired)


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.env.client.disconnect()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
