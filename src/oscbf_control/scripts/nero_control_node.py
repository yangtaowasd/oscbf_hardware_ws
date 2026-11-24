#!/usr/bin/env python3
"""Nero flange OSCBF controller for the agx_arm_ctrl ROS 2 MIT interface."""

import signal
from time import monotonic

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from agx_arm_msgs.msg import MoveMITMsg
from oscbf_control_msgs.msg import EEState

from oscbf.core.manipulator import NERO_READY, load_nero
from oscbf.core.nero_control import NeroController
from oscbf_control.utils.rotations_and_transforms import xyzw_to_rotation_numpy


class NeroNode(Node):
    def __init__(self):
        super().__init__("nero_oscbf")
        self.hold = self.declare_parameter("hold_current_pose", False).value
        self.controller = NeroController(load_nero())
        q = np.asarray(NERO_READY)
        tf = np.asarray(self.controller.robot.ee_transform(q))
        state = np.r_[q, np.zeros(7)]
        self.controller.check_start(state)
        self.controller.torque(state, np.r_[tf[:3, 3], tf[:3, :3].ravel(), np.zeros(6)], q)
        self.joints = [f"joint{i}" for i in range(1, 8)]
        self.state = self.target = self.posture = None
        self.state_stamp = self.target_stamp = 0
        self.last_sent_stamp = 0
        self.active = self.faulted = False
        self.timeout = 0.1
        self.stop_request = None
        self.stop_client = self.create_client(Empty, "emergency_stop")
        self.commands = self.create_publisher(MoveMITMsg, "control/move_mit", 1)
        self.create_subscription(JointState, "feedback/joint_states", self.feedback, qos_profile_sensor_data)
        if not self.hold:
            self.create_subscription(EEState, "ee_state", self.set_target, qos_profile_sensor_data)
        self.create_timer(0.01, self.tick)
        mode = "hold initial flange pose" if self.hold else "track stamped flange targets"
        self.get_logger().info(f"Ready to {mode}")

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec

    def feedback(self, msg):
        try:
            width = len(msg.name)
            if len(set(msg.name)) != width or len(msg.position) != width or len(msg.velocity) != width:
                raise ValueError("Malformed Nero joint feedback")
            idx = [msg.name.index(name) for name in self.joints]
            state = np.r_[np.asarray(msg.position)[idx], np.asarray(msg.velocity)[idx]]
            if not np.isfinite(state).all():
                raise ValueError("Nonfinite Nero joint feedback")
        except ValueError as exc:
            self.state = None
            if self.active:
                self.fault(str(exc))
            return
        stamp = self.stamp(msg)
        if stamp > self.state_stamp:
            self.state, self.state_stamp = state, stamp

    def set_target(self, msg):
        p, quat = msg.pose.position, msg.pose.orientation
        v, w = msg.twist.linear, msg.twist.angular
        xyzw = np.array([quat.x, quat.y, quat.z, quat.w])
        target = np.array([p.x, p.y, p.z, v.x, v.y, v.z, w.x, w.y, w.z])
        if (msg.header.frame_id not in ("", "base_link", "world")
                or not np.isfinite(target).all() or not np.isclose(np.linalg.norm(xyzw), 1., atol=1e-3)):
            self.target = None
            if self.active:
                self.fault("Invalid Nero flange target or reference frame")
            return
        stamp = self.stamp(msg)
        if stamp > self.target_stamp:
            rot = xyzw_to_rotation_numpy(xyzw)
            self.target = np.r_[target[:3], rot.ravel(), target[3:]]
            self.target_stamp = stamp

    def fresh(self):
        now = self.get_clock().now().nanoseconds
        stamps = (self.state_stamp,) if self.hold else (self.state_stamp, self.target_stamp)
        return all(0 <= now - stamp <= self.timeout * 1e9 for stamp in stamps)

    def fault(self, reason):
        if not self.faulted:
            self.get_logger().error(reason)
            self.faulted = True
        if self.active and self.stop_request is None and self.stop_client.service_is_ready():
            self.stop_request = self.stop_client.call_async(Empty.Request())

    def tick(self):
        if self.faulted:
            self.fault("Nero controller stopped")
            return
        missing_input = self.state is None or (self.target is None and not self.hold)
        if missing_input or not self.fresh() or not self.stop_client.service_is_ready():
            if self.active:
                self.fault("Nero input timed out or driver stop service unavailable")
            return
        if self.state_stamp == self.last_sent_stamp:
            return
        try:
            if self.posture is None:
                self.controller.check_start(self.state)
                self.posture = self.state[:7].copy()
                if self.hold:
                    tf = np.asarray(self.controller.robot.ee_transform(self.posture))
                    self.target = np.r_[tf[:3, 3], tf[:3, :3].ravel(), np.zeros(6)]
            tau = self.controller.torque(self.state, self.target, self.posture)
        except ValueError as exc:
            self.fault(str(exc))
            return
        if not self.fresh():
            if self.active:
                self.fault("Nero input expired during solve")
            return
        msg = MoveMITMsg(joint_index=list(range(1, 8)), p_des=self.state[:7].tolist(),
                         v_des=[0.] * 7, kp=[0.] * 7, kd=[0.] * 7, torque=tau.tolist())
        self.commands.publish(msg)
        self.active = True
        self.last_sent_stamp = self.state_stamp


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    running = True

    def shutdown(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    node = NeroNode()
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node.active and rclpy.ok():
            node.fault("Nero OSCBF shutting down, requesting driver stop")
            until = monotonic() + 1.
            while monotonic() < until and (node.stop_request is None or not node.stop_request.done()):
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
