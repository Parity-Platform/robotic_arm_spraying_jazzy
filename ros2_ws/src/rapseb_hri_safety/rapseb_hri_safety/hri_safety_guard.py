#!/usr/bin/env python3
"""
HRI safety guard for the RAPSEB UR10e workcell.

Monitors human proximity via ros4hri tracked IDs and enforces three
ISO 10218-compliant safety zones by switching the active trajectory
controller through the controller_manager service interface.

Zones (configurable via ROS parameters):
    Z1 (< stop_distance)  : pause controller, publish STOPPED
    Z2 (< warn_distance)  : reduce speed via UR dashboard, publish REDUCED
    Z3 (>= warn_distance) : normal operation, publish NORMAL

Publishes:
    /rapseb/robot_mode  (std_msgs/String)  -  NORMAL | REDUCED | STOPPED

Requires:
    - ros4hri (hri_msgs) for human tracking
    - controller_manager_msgs for SwitchController
    - A running controller_manager (from ros2_control)
    - UR dashboard_client (optional, for speed slider on real hardware)
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from std_msgs.msg import String
from hri_msgs.msg import IdsList
from controller_manager_msgs.srv import SwitchController


class HRISafetyGuard(Node):

    STOPPED = 'STOPPED'
    REDUCED = 'REDUCED'
    NORMAL  = 'NORMAL'

    def __init__(self):
        super().__init__('hri_safety_guard')

        # parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('warn_distance_m', 1.5)
        self.declare_parameter('controller_manager_ns', '/controller_manager')
        self.declare_parameter('trajectory_controller',
                               'scaled_joint_trajectory_controller')
        self.declare_parameter('speed_slider_service', '')
        self.declare_parameter('reduced_speed_pct', 20)
        self.declare_parameter('monitor_rate_hz', 20.0)

        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.stop_dist = self.get_parameter(
            'stop_distance_m').get_parameter_value().double_value
        self.warn_dist = self.get_parameter(
            'warn_distance_m').get_parameter_value().double_value
        cm_ns = self.get_parameter(
            'controller_manager_ns').get_parameter_value().string_value
        self.traj_ctrl = self.get_parameter(
            'trajectory_controller').get_parameter_value().string_value
        self.speed_srv_name = self.get_parameter(
            'speed_slider_service').get_parameter_value().string_value
        self.reduced_pct = self.get_parameter(
            'reduced_speed_pct').get_parameter_value().integer_value
        rate_hz = self.get_parameter(
            'monitor_rate_hz').get_parameter_value().double_value

        if self.warn_dist <= self.stop_dist:
            self.get_logger().warn(
                'warn_distance_m <= stop_distance_m, adjusting to stop + 0.2')
            self.warn_dist = self.stop_dist + 0.2

        # state
        self._mode = self.NORMAL
        self._tracked_ids: List[str] = []
        self._ctrl_active = True

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # publisher
        self._mode_pub = self.create_publisher(String, '/rapseb/robot_mode', 10)

        # ros4hri subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(
            IdsList, '/humans/bodies/tracked', self._on_tracked, qos)

        # controller_manager switch service
        self._switch_cli = self.create_client(
            SwitchController, f'{cm_ns}/switch_controller')

        # UR speed slider (optional, only on real hardware with ur_robot_driver)
        self._speed_cli = None
        self._speed_srv_type = None
        if self.speed_srv_name:
            try:
                from ur_dashboard_msgs.srv import SetSpeedSliderFraction
                self._speed_cli = self.create_client(
                    SetSpeedSliderFraction, self.speed_srv_name)
                self._speed_srv_type = SetSpeedSliderFraction
            except ImportError:
                self.get_logger().warn(
                    'ur_dashboard_msgs not found, speed slider disabled')

        # monitor timer
        period = 1.0 / max(rate_hz, 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'HRI guard active: stop={self.stop_dist}m '
            f'warn={self.warn_dist}m ctrl={self.traj_ctrl}')

    # -- callbacks -------------------------------------------------------

    def _on_tracked(self, msg: IdsList):
        self._tracked_ids = list(msg.ids)

    def _tick(self):
        min_d = self._closest_human()
        prev = self._mode

        if min_d < self.stop_dist:
            self._mode = self.STOPPED
        elif min_d < self.warn_dist:
            self._mode = self.REDUCED
        else:
            self._mode = self.NORMAL

        if self._mode != prev:
            self.get_logger().info(
                f'{prev} -> {self._mode} (d={min_d:.2f}m)')
            self._apply()

        self._mode_pub.publish(String(data=self._mode))

    # -- distance --------------------------------------------------------

    def _closest_human(self) -> float:
        if not self._tracked_ids:
            return float('inf')

        best = float('inf')
        for hid in self._tracked_ids:
            try:
                t = self._tf_buf.lookup_transform(
                    self.base_frame, f'human_{hid}',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
                v = t.transform.translation
                d = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                best = min(best, d)
            except tf2_ros.TransformException:
                continue
        return best

    # -- actuation -------------------------------------------------------

    def _apply(self):
        if self._mode == self.STOPPED:
            self._deactivate_ctrl()
            self._speed_slider(0.0)
        elif self._mode == self.REDUCED:
            self._activate_ctrl()
            self._speed_slider(self.reduced_pct / 100.0)
        else:
            self._activate_ctrl()
            self._speed_slider(1.0)

    def _deactivate_ctrl(self):
        if not self._ctrl_active:
            return
        self._call_switch(deactivate=[self.traj_ctrl])
        self._ctrl_active = False

    def _activate_ctrl(self):
        if self._ctrl_active:
            return
        self._call_switch(activate=[self.traj_ctrl])
        self._ctrl_active = True

    def _call_switch(self, activate=None, deactivate=None):
        if not self._switch_cli.service_is_ready():
            self.get_logger().warn('controller_manager unavailable')
            return
        req = SwitchController.Request()
        req.activate_controllers = activate or []
        req.deactivate_controllers = deactivate or []
        req.strictness = SwitchController.Request.STRICT
        fut = self._switch_cli.call_async(req)
        fut.add_done_callback(self._log_result)

    def _speed_slider(self, fraction: float):
        if self._speed_cli is None or not self._speed_cli.service_is_ready():
            return
        req = self._speed_srv_type.Request()
        req.speed_slider_fraction = max(0.0, min(1.0, fraction))
        self._speed_cli.call_async(req)

    @staticmethod
    def _log_result(future):
        try:
            r = future.result()
            if r is not None and not r.ok:
                rclpy.logging.get_logger('hri_safety_guard').warn(
                    'SwitchController ok=False')
        except Exception as e:
            rclpy.logging.get_logger('hri_safety_guard').error(
                f'SwitchController error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HRISafetyGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
