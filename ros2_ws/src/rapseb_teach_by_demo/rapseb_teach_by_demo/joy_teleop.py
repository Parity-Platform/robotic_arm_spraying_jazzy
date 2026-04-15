"""Logitech F710 gamepad teleop for the RAPSEB teach-by-demo workflow.

Maps gamepad axes to a Cartesian twist for MoveIt Servo and publishes the
spray trigger (right trigger press-and-hold) and segment markers on
std_msgs/Bool topics that the demo recorder listens to.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger


class JoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__('joy_teleop')

        self.declare_parameters(namespace='', parameters=[
            ('twist_topic', '/servo_node/delta_twist_cmds'),
            ('spray_topic', '/rapseb/spray_trigger'),
            ('segment_topic', '/rapseb/segment_marker'),
            ('record_start_topic', '/rapseb/record_start'),
            ('record_stop_topic', '/rapseb/record_stop'),
            ('freedrive_service', '/rapseb/toggle_freedrive'),
            ('command_frame', 'tool0'),
            ('base_frame', 'base_link'),
            ('publish_rate_hz', 100.0),
            ('axis_deadzone', 0.08),
            ('linear_scale_xy', 0.08),
            ('linear_scale_z', 0.04),
            ('angular_scale', 0.4),
            ('boost_linear', 2.0),
            ('boost_angular', 1.5),
            ('axis_left_x', 0),
            ('axis_left_y', 1),
            ('axis_right_x', 3),
            ('axis_right_y', 4),
            ('axis_triggers_lt', 2),
            ('axis_triggers_rt', 5),
            ('button_a', 0),
            ('button_b', 1),
            ('button_x', 2),
            ('button_y', 3),
            ('button_lb', 4),
            ('button_rb', 5),
            ('button_back', 6),
            ('button_start', 7),
            ('button_logo', 8),
            ('spray_axis_threshold', 0.2),
        ])

        p = self.get_parameter
        self.command_frame = p('command_frame').value
        self.rate_hz = float(p('publish_rate_hz').value)
        self.dead = float(p('axis_deadzone').value)
        self.lin_xy = float(p('linear_scale_xy').value)
        self.lin_z = float(p('linear_scale_z').value)
        self.ang = float(p('angular_scale').value)
        self.boost_lin = float(p('boost_linear').value)
        self.boost_ang = float(p('boost_angular').value)
        self.spray_thr = float(p('spray_axis_threshold').value)

        self.ax_lx = int(p('axis_left_x').value)
        self.ax_ly = int(p('axis_left_y').value)
        self.ax_rx = int(p('axis_right_x').value)
        self.ax_ry = int(p('axis_right_y').value)
        self.ax_lt = int(p('axis_triggers_lt').value)
        self.ax_rt = int(p('axis_triggers_rt').value)

        self.bt_a = int(p('button_a').value)
        self.bt_b = int(p('button_b').value)
        self.bt_x = int(p('button_x').value)
        self.bt_y = int(p('button_y').value)
        self.bt_lb = int(p('button_lb').value)
        self.bt_rb = int(p('button_rb').value)
        self.bt_back = int(p('button_back').value)
        self.bt_start = int(p('button_start').value)
        self.bt_logo = int(p('button_logo').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.twist_pub = self.create_publisher(
            TwistStamped, p('twist_topic').value, 10)
        self.spray_pub = self.create_publisher(
            Bool, p('spray_topic').value, qos)
        self.segment_pub = self.create_publisher(
            Empty, p('segment_topic').value, qos)
        self.record_start_pub = self.create_publisher(
            Empty, p('record_start_topic').value, qos)
        self.record_stop_pub = self.create_publisher(
            Empty, p('record_stop_topic').value, qos)

        self.freedrive_cli = self.create_client(
            Trigger, p('freedrive_service').value)

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self._on_joy, 10)

        self.last_buttons: list[int] = []
        self.spray_active = False
        self.scale_mult = 1.0

        self.latest_twist = TwistStamped()
        self.latest_twist.header.frame_id = self.command_frame

        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_twist)

        self.get_logger().info(
            f'joy_teleop ready. command_frame={self.command_frame}, '
            f'rate={self.rate_hz} Hz, lin_xy={self.lin_xy} m/s')

    @staticmethod
    def _apply_deadzone(v: float, d: float) -> float:
        if abs(v) < d:
            return 0.0
        s = 1.0 if v > 0 else -1.0
        return s * (abs(v) - d) / (1.0 - d)

    def _edge(self, buttons: list[int], idx: int) -> bool:
        if idx >= len(buttons):
            return False
        prev = self.last_buttons[idx] if idx < len(self.last_buttons) else 0
        return buttons[idx] == 1 and prev == 0

    def _on_joy(self, msg: Joy) -> None:
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Spray signal: right trigger axis below threshold (press and hold).
        rt = axes[self.ax_rt] if self.ax_rt < len(axes) else 1.0
        spray_now = rt < self.spray_thr
        if spray_now != self.spray_active:
            self.spray_active = spray_now
            self.spray_pub.publish(Bool(data=spray_now))

        # Discrete buttons (rising edge).
        if self._edge(buttons, self.bt_a):
            self.record_start_pub.publish(Empty())
            self.get_logger().info('record start')
        if self._edge(buttons, self.bt_b):
            self.record_stop_pub.publish(Empty())
            self.get_logger().info('record stop and save')
        if self._edge(buttons, self.bt_start):
            self.segment_pub.publish(Empty())
            self.get_logger().info('segment marker')
        if self._edge(buttons, self.bt_y):
            self._request_freedrive_toggle()
        if self._edge(buttons, self.bt_lb):
            self.scale_mult = max(0.25, self.scale_mult * 0.8)
            self.get_logger().info(f'scale x{self.scale_mult:.2f}')
        if self._edge(buttons, self.bt_rb):
            self.scale_mult = min(4.0, self.scale_mult * 1.25)
            self.get_logger().info(f'scale x{self.scale_mult:.2f}')

        self.last_buttons = buttons

        # Cartesian twist mapping.
        lx = self._apply_deadzone(axes[self.ax_lx], self.dead)
        ly = self._apply_deadzone(axes[self.ax_ly], self.dead)
        rx = self._apply_deadzone(axes[self.ax_rx], self.dead)
        ry = self._apply_deadzone(axes[self.ax_ry], self.dead)

        panic = (self.bt_x < len(buttons) and buttons[self.bt_x] == 1)
        boost = (self.bt_back < len(buttons) and buttons[self.bt_back] == 1)

        k_lin = self.lin_xy * self.scale_mult * (self.boost_lin if boost else 1.0)
        k_z = self.lin_z * self.scale_mult * (self.boost_lin if boost else 1.0)
        k_ang = self.ang * self.scale_mult * (self.boost_ang if boost else 1.0)

        t = TwistStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.command_frame
        if not panic:
            t.twist.linear.x = float(ly) * k_lin
            t.twist.linear.y = float(lx) * k_lin
            t.twist.linear.z = float(ry) * k_z
            t.twist.angular.z = float(rx) * k_ang
        self.latest_twist = t

    def _request_freedrive_toggle(self) -> None:
        if not self.freedrive_cli.service_is_ready():
            self.get_logger().warn('freedrive service not available')
            return
        req = Trigger.Request()
        future = self.freedrive_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'freedrive toggle: {f.result().message if f.result() else "no result"}'))

    def _publish_twist(self) -> None:
        self.latest_twist.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(self.latest_twist)


def main() -> None:
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
