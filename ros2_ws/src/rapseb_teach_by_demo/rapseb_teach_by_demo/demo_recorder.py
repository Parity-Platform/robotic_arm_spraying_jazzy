"""Records the end-effector trajectory and spray trigger during a demonstration.

Subscribes:
  /tf, /tf_static                         -- used to read tool0 in base_link
  /rapseb/spray_trigger (std_msgs/Bool)   -- press-and-hold spray signal
  /rapseb/segment_marker (std_msgs/Empty) -- boundary between spraying passes
  /rapseb/record_start (std_msgs/Empty)   -- start new recording
  /rapseb/record_stop (std_msgs/Empty)    -- stop and flush to CSV

Writes a raw demonstration CSV:
  demos/demo_YYYYMMDD_HHMMSS_raw.csv

Columns:
  time_s, x_m, y_m, z_m, qx, qy, qz, qw, spray, segment_id

Sampling rate is fixed by the 'sample_rate_hz' parameter (default 100 Hz).
TF lookups use the latest available transform; if TF is stale the row is
skipped.
"""
from __future__ import annotations

import csv
import math
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener, LookupException, \
    ExtrapolationException, ConnectivityException


class DemoRecorder(Node):
    def __init__(self) -> None:
        super().__init__('demo_recorder')

        self.declare_parameters(namespace='', parameters=[
            ('base_frame', 'base_link'),
            ('ee_frame', 'tool0'),
            ('sample_rate_hz', 100.0),
            ('output_dir', 'demos'),
            ('autostart', False),
        ])

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.out_dir = Path(self.get_parameter('output_dir').value)
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spray_sub = self.create_subscription(
            Bool, '/rapseb/spray_trigger', self._on_spray, 10)
        self.segment_sub = self.create_subscription(
            Empty, '/rapseb/segment_marker', self._on_segment, 10)
        self.start_sub = self.create_subscription(
            Empty, '/rapseb/record_start', self._on_start, 10)
        self.stop_sub = self.create_subscription(
            Empty, '/rapseb/record_stop', self._on_stop, 10)

        self.spray_active = False
        self.segment_id = 0
        self.recording = False
        self.t0 = None
        self.writer = None
        self.file = None
        self.path = None
        self.rows_written = 0

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        if bool(self.get_parameter('autostart').value):
            self._start_recording()

        self.get_logger().info(
            f'demo_recorder ready. out_dir={self.out_dir.resolve()} '
            f'rate={self.rate_hz} Hz')

    def _on_spray(self, msg: Bool) -> None:
        self.spray_active = bool(msg.data)

    def _on_segment(self, _msg: Empty) -> None:
        self.segment_id += 1
        self.get_logger().info(f'segment -> {self.segment_id}')

    def _on_start(self, _msg: Empty) -> None:
        self._start_recording()

    def _on_stop(self, _msg: Empty) -> None:
        self._stop_recording()

    def _start_recording(self) -> None:
        if self.recording:
            self.get_logger().warn('already recording')
            return
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = self.out_dir / f'demo_{ts}_raw.csv'
        self.file = open(self.path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_s', 'x_m', 'y_m', 'z_m',
            'qx', 'qy', 'qz', 'qw', 'spray', 'segment_id',
        ])
        self.t0 = self.get_clock().now()
        self.segment_id = 0
        self.rows_written = 0
        self.recording = True
        self.get_logger().info(f'recording -> {self.path.name}')

    def _stop_recording(self) -> None:
        if not self.recording:
            return
        self.recording = False
        if self.file:
            self.file.flush()
            self.file.close()
        self.get_logger().info(
            f'recording stopped. rows={self.rows_written} file={self.path}')
        self.file = None
        self.writer = None
        self.t0 = None

    def _tick(self) -> None:
        if not self.recording or self.writer is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, Time())
        except (LookupException, ExtrapolationException,
                ConnectivityException):
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        tr = tf.transform.translation
        r = tf.transform.rotation
        self.writer.writerow([
            f'{t:.6f}',
            f'{tr.x:.6f}', f'{tr.y:.6f}', f'{tr.z:.6f}',
            f'{r.x:.6f}', f'{r.y:.6f}', f'{r.z:.6f}', f'{r.w:.6f}',
            1 if self.spray_active else 0,
            self.segment_id,
        ])
        self.rows_written += 1

    def destroy_node(self) -> bool:
        self._stop_recording()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
