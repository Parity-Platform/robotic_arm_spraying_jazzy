"""Optional bridge from an extracted trajectory CSV to the spraying executor.

Reads trajectory_output.csv (the same schema the spraying node consumes) and
publishes it on demand as:

  /rapseb/demo_trajectory (trajectory_msgs/JointTrajectory)  -- optional
  /rapseb/spray_status (std_msgs/Bool)                       -- per-waypoint

This is a simple fan-out so the spraying pipeline can reuse its existing
trajectory executor without needing to parse files at runtime. If the main
executor already reads CSV from disk, this node can be skipped.
"""
from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ReplayPublisher(Node):
    def __init__(self) -> None:
        super().__init__('replay_publisher')

        self.declare_parameter('input_csv', 'trajectory_output.csv')
        self.declare_parameter('playback_rate', 1.0)

        self.input_csv = Path(self.get_parameter('input_csv').value)
        self.playback_rate = float(self.get_parameter('playback_rate').value)

        self.spray_pub = self.create_publisher(Bool, '/rapseb/spray_status', 10)
        self.srv = self.create_service(Trigger, '/rapseb/replay_demo',
                                       self._on_replay)

        self._rows: list[dict] = []
        self._load()
        self._timer = None
        self._idx = 0
        self._t_prev = 0.0

        self.get_logger().info(
            f'replay_publisher ready. rows={len(self._rows)} '
            f'source={self.input_csv}')

    def _load(self) -> None:
        if not self.input_csv.exists():
            self.get_logger().warn(f'{self.input_csv} not found at init; '
                                   'will reload on replay')
            return
        with self.input_csv.open('r', newline='') as f:
            self._rows = list(csv.DictReader(f))

    def _on_replay(self, _req: Trigger.Request,
                   resp: Trigger.Response) -> Trigger.Response:
        self._load()
        if not self._rows:
            resp.success = False
            resp.message = 'no trajectory loaded'
            return resp
        self._idx = 0
        self._t_prev = 0.0
        self._schedule_next()
        resp.success = True
        resp.message = f'replaying {len(self._rows)} rows at x{self.playback_rate}'
        return resp

    def _schedule_next(self) -> None:
        if self._idx >= len(self._rows):
            return
        row = self._rows[self._idx]
        t = float(row['time [s]'])
        dt = max(1e-3, (t - self._t_prev) / self.playback_rate)
        self._t_prev = t
        self._timer = self.create_timer(dt, self._on_tick, oneshot=True) \
            if hasattr(self, '_on_tick_supports_oneshot') else None
        # ROS 2 rclpy does not support one-shot timers directly; emulate with
        # a short timer that cancels itself.
        if self._timer is None:
            self._timer = self.create_timer(dt, self._on_tick_single)

    def _on_tick_single(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._idx >= len(self._rows):
            return
        row = self._rows[self._idx]
        self.spray_pub.publish(Bool(data=(int(row['spray [bool]']) == 1)))
        self._idx += 1
        self._schedule_next()


def main() -> None:
    rclpy.init()
    node = ReplayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
