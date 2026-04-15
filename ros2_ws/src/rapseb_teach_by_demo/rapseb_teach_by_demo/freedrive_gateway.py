"""Thin gateway that toggles UR freedrive via the ur_robot_driver dashboard.

Exposes a std_srvs/Trigger service '/rapseb/toggle_freedrive'. When called,
it loads the 'freedrive.urp' program on the teach pendant and starts it,
or stops the currently running program if freedrive is already active.

The gateway assumes the ur_robot_driver is running and publishes the dashboard
services. In simulation this node stays idle and reports 'freedrive not
available in simulation'.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

try:
    from ur_dashboard_msgs.srv import Load
    from std_srvs.srv import Trigger as UrTrigger  # play/stop
    HAVE_UR = True
except Exception:
    HAVE_UR = False


class FreedriveGateway(Node):
    def __init__(self) -> None:
        super().__init__('freedrive_gateway')

        self.declare_parameter('freedrive_program', 'freedrive.urp')
        self.declare_parameter('simulation', False)
        self.program_name = self.get_parameter('freedrive_program').value
        self.simulation = bool(self.get_parameter('simulation').value)

        self.srv = self.create_service(
            Trigger, '/rapseb/toggle_freedrive', self._on_toggle)

        self.active = False

        if not self.simulation and HAVE_UR:
            self.load_cli = self.create_client(Load, '/dashboard_client/load_program')
            self.play_cli = self.create_client(UrTrigger, '/dashboard_client/play')
            self.stop_cli = self.create_client(UrTrigger, '/dashboard_client/stop')

        self.get_logger().info(
            f'freedrive_gateway ready (simulation={self.simulation}, '
            f'ur_msgs_available={HAVE_UR})')

    def _on_toggle(self, _req: Trigger.Request,
                   resp: Trigger.Response) -> Trigger.Response:
        if self.simulation or not HAVE_UR:
            resp.success = False
            resp.message = 'freedrive not available in simulation'
            return resp

        if not self.active:
            if not self.load_cli.wait_for_service(timeout_sec=1.0):
                resp.success = False
                resp.message = 'dashboard load_program not ready'
                return resp
            req = Load.Request()
            req.filename = self.program_name
            self.load_cli.call_async(req)
            self.play_cli.call_async(UrTrigger.Request())
            self.active = True
            resp.success = True
            resp.message = f'freedrive ON ({self.program_name})'
        else:
            if self.stop_cli.wait_for_service(timeout_sec=1.0):
                self.stop_cli.call_async(UrTrigger.Request())
            self.active = False
            resp.success = True
            resp.message = 'freedrive OFF'
        return resp


def main() -> None:
    rclpy.init()
    node = FreedriveGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
