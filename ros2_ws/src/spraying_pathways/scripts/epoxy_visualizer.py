#!/usr/bin/env python3
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray, Int32

LAYER_FILE = "/tmp/epoxy_layers.json"
# box_0_5 panel: center (1.0, 0.2, 0.77) in world, size 0.4x0.4x0.05 -> top surface at 0.795
# spray_centers from the C++ node are already in world x/y frame (corners 0.8..1.2, 0.0..0.4)
Z_BASE = 0.795
LAYER_HEIGHT = 0.015 # coating thickness per layer (matches standard_h in v4)

# Color per cumulative layer count: (R, G, B, A)
LAYER_COLORS = [
    (1.0, 1.0, 0.2, 0.45),  # layer 1: yellow
    (1.0, 0.6, 0.0, 0.60),  # layer 2: orange
    (1.0, 0.3, 0.0, 0.75),  # layer 3: deep orange
    (0.7, 0.0, 0.0, 0.88),  # layer 4+: dark red
]


class EpoxyVisualizer(Node):
    def __init__(self):
        super().__init__('epoxy_visualizer')

        self.spray_centers = []   # list of (x, y) in world frame, matching panel extents [0.8-1.2, 0.0-0.4]
        self.cube_size_x = 0.0
        self.cube_size_y = 0.0
        self.cell_layers = {}     # cell_idx -> cumulative layer count (from JSON)
        self.current_run_cells = set()  # cells touched in this process lifetime
        self.plan_received = False

        self._load_layers()

        # Transient-local QoS so we receive the plan even if we start after the spray node
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Float64MultiArray, '/spray_plan', self._plan_cb, latched)
        self.create_subscription(Int32, '/spray_current_idx', self._idx_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/epoxy_coating_markers', 10)

        self.create_timer(0.2, self._publish_markers)   # 5 Hz
        self.create_timer(30.0, self._periodic_save)    # autosave every 30 s

        self.get_logger().info('Epoxy visualizer ready. Waiting for /spray_plan ...')

    # ------------------------------------------------------------------ load/save

    def _load_layers(self):
        if not os.path.exists(LAYER_FILE):
            return
        try:
            with open(LAYER_FILE, 'r') as f:
                data = json.load(f)
            self.cell_layers = {int(k): v for k, v in data.get('cells', {}).items()}
            centers = data.get('spray_centers', [])
            if centers:
                self.spray_centers = [tuple(c) for c in centers]
                self.cube_size_x = data.get('cube_size_x', 0.0)
                self.cube_size_y = data.get('cube_size_y', 0.0)
                self.plan_received = True
            self.get_logger().info(
                f'Loaded layer data: {len(self.cell_layers)} cells, '
                f'{len(self.spray_centers)} centers'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not load {LAYER_FILE}: {e}')

    def _save_layers(self):
        # Commit current-run cells into permanent record before saving
        for idx in self.current_run_cells:
            self.cell_layers[idx] = self.cell_layers.get(idx, 0) + 1
        self.current_run_cells.clear()

        data = {
            'cells': {str(k): v for k, v in self.cell_layers.items()},
            'spray_centers': [list(c) for c in self.spray_centers],
            'cube_size_x': self.cube_size_x,
            'cube_size_y': self.cube_size_y,
        }
        try:
            with open(LAYER_FILE, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f'Saved layer data to {LAYER_FILE}')
        except Exception as e:
            self.get_logger().error(f'Could not save {LAYER_FILE}: {e}')

    def _periodic_save(self):
        if self.current_run_cells:
            self._save_layers()

    # ------------------------------------------------------------------ callbacks

    def _plan_cb(self, msg):
        if len(msg.data) < 3:
            return
        self.cube_size_x = msg.data[0]
        self.cube_size_y = msg.data[1]
        n = int(msg.data[2])
        self.spray_centers = [
            (msg.data[3 + i * 2], msg.data[3 + i * 2 + 1]) for i in range(n)
        ]
        self.plan_received = True
        self.get_logger().info(
            f'Spray plan received: {n} centers, '
            f'cell ({self.cube_size_x:.4f} x {self.cube_size_y:.4f}) m'
        )

    def _idx_cb(self, msg):
        idx = msg.data
        if 0 <= idx < len(self.spray_centers):
            self.current_run_cells.add(idx)

    # ------------------------------------------------------------------ markers

    def _cell_total_layers(self, idx):
        base = self.cell_layers.get(idx, 0)
        current = 1 if idx in self.current_run_cells else 0
        return base + current

    def _publish_markers(self):
        if not self.plan_received:
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for idx, (cx, cy) in enumerate(self.spray_centers):
            total = self._cell_total_layers(idx)
            if total == 0:
                continue

            color = LAYER_COLORS[min(total - 1, len(LAYER_COLORS) - 1)]
            height = total * LAYER_HEIGHT

            # Coating cube
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = now
            m.ns = 'epoxy_coating'
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = Z_BASE + height / 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = self.cube_size_x
            m.scale.y = self.cube_size_y
            m.scale.z = height
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.lifetime.sec = 0  # persistent
            markers.markers.append(m)

            # Layer-count text (only when more than one layer)
            if total > 1:
                t = Marker()
                t.header.frame_id = 'world'
                t.header.stamp = now
                t.ns = 'epoxy_labels'
                t.id = idx + 100000
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = cx
                t.pose.position.y = cy
                t.pose.position.z = Z_BASE + height + 0.025
                t.pose.orientation.w = 1.0
                t.scale.z = 0.018  # text height in metres
                t.color.r = t.color.g = t.color.b = t.color.a = 1.0
                t.text = f'L{total}'
                t.lifetime.sec = 0
                markers.markers.append(t)

        if markers.markers:
            self.marker_pub.publish(markers)

    # ------------------------------------------------------------------ shutdown

    def destroy_node(self):
        if self.current_run_cells:
            self._save_layers()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EpoxyVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
