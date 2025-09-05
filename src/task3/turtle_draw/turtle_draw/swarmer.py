#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute, Kill, SetPen


class Swarmer(Node):
    def __init__(self):
        super().__init__("swarmer")

        # Shifted center so radius 5 fits nicely inside
        self.center_x = 6.0
        self.center_y = 6.0
        self.R = 5.0  # radius

        # Service clients
        self.spawn_cli = self.create_client(Spawn, "/spawn")
        self.kill_cli = self.create_client(Kill, "/kill")

        for cli, name in [(self.spawn_cli, "/spawn"), (self.kill_cli, "/kill")]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name} service...")

        self.teleport_clients = {}
        self.setpen_clients = {}

        # Circle config
        self.circle_cfg = {
            "name": "circle",
            "color": (255, 0, 0),
            "width": 3,
        }

        # Roses (n=2,3,4)
        self.rose_cfgs = [
            {"name": "rose2", "n": 2, "color": (0, 0, 255), "width": 3},
            {"name": "rose3", "n": 3, "color": (0, 180, 0), "width": 3},
            {"name": "rose4", "n": 4, "color": (0, 0, 0), "width": 3},
        ]

        # Kill/re-spawn turtles
        self._spawn_and_setup(self.circle_cfg["name"], self.circle_cfg["color"], self.circle_cfg["width"])
        for cfg in self.rose_cfgs:
            self._spawn_and_setup(cfg["name"], cfg["color"], cfg["width"])

        self.theta = 0.0
        self.timer = self.create_timer(0.05, self.update_positions)

    def _spawn_and_setup(self, name, color, width):
        # kill old
        try:
            kreq = Kill.Request()
            kreq.name = name
            fut_k = self.kill_cli.call_async(kreq)
            rclpy.spin_until_future_complete(self, fut_k, timeout_sec=0.5)
        except Exception:
            pass

        # spawn
        sreq = Spawn.Request()
        sreq.x = self.center_x
        sreq.y = self.center_y
        sreq.theta = 0.0
        sreq.name = name
        fut = self.spawn_cli.call_async(sreq)
        rclpy.spin_until_future_complete(self, fut)

        if fut.result() is not None:
            tp_client = self.create_client(TeleportAbsolute, f"/{name}/teleport_absolute")
            sp_client = self.create_client(SetPen, f"/{name}/set_pen")
            for cli, srv in [(tp_client, f"/{name}/teleport_absolute"), (sp_client, f"/{name}/set_pen")]:
                while not cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f"Waiting for {srv}...")
            self.teleport_clients[name] = tp_client
            self.setpen_clients[name] = sp_client

            # pen setup
            pen_req = SetPen.Request()
            pen_req.r, pen_req.g, pen_req.b = color
            pen_req.width = width
            pen_req.off = 0
            sp_client.call_async(pen_req)

    def teleport(self, name, x, y):
        if name not in self.teleport_clients:
            return
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0
        self.teleport_clients[name].call_async(req)

    def update_positions(self):
        theta = self.theta

        # Circle r=5
        cx = self.center_x + self.R * math.cos(theta)
        cy = self.center_y + self.R * math.sin(theta)
        self.teleport(self.circle_cfg["name"], cx, cy)

        # Roses r = 5*sin(nθ)
        for cfg in self.rose_cfgs:
            r = self.R * math.sin(cfg["n"] * theta)
            x = self.center_x + r * math.cos(theta)
            y = self.center_y + r * math.sin(theta)
            self.teleport(cfg["name"], x, y)

        self.theta += 0.02  # slow for nice drawing


def main(args=None):
    rclpy.init(args=args)
    node = Swarmer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

