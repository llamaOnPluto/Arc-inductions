#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute


class Swarmer(Node):
    def __init__(self):
        super().__init__("swarmer")

        # Wait for spawn service
        self.spawn_cli = self.create_client(Spawn, "/spawn")
        while not self.spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spawn service...")

        # Dictionary to store teleport service clients
        self.teleport_clients = {}

        # Spawn turtles for the orbits
        for name in ["circle", "rose2", "rose3", "rose5"]:
            self.spawn_turtle(name, 5.0, 5.0)

        # Timer to update positions (slower so you can enjoy the tracing)
        self.t = 0.0
        self.timer = self.create_timer(0.1, self.update_positions)  # 10 Hz

    def spawn_turtle(self, name, x, y):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0
        req.name = name

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawned turtle: {name}")
            # Create teleport client for this turtle
            self.teleport_clients[name] = self.create_client(
                TeleportAbsolute, f"/{name}/teleport_absolute"
            )
            while not self.teleport_clients[name].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for /{name}/teleport_absolute...")
        else:
            self.get_logger().error(f"Failed to spawn turtle {name}")

    def teleport(self, name, x, y):
        if name not in self.teleport_clients:
            return
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = 0.0
        self.teleport_clients[name].call_async(req)

    def update_positions(self):
        # Circle orbit (radius 3)
        x = 5.0 + 3.0 * math.cos(self.t)
        y = 5.0 + 3.0 * math.sin(self.t)
        self.teleport("circle", x, y)

        # Rose curves (scaled up so they don’t overlap)
        scale = 3.0
        for name, n in [("rose2", 4), ("rose3", 7), ("rose5", 9)]:
            r = math.sin(n * self.t) * scale
            x = 5.0 + r * math.cos(self.t)
            y = 5.0 + r * math.sin(self.t)
            self.teleport(name, x, y)

        # Slow angular progression
        self.t += 0.01


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

