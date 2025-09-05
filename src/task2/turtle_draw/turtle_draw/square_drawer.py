#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

def _publish_for(pub, node, msg, duration, rate=10):
    """Publish `msg` at `rate` Hz for `duration` seconds."""
    interval = 1.0 / rate
    end_t = time.time() + duration
    while rclpy.ok() and time.time() < end_t:
        pub.publish(msg)
        # let ROS do any housekeeping (not strictly necessary here but polite)
        time.sleep(interval)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('simple_square_mover')
    pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # Messages
    forward = Twist()
    forward.linear.x = 2.0   # forward speed (units/sec)
    turn = Twist()
    turn.angular.z = 1.57    # rad/sec (~90°/sec)
    stop = Twist()           # all zeros => stop

    # allow publisher/subscriber connections to establish
    time.sleep(1.0)

    try:
        for side in range(4):
            node.get_logger().info(f'Drawing side {side+1} / 4')

            # move forward for 2 seconds (publish repeatedly)
            _publish_for(pub, node, forward, duration=2.0, rate=10)

            # ensure fully stopped for a moment
            _publish_for(pub, node, stop, duration=0.2, rate=10)

            # turn ~90 degrees (1.57 rad/sec * 1.0s ≈ 90°)
            _publish_for(pub, node, turn, duration=1.0, rate=10)

            # small pause and stop before next side
            _publish_for(pub, node, stop, duration=0.2, rate=10)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
