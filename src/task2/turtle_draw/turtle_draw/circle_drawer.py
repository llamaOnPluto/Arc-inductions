#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

def main(args=None):
    # initialize ROS 2 
    rclpy.init(args=args)

    # create a node
    node = rclpy.create_node('simple_circle_mover')

    # create a publisher to publish messages
    pub = node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    # actual motion of turtle
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = 1.0

    try:
        # keep publishing the message so that the turtle continues to move. Small sleep timer to control the speed.
        while rclpy.ok():
            pub.publish(msg)
            time.sleep(0.1)  # 10 Hz
    except KeyboardInterrupt:
        # allow Ctrl+C in the terminal to stop it cleanly
        pass
    finally:
        # cleanup (destroy the node when the program is done)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
