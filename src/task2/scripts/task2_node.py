#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node('task2_node')
    rospy.loginfo("Task 2 node is running!")
    rospy.spin()

if __name__ == "__main__":
    main()

