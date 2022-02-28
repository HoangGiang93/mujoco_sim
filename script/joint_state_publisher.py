#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

joint_trajectory_point = JointTrajectoryPoint()
pub = rospy.Publisher('joint_trajectory_points', JointTrajectoryPoint, queue_size=10)

def get_joint_state_cb(data):
    joint_trajectory_point.positions = data.position
    joint_trajectory_point.velocities = [0 for _ in range(len(data.position))]
    joint_trajectory_point.accelerations = [0 for _ in range(len(data.position))]
    pub.publish(joint_trajectory_point)

if __name__ == '__main__':
    rospy.init_node('joint_state_publisher', anonymous=True)
    rospy.Subscriber('desired_joint_states', JointState, get_joint_state_cb, queue_size=10)
    rospy.spin()
