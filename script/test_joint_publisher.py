#!/usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectoryPoint

pub = rospy.Publisher('mujoco/joint_trajectory_points', JointTrajectoryPoint, queue_size=10)
rospy.init_node('test_joint_publisher')
rate = rospy.Rate(1000) # 10hz
joint_trajectory_point = JointTrajectoryPoint()
t_start = rospy.get_rostime()

q_0 = math.pi / 3
omega = 2 * math.pi * 0.1
while not rospy.is_shutdown():
    t = rospy.get_rostime() - t_start
    q = q_0 * math.sin(omega * t.to_sec())
    dq = omega * q_0 * math.cos(omega * t.to_sec())
    ddq = -omega * omega * q_0 * math.sin(omega * t.to_sec())
    joint_trajectory_point.positions = [q, q, 3*q, 4*q, q ,q, q]
    joint_trajectory_point.velocities = [dq, dq, 3*dq, 4*dq, dq ,dq, dq]
    joint_trajectory_point.accelerations = [ddq, ddq, 3*ddq, 4*ddq, ddq, ddq, ddq]
    joint_trajectory_point.time_from_start = t
    pub.publish(joint_trajectory_point)
    rate.sleep()