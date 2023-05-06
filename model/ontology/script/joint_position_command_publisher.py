#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_names = ['box_flap_1_joint', 'box_flap_2_joint']
pubs = [rospy.Publisher(joint_name + '_pos_controller/command', Float64, queue_size=10) for joint_name in joint_names]

def get_joint_state_cb(data):
    for i in range(len(pubs)):
        pubs[i].publish(data.position[i])

if __name__ == '__main__':
    rospy.init_node('joint_position_command', anonymous=True)
    rospy.Subscriber('desired_joint_states', JointState, get_joint_state_cb, queue_size=10)
    rospy.spin()