#!/usr/bin/env python3

import sys
import os
import rospy
from owlready2 import onto_path, get_ontology, declare_datatype, destroy_entity
from numpy import float64, pi
import rospkg
from sensor_msgs.msg import JointState

onto_file = None

def double_parser(string: str):
    return float64(string)


def double_unparser(x: float64):
    return str(x)


declare_datatype(float64, 'http://www.w3.org/2001/XMLSchema#double',
                 double_parser, double_unparser)


def update_joint_value(dul_onto, usd_onto, joint_name, joint_value):
    joint_quality = joint_name + '_jointValue'
    jointValue_inst = dul_onto.Quality(joint_quality, namespace=usd_onto)
    destroy_entity(jointValue_inst) #owlready does not allow to update data
    jointValue_inst = dul_onto.Quality(joint_quality, namespace=usd_onto)
    usd_onto[joint_name].hasQuality.append(jointValue_inst)
    jointValue_inst.hasJointValue = [float64(joint_value) / (2*pi)]
    return None

def mjcf_to_ABox():
    if not os.path.exists(onto_file):
        return None
    joint_states = rospy.wait_for_message('joint_states', JointState, 1)
    rospack = rospkg.RosPack()

    upper_onto_path = rospack.get_path('mujoco_sim') + '/model/owl/'
    onto_path.append(upper_onto_path)

    ABox_onto = get_ontology('file://' + onto_file)
    ABox_onto.load()

    dul_onto = get_ontology('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl')

    usd_onto = get_ontology('https://ease-crc.org/ont/USD.owl')

    with ABox_onto:
        for i in range(len(joint_states.name)):
            joint_name = joint_states.name[i]
            joint_value = joint_states.position[i]
            update_joint_value(dul_onto, usd_onto, joint_name, joint_value)

    ABox_onto.save(onto_file)
    return None

if __name__ == '__main__':
    if len(sys.argv) >= 2:
        onto_file = sys.argv[1]
    else:
        print('Usage: in_onto.owl')
        sys.exit(1)
    rospy.init_node('mujoco_to_ABox')
    mjcf_to_ABox()
