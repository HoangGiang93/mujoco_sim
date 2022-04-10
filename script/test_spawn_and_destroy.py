#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA

from mujoco_msgs.msg import ObjectState
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

from random import random, uniform, randint

from math import pi, sin, cos

object = ObjectState()
types = [ObjectState.CUBE, ObjectState.SPHERE, ObjectState.CYLINDER]



color = [
    ColorRGBA(0, 0, 1, 1),
    ColorRGBA(0, 1, 1, 1),
    ColorRGBA(0, 1, 0, 1),
    ColorRGBA(1, 0, 0.5, 1),
    ColorRGBA(0.5, 0, 1, 1),
    ColorRGBA(1, 0, 0, 1),
    ColorRGBA(1, 1, 0, 1),
]

def spawn_object(i):
    object.name = "object_" + str(i)
    alpha = uniform(-pi, pi)
    r = uniform(1.5, 2)
    object.pose.position.x = r * sin(alpha)
    object.pose.position.y = r * cos(alpha)
    object.pose.position.z = 2
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    object.type = ObjectState.SPHERE
    object.scale.x = 0.1
    object.scale.y = 0.1
    object.scale.z = 0.1
    object.color = color[randint(0, len(color) - 1)]

    objects = SpawnObjectRequest()
    objects.object_states = [object]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy(
            "/mujoco/spawn_objects", SpawnObject
        )
        spawn_objects(objects)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def destroy_object(i):
    rospy.wait_for_service("/mujoco/destroy_objects")
    objects = DestroyObjectRequest()
    objects.names = ["object_" + str(i)]
    try:
        destroy_objects = rospy.ServiceProxy(
            "/mujoco/destroy_objects", DestroyObject
        )
        destroy_objects(objects)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("test")
    i = 0
    while not rospy.is_shutdown():
        spawn_object(i)
        if i > 20:
            destroy_object(i-20)
        i+=1
        rospy.sleep(0.1)
