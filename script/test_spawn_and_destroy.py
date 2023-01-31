#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

from random import random, uniform, randint

from math import pi, sin, cos

object = ObjectStatus()
types = [ObjectInfo.CUBE, ObjectInfo.SPHERE, ObjectInfo.CYLINDER, ObjectInfo.MESH, ObjectInfo.MESH]
meshes = ["../test/cup.xml", "../test/bowl_small.xml"]

colors = [
    ColorRGBA(0, 0, 1, 1),
    ColorRGBA(0, 1, 1, 1),
    ColorRGBA(0, 1, 0, 1),
    ColorRGBA(1, 0, 0.5, 1),
    ColorRGBA(0.5, 0, 1, 1),
    ColorRGBA(1, 0, 0, 1),
    ColorRGBA(1, 1, 0, 1),
]

def spawn_object(i):
    object.info.name = "object_" + str(i)
    object.info.type = types[randint(0, len(types) - 1)]
    object.info.movable = True
    if object.info.type != ObjectInfo.MESH:
        object.info.size.x = 0.05
        object.info.size.y = 0.05
        object.info.size.z = 0.05
    else:
        object.info.size.x = 1
        object.info.size.y = 1
        object.info.size.z = 1
    object.info.rgba = colors[randint(0, len(colors) - 1)]
    alpha = uniform(-pi, pi)
    r = uniform(1.5, 2)
    object.info.mesh = meshes[randint(0, len(meshes) - 1)]
    object.pose.position.x = r * sin(alpha)
    object.pose.position.y = r * cos(alpha)
    object.pose.position.z = 2
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy(
            "/mujoco/spawn_objects", SpawnObject
        )
        spawn_resp = spawn_objects(objects)
        rospy.loginfo("Spawn response: " + str(spawn_resp))
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
        destroy_resp = destroy_objects(objects)
        rospy.loginfo("Destroy response: " + str(destroy_resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("test")
    i = 0
    while not rospy.is_shutdown():
        spawn_object(i)
        if i > 10:
            rospy.sleep(0.2)
            destroy_object(i-20)
            rospy.sleep(0.2)
        else:
            rospy.sleep(0.4)
        i+=1
        
