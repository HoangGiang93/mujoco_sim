#!/usr/bin/env python

import rospy
from std_msgs.msg import ColorRGBA

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

from random import random, uniform, randint, seed, shuffle

from math import pi, sin, cos

object = ObjectStatus()
mehes = ["../test/pr2/pr2.xml", 
         "../test/hsrb4s/hsrb4s.xml",
         "../test/tiago/tiago.xml",
         "../test/ridgeback_panda/ridgeback_panda.xml",
         "../test/armar/armar6.xml"]

seed(10)

def spawn_object(i):
    object.info.name = "robot_" + str(i)
    object.info.type = ObjectInfo.MESH
    object.info.movable = True

    x = random()*0.3+0.7
    y = random()*0.7
    z = 1 - x*x - y*y

    r_g_b = [x, y, z]
    shuffle(r_g_b)

    r = r_g_b[0]
    g = r_g_b[1]
    b = r_g_b[2]
    object.info.rgba = ColorRGBA(r, g, b, 1)
    object.info.mesh = mehes[randint(0, len(mehes) - 1)]
    r = uniform(1.5, 2)
    alpha = uniform(-pi, pi)

    object.pose.position.x = r * sin(alpha)
    object.pose.position.y = r * cos(alpha)
    object.pose.position.z = 5
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
    objects.names = ["robot_" + str(i)]
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
        if i >= 8:
            rospy.sleep(0.1)
            destroy_object(i-8)
            rospy.sleep(0.1)
        else:
            rospy.sleep(0.2)
        i+=1
        
