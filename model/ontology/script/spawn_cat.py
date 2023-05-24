#!/usr/bin/env python3

import rospy

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

from random import uniform
from math import pi, sin, cos

cat_name = 'cat'

def spawn_cat():
    cat = ObjectStatus()
    cat.info.name = cat_name
    cat.info.type = ObjectInfo.MESH
    cat.info.movable = True
    cat.info.mesh = 'cat/cat.xml'
    alpha = uniform(-pi, pi)
    r = uniform(0.0, 0.5)
    cat.pose.position.x = r * sin(alpha)
    cat.pose.position.y = r * cos(alpha)
    cat.pose.position.z = 0.5
    cat.pose.orientation.x = 0.0
    cat.pose.orientation.y = 0.0
    cat.pose.orientation.z = 0.0
    cat.pose.orientation.w = 1.0

    cats = SpawnObjectRequest()
    cats.objects = [cat]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy(
            "/mujoco/spawn_objects", SpawnObject
        )
        spawn_resp = spawn_objects(cats)
        rospy.loginfo("Spawn response: " + str(spawn_resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def destroy_cat():
    rospy.wait_for_service("/mujoco/destroy_objects")
    cats = DestroyObjectRequest()
    cats.names = [cat_name]
    try:
        destroy_objects = rospy.ServiceProxy(
            "/mujoco/destroy_objects", DestroyObject
        )
        destroy_resp = destroy_objects(cats)
        rospy.loginfo("Destroy response: " + str(destroy_resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("spawn_cat")
    destroy_cat()
    rospy.sleep(1)
    spawn_cat()