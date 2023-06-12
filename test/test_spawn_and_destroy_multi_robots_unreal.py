#!/usr/bin/env python3

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

mehes_UE = ["/Game/SkeletalMeshes/Robots/pr2/pr2.pr2",
            "/Game/SkeletalMeshes/Robots/hsrb4s/hsrb4s.hsrb4s",
            "/Game/SkeletalMeshes/Robots/tiago/tiago.tiago",
            "/Game/SkeletalMeshes/Robots/ridgeback_panda/ridgeback_panda.ridgeback_panda",
            "/Game/SkeletalMeshes/Robots/armar6/armar6.armar6"]

seed(10)

def spawn_object_mujoco(mesh_idx, i, r, alpha):
    object.info.name = "robot_" + str(i)
    object.info.type = ObjectInfo.MESH
    object.info.movable = True

    object.info.mesh = mehes[mesh_idx]

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

def spawn_object_unreal(mesh_idx, i, r, alpha):
    object.info.name = "robot_" + str(i)
    object.info.type = ObjectInfo.MESH
    object.info.movable = True
    object.info.size.x = 1
    object.info.size.y = 1
    object.info.size.z = 1
    object.info.mesh = mehes_UE[mesh_idx]

    object.pose.position.x = r * sin(alpha)
    object.pose.position.y = r * cos(alpha)
    object.pose.position.z = 5
    object.pose.orientation.x = 0.0
    object.pose.orientation.y = 0.0
    object.pose.orientation.z = 0.0
    object.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object]
    rospy.wait_for_service("/unreal/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy(
            "/unreal/spawn_objects", SpawnObject
        )
        spawn_resp = spawn_objects(objects)
        rospy.loginfo("Spawn response: " + str(spawn_resp))
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node("test")
    i = 0
    while not rospy.is_shutdown():
        mesh_idx = randint(0, len(mehes) - 1)
        r = uniform(1.5, 2)
        alpha = uniform(-pi, pi)
        spawn_object_mujoco(mesh_idx, i, r, alpha)
        spawn_object_unreal(mesh_idx, i, r, alpha)
        rospy.sleep(1)
        i+=1
        if i > 10:
            break
        
