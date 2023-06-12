#!/usr/bin/env python3

from random import uniform, randint
from math import pi, sin, cos

import rospy
from std_msgs.msg import ColorRGBA

from mujoco_msgs.msg import ObjectStatus, ObjectInfo
from mujoco_msgs.srv import SpawnObject, SpawnObjectRequest, DestroyObject, DestroyObjectRequest

object_status = ObjectStatus()
types = [ObjectInfo.CUBE, ObjectInfo.SPHERE,
         ObjectInfo.CYLINDER, ObjectInfo.MESH]
meshes = ["../test/box.xml", "../test/cup.xml"]

colors = [
    ColorRGBA(0, 0, 1, 1),
    ColorRGBA(0, 1, 1, 1),
    ColorRGBA(0, 1, 0, 1),
    ColorRGBA(1, 0, 0.5, 1),
    ColorRGBA(0.5, 0, 1, 1),
    ColorRGBA(1, 0, 0, 1),
    ColorRGBA(1, 1, 0, 1),
]


def spawn_object(i: int) -> None:
    object_status.info.name = "object_" + str(i)
    object_status.info.type = types[randint(0, len(types) - 1)]
    object_status.info.movable = True
    scale = randint(200, 500) / 100.0
    object_status.info.rgba = colors[randint(0, len(colors) - 1)]
    object_status.info.mesh = meshes[randint(0, len(meshes) - 1)]
    object_status.info.size.x = 1
    object_status.info.size.y = 1
    object_status.info.size.z = 1
    if object_status.info.type != ObjectInfo.MESH:
        object_status.info.size.x = 0.05 * scale
        object_status.info.size.y = 0.05 * scale
        object_status.info.size.z = 0.05 * scale
    elif object_status.info.mesh != "../test/box.xml":
        object_status.info.size.x = scale
        object_status.info.size.y = scale
        object_status.info.size.z = scale
    alpha = uniform(-pi, pi)
    r = uniform(1.5, 2)
    object_status.pose.position.x = r * sin(alpha)
    object_status.pose.position.y = r * cos(alpha)
    object_status.pose.position.z = 5
    object_status.pose.orientation.x = 0.0
    object_status.pose.orientation.y = 0.0
    object_status.pose.orientation.z = 0.0
    object_status.pose.orientation.w = 1.0

    objects = SpawnObjectRequest()
    objects.objects = [object_status]
    rospy.wait_for_service("/mujoco/spawn_objects")
    try:
        spawn_objects = rospy.ServiceProxy(
            "/mujoco/spawn_objects", SpawnObject
        )
        spawn_resp = spawn_objects(objects)
        rospy.loginfo("Spawn response: " + str(spawn_resp))
    except rospy.ServiceException as error:
        print(f"Service call failed: {error}")


def destroy_object(i: int) -> None:
    rospy.wait_for_service("/mujoco/destroy_objects")
    objects = DestroyObjectRequest()
    objects.names = ["object_" + str(i)]
    try:
        destroy_objects = rospy.ServiceProxy(
            "/mujoco/destroy_objects", DestroyObject
        )
        destroy_resp = destroy_objects(objects)
        rospy.loginfo("Destroy response: " + str(destroy_resp))
    except rospy.ServiceException as error:
        print(f"Service call failed: {error}")


if __name__ == "__main__":
    rospy.init_node("test")
    object_id = 0
    while not rospy.is_shutdown():
        spawn_object(object_id)
        if object_id >= 20:
            rospy.sleep(0.15)
            destroy_object(object_id-20)
            rospy.sleep(0.15)
        else:
            rospy.sleep(0.3)
        object_id += 1
