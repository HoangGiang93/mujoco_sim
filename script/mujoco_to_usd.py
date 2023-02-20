#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pxr import Usd, UsdGeom
import mujoco

def mjcf_to_usd_client():
    rospy.wait_for_service('/mujoco/screenshot')
    try:
        screenshot_srv = rospy.ServiceProxy('/mujoco/screenshot', Trigger)
        resp: TriggerResponse = screenshot_srv()
        stage = Usd.Stage.CreateNew('HelloWorld.usda')
        xformPrim = UsdGeom.Xform.Define(stage, '/hello')
        spherePrim = UsdGeom.Sphere.Define(stage, '/hello/world')
        stage.GetRootLayer().Save()
        model = mujoco.MjModel.from_xml_path(resp.message)
        data = mujoco.MjData(model)
        while data.time < 1:
            mujoco.mj_step(model, data)
            print(data.geom_xpos)
        return
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" %e)


if __name__ == "__main__":
    mjcf_to_usd_client()
