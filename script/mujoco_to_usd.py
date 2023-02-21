#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pxr import Usd, UsdGeom, Sdf, Gf, Vt
import mujoco
import numpy


def mjcf_to_usd_handle(xml_path: str):
    usd_path = xml_path
    if usd_path.endswith('.xml'):
        usd_path = usd_path.replace('xml', 'usda')
    model = mujoco.MjModel.from_xml_path(xml_path)
    dataid = model.geom('bowl_bottom').dataid[0]

    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    mujocoPrim = UsdGeom.Xform.Define(stage, '/mujoco')

    meshPrim = UsdGeom.Mesh.Define(stage, '/mujoco/mesh')

    print(model.mesh(dataid))
    points = numpy.empty(shape=[model.mesh(dataid).vertnum[0], 3], dtype=float)

    face_vertex_counts = numpy.empty(
        shape=model.mesh(dataid).facenum[0], dtype=float)
    face_vertex_counts.fill(3)
    face_vertex_indices = numpy.empty(
        shape=model.mesh(dataid).facenum[0] * 3, dtype=float)

    for i in range(model.mesh(dataid).vertnum[0]):
        vertid = model.mesh(dataid).vertadr[0] + i
        points[i] = model.mesh_vert[vertid]

    for i in range(model.mesh(dataid).facenum[0]):
        faceid = model.mesh(dataid).faceadr[0] + i
        face_vertex_indices[3*i] = model.mesh_face[faceid][0]
        face_vertex_indices[3*i + 1] = model.mesh_face[faceid][1]
        face_vertex_indices[3*i + 2] = model.mesh_face[faceid][2]

    meshPrim.CreatePointsAttr(points)
    meshPrim.CreateFaceVertexCountsAttr(face_vertex_counts)
    meshPrim.CreateFaceVertexIndicesAttr(face_vertex_indices)

    stage.GetRootLayer().Save()

    return


def mjcf_to_usd_client():
    rospy.wait_for_service('/mujoco/screenshot')
    try:
        screenshot_srv = rospy.ServiceProxy('/mujoco/screenshot', Trigger)
        resp: TriggerResponse = screenshot_srv()
        mjcf_to_usd_handle(resp.message)
        return
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


if __name__ == "__main__":
    mjcf_to_usd_client()
