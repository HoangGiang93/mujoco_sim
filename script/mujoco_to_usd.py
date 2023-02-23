#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pxr import Usd, UsdGeom, Sdf, Gf, Vt
import mujoco
import numpy
import xml.etree.ElementTree as ET
import os
import csv


def get_data(data, key: list, i: int, length: int) -> None:
    i += 1
    while True:
        try:
            if length != 9:
                key.append([float(data[i][j]) for j in range(length)])
            else:
                key.append([[float(data[i][3*k + j])
                           for j in range(3)] for k in range(3)])
            i += 1
        except ValueError:
            return None


def mjcf_to_usd_handle(xml_path: str):
    usd_dir = os.path.dirname(xml_path)
    usd_file = os.path.basename(xml_path)
    usd_file = usd_file.replace("xml", "usda")

    mesh_dict = {}
    tree = ET.parse(xml_path)
    root = tree.getroot()

    mesh_root_dir = usd_dir
    for compiler in root.iter("compiler"):
        if compiler.attrib.get("meshdir") is not None:
            mesh_root_dir = compiler.attrib.get("meshdir")
            break

    for asset in root.iter("asset"):
        for mesh in asset.iter("mesh"):
            mesh_name = mesh.attrib.get("name")
            mesh_dir = os.path.dirname(mesh.attrib.get("file"))
            mesh_file = os.path.basename(mesh.attrib.get("file"))
            mesh_file = mesh_file.replace("stl", "usda")
            mesh_dir = os.path.dirname(mesh_dir) + "/usd"
            mesh_dict[mesh_name] = os.path.join(
                mesh_root_dir, mesh_dir, mesh_file)

    model = mujoco.MjModel.from_xml_path(xml_path)

    for mesh_id in range(model.nmesh):
        mj_mesh = model.mesh(mesh_id)

        if os.path.exists(mesh_dict[mj_mesh.name]):
            continue

        stage = Usd.Stage.CreateNew(mesh_dict[mj_mesh.name])

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
        usd_mesh = UsdGeom.Mesh.Define(
            stage, "/Mesh_" + mj_mesh.name.replace('-', '_'))
        stage.SetDefaultPrim(usd_mesh.GetPrim())

        points = numpy.empty(
            shape=[model.mesh(mesh_id).vertnum[0], 3], dtype=float)
        normals = numpy.empty(
            shape=[model.mesh(mesh_id).vertnum[0], 3], dtype=float)

        face_vertex_counts = numpy.empty(
            shape=model.mesh(mesh_id).facenum[0], dtype=float
        )
        face_vertex_counts.fill(3)
        face_vertex_indices = numpy.empty(
            shape=model.mesh(mesh_id).facenum[0] * 3, dtype=float
        )

        for i in range(model.mesh(mesh_id).vertnum[0]):
            vertid = model.mesh(mesh_id).vertadr[0] + i
            points[i] = model.mesh_vert[vertid]
            normals[i] = model.mesh_normal[vertid]

        for i in range(model.mesh(mesh_id).facenum[0]):
            faceid = model.mesh(mesh_id).faceadr[0] + i
            face_vertex_indices[3 * i] = model.mesh_face[faceid][0]
            face_vertex_indices[3 * i + 1] = model.mesh_face[faceid][1]
            face_vertex_indices[3 * i + 2] = model.mesh_face[faceid][2]

        usd_mesh.CreatePointsAttr(points)
        usd_mesh.CreateNormalsAttr(normals)
        usd_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
        usd_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)

        stage.Save()

    xpos = []
    xmat = []

    data_dir = os.path.dirname(xml_path)
    data_file = os.path.basename(xml_path)
    data_file = data_file.replace(".xml", "_data.txt")

    with open(os.path.join(data_dir, data_file), newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=' ',
                            skipinitialspace=True, quoting=csv.QUOTE_NONE)
        data = [row for row in reader]

    for i in range(len(data)):
        if len(data[i]) == 0:
            continue
        if data[i][0] == 'XPOS':
            get_data(data, xpos, i, 3)

        elif data[i][0] == 'XMAT':
            get_data(data, xmat, i, 9)

    stage = Usd.Stage.CreateNew(os.path.join(usd_dir, usd_file))

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)

    root_path = Sdf.Path("/Root")
    root_prim = UsdGeom.Xform.Define(stage, root_path)

    for body_id in range(1, model.nbody):
        body = model.body(body_id)
        parent_body_id = body.parentid[0]
        body_path = root_path
        parent_path = ""
        while parent_body_id != 0:
            parent_body_name = model.body(parent_body_id).name
            if parent_body_name == "":
                parent_body_name = "body_" + str(parent_body_id)
            parent_path = parent_body_name + "/" + parent_path
            parent_body_id = model.body(parent_body_id).parentid[0]

        body_name = body.name
        if body_name == "":
            body_name = "body_" + str(body_id)

        if parent_path == "":
            body_path = body_path.AppendPath(body_name)
        else:
            parent_path = parent_path[:-1]
            body_path = body_path.AppendPath(parent_path).AppendPath(body_name)

        body_prim = UsdGeom.Xform.Define(stage, body_path)
        transform = body_prim.AddTransformOp()
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(body.pos[0], body.pos[1], body.pos[2]))
        mat.SetRotateOnly(
            Gf.Quatd(body.quat[0], body.quat[1], body.quat[2], body.quat[3]))
        transform.Set(mat)

        for i in range(body.geomnum[0]):
            geom_id = body.geomadr[0] + i
            geom = model.geom(geom_id)
            if geom.name == "":
                geom_path = body_path.AppendPath("geom_" + str(geom_id))
            else:
                geom_path = body_path.AppendPath(geom.name.replace('-', '_'))

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(
                Gf.Vec3d(geom.pos[0], geom.pos[1], geom.pos[2]))
            mat.SetRotateOnly(
                Gf.Quatd(geom.quat[0], geom.quat[1],
                         geom.quat[2], geom.quat[3])
            )

            if geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_prim = UsdGeom.Cube.Define(stage, geom_path)
                mat_scale = Gf.Matrix4d()
                mat_scale.SetScale(
                    Gf.Vec3d(geom.size[0], geom.size[1], geom.size[2]))
                mat = mat_scale * mat

            elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                geom_prim = UsdGeom.Sphere.Define(stage, geom_path)
                geom_prim.CreateRadiusAttr(geom.size[0])
                geom_prim.CreateExtentAttr(
                    numpy.array([-1, -1, -1, 1, 1, 1]) * geom.size[0])

            elif geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                geom_prim = UsdGeom.Cylinder.Define(stage, geom_path)
                geom_prim.CreateRadiusAttr(geom.size[0])
                geom_prim.CreateHeightAttr(geom.size[1]*2)
                geom_prim.CreateExtentAttr(numpy.array(
                    [[-geom.size[0], -geom.size[0], -geom.size[1]],
                        [geom.size[0], geom.size[0], geom.size[1]]]
                ))

            elif geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = geom.dataid[0]
                mesh_name = model.mesh(mesh_id).name
                geom_prim = UsdGeom.Mesh.Define(stage, geom_path)
                geom_prim.GetPrim().GetReferences(
                ).AddReference(mesh_dict[mesh_name])

            transform = geom_prim.AddTransformOp()
            transform.Set(mat)

            geom_prim.CreateDisplayColorAttr(geom.rgba[:3])
            geom_prim.CreateDisplayOpacityAttr(geom.rgba[3])

    stage.SetDefaultPrim(root_prim.GetPrim())

    stage.Save()

    return


def mjcf_to_usd_client():
    rospy.wait_for_service("/mujoco/screenshot")
    try:
        screenshot_srv = rospy.ServiceProxy("/mujoco/screenshot", Trigger)
        resp: TriggerResponse = screenshot_srv()
        mjcf_to_usd_handle(resp.message)
        return
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s" % e)


if __name__ == "__main__":
    mjcf_to_usd_client()
