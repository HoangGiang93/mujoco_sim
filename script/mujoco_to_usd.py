#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from pxr import Usd, UsdGeom, Sdf, Gf, Vt
import mujoco
import numpy
import xml.etree.ElementTree as ET
import os


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
            mesh_dict[mesh_name] = os.path.join(mesh_root_dir, mesh_dir, mesh_file)

    model = mujoco.MjModel.from_xml_path(xml_path)

    for mesh_id in range(model.nmesh):
        mj_mesh = model.mesh(mesh_id)

        if os.path.exists(mesh_dict[mj_mesh.name]):
            continue

        stage = Usd.Stage.CreateNew(mesh_dict[mj_mesh.name])

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)
        usd_mesh = UsdGeom.Mesh.Define(stage, "/" + mj_mesh.name)
        stage.SetDefaultPrim(usd_mesh.GetPrim())

        points = numpy.empty(shape=[model.mesh(mesh_id).vertnum[0], 3], dtype=float)
        normals = numpy.empty(shape=[model.mesh(mesh_id).vertnum[0], 3], dtype=float)

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
        mat.SetTranslateOnly(
            Gf.Vec3d(
                model.body(body_id).pos[0],
                model.body(body_id).pos[1],
                model.body(body_id).pos[2],
            )
        )
        mat.SetRotateOnly(
            Gf.Quatd(
                model.body(body_id).quat[0],
                model.body(body_id).quat[1],
                model.body(body_id).quat[2],
                model.body(body_id).quat[3],
            )
        )
        transform.Set(mat)

        for i in range(body.geomnum[0]):
            geom_id = body.geomadr[0] + i
            geom = model.geom(geom_id)
            if geom.name == "":
                geom_path = body_path.AppendPath("geom_" + str(geom_id))
            else:
                geom_path = body_path.AppendPath(geom.name)

            mat = Gf.Matrix4d()
            mat.SetTranslateOnly(
                Gf.Vec3d(
                    model.geom(geom_id).pos[0],
                    model.geom(geom_id).pos[1],
                    model.geom(geom_id).pos[2],
                )
            )
            mat.SetRotateOnly(
                Gf.Quatd(
                    model.geom(geom_id).quat[0],
                    model.geom(geom_id).quat[1],
                    model.geom(geom_id).quat[2],
                    model.geom(geom_id).quat[3],
                )
            )

            if geom.type == mujoco.mjtGeom.mjGEOM_BOX:
                geom_prim = UsdGeom.Cube.Define(stage, geom_path)
                mat_scale = Gf.Matrix4d()
                mat_scale.SetScale(
                    Gf.Vec3d(
                        model.geom(geom_id).size[0],
                        model.geom(geom_id).size[1],
                        model.geom(geom_id).size[2],
                    )
                )
                mat = mat_scale * mat
                extend_attr = geom_prim.GetExtentAttr()
                extend_attr.Set(
                    extend_attr.Get()
                    * numpy.array(
                        [
                            [
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[1],
                            ],
                            [
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[1],
                            ],
                        ]
                    )
                )

            elif geom.type == mujoco.mjtGeom.mjGEOM_SPHERE:
                geom_prim = UsdGeom.Sphere.Define(stage, geom_path)
                radius_attr = geom_prim.GetRadiusAttr()
                radius_attr.Set(radius_attr.Get() * model.geom(geom_id).size[0])
                extend_attr = geom_prim.GetExtentAttr()
                extend_attr.Set(extend_attr.Get() * model.geom(geom_id).size[0])

            elif geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                geom_prim = UsdGeom.Cylinder.Define(stage, geom_path)
                radius_attr = geom_prim.GetRadiusAttr()
                radius_attr.Set(radius_attr.Get() * model.geom(geom_id).size[0])
                height_attr = geom_prim.GetHeightAttr()
                height_attr.Set(height_attr.Get() * model.geom(geom_id).size[1])
                extend_attr = geom_prim.GetExtentAttr()
                extend_attr.Set(
                    extend_attr.Get()
                    * numpy.array(
                        [
                            [
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[1],
                            ],
                            [
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[0],
                                model.geom(geom_id).size[1],
                            ],
                        ]
                    )
                )

            elif geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = geom.dataid[0]
                mesh_name = model.mesh(mesh_id).name
                geom_prim = UsdGeom.Mesh.Define(stage, geom_path)
                geom_prim.GetPrim().GetReferences().AddReference(mesh_dict[mesh_name])

            transform = geom_prim.AddTransformOp()
            transform.Set(mat)

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
