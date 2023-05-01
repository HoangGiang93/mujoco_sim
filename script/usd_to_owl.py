#!/usr/bin/env python3

import sys
from pxr import Usd, UsdOntology, UsdGeom, UsdPhysics, Gf, Vt
from owlready2 import onto_path, get_ontology, declare_datatype
import types
import rospkg


def point3f_parser(string: str):
    return Gf.Vec3f(eval(string))


def point3f_unparser(vec3f: Gf.Vec3f):
    return '[' + ', '.join(str(x) for x in vec3f) + ']'


declare_datatype(Gf.Vec3f, 'https://ease-crc.org/ont/USD.owl#point3f',
                 point3f_parser, point3f_unparser)


def token_array_parser(string: str):
    return Vt.TokenArray(eval(string))


def token_array_unparser(token_array: Vt.TokenArray):
    return '[' + ', '.join([str(x) for x in token_array]) + ']'


declare_datatype(Vt.TokenArray, 'https://ease-crc.org/ont/USD.owl#token_array',
                 token_array_parser, token_array_unparser)


def matrix4d_parser(string: str):
    return Gf.Matrix4d(eval(string))


def matrix4d_unparser(matrix4d: Gf.Matrix4d):
    return '[' + ', '.join(str(row) for row in [list(row) for row in matrix4d]) + ']'


declare_datatype(Gf.Matrix4d, 'https://ease-crc.org/ont/USD.owl#matrix4d',
                 matrix4d_parser, matrix4d_unparser)


def color3f_array_parser(string: str):
    return Vt.Vec3fArray(eval(string))


def color3f_array_unparser(vec3f_array: Vt.Vec3fArray):
    return '[' + ', '.join(str(row) for row in [list(row) for row in vec3f_array]) + ']'


declare_datatype(Vt.Vec3fArray, 'https://ease-crc.org/ont/USD.owl#color3f_array',
                 color3f_array_parser, color3f_array_unparser)


def float_array_parser(string: str):
    return Vt.FloatArray(eval(string))


def float_array_unparser(float_array: Vt.FloatArray):
    return '[' + ', '.join([str(x) for x in float_array]) + ']'


declare_datatype(Vt.FloatArray, 'https://ease-crc.org/ont/USD.owl#float_array',
                 float_array_parser, float_array_unparser)


def quatf_parser(string: str):
    return Gf.Quatf(eval(string))


def quatf_unparser(quatf: Gf.Quatf):
    return '[' + str(quatf.GetReal()) + ', '.join([str(x) for x in quatf.GetImaginary()]) + ']'


declare_datatype(Gf.Quatf, 'https://ease-crc.org/ont/USD.owl#quatf',
                 quatf_parser, quatf_unparser)


def usd_to_owl(file_path: str) -> None:
    rospack = rospkg.RosPack()
    save_path = rospack.get_path('mujoco_sim') + '/model/ontology/'
    onto_path.append(save_path)

    ABox_onto = get_ontology(
        'https://ease-crc.org/ont/usd/BoxScenarioTest1.owl')

    # usd_onto = get_ontology('https://ease-crc.org/ont/USD.owl')
    usd_onto = get_ontology(
        'file:///' + rospack.get_path('mujoco_sim') + '/model/owl/USD.owl')
    usd_onto.load()

    owl_onto = get_ontology('http://www.w3.org/2002/07/owl')
    owl_onto.load()

    dul_onto = get_ontology(
        'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl')
    dul_onto.load()

    ABox_onto.imported_ontologies.append(usd_onto)

    iri_map = dict()

    stage = Usd.Stage.Open(file_path)
    with ABox_onto:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdOntology.RdfAPI):
                rdfAPI = UsdOntology.RdfAPI.Apply(prim)
                iri = rdfAPI.GetRdfNamespaceAttr().Get() + prim.GetName()
                prim_inst = usd_onto.Prim(iri)

                if prim.IsA(UsdGeom.Xformable):
                    xformable = UsdGeom.Xformable(prim)
                    xformOpOrderAttr = xformable.GetXformOpOrderAttr().Get()
                    if xformOpOrderAttr is not None:
                        xformOpOrder_inst = dul_onto.Quality(
                            iri + '_xformOpOrder')
                        prim_inst.hasQuality.append(xformOpOrder_inst)
                        xformOpOrder_inst.xformOpOrder = [xformOpOrderAttr]
                        for xformOp in xformOpOrderAttr:
                            if prim.HasAttribute(xformOp):
                                if xformOp == 'xformOp:transform':
                                    xformOpTransform_inst = dul_onto.Quality(
                                        iri + '_xformOp_transform')
                                    prim_inst.hasQuality.append(
                                        xformOpTransform_inst)
                                    xformOpTransform_inst.xformOp_transform = [
                                        prim.GetAttribute(xformOp).Get()]

                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
                    rigidBodyEnabled_inst = dul_onto.Quality(
                        iri + '_rigidBodyEnabled')
                    prim_inst.hasQuality.append(rigidBodyEnabled_inst)
                    rigidBodyEnabled_inst.physics_rigidBodyEnabled = [
                        rigidBodyAPI.GetRigidBodyEnabledAttr().Get()]

                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
                    rigidCollisionEnabled_inst = dul_onto.Quality(
                        iri + '_rigidBodyEnabled')
                    prim_inst.hasQuality.append(rigidCollisionEnabled_inst)
                    rigidCollisionEnabled_inst.physics_collisionEnabled = [
                        collisionAPI.GetCollisionEnabledAttr().Get()]

                if prim.HasAPI(UsdPhysics.MassAPI):
                    massAPI = UsdPhysics.MassAPI.Apply(prim)
                    mass_inst = dul_onto.Quality(iri + '_mass')
                    prim_inst.hasQuality.append(mass_inst)
                    mass_inst.physics_mass = [massAPI.GetMassAttr().Get()]

                    com_inst = dul_onto.Quality(iri + '_centerOfMass')
                    prim_inst.hasQuality.append(com_inst)
                    com_inst.physics_centerOfMass = [
                        massAPI.GetCenterOfMassAttr().Get()]

                if prim.IsA(UsdGeom.Gprim):
                    color_inst = usd_onto.Color(iri + '_color')
                    prim_inst.hasColor = color_inst
                    gprim = UsdGeom.Gprim(prim)
                    color_inst.primvars_displayColor = [
                        gprim.GetDisplayColorPrimvar().Get()]
                    color_inst.primvars_displayOpacity = [
                        gprim.GetDisplayOpacityPrimvar().Get()]

                if prim.IsA(UsdGeom.Xform):
                    prim_inst.hasTypedSchema = usd_onto.XformSchema
                elif prim.IsA(UsdGeom.Cube):
                    prim_inst.hasTypedSchema = usd_onto.CubeSchema
                    size_inst = dul_onto.Quality(iri + '_size')
                    prim_inst.hasQuality.append(size_inst)
                    cube = UsdGeom.Cube(prim)
                    size_inst.size = [cube.GetSizeAttr().Get()]
                elif prim.IsA(UsdGeom.Sphere):
                    prim_inst.hasTypedSchema = usd_onto.SphereSchema
                    radius_inst = dul_onto.Quality(iri + '_radius')
                    prim_inst.hasQuality.append(radius_inst)
                    sphere = UsdGeom.Sphere(prim)
                    radius_inst.radius = [sphere.GetRadiusAttr().Get()]
                elif prim.IsA(UsdGeom.Cylinder):
                    prim_inst.hasTypedSchema = usd_onto.CylinderSchema
                    radius_inst = dul_onto.Quality(iri + '_radius')
                    prim_inst.hasQuality.append(radius_inst)
                    height_inst = dul_onto.Quality(iri + '_height')
                    prim_inst.hasQuality.append(height_inst)
                    cylinder = UsdGeom.Cylinder(prim)
                    radius_inst.radius = [cylinder.GetRadiusAttr().Get()]
                    height_inst.height = [cylinder.GetHeightAttr().Get()]

        for prim in stage.Traverse():
            if prim.HasAPI(UsdOntology.RdfAPI) and prim.IsA(UsdPhysics.RevoluteJoint):
                rdfAPI = UsdOntology.RdfAPI.Apply(prim)
                iri = rdfAPI.GetRdfNamespaceAttr().Get() + prim.GetName()
                prim_inst = ABox_onto[iri]

                prim_inst.hasTypedSchema = usd_onto.RevoluteJoint
                revoluteJoint = UsdPhysics.RevoluteJoint(prim)
                body0 = stage.GetPrimAtPath(
                    revoluteJoint.GetBody0Rel().GetTargets()[0])
                if body0.HasAPI(UsdOntology.RdfAPI):
                    body0_rdfAPI = UsdOntology.RdfAPI.Apply(body0)
                    body0_iri = body0_rdfAPI.GetRdfNamespaceAttr().Get() + body0.GetName()
                    prim_inst.physics_body0 = ABox_onto[body0_iri]
                body1 = stage.GetPrimAtPath(
                    revoluteJoint.GetBody1Rel().GetTargets()[0])
                if body1.HasAPI(UsdOntology.RdfAPI):
                    body1_rdfAPI = UsdOntology.RdfAPI.Apply(body1)
                    body1_iri = body1_rdfAPI.GetRdfNamespaceAttr().Get() + body1.GetName()
                    prim_inst.physics_body1 = ABox_onto[body1_iri]

                prim_inst.physics_collisionEnabled = [
                    revoluteJoint.GetCollisionEnabledAttr().Get()]
                prim_inst.physics_localPos0 = [
                    revoluteJoint.GetLocalPos0Attr().Get()]
                prim_inst.physics_localPos1 = [
                    revoluteJoint.GetLocalPos1Attr().Get()]
                prim_inst.physics_localRot0 = [
                    revoluteJoint.GetLocalRot0Attr().Get()]
                prim_inst.physics_localRot1 = [
                    revoluteJoint.GetLocalRot1Attr().Get()]

    ABox_onto.save(file=save_path + 'BoxScenarioTest1.owl', format="rdfxml")

    return None


if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        print('Usage: file_path.usda')
        sys.exit(1)
    usd_to_owl(file_path)
