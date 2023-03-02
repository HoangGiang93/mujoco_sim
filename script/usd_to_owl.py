#!/usr/bin/env python3

import sys
from pxr import Usd
from owlready2 import onto_path, get_ontology

types = set()
prim_types = dict()
def usd_to_owl(file_path: str) -> None:
    stage = Usd.Stage.Open(file_path)
    for prim in stage.Traverse():
        for prop in prim.GetPropertyNames():
            if 'semanticType' in prop and prim.GetAttribute(prop).Get() == 'class':
                prop = prop.replace('Type', 'Data')
                type = prim.GetAttribute(prop).Get()
                types.add(type)
                prim_types[prim.GetName()] = type
                break

    onto_path.append("/home/giang/Workspace/mujoco_ws")
    onto = get_ontology("http://www.lesfleursdunormal.fr/static/_downloads/pizza_onto.owl")
    onto.load()
    onto.save()
    return None


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        print('Usage: file_path.usda')
        sys.exit(1)
    usd_to_owl(file_path)
