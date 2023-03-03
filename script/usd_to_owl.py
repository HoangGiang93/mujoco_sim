#!/usr/bin/env python3

import sys
from pxr import Usd
from owlready2 import onto_path, get_ontology
import types

type_set = set()
prim_types = dict()


def usd_to_owl(file_path: str) -> None:
    stage = Usd.Stage.Open(file_path)
    for prim in stage.Traverse():
        for prop in prim.GetPropertyNames():
            if 'semanticType' in prop and prim.GetAttribute(prop).Get() == 'class':
                prop = prop.replace('Type', 'Data')
                prim_type = prim.GetAttribute(prop).Get()
                type_set.add(prim_type)
                prim_types[prim.GetName()] = prim_type
                break

    onto_path.append("/home/giangnguyen/Workspace/mujoco_ws")
    TBox_onto = get_ontology('file:///home/giangnguyen/Workspace/mujoco_ws/TBox.owl')
    ABox_onto = get_ontology('file:///home/giangnguyen/Workspace/mujoco_ws/ABox.owl')

    soma_onto = get_ontology(
        'http://www.ease-crc.org/ont/SOMA.owl')
    soma_onto.load()
    soma_home_onto = get_ontology(
        'http://www.ease-crc.org/ont/SOMA-HOME.owl')
    soma_home_onto.load()

    TBox_onto.imported_ontologies.append(soma_onto)
    TBox_onto.imported_ontologies.append(soma_home_onto)

    ABox_onto.imported_ontologies.append(TBox_onto)

    with TBox_onto:
        class CabinetDrawer(soma_onto.Drawer):
            pass

        class FridgeDrawer(soma_onto.Drawer):
            pass

        class CoffeeTableDrawer(soma_onto.Drawer):
            pass

        class SinkDrawer(soma_onto.Drawer):
            pass

        class CabinetDoor(soma_onto.Door):
            pass

        class WardrobeDoor(soma_onto.Door):
            pass

        class OvenDoor(soma_onto.Door):
            pass

        class FridgeDoor(soma_onto.Door):
            pass

        class IslandCover(soma_onto.Cover):
            pass

        class Window(soma_onto.DesignedComponent):
            pass

        class WindowFrame(soma_onto.Rack):
            pass
    
    with ABox_onto:
        for prim_name, prim_type in prim_types.items():
            if prim_type == 'Wardrobe':
                prim = soma_home_onto.Wardrobe(prim_name)
            elif prim_type == 'Sink':
                prim = soma_home_onto.Sink(prim_name)
            elif prim_type == 'Sofa':
                prim = soma_home_onto.Sofa(prim_name)
            elif prim_type == 'Dishwasher':
                prim = soma_onto.Dishwasher(prim_name)
            elif prim_type == 'CabinetDoor':
                prim = CabinetDoor(prim_name)
            elif prim_type == 'Handle':
                prim = soma_onto.DesignedHandle(prim_name)
            elif prim_type == 'Countertop':
                prim = soma_onto.Countertop(prim_name)
            elif prim_type == 'Armchair':
                prim = soma_home_onto.Armchair(prim_name)
            elif prim_type == 'WardrobeDoor':
                prim = WardrobeDoor(prim_name)
            elif prim_type == 'FridgeDrawer':
                prim = FridgeDrawer(prim_name)
            elif prim_type == 'CabinetDrawer':
                prim = CabinetDrawer(prim_name)
            elif prim_type == 'CoffeeTable':
                prim = soma_home_onto.CoffeeTable(prim_name)
            elif prim_type == 'OvenDoor':
                prim = OvenDoor(prim_name)
            elif prim_type == 'FridgeDoor':
                prim = FridgeDoor(prim_name)
            elif prim_type == 'IslandCover':
                prim = IslandCover(prim_name)
            elif prim_type == 'Watertab':
                prim = soma_onto.DishwasherTab(prim_name)
            elif prim_type == 'Floor':
                prim = soma_onto.Floor(prim_name)
            elif prim_type == 'Wall':
                prim = soma_onto.Wall(prim_name)
            elif prim_type == 'Cabinet':
                prim = soma_home_onto.KitchenCabinet(prim_name)
            elif prim_type == 'BedsideTable':
                prim = soma_home_onto.BedsideTable(prim_name)
            elif prim_type == 'SinkDrawer':
                prim = SinkDrawer(prim_name)
            elif prim_type == 'Window':
                prim = Window(prim_name)
            elif prim_type == 'WindowFrame':
                prim = WindowFrame(prim_name)
            elif prim_type == 'CoffeeTableDrawer':
                prim = CoffeeTableDrawer(prim_name)
            

    TBox_onto.save(file='TBox.owl')
    ABox_onto.save(file='ABox.owl')
    return None


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        print('Usage: file_path.usda')
        sys.exit(1)
    usd_to_owl(file_path)
