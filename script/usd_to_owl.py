#!/usr/bin/env python3

import sys
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics


def usd_to_owl(file_path: str) -> None:
    stage = Usd.Stage.Open(file_path)
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if isinstance(attr.Get(), str):
                print(attr, dir(attr))
    return None


if __name__ == "__main__":
    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        print('Usage: file_path.usda')
        sys.exit(1)
    usd_to_owl(file_path)
