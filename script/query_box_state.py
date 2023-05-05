#!/usr/bin/env python3

import sys
from owlready2 import onto_path, get_ontology, World, sync_reasoner_pellet
import rospkg


def query_box_state(file_path: str) -> None:
    rospack = rospkg.RosPack()

    save_path = rospack.get_path('mujoco_sim') + '/model/ontology/'
    onto_path.append(save_path)

    onto = get_ontology(
        'https://ease-crc.org/ont/usd/BoxScenario_ABox.owl').load()

    sync_reasoner_pellet(onto)

    onto.save(file=save_path + 'BoxScenario_DBox.xml')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_path = sys.argv[1]
    else:
        print('Usage: file_path.usda')
        sys.exit(1)
    query_box_state(file_path)
