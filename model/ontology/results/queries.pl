:- use_module(library(semweb/rdf11)).

:- rdf_load('../../owl/DUL.owl').

:- rdf_load('../../owl/USD.owl').

:- rdf_load('../../owl/box_scenario_TBox.owl').

:- rdf_load('box_scenario_reasoned.owl').

:- rdf_register_prefix(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#').

:- rdf_register_prefix(usd, 'https://ease-crc.org/ont/USD.owl#').

:- rdf_register_prefix(scene, 'https://ease-crc.org/ont/usd/BoxScenario.owl#').

get_box_flap_joints(Joints) :-
  findall(Joint, rdf(usd:'box', usd:'hasConnection', Joint), Joints).

get_xform_prims(Prims) :-
  findall(Prim, (
    rdf(Class, owl:'someValuesFrom', usd:'XformSchema'), 
    rdf(Prim, rdf:'type', Class)), Prims).

get_mass_of_box(Mass) :-
  rdf(Quality, usd:'physics_mass', Mass),
  rdf(usd:'box', usd:'hasPhysics', Quality).

get_linked_prim(Prim1, Prim2) :-
  rdf(Prim1, usd:'hasConnection', PrimM),
  rdf(PrimM, usd:'hasConnection', Prim2),
  \=(Prim1, Prim2).