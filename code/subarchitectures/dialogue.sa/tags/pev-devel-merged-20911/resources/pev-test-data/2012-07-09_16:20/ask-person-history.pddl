END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-contains-a-person-prior-room_0__a1-true 0:C@spatial.sa
2: SUCCEEDED __commit-sample_object_existence-default-0 0:C@spatial.sa 6:w@planner.sa container in 0:A1@coma false
3: SUCCEEDED __commit-person_in_room-0 0:C@spatial.sa 0:S@vision.sa 6:B@spatial.sa 0:A1@coma
4: SUCCEEDED __commit-entity-nonexistence-in-nonexistence-0 0:C@spatial.sa 7:w@planner.sa 6:w@planner.sa
5: SUCCEEDED look-for-people 0:C@spatial.sa 6:B@spatial.sa 0:A1@coma 0:S@vision.sa
6: PENDING engage 0:C@spatial.sa 0:S@vision.sa 6:B@spatial.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa container 6:w@planner.sa in 0:A1@coma 6:B@spatial.sa 0:S@vision.sa
8: SUCCEEDED __knowledge-entity-nonexistence-in-nonexistence-0 0:C@spatial.sa 7:w@planner.sa 6:w@planner.sa
9: PENDING goal 
links:
4 5 THREATENS started  VALUE: true
4 8 DEPENDS entity-exists 7:w@planner.sa MODALITY: poss false VALUE: true
0 4 DEPENDS related-to 7:w@planner.sa MODALITY: poss 6:w@planner.sa VALUE: true
0 4 DEPENDS entity-exists 7:w@planner.sa MODALITY: committed  VALUE: false
0 4 DEPENDS started  VALUE: false
0 8 DEPENDS related-to 7:w@planner.sa MODALITY: poss 6:w@planner.sa VALUE: true
0 8 DEPENDS related-to 7:w@planner.sa VALUE: 6:w@planner.sa
0 1 DEPENDS contains-a-person-prior 0:A1@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS in-room 6:B@spatial.sa VALUE: 0:A1@coma
0 5 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 5 DEPENDS in-room 6:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 7 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to 6:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS entity-exists 0:A1@coma VALUE: true
0 7 DEPENDS in-room 6:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 7 DEPENDS label 6:w@planner.sa VALUE: container
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 6 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 6 DEPENDS done  VALUE: false
0 2 DEPENDS relation 6:w@planner.sa MODALITY: poss in VALUE: true
0 2 DEPENDS related-to 6:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS entity-exists 6:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists container in 0:A1@coma MODALITY: committed  VALUE: false
0 2 DEPENDS label 6:w@planner.sa VALUE: container
0 3 DEPENDS associated-with 0:S@vision.sa VALUE: 0:A1@coma
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS is-virtual 0:S@vision.sa VALUE: true
0 3 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 3 DEPENDS in-room 6:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
7 8 DEPENDS entity-exists 6:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
7 9 DEPENDS entity-exists 6:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 3 DEPENDS contains-a-person-prior 0:A1@coma MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
5 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS engaged 0:S@vision.sa VALUE: true
2 4 DEPENDS entity-exists 6:w@planner.sa MODALITY: poss false VALUE: true
2 5 THREATENS started  VALUE: true
2 8 DEPENDS entity-exists 6:w@planner.sa MODALITY: poss false VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
3 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
8 9 DEPENDS entity-exists 7:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-sample_object_existence-default-0 0:C@spatial.sa 6:w@planner.sa container in 0:A1@coma false
2: SUCCEEDED __commit-entity-nonexistence-in-nonexistence-0 0:C@spatial.sa 7:w@planner.sa 6:w@planner.sa
3: SUCCEEDED engage 0:C@spatial.sa 0:S@vision.sa 6:B@spatial.sa
4: SUCCEEDED ask-for-object-existence 0:C@spatial.sa container 6:w@planner.sa in 0:A1@coma 6:B@spatial.sa 0:S@vision.sa
5: SUCCEEDED __knowledge-entity-nonexistence-in-nonexistence-0 0:C@spatial.sa 7:w@planner.sa 6:w@planner.sa
6: PENDING goal 
links:
1 2 DEPENDS entity-exists 6:w@planner.sa MODALITY: poss false VALUE: true
1 3 THREATENS started  VALUE: true
1 5 DEPENDS entity-exists 6:w@planner.sa MODALITY: poss false VALUE: true
5 6 DEPENDS entity-exists 7:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS engaged 0:S@vision.sa VALUE: true
2 5 DEPENDS entity-exists 7:w@planner.sa MODALITY: poss false VALUE: true
2 3 THREATENS started  VALUE: true
0 5 DEPENDS related-to 7:w@planner.sa MODALITY: poss 6:w@planner.sa VALUE: true
0 5 DEPENDS related-to 7:w@planner.sa VALUE: 6:w@planner.sa
0 2 DEPENDS related-to 7:w@planner.sa MODALITY: poss 6:w@planner.sa VALUE: true
0 2 DEPENDS entity-exists 7:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 1 DEPENDS relation 6:w@planner.sa MODALITY: poss in VALUE: true
0 1 DEPENDS related-to 6:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS entity-exists 6:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS obj_exists container in 0:A1@coma MODALITY: committed  VALUE: false
0 1 DEPENDS label 6:w@planner.sa VALUE: container
0 4 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS related-to 6:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
0 4 DEPENDS entity-exists 0:A1@coma VALUE: true
0 4 DEPENDS is-in 0:S@vision.sa VALUE: 6:B@spatial.sa
0 4 DEPENDS in-room 6:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 4 DEPENDS label 6:w@planner.sa VALUE: container
0 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 3 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:S@vision.sa VALUE: 6:B@spatial.sa
4 5 DEPENDS entity-exists 6:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 6 DEPENDS entity-exists 6:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
END_PLAN
actions:
0: SUCCEEDED init 
1: UNSUCCESSFUL ask-for-object-existence 0:C@spatial.sa magazine 7:w@planner.sa in 6:w@planner.sa 6:B@spatial.sa 0:S@vision.sa
2: PENDING goal 
links:
0 1 DEPENDS is-in 0:S@vision.sa VALUE: 6:B@spatial.sa
0 1 DEPENDS done  VALUE: false
0 1 DEPENDS related-to 6:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: poss 6:B@spatial.sa VALUE: true
0 1 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 1 DEPENDS engaged 0:S@vision.sa VALUE: true
0 1 DEPENDS entity-exists 6:w@planner.sa MODALITY: poss true VALUE: true
0 1 DEPENDS label 7:w@planner.sa VALUE: magazine
0 1 DEPENDS related-to 7:w@planner.sa MODALITY: poss 6:w@planner.sa VALUE: true
0 1 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 1 DEPENDS entity-exists 6:w@planner.sa VALUE: true
0 2 DEPENDS entity-exists 6:w@planner.sa VALUE: true
1 2 DEPENDS entity-exists 7:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
