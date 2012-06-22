(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (connected place_0__b place_1__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.1624  (assign (category room_0_91) meetingroom)
                        0.1690  (assign (category room_0_91) corridor)
                        0.6685  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.8001  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3023  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3023  (assign (leads_to_room place_1__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_1__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_1__b room0 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room0 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room0), EXECUTED
(move robot_0__c place_1__b place_0__b), EXECUTED
(create_cones_in_room robot_0__c magazine room0 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room0 place_1__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_1__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa 1:B@spatial.sa room0 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room0 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room0
5: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room0 1:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room0 1:B@spatial.sa visualobject3
8: PENDING goal 
links:
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
5 6 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
5 7 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
2 3 DEPENDS category room0 MODALITY: poss meetingroom VALUE: true
2 5 THREATENS started  VALUE: true
2 7 DEPENDS in-room 1:B@spatial.sa MODALITY: poss room0 VALUE: true
2 6 DEPENDS in-room 1:B@spatial.sa MODALITY: poss room0 VALUE: true
6 7 DEPENDS cones_created magazine in room0 VALUE: true
3 6 DEPENDS obj_exists magazine in room0 MODALITY: poss true VALUE: true
3 4 DEPENDS obj_exists magazine in room0 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
0 6 DEPENDS not_fully_explored room0 VALUE: false
0 6 DEPENDS done  VALUE: false
0 2 DEPENDS virtual-place room0 VALUE: 1:B@spatial.sa
0 2 DEPENDS placestatus 1:B@spatial.sa VALUE: placeholder
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS in-room 1:B@spatial.sa MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual room0 VALUE: true
0 2 DEPENDS category room0 MODALITY: committed  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS obj_exists magazine in room0 MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS p-obj_exists magazine in room0 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS started  VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS search_cost magazine in room0 VALUE: 200.0
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 1 DEPENDS leads_to_room 1:B@spatial.sa meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
1 2 DEPENDS leads_to_room 1:B@spatial.sa meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room0 VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           room0 room1 room_0_91 - room
           place_0__b place_1__b place_2__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.1121  (assign (category room_0_91) meetingroom)
                        0.1506  (assign (category room_0_91) corridor)
                        0.7373  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2972  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8096  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2972  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_2__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_2__b room1 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room1), EXECUTED
(move_direct robot_0__c place_2__b place_1__b place_0__b), EXECUTED
(create_cones_in_room robot_0__c magazine room1 place_2__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room1 place_2__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_2__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa 2:B@spatial.sa room1 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room1 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room1
5: SUCCEEDED move_direct 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room1 2:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room1 2:B@spatial.sa visualobject3
8: PENDING goal 
links:
6 7 DEPENDS cones_created magazine in room1 VALUE: true
5 6 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
5 7 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room1 VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room1 VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room1 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS obj_exists magazine in room1 MODALITY: committed  VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS search_cost magazine in room1 VALUE: 200.0
0 7 DEPENDS done  VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS category room1 MODALITY: committed  VALUE: false
0 2 DEPENDS in-room 2:B@spatial.sa MODALITY: committed  VALUE: false
0 2 DEPENDS placestatus 2:B@spatial.sa VALUE: placeholder
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS is-virtual room1 VALUE: true
0 2 DEPENDS virtual-place room1 VALUE: 2:B@spatial.sa
0 5 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS connected 1:B@spatial.sa 0:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 1 DEPENDS leads_to_room 2:B@spatial.sa meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
2 6 DEPENDS in-room 2:B@spatial.sa MODALITY: poss room1 VALUE: true
2 3 DEPENDS category room1 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room 2:B@spatial.sa MODALITY: poss room1 VALUE: true
2 5 THREATENS placestatus 2:B@spatial.sa VALUE: trueplace
3 6 DEPENDS obj_exists magazine in room1 MODALITY: poss true VALUE: true
3 4 DEPENDS obj_exists magazine in room1 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
1 2 DEPENDS leads_to_room 2:B@spatial.sa meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room1 room2 room3 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b place_2__b place_3__b place_4__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_3__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (is-in robot_0__c) place_2__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room3) place_3__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2615  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.0420  (assign (category room_0_91) meetingroom)
                        0.0364  (assign (category room_0_91) corridor)
                        0.9216  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2615  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.6570  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8760  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.1962  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.1962  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_4__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_4__b room2 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room2 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room2), EXECUTED
(move robot_0__c place_4__b place_2__b), EXECUTED
(create_cones_in_room robot_0__c magazine room2 place_4__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room2 place_4__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_4__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa 4:B@spatial.sa room2 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room2 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room2
5: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 2:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room2 4:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room2 4:B@spatial.sa visualobject3
8: PENDING goal 
links:
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room2 VALUE: true
2 5 THREATENS placestatus 4:B@spatial.sa VALUE: trueplace
2 7 DEPENDS in-room 4:B@spatial.sa MODALITY: poss room2 VALUE: true
2 3 DEPENDS category room2 MODALITY: poss meetingroom VALUE: true
2 6 DEPENDS in-room 4:B@spatial.sa MODALITY: poss room2 VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
5 6 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
5 7 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 1 DEPENDS leads_to_room 4:B@spatial.sa meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 2:B@spatial.sa 4:B@spatial.sa VALUE: true
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS search_cost magazine in room2 VALUE: 200.0
0 7 DEPENDS done  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS p-obj_exists magazine in room2 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS obj_exists magazine in room2 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room2 VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 2 DEPENDS placestatus 4:B@spatial.sa VALUE: placeholder
0 2 DEPENDS category room2 MODALITY: committed  VALUE: false
0 2 DEPENDS in-room 4:B@spatial.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS virtual-place room2 VALUE: 4:B@spatial.sa
0 2 DEPENDS is-virtual room2 VALUE: true
3 4 DEPENDS obj_exists magazine in room2 MODALITY: poss true VALUE: true
3 6 DEPENDS obj_exists magazine in room2 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
6 7 DEPENDS cones_created magazine in room2 VALUE: true
1 5 THREATENS started  VALUE: true
1 2 DEPENDS leads_to_room 4:B@spatial.sa meetingroom MODALITY: poss true VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room1 room2 room3 room4 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b place_2__b place_4__b place_5__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_5__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room4) place_5__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2572  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.0161  (assign (category room_0_91) meetingroom)
                        0.0240  (assign (category room_0_91) corridor)
                        0.9600  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2572  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.8841  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_5__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_5__b room4 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room4 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room4), EXECUTED
(move robot_0__c place_5__b place_4__b), FAILED
(create_cones_in_room robot_0__c magazine room4 place_5__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room4 place_5__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_5__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa place_5__b room4 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room4 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room4
5: FAILED move 0:C@spatial.sa place_5__b 4:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room4 place_5__b
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room4 place_5__b visualobject3
8: PENDING goal 
links:
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS in-room place_5__b MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual room4 VALUE: true
0 2 DEPENDS placestatus place_5__b VALUE: placeholder
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS category room4 MODALITY: committed  VALUE: false
0 2 DEPENDS virtual-place room4 VALUE: place_5__b
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room4 VALUE: false
0 7 DEPENDS search_cost magazine in room4 VALUE: 200.0
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 3 DEPENDS obj_exists magazine in room4 MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS p-obj_exists magazine in room4 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 4:B@spatial.sa place_5__b VALUE: true
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 1 DEPENDS leads_to_room place_5__b meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room4 VALUE: true
2 5 THREATENS placestatus place_5__b VALUE: trueplace
2 3 DEPENDS category room4 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room place_5__b MODALITY: poss room4 VALUE: true
2 6 DEPENDS in-room place_5__b MODALITY: poss room4 VALUE: true
6 7 DEPENDS cones_created magazine in room4 VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS obj_exists magazine in room4 MODALITY: poss true VALUE: true
3 6 DEPENDS obj_exists magazine in room4 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: place_5__b
5 7 DEPENDS in-room place_5__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place_5__b
5 6 DEPENDS in-room place_5__b MODALITY: kval 0:C@spatial.sa VALUE: true
1 5 THREATENS started  VALUE: true
1 2 DEPENDS leads_to_room place_5__b meetingroom MODALITY: poss true VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room1 room2 room3 room4 room5 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b place_2__b place_4__b place_6__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room5) place_6__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2569  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.8847  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.0212  (assign (category room_0_91) meetingroom)
                        0.0231  (assign (category room_0_91) corridor)
                        0.9556  (assign (category room_0_91) office)
        )
        (probabilistic  0.2569  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_6__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_6__b room5 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room5 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room5), EXECUTED
(move robot_0__c place_6__b place_4__b), FAILED
(create_cones_in_room robot_0__c magazine room5 place_6__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room5 place_6__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_6__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa place_6__b room5 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room5 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room5
5: FAILED move 0:C@spatial.sa place_6__b 4:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room5 place_6__b
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room5 place_6__b visualobject3
8: PENDING goal 
links:
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS obj_exists magazine in room5 MODALITY: poss true VALUE: true
3 4 DEPENDS obj_exists magazine in room5 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room5 VALUE: true
4 5 THREATENS started  VALUE: true
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS search_cost magazine in room5 VALUE: 200.0
0 7 DEPENDS done  VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 5 DEPENDS connected 4:B@spatial.sa place_6__b VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS obj_exists magazine in room5 MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS p-obj_exists magazine in room5 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS started  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room5 VALUE: false
0 2 DEPENDS in-room place_6__b MODALITY: committed  VALUE: false
0 2 DEPENDS placestatus place_6__b VALUE: placeholder
0 2 DEPENDS virtual-place room5 VALUE: place_6__b
0 2 DEPENDS category room5 MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS is-virtual room5 VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS leads_to_room place_6__b meetingroom MODALITY: committed  VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
5 7 DEPENDS in-room place_6__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: place_6__b
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place_6__b
5 6 DEPENDS in-room place_6__b MODALITY: kval 0:C@spatial.sa VALUE: true
2 7 DEPENDS in-room place_6__b MODALITY: poss room5 VALUE: true
2 6 DEPENDS in-room place_6__b MODALITY: poss room5 VALUE: true
2 3 DEPENDS category room5 MODALITY: poss meetingroom VALUE: true
2 5 THREATENS placestatus place_6__b VALUE: trueplace
1 2 DEPENDS leads_to_room place_6__b meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
6 7 DEPENDS cones_created magazine in room5 VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room1 room2 room3 room4 room5 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b place_2__b place_4__b place_7__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_7__b)
        (connected place_7__b place_4__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.0228  (assign (category room_0_91) meetingroom)
                        0.0245  (assign (category room_0_91) corridor)
                        0.9527  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 office), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(move robot_0__c place_4__b place_7__b), EXECUTED
(create_cones_in_room robot_0__c magazine room_0_91 place_4__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_91 place_4__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma office
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
3: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 7:B@spatial.sa
4: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 4:B@spatial.sa
5: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 4:B@spatial.sa visualobject3
6: PENDING goal 
links:
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS connected 7:B@spatial.sa 4:B@spatial.sa VALUE: true
0 5 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS category 0:91@coma MODALITY: poss office VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:91@coma office MODALITY: defined  VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:91@coma office VALUE: 0.180000005631
0 1 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 6 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS not_fully_explored 0:91@coma VALUE: false
0 4 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 2 DEPENDS label visualobject3 VALUE: magazine
0 2 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject3 VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
5 6 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 4 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 5 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
1 4 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
4 5 DEPENDS cones_created magazine in 0:91@coma VALUE: true
2 3 THREATENS started  VALUE: true
2 5 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
2 5 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           room0 room1 room2 room3 room4 room5 room6 room7 room_0_91 - room
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           place_0__b place_1__b place_2__b place_4__b place_7__b place_8__b place_9__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists place_9__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) placeholder)
        (= (placestatus place_9__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room6) 200.0000)
        (= (search_cost magazine in room7) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room6) place_9__b)
        (= (virtual-place room7) place_8__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_8__b)
        (connected place_7__b place_9__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual room7)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.8838  (assign (leads_to_room place_9__b corridor) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2574  (assign (leads_to_room place_9__b meetingroom) true))
        (probabilistic  0.0228  (assign (category room_0_91) meetingroom)
                        0.0245  (assign (category room_0_91) corridor)
                        0.9527  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2574  (assign (leads_to_room place_8__b meetingroom) true))
        (probabilistic  0.2574  (assign (leads_to_room place_9__b office) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2574  (assign (leads_to_room place_8__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.8838  (assign (leads_to_room place_8__b corridor) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_8__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_8__b room7 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room7 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room7), EXECUTED
(move robot_0__c place_8__b place_7__b), FAILED
(create_cones_in_room robot_0__c magazine room7 place_8__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room7 place_8__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_8__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa place_8__b room7 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room7 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room7
5: FAILED move 0:C@spatial.sa place_8__b 7:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room7 place_8__b
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room7 place_8__b visualobject3
8: PENDING goal 
links:
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room7 VALUE: true
6 7 DEPENDS cones_created magazine in room7 VALUE: true
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room7 VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS obj_exists magazine in room7 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room7 meetingroom MODALITY: defined  VALUE: false
0 2 DEPENDS category room7 MODALITY: committed  VALUE: false
0 2 DEPENDS in-room place_8__b MODALITY: committed  VALUE: false
0 2 DEPENDS placestatus place_8__b VALUE: placeholder
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS is-virtual room7 VALUE: true
0 2 DEPENDS virtual-place room7 VALUE: place_8__b
0 1 DEPENDS leads_to_room place_8__b meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS search_cost magazine in room7 VALUE: 200.0
0 8 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS connected 7:B@spatial.sa place_8__b VALUE: true
0 5 DEPENDS done  VALUE: false
3 4 DEPENDS obj_exists magazine in room7 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
3 6 DEPENDS obj_exists magazine in room7 MODALITY: poss true VALUE: true
1 2 DEPENDS leads_to_room place_8__b meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place_8__b
5 6 DEPENDS in-room place_8__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: place_8__b
5 7 DEPENDS in-room place_8__b MODALITY: kval 0:C@spatial.sa VALUE: true
2 6 DEPENDS in-room place_8__b MODALITY: poss room7 VALUE: true
2 3 DEPENDS category room7 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room place_8__b MODALITY: poss room7 VALUE: true
2 5 THREATENS placestatus place_8__b VALUE: trueplace
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room7 room8 room_0_91 - room
           place_0__b place_1__b place_2__b place_4__b place_7__b place_9__b place__a__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_9__b) true)
        (= (entity-exists place__a__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_9__b) placeholder)
        (= (placestatus place__a__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room8) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room6) 200.0000)
        (= (search_cost magazine in room7) 200.0000)
        (= (search_cost magazine in room8) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room6) place_9__b)
        (= (virtual-place room8) place__a__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_9__b)
        (connected place_7__b place__a__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual room7)
        (is-virtual room8)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.8838  (assign (leads_to_room place_9__b corridor) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2574  (assign (leads_to_room place__a__b meetingroom) true))
        (probabilistic  0.0228  (assign (category room_0_91) meetingroom)
                        0.0245  (assign (category room_0_91) corridor)
                        0.9527  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2574  (assign (leads_to_room place_9__b office) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2574  (assign (leads_to_room place_9__b meetingroom) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2574  (assign (leads_to_room place__a__b office) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.8838  (assign (leads_to_room place__a__b corridor) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place_9__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place_9__b room6 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room6 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room6), EXECUTED
(move robot_0__c place_9__b place_7__b), FAILED
(create_cones_in_room robot_0__c magazine room6 place_9__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room6 place_9__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_9__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa place_9__b room6 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room6 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room6
5: FAILED move 0:C@spatial.sa place_9__b 7:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room6 place_9__b
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room6 place_9__b visualobject3
8: PENDING goal 
links:
5 7 DEPENDS in-room place_9__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: place_9__b
5 6 DEPENDS in-room place_9__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place_9__b
2 5 THREATENS placestatus place_9__b VALUE: trueplace
2 7 DEPENDS in-room place_9__b MODALITY: poss room6 VALUE: true
2 3 DEPENDS category room6 MODALITY: poss meetingroom VALUE: true
2 6 DEPENDS in-room place_9__b MODALITY: poss room6 VALUE: true
6 7 DEPENDS cones_created magazine in room6 VALUE: true
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room6 VALUE: true
3 5 THREATENS started  VALUE: true
3 4 DEPENDS obj_exists magazine in room6 MODALITY: poss true VALUE: true
3 6 DEPENDS obj_exists magazine in room6 MODALITY: poss true VALUE: true
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 7:B@spatial.sa place_9__b VALUE: true
0 2 DEPENDS in-room place_9__b MODALITY: committed  VALUE: false
0 2 DEPENDS category room6 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual room6 VALUE: true
0 2 DEPENDS placestatus place_9__b VALUE: placeholder
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS virtual-place room6 VALUE: place_9__b
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room6 VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room6 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS obj_exists magazine in room6 MODALITY: committed  VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS search_cost magazine in room6 VALUE: 200.0
0 1 DEPENDS leads_to_room place_9__b meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
1 5 THREATENS started  VALUE: true
1 2 DEPENDS leads_to_room place_9__b meetingroom MODALITY: poss true VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room7 room8 room9 room_0_91 - room
           place_0__b place_1__b place_2__b place_4__b place_7__b place__a__b place__b__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place__a__b) true)
        (= (entity-exists place__b__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place__a__b) placeholder)
        (= (placestatus place__b__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room8) 200.0000)
        (= (search_cost book in room9) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room9) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room9) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room6) 200.0000)
        (= (search_cost magazine in room7) 200.0000)
        (= (search_cost magazine in room8) 200.0000)
        (= (search_cost magazine in room9) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room9) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room8) place__a__b)
        (= (virtual-place room9) place__b__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place__a__b)
        (connected place_7__b place__b__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual room7)
        (is-virtual room8)
        (is-virtual room9)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2574  (assign (leads_to_room place__b__b office) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2574  (assign (leads_to_room place__a__b meetingroom) true))
        (probabilistic  0.0228  (assign (category room_0_91) meetingroom)
                        0.0245  (assign (category room_0_91) corridor)
                        0.9527  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2574  (assign (leads_to_room place__b__b meetingroom) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8838  (assign (leads_to_room place__b__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2574  (assign (leads_to_room place__a__b office) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.8838  (assign (leads_to_room place__a__b corridor) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place__a__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place__a__b room8 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room8 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room8), EXECUTED
(move robot_0__c place__a__b place_7__b), FAILED
(create_cones_in_room robot_0__c magazine room8 place__a__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room8 place__a__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place__a__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa place__a__b room8 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room8 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room8
5: FAILED move 0:C@spatial.sa place__a__b 7:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room8 place__a__b
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room8 place__a__b visualobject3
8: PENDING goal 
links:
3 4 DEPENDS obj_exists magazine in room8 MODALITY: poss true VALUE: true
3 6 DEPENDS obj_exists magazine in room8 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
2 3 DEPENDS category room8 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room place__a__b MODALITY: poss room8 VALUE: true
2 6 DEPENDS in-room place__a__b MODALITY: poss room8 VALUE: true
2 5 THREATENS placestatus place__a__b VALUE: trueplace
1 2 DEPENDS leads_to_room place__a__b meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS cones_created magazine in room8 VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: place__a__b
5 7 DEPENDS in-room place__a__b MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place__a__b
5 6 DEPENDS in-room place__a__b MODALITY: kval 0:C@spatial.sa VALUE: true
0 1 DEPENDS leads_to_room place__a__b meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 2 DEPENDS is-virtual room8 VALUE: true
0 2 DEPENDS category room8 MODALITY: committed  VALUE: false
0 2 DEPENDS virtual-place room8 VALUE: place__a__b
0 2 DEPENDS in-room place__a__b MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS placestatus place__a__b VALUE: placeholder
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS obj_exists magazine in room8 MODALITY: committed  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room8 meetingroom MODALITY: defined  VALUE: false
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS search_cost magazine in room8 VALUE: 200.0
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room8 VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 7:B@spatial.sa place__a__b VALUE: true
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room8 VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  person0 person1 - person
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           book cerealbox magazine table - label
           robot_0__c - robot
           corridor meetingroom office - category
           room0 room1 room10 room2 room3 room4 room5 room6 room7 room8 room9 room_0_91 - room
           place_0__b place_1__b place_2__b place_4__b place_7__b place__b__b place__c__b - place
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (dora__cost_inroom book) 200.0000)
        (= (dora__cost_inroom cerealbox) 200.0000)
        (= (dora__cost_inroom magazine) 200.0000)
        (= (dora__cost_on book table) 30.0000)
        (= (dora__cost_on cerealbox table) 30.0000)
        (= (dora__cost_on magazine table) 30.0000)
        (= (dora__inroom book corridor) 0.0025)
        (= (dora__inroom book meetingroom) 0.0154)
        (= (dora__inroom book office) 0.1285)
        (= (dora__inroom cerealbox corridor) 0.0519)
        (= (dora__inroom cerealbox meetingroom) 0.0600)
        (= (dora__inroom cerealbox office) 0.0253)
        (= (dora__inroom magazine corridor) 0.0025)
        (= (dora__inroom magazine meetingroom) 0.8000)
        (= (dora__inroom magazine office) 0.1800)
        (= (dora__inroom table corridor) 0.0594)
        (= (dora__inroom table meetingroom) 0.1042)
        (= (dora__inroom table office) 0.5200)
        (= (dora__not_inroom book corridor) 0.9975)
        (= (dora__not_inroom book meetingroom) 0.9846)
        (= (dora__not_inroom book office) 0.8715)
        (= (dora__not_inroom cerealbox corridor) 0.9481)
        (= (dora__not_inroom cerealbox meetingroom) 0.9400)
        (= (dora__not_inroom cerealbox office) 0.9747)
        (= (dora__not_inroom magazine corridor) 0.9975)
        (= (dora__not_inroom magazine meetingroom) 0.2000)
        (= (dora__not_inroom magazine office) 0.8200)
        (= (dora__not_inroom table corridor) 0.9406)
        (= (dora__not_inroom table meetingroom) 0.8958)
        (= (dora__not_inroom table office) 0.4800)
        (= (dora__on book table corridor) 0.0300)
        (= (dora__on book table meetingroom) 0.3000)
        (= (dora__on book table office) 0.1000)
        (= (dora__on cerealbox table corridor) 0.0300)
        (= (dora__on cerealbox table meetingroom) 0.0300)
        (= (dora__on cerealbox table office) 0.0300)
        (= (dora__on magazine table corridor) 0.0300)
        (= (dora__on magazine table meetingroom) 0.8000)
        (= (dora__on magazine table office) 0.1000)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place__b__b) true)
        (= (entity-exists place__c__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_4__b) room_0_91)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) magazine)
        (= (label visualobject4) table)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place__b__b) placeholder)
        (= (placestatus place__c__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_91) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room10) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room8) 200.0000)
        (= (search_cost book in room9) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) 30.0000)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room10) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room9) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) 30.0000)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room10) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room9) unknown)
        (= (search_cost container in room_0_91) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject3) unknown)
        (= (search_cost container in visualobject4) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject3) unknown)
        (= (search_cost container on visualobject4) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room0) 200.0000)
        (= (search_cost magazine in room1) 200.0000)
        (= (search_cost magazine in room10) 200.0000)
        (= (search_cost magazine in room2) 200.0000)
        (= (search_cost magazine in room3) 200.0000)
        (= (search_cost magazine in room4) 200.0000)
        (= (search_cost magazine in room5) 200.0000)
        (= (search_cost magazine in room6) 200.0000)
        (= (search_cost magazine in room7) 200.0000)
        (= (search_cost magazine in room8) 200.0000)
        (= (search_cost magazine in room9) 200.0000)
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) 30.0000)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room10) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room9) unknown)
        (= (search_cost table in room_0_91) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject3) unknown)
        (= (search_cost table in visualobject4) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject3) unknown)
        (= (search_cost table on visualobject4) unknown)
        (= (virtual-place room0) place_1__b)
        (= (virtual-place room1) place_2__b)
        (= (virtual-place room10) place__c__b)
        (= (virtual-place room2) place_4__b)
        (= (virtual-place room9) place__b__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_4__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place__b__b)
        (connected place_7__b place__c__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room10)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual room7)
        (is-virtual room8)
        (is-virtual room9)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.2574  (assign (leads_to_room place__b__b office) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2574  (assign (leads_to_room place__c__b meetingroom) true))
        (probabilistic  0.0228  (assign (category room_0_91) meetingroom)
                        0.0245  (assign (category room_0_91) corridor)
                        0.9527  (assign (category room_0_91) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2574  (assign (leads_to_room place__b__b meetingroom) true))
        (probabilistic  0.2631  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8731  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.0745  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8838  (assign (leads_to_room place__b__b corridor) true))
        (probabilistic  0.2574  (assign (leads_to_room place__c__b office) true))
        (probabilistic  0.8838  (assign (leads_to_room place__c__b corridor) true))
        (probabilistic  0.8214  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0297  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.2908  (assign (leads_to_room place_2__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (and  (= (label ?o) magazine)
                                                 (kval robot_0__c (related-to ?o))
                                           )
              )
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-leads_to_room-place__c__b-meetingroom-true robot_0__c), EXECUTED
(__commit-room_from_placeholder-0 robot_0__c place__c__b room10 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room10 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room10), EXECUTED
(move robot_0__c place__c__b place_7__b), IN_PROGRESS
(create_cones_in_room robot_0__c magazine room10 place__c__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room10 place__c__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place__c__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa C:B@spatial.sa room10 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room10 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room10
5: INPROGRESS move 0:C@spatial.sa C:B@spatial.sa 7:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room10 C:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room10 C:B@spatial.sa visualobject3
8: PENDING goal 
links:
6 7 DEPENDS cones_created magazine in room10 VALUE: true
2 3 DEPENDS category room10 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room C:B@spatial.sa MODALITY: poss room10 VALUE: true
2 5 THREATENS started  VALUE: true
2 6 DEPENDS in-room C:B@spatial.sa MODALITY: poss room10 VALUE: true
3 4 DEPENDS obj_exists magazine in room10 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
3 6 DEPENDS obj_exists magazine in room10 MODALITY: poss true VALUE: true
5 7 DEPENDS in-room C:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: C:B@spatial.sa
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: C:B@spatial.sa
5 6 DEPENDS in-room C:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room10 VALUE: true
4 5 THREATENS started  VALUE: true
1 2 DEPENDS leads_to_room C:B@spatial.sa meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
0 2 DEPENDS is-virtual room10 VALUE: true
0 2 DEPENDS in-room C:B@spatial.sa MODALITY: committed  VALUE: false
0 2 DEPENDS category room10 MODALITY: committed  VALUE: false
0 2 DEPENDS virtual-place room10 VALUE: C:B@spatial.sa
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS placestatus C:B@spatial.sa VALUE: placeholder
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room10 VALUE: false
0 8 DEPENDS label visualobject3 VALUE: magazine
0 1 DEPENDS leads_to_room C:B@spatial.sa meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS obj_exists magazine in room10 MODALITY: committed  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room10 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS started  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 7:B@spatial.sa C:B@spatial.sa VALUE: true
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS search_cost magazine in room10 VALUE: 200.0
0 7 DEPENDS done  VALUE: false
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
