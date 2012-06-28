(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room_0_91 - room
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
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) placeholder)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
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
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
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
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.3046  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.3324  (assign (category room_0_91) office)
                        0.4902  (assign (category room_0_91) meetingroom)
                        0.1774  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.7959  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.7959  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3046  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3046  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.3046  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3046  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.3046  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7959  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(move robot_0__c place_1__b place_0__b), EXECUTED
(move_direct robot_0__c place_2__b place_1__b place_0__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_2__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_91 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_91 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
4: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa 0:B@spatial.sa
7: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa visualobject3
10: PENDING goal 
links:
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 8 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
8 9 DEPENDS cones_created magazine in 0:91@coma VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 8 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 9 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
9 10 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 4 THREATENS started  VALUE: true
3 9 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 9 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
0 6 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.800000012159
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 8 DEPENDS not_fully_explored 0:91@coma VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 10 DEPENDS label visualobject3 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 9 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 9 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 5 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room_0_91 - room
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
        (= (entity-exists place_5__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) placeholder)
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
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
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
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
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
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room4) place_4__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_5__b)
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
        (probabilistic  0.3000  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6263  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2126  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.3982  (assign (category room_0_91) office)
                        0.4978  (assign (category room_0_91) meetingroom)
                        0.1040  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.3000  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7873  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8350  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2835  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2835  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2126  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2835  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.2835  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.8350  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(move robot_0__c place_2__b place_1__b), EXECUTED
(move_direct robot_0__c place_4__b place_2__b place_1__b), EXECUTABLE
(move_direct robot_0__c place_5__b place_4__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_1__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_91 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_91 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
4: SUCCEEDED move 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa place_4__b 2:B@spatial.sa 1:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 5:B@spatial.sa place_4__b 1:B@spatial.sa
7: PENDING move 0:C@spatial.sa 1:B@spatial.sa 5:B@spatial.sa
8: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
9: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
10: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa
11: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa visualobject3
12: PENDING goal 
links:
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 10 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
3 4 THREATENS started  VALUE: true
3 11 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 11 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
11 12 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: place_4__b
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
10 11 DEPENDS cones_created magazine in 0:91@coma VALUE: true
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.8
0 7 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa place_4__b VALUE: true
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 10 DEPENDS done  VALUE: false
0 10 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 10 DEPENDS not_fully_explored 0:91@coma VALUE: false
0 12 DEPENDS label visualobject3 VALUE: magazine
0 8 DEPENDS connected 1:B@spatial.sa 0:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 5 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 1:B@spatial.sa place_4__b VALUE: true
0 11 DEPENDS done  VALUE: false
0 11 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 11 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 11 DEPENDS label visualobject3 VALUE: magazine
9 10 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 10 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
9 11 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 11 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_5__b place_6__b place_7__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room_0_91 - room
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
        (= (entity-exists place_5__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (is-in robot_0__c) place_2__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
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
        (= (placestatus place_5__b) placeholder)
        (= (placestatus place_6__b) placeholder)
        (= (placestatus place_7__b) placeholder)
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
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
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
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_7__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.5870  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.2295  (assign (category room_0_91) office)
                        0.6752  (assign (category room_0_91) meetingroom)
                        0.0953  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.1962  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7873  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8505  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2808  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2808  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.2732  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.2732  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.1962  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.6300  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2106  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.2106  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.8171  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8400  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(move robot_0__c place_5__b place_2__b), EXECUTED
(move_direct robot_0__c place_6__b place_5__b place_2__b), EXECUTABLE
(move robot_0__c place_2__b place_6__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_2__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(move_direct robot_0__c place_7__b place_0__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_7__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_91 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_91 place_1__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
4: SUCCEEDED move 0:C@spatial.sa 5:B@spatial.sa 2:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 6:B@spatial.sa 5:B@spatial.sa 2:B@spatial.sa
6: PENDING move 0:C@spatial.sa 2:B@spatial.sa 6:B@spatial.sa
7: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa 0:B@spatial.sa
8: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
9: PENDING move_direct 0:C@spatial.sa 7:B@spatial.sa 0:B@spatial.sa 1:B@spatial.sa
10: PENDING move 0:C@spatial.sa 1:B@spatial.sa 7:B@spatial.sa
11: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 1:B@spatial.sa
12: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 1:B@spatial.sa visualobject3
13: PENDING goal 
links:
10 11 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
10 11 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
10 12 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
10 12 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
11 12 DEPENDS cones_created magazine in 0:91@coma VALUE: true
0 10 DEPENDS done  VALUE: false
0 10 DEPENDS connected 1:B@spatial.sa 7:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 11 DEPENDS not_fully_explored 0:91@coma VALUE: false
0 11 DEPENDS done  VALUE: false
0 11 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 2:B@spatial.sa 5:B@spatial.sa VALUE: true
0 5 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.799999982344
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 2:B@spatial.sa 5:B@spatial.sa VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS connected 1:B@spatial.sa 7:B@spatial.sa VALUE: true
0 9 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 12 DEPENDS done  VALUE: false
0 12 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 12 DEPENDS label visualobject3 VALUE: magazine
0 12 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 7 DEPENDS connected 2:B@spatial.sa 0:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 13 DEPENDS label visualobject3 VALUE: magazine
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 11 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
9 10 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
3 12 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 12 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
3 4 THREATENS started  VALUE: true
12 13 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_5__b place_6__b place_7__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room_0_91 - room
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
        (= (entity-exists place_5__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_5__b) room_0_91)
        (= (is-in robot_0__c) place_5__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
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
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) placeholder)
        (= (placestatus place_7__b) placeholder)
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
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
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
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_5__b)
        (connected place_1__b place_7__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_5__b)
        (connected place_5__b place_1__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_6__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.6079  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.4070  (assign (category room_0_91) office)
                        0.5663  (assign (category room_0_91) meetingroom)
                        0.0267  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.1850  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7873  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8505  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2581  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2581  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.3000  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.1850  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.6617  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.1936  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.1936  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8823  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(move robot_0__c place_6__b place_5__b), EXECUTED
(move_direct robot_0__c place_1__b place_6__b place_5__b), EXECUTED
(move robot_0__c place_7__b place_1__b), EXECUTED
(move robot_0__c place_1__b place_7__b), EXECUTED
(move_direct robot_0__c place_3__b place_1__b place_0__b), EXECUTED
(move robot_0__c place_0__b place_3__b), EXECUTED
(create_cones_in_room robot_0__c magazine room_0_91 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_91 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
4: SUCCEEDED move 0:C@spatial.sa 6:B@spatial.sa 5:B@spatial.sa
5: SUCCEEDED move_direct 0:C@spatial.sa 1:B@spatial.sa 6:B@spatial.sa 5:B@spatial.sa
6: SUCCEEDED move 0:C@spatial.sa 7:B@spatial.sa 1:B@spatial.sa
7: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 7:B@spatial.sa
8: SUCCEEDED move_direct 0:C@spatial.sa 3:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
9: SUCCEEDED move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
10: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa
11: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa visualobject3
12: PENDING goal 
links:
9 10 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 10 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
9 11 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 11 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
2 4 THREATENS started  VALUE: true
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 10 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
11 12 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 1:B@spatial.sa 7:B@spatial.sa VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.799999989474
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 5:B@spatial.sa 6:B@spatial.sa VALUE: true
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 8 DEPENDS connected 1:B@spatial.sa 0:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 11 DEPENDS done  VALUE: false
0 11 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 11 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 11 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS connected 5:B@spatial.sa 1:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 5:B@spatial.sa 6:B@spatial.sa VALUE: true
0 6 DEPENDS connected 1:B@spatial.sa 7:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 12 DEPENDS label visualobject3 VALUE: magazine
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 10 DEPENDS done  VALUE: false
0 10 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 10 DEPENDS not_fully_explored 0:91@coma VALUE: false
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 4 THREATENS started  VALUE: true
3 11 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 11 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
10 11 DEPENDS cones_created magazine in 0:91@coma VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_5__b place_6__b place_7__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room_0_91 - room
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
        (= (entity-exists place_5__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_3__b) room_0_91)
        (= (in-room place_5__b) room_0_91)
        (= (in-room place_6__b) room_0_91)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
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
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
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
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_5__b)
        (connected place_1__b place_7__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_5__b place_1__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_7__b place_1__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.6079  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.1822  (assign (category room_0_91) office)
                        0.7943  (assign (category room_0_91) meetingroom)
                        0.0235  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.1850  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7873  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8505  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2570  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2570  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.1036  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1036  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.1850  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.4876  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.1412  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.1412  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.3433  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8844  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b office) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(create_cones_in_room robot_0__c magazine room_0_91 place_0__b), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0_91 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:91@coma
4: SUCCEEDED create_cones_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa
5: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa visualobject3
6: PENDING goal 
links:
5 6 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
4 5 DEPENDS cones_created magazine in 0:91@coma VALUE: true
3 5 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 5 DEPENDS related-to visualobject3 MODALITY: poss 0:91@coma VALUE: true
3 4 THREATENS started  VALUE: true
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 4 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 5 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 5 DEPENDS in-room 0:B@spatial.sa VALUE: 0:91@coma
0 5 DEPENDS label visualobject3 VALUE: magazine
0 6 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 4 DEPENDS in-room 0:B@spatial.sa VALUE: 0:91@coma
0 4 DEPENDS not_fully_explored 0:91@coma VALUE: false
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.8
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_5__b place_6__b place_7__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l - conegroup
           room0 room1 room2 room3 room4 room5 room6 room_0_91 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_91)
        (= (cg-label conegroup_0__l) magazine)
        (= (cg-label conegroup_1__l) magazine)
        (= (cg-place conegroup_0__l) place_2__b)
        (= (cg-place conegroup_1__l) place_5__b)
        (= (cg-related-to conegroup_0__l) room_0_91)
        (= (cg-related-to conegroup_1__l) room_0_91)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
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
        (= (entity-exists conegroup_0__l) true)
        (= (entity-exists conegroup_1__l) true)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_3__b) true)
        (= (entity-exists place_5__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_91) true)
        (= (in-room place_0__b) room_0_91)
        (= (in-room place_1__b) room_0_91)
        (= (in-room place_2__b) room_0_91)
        (= (in-room place_3__b) room_0_91)
        (= (in-room place_5__b) room_0_91)
        (= (in-room place_6__b) room_0_91)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_91 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_91 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_91 office) 0.0253)
        (= (p-obj_exists magazine in room_0_91 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_91 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_91 office) 0.1800)
        (= (p-visible conegroup_0__l) 0.3699)
        (= (p-visible conegroup_1__l) 0.2984)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
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
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0_91) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject3) unknown)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0_91) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject3) unknown)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
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
        (= (search_cost magazine in room_0_91) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject3) unknown)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
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
        (= (virtual-place room0) place_3__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_2__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_5__b)
        (connected place_1__b place_7__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_5__b place_1__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_7__b place_1__b)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual room0)
        (is-virtual room1)
        (is-virtual room2)
        (is-virtual room3)
        (is-virtual room4)
        (is-virtual room5)
        (is-virtual room6)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (probabilistic  0.6079  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_91) true))
        (probabilistic  0.1822  (assign (category room_0_91) office)
                        0.7943  (assign (category room_0_91) meetingroom)
                        0.0235  (assign (category room_0_91) corridor)
        )
        (probabilistic  0.1850  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7873  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8505  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2570  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2570  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.2752  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.1036  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1036  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.1850  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.1412  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.3433  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8844  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.1412  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3093  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.4876  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-category-room_0_91-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_91 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_91), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0_91 place_0__b visualobject3), UNSUCCESSFUL
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_91-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:91@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa 0:Q@vision.sa magazine in 0:91@coma
4: UNSUCCESSFUL search_for_object_in_room 0:C@spatial.sa magazine 0:91@coma 0:B@spatial.sa 0:Q@vision.sa
5: PENDING goal 
links:
1 4 THREATENS started  VALUE: true
1 2 DEPENDS category 0:91@coma MODALITY: poss meetingroom VALUE: true
3 4 DEPENDS relation 0:Q@vision.sa MODALITY: poss in VALUE: true
3 4 DEPENDS related-to 0:Q@vision.sa MODALITY: poss 0:91@coma VALUE: true
2 3 DEPENDS obj_exists magazine in 0:91@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
4 5 DEPENDS related-to 0:Q@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
0 5 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 1 DEPENDS category 0:91@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS search_cost magazine in 0:91@coma VALUE: 200.0
0 4 DEPENDS cg-label 0:L@binder VALUE: magazine
0 4 DEPENDS cg-related-to 0:L@binder VALUE: 0:91@coma
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:91@coma VALUE: true
0 4 DEPENDS in-room 0:B@spatial.sa VALUE: 0:91@coma
0 4 DEPENDS cg-relation 0:L@binder VALUE: in
0 4 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:91@coma MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:91@coma meetingroom VALUE: 0.8
0 3 DEPENDS entity-exists 0:Q@vision.sa MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual 0:Q@vision.sa VALUE: true
0 3 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 3 DEPENDS related-to 0:Q@vision.sa MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation 0:Q@vision.sa MODALITY: committed  VALUE: false
END_POPLAN
