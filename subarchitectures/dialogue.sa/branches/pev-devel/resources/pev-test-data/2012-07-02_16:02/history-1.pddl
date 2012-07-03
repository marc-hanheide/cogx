(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room_0_81 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) placeholder)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room_0_81) unknown)
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
        (probabilistic  0.3047  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3371  (assign (category room_0_81) office)
                        0.4850  (assign (category room_0_81) meetingroom)
                        0.1779  (assign (category room_0_81) corridor)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7956  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.7956  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3000  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.3000  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.3047  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3047  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_81-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(move robot_0__c place_1__b place_0__b), EXECUTED
(move_direct robot_0__c place_2__b place_1__b place_0__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_2__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_81 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_81 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_81-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
4: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa 0:B@spatial.sa
7: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 0:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 0:B@spatial.sa visualobject3
10: PENDING goal 
links:
3 4 THREATENS started  VALUE: true
3 9 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 9 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
1 2 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
8 9 DEPENDS cones_created magazine in 0:81@coma VALUE: true
9 10 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
2 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 8 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 8 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 9 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 10 DEPENDS label visualobject3 VALUE: magazine
0 1 DEPENDS category 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 8 DEPENDS not_fully_explored 0:81@coma VALUE: false
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 9 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 9 DEPENDS label visualobject3 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 6 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.8
0 5 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room_0_81 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
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
        (probabilistic  0.7588  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.4938  (assign (category room_0_81) office)
                        0.2497  (assign (category room_0_81) meetingroom)
                        0.2565  (assign (category room_0_81) corridor)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.7588  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3247  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3247  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.3247  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3247  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.3247  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7588  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-room_from_placeholder-0 robot_0__c place_4__b room3 meetingroom), EXECUTED
(__commit-obj_in_room-0 robot_0__c magazine room3 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room3), EXECUTED
(move robot_0__c place_4__b place_1__b), EXECUTED
(create_cones_in_room robot_0__c magazine room3 place_4__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room3 place_4__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-leads_to_room-place_4__b-meetingroom-true 0:C@spatial.sa
2: SUCCEEDED __commit-room_from_placeholder-0 0:C@spatial.sa 4:B@spatial.sa room3 meetingroom
3: SUCCEEDED __commit-obj_in_room-0 0:C@spatial.sa magazine room3 meetingroom
4: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in room3
5: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
6: PENDING create_cones_in_room 0:C@spatial.sa magazine room3 4:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine room3 4:B@spatial.sa visualobject3
8: PENDING goal 
links:
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 5 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 8 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS search_cost magazine in room3 VALUE: 200.0
0 7 DEPENDS label visualobject3 VALUE: magazine
0 7 DEPENDS done  VALUE: false
0 3 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 3 DEPENDS dora__inroom magazine meetingroom MODALITY: defined  VALUE: true
0 3 DEPENDS dora__inroom magazine meetingroom VALUE: 0.8
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS p-obj_exists magazine in room3 meetingroom MODALITY: defined  VALUE: false
0 3 DEPENDS obj_exists magazine in room3 MODALITY: committed  VALUE: false
0 4 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS is-virtual visualobject3 VALUE: true
0 4 DEPENDS label visualobject3 VALUE: magazine
0 4 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 4 DEPENDS started  VALUE: false
0 4 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS category room3 MODALITY: committed  VALUE: false
0 2 DEPENDS in-room 4:B@spatial.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS placestatus 4:B@spatial.sa VALUE: placeholder
0 2 DEPENDS virtual-place room3 VALUE: 4:B@spatial.sa
0 2 DEPENDS is-virtual room3 VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored room3 VALUE: false
0 1 DEPENDS leads_to_room 4:B@spatial.sa meetingroom MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
5 6 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
5 7 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
1 2 DEPENDS leads_to_room 4:B@spatial.sa meetingroom MODALITY: poss true VALUE: true
1 5 THREATENS started  VALUE: true
7 8 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS obj_exists magazine in room3 MODALITY: poss true VALUE: true
3 4 DEPENDS obj_exists magazine in room3 MODALITY: poss true VALUE: true
3 5 THREATENS started  VALUE: true
4 5 THREATENS started  VALUE: true
4 7 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
4 7 DEPENDS related-to visualobject3 MODALITY: poss room3 VALUE: true
6 7 DEPENDS cones_created magazine in room3 VALUE: true
2 3 DEPENDS category room3 MODALITY: poss meetingroom VALUE: true
2 7 DEPENDS in-room 4:B@spatial.sa MODALITY: poss room3 VALUE: true
2 5 THREATENS started  VALUE: true
2 6 DEPENDS in-room 4:B@spatial.sa MODALITY: poss room3 VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room_0_81 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room4) place_5__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_3__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_4__b place_1__b)
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
        (probabilistic  0.3000  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.4259  (assign (category room_0_81) office)
                        0.4416  (assign (category room_0_81) meetingroom)
                        0.1325  (assign (category room_0_81) corridor)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.3000  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8192  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2920  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2920  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2920  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.2920  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.3000  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.8192  (assign (leads_to_room place_3__b corridor) true))
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
(__commit-category-room_0_81-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(move_direct robot_0__c place_2__b place_4__b place_1__b), EXECUTED
(move robot_0__c place_1__b place_2__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_1__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_81 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_81 place_0__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_81-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
4: SUCCEEDED move_direct 0:C@spatial.sa 2:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
5: PENDING move 0:C@spatial.sa 1:B@spatial.sa 2:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
7: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 0:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 0:B@spatial.sa visualobject3
10: PENDING goal 
links:
9 10 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
3 9 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 9 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
3 4 THREATENS started  VALUE: true
8 9 DEPENDS cones_created magazine in 0:81@coma VALUE: true
2 8 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 9 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 9 DEPENDS label visualobject3 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 4 DEPENDS connected 4:B@spatial.sa 1:B@spatial.sa VALUE: true
0 4 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.80000005399
0 10 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa 0:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 8 DEPENDS not_fully_explored 0:81@coma VALUE: false
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 1 DEPENDS category 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 9 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 8 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
1 2 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           room0 room1 room2 room3 room4 room5 room6 room_0_81 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (in-room place_2__b) room_0_81)
        (= (is-in robot_0__c) place_2__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) placeholder)
        (= (placestatus place_7__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_4__b place_1__b)
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
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.4102  (assign (category room_0_81) office)
                        0.5574  (assign (category room_0_81) meetingroom)
                        0.0324  (assign (category room_0_81) corridor)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.8247  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.1892  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.1892  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.2549  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  )
        (probabilistic  0.6391  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.2549  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.8608  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-category-room_0_81-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(move robot_0__c place_3__b place_2__b), EXECUTED
(move_direct robot_0__c place_6__b place_3__b place_2__b), EXECUTABLE
(move_direct robot_0__c place_7__b place_6__b place_2__b), EXECUTABLE
(move robot_0__c place_2__b place_7__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_81 place_2__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_81 place_2__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_81-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
4: SUCCEEDED move 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 6:B@spatial.sa 3:B@spatial.sa 2:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 7:B@spatial.sa 6:B@spatial.sa 2:B@spatial.sa
7: PENDING move 0:C@spatial.sa 2:B@spatial.sa 7:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 2:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 2:B@spatial.sa visualobject3
10: PENDING goal 
links:
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
2 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 8 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
9 10 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 4 DEPENDS connected 2:B@spatial.sa 3:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
0 10 DEPENDS label visualobject3 VALUE: magazine
0 5 DEPENDS connected 2:B@spatial.sa 3:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.800000042773
0 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 9 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 9 DEPENDS in-room 2:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 9 DEPENDS label visualobject3 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 2:B@spatial.sa 7:B@spatial.sa VALUE: true
0 6 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 8 DEPENDS in-room 2:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 8 DEPENDS not_fully_explored 0:81@coma VALUE: false
0 8 DEPENDS done  VALUE: false
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 1 DEPENDS category 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 2:B@spatial.sa 7:B@spatial.sa VALUE: true
8 9 DEPENDS cones_created magazine in 0:81@coma VALUE: true
3 4 THREATENS started  VALUE: true
3 9 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 9 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
1 2 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
7 8 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
7 9 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  room0 room1 room2 room3 room4 room5 room6 room7 room_0_81 - room
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b place_8__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (in-room place_2__b) room_0_81)
        (= (in-room place_3__b) room_0_81)
        (= (is-in robot_0__c) place_3__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) placeholder)
        (= (placestatus place_7__b) placeholder)
        (= (placestatus place_8__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (= (virtual-place room7) place_8__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_3__b place_8__b)
        (connected place_4__b place_1__b)
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
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1109  (assign (category room_0_81) office)
                        0.7887  (assign (category room_0_81) meetingroom)
                        0.1005  (assign (category room_0_81) corridor)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  )
        (probabilistic  0.8247  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b office) true))
        (probabilistic  )
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.2816  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  )
        (probabilistic  0.6188  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.2816  (assign (leads_to_room place_7__b office) true))
        (probabilistic  )
        (probabilistic  0.8345  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-category-room_0_81-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(move robot_0__c place_7__b place_3__b), EXECUTED
(move_direct robot_0__c place_8__b place_7__b place_3__b), EXECUTABLE
(move robot_0__c place_3__b place_8__b), EXECUTABLE
(move_direct robot_0__c place_6__b place_3__b place_2__b), EXECUTABLE
(move robot_0__c place_2__b place_6__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0_81 place_2__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_81 place_2__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_81-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
4: SUCCEEDED move 0:C@spatial.sa 7:B@spatial.sa 3:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 8:B@spatial.sa 7:B@spatial.sa 3:B@spatial.sa
6: PENDING move 0:C@spatial.sa 3:B@spatial.sa 8:B@spatial.sa
7: PENDING move_direct 0:C@spatial.sa place_6__b 3:B@spatial.sa 2:B@spatial.sa
8: PENDING move 0:C@spatial.sa 2:B@spatial.sa place_6__b
9: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 2:B@spatial.sa
10: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 2:B@spatial.sa visualobject3
11: PENDING goal 
links:
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 8:B@spatial.sa
2 9 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: place_6__b
3 10 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 10 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
3 4 THREATENS started  VALUE: true
9 10 DEPENDS cones_created magazine in 0:81@coma VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.800000045346
0 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 3:B@spatial.sa 2:B@spatial.sa VALUE: true
0 7 DEPENDS connected 2:B@spatial.sa place_6__b VALUE: true
0 5 DEPENDS connected 3:B@spatial.sa 7:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 3:B@spatial.sa 8:B@spatial.sa VALUE: true
0 9 DEPENDS in-room 2:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 9 DEPENDS not_fully_explored 0:81@coma VALUE: false
0 9 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 3:B@spatial.sa 7:B@spatial.sa VALUE: true
0 1 DEPENDS category 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 2:B@spatial.sa place_6__b VALUE: true
0 11 DEPENDS label visualobject3 VALUE: magazine
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 3:B@spatial.sa 8:B@spatial.sa VALUE: true
0 10 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 10 DEPENDS in-room 2:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 10 DEPENDS label visualobject3 VALUE: magazine
0 10 DEPENDS done  VALUE: false
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
8 9 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
8 10 DEPENDS in-room 2:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
8 10 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
1 2 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
10 11 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  room0 room1 room2 room3 room4 room5 room6 room7 room8 room_0_81 - room
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_7__b place_8__b place_9__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists place_9__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (in-room place_2__b) room_0_81)
        (= (in-room place_3__b) room_0_81)
        (= (in-room place_7__b) room_0_81)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) placeholder)
        (= (placestatus place_9__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
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
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room6) place_7__b)
        (= (virtual-place room7) place_8__b)
        (= (virtual-place room8) place_9__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_3__b place_8__b)
        (connected place_4__b place_1__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
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
        (is-virtual room8)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject3)
        (is-virtual visualobject4)
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.3840  (assign (category room_0_81) office)
                        0.5799  (assign (category room_0_81) meetingroom)
                        0.0361  (assign (category room_0_81) corridor)
        )
        (probabilistic  )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  )
        (probabilistic  0.8247  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b office) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b office) true))
        (probabilistic  )
        (probabilistic  0.2890  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.6188  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  )
        (probabilistic  0.8392  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-category-room_0_81-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(move robot_0__c place_9__b place_7__b), EXECUTED
(move robot_0__c place_7__b place_9__b), EXECUTED
(move_direct robot_0__c place_8__b place_7__b place_3__b), EXECUTED
(move robot_0__c place_3__b place_8__b), EXECUTED
(create_cones_in_room robot_0__c magazine room_0_81 place_3__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0_81 place_3__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0_81-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
4: SUCCEEDED move 0:C@spatial.sa 9:B@spatial.sa 7:B@spatial.sa
5: SUCCEEDED move 0:C@spatial.sa 7:B@spatial.sa 9:B@spatial.sa
6: SUCCEEDED move_direct 0:C@spatial.sa 8:B@spatial.sa 7:B@spatial.sa 3:B@spatial.sa
7: SUCCEEDED move 0:C@spatial.sa 3:B@spatial.sa 8:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 3:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 3:B@spatial.sa visualobject3
10: PENDING goal 
links:
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 9:B@spatial.sa
9 10 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
1 2 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
2 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 8 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
0 5 DEPENDS connected 7:B@spatial.sa 9:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 7:B@spatial.sa 9:B@spatial.sa VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 9 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 9 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 9 DEPENDS label visualobject3 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 10 DEPENDS label visualobject3 VALUE: magazine
0 1 DEPENDS category 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.800000041116
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 3:B@spatial.sa 8:B@spatial.sa VALUE: true
0 3 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject3 VALUE: true
0 6 DEPENDS connected 7:B@spatial.sa 3:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 3:B@spatial.sa 8:B@spatial.sa VALUE: true
0 8 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS not_fully_explored 0:81@coma VALUE: false
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 8:B@spatial.sa
3 9 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
3 9 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
3 4 THREATENS started  VALUE: true
8 9 DEPENDS cones_created magazine in 0:81@coma VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
7 9 DEPENDS in-room 3:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
7 8 DEPENDS in-room 3:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  room0 room1 room2 room3 room4 room5 room6 room7 room8 room_0_81 - room
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_7__b place_8__b place_9__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
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
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists place_9__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (in-room place_2__b) room_0_81)
        (= (in-room place_3__b) room_0_81)
        (= (in-room place_7__b) room_0_81)
        (= (in-room place_8__b) room_0_81)
        (= (is-in robot_0__c) place_3__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) trueplace)
        (= (placestatus place_9__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
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
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room6) place_7__b)
        (= (virtual-place room7) place_8__b)
        (= (virtual-place room8) place_9__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_8__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_3__b place_8__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_9__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_9__b)
        (connected place_8__b place_0__b)
        (connected place_8__b place_3__b)
        (connected place_9__b place_4__b)
        (connected place_9__b place_7__b)
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
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.0368  (assign (category room_0_81) office)
                        0.9333  (assign (category room_0_81) meetingroom)
                        0.0299  (assign (category room_0_81) corridor)
        )
        (probabilistic  )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  )
        (probabilistic  0.8247  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b office) true))
        (probabilistic  )
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b office) true))
        (probabilistic  )
        (probabilistic  0.2890  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.6188  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.8392  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(create_cones_in_room robot_0__c magazine room_0_81 place_3__b), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0_81 place_3__b visualobject3), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject3 magazine in 0:81@coma
3: SUCCEEDED create_cones_in_room 0:C@spatial.sa magazine 0:81@coma 3:B@spatial.sa
4: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 3:B@spatial.sa visualobject3
5: PENDING goal 
links:
4 5 DEPENDS related-to visualobject3 MODALITY: kd 0:C@spatial.sa VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.800000025545
0 4 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 4 DEPENDS in-room 3:B@spatial.sa VALUE: 0:81@coma
0 4 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS label visualobject3 VALUE: magazine
0 2 DEPENDS entity-exists visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS label visualobject3 VALUE: magazine
0 2 DEPENDS related-to visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS relation visualobject3 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject3 VALUE: true
0 5 DEPENDS label visualobject3 VALUE: magazine
0 3 DEPENDS in-room 3:B@spatial.sa VALUE: 0:81@coma
0 3 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS not_fully_explored 0:81@coma VALUE: false
3 4 DEPENDS cones_created magazine in 0:81@coma VALUE: true
2 4 DEPENDS relation visualobject3 MODALITY: poss in VALUE: true
2 4 DEPENDS related-to visualobject3 MODALITY: poss 0:81@coma VALUE: true
2 3 THREATENS started  VALUE: true
1 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
1 3 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  room0 room1 room2 room3 room4 room5 room6 room7 room8 room_0_81 - room
           person0 person1 - person
           robot_0__c - robot
           place_0__b place_1__b place_2__b place_3__b place_4__b place_7__b place_8__b place_9__b - place
           book cerealbox magazine table - label
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l - conegroup
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0_81)
        (= (cg-label conegroup_0__l) magazine)
        (= (cg-label conegroup_1__l) magazine)
        (= (cg-label conegroup_2__l) magazine)
        (= (cg-label conegroup_3__l) magazine)
        (= (cg-place conegroup_0__l) place_2__b)
        (= (cg-place conegroup_1__l) place_3__b)
        (= (cg-place conegroup_2__l) place_7__b)
        (= (cg-place conegroup_3__l) place_8__b)
        (= (cg-related-to conegroup_0__l) room_0_81)
        (= (cg-related-to conegroup_1__l) room_0_81)
        (= (cg-related-to conegroup_2__l) room_0_81)
        (= (cg-related-to conegroup_3__l) room_0_81)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
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
        (= (entity-exists conegroup_2__l) true)
        (= (entity-exists conegroup_3__l) true)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_3__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists place_9__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0_81) true)
        (= (in-room place_0__b) room_0_81)
        (= (in-room place_1__b) room_0_81)
        (= (in-room place_2__b) room_0_81)
        (= (in-room place_3__b) room_0_81)
        (= (in-room place_7__b) room_0_81)
        (= (in-room place_8__b) room_0_81)
        (= (is-in robot_0__c) place_3__b)
        (= (label visualobject0) container)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject3) magazine)
        (= (label visualobject4) book)
        (= (p-obj_exists cerealbox in room_0_81 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0_81 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0_81 office) 0.0253)
        (= (p-obj_exists magazine in room_0_81 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0_81 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0_81 office) 0.1800)
        (= (p-visible conegroup_0__l) 0.1713)
        (= (p-visible conegroup_1__l) 0.3317)
        (= (p-visible conegroup_2__l) 0.1788)
        (= (p-visible conegroup_3__l) 0.0716)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) trueplace)
        (= (placestatus place_9__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0_81) 0)
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
        (= (search_cost book in room_0_81) 200.0000)
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
        (= (search_cost cerealbox in room7) 200.0000)
        (= (search_cost cerealbox in room8) 200.0000)
        (= (search_cost cerealbox in room_0_81) 200.0000)
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
        (= (search_cost container in room7) unknown)
        (= (search_cost container in room8) unknown)
        (= (search_cost container in room_0_81) unknown)
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
        (= (search_cost magazine in room_0_81) 200.0000)
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
        (= (search_cost table in room7) unknown)
        (= (search_cost table in room8) unknown)
        (= (search_cost table in room_0_81) unknown)
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
        (= (virtual-place room3) place_4__b)
        (= (virtual-place room6) place_7__b)
        (= (virtual-place room7) place_8__b)
        (= (virtual-place room8) place_9__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_8__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_3__b place_8__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_9__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_9__b)
        (connected place_8__b place_0__b)
        (connected place_8__b place_3__b)
        (connected place_9__b place_4__b)
        (connected place_9__b place_7__b)
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
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (probabilistic  0.8245  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.2891  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.0368  (assign (category room_0_81) office)
                        0.9333  (assign (category room_0_81) meetingroom)
                        0.0299  (assign (category room_0_81) corridor)
        )
        (probabilistic  )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0_81) true))
        (probabilistic  0.7932  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  )
        (probabilistic  0.8247  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.2001  (assign (leads_to_room place_3__b office) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.2891  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2890  (assign (leads_to_room place_2__b office) true))
        (probabilistic  )
        (probabilistic  0.2890  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.3061  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.6188  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.2790  (assign (leads_to_room place_7__b office) true))
        (probabilistic  )
        (probabilistic  0.8392  (assign (leads_to_room place_7__b corridor) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0_81 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject3 magazine in room_0_81), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0_81 place_3__b visualobject3), UNSUCCESSFUL
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:81@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa 0:Q@vision.sa magazine in 0:81@coma
3: UNSUCCESSFUL search_for_object_in_room 0:C@spatial.sa magazine 0:81@coma 3:B@spatial.sa 0:Q@vision.sa
4: PENDING goal 
links:
1 2 DEPENDS obj_exists magazine in 0:81@coma MODALITY: poss true VALUE: true
1 3 THREATENS started  VALUE: true
2 3 DEPENDS relation 0:Q@vision.sa MODALITY: poss in VALUE: true
2 3 DEPENDS related-to 0:Q@vision.sa MODALITY: poss 0:81@coma VALUE: true
0 2 DEPENDS entity-exists 0:Q@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 2 DEPENDS related-to 0:Q@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS relation 0:Q@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual 0:Q@vision.sa VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS category 0:81@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS obj_exists magazine in 0:81@coma MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:81@coma meetingroom VALUE: 0.800000025545
0 4 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 3 DEPENDS search_cost magazine in 0:81@coma VALUE: 200.0
0 3 DEPENDS in-room 3:B@spatial.sa VALUE: 0:81@coma
0 3 DEPENDS cg-related-to 2:L@binder VALUE: 0:81@coma
0 3 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:81@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS cg-label 2:L@binder VALUE: magazine
0 3 DEPENDS cg-relation 2:L@binder VALUE: in
0 3 DEPENDS label 0:Q@vision.sa VALUE: magazine
3 4 DEPENDS related-to 0:Q@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
