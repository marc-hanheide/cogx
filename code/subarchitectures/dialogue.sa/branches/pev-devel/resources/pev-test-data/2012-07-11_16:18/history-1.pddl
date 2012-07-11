(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) placeholder)
        (= (placestatus place_2__b) placeholder)
        (= (placestatus place_3__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
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
        (probabilistic  0.3042  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7967  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3042  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.7967  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.5070  (assign (category room_0__a1) meetingroom)
                        0.3172  (assign (category room_0__a1) office)
                        0.1758  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.3042  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.7967  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3042  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.3042  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3042  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-category-room_0__a1-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_1__b place_0__b), EXECUTED
(move_direct robot_0__c place_2__b place_1__b place_0__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_2__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_0__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0__a1-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
4: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa 0:B@spatial.sa
7: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 0:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 0:B@spatial.sa visualobject4
10: PENDING goal 
links:
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: committed  VALUE: false
0 7 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS label visualobject4 VALUE: magazine
0 3 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject4 VALUE: true
0 3 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 9 DEPENDS label visualobject4 VALUE: magazine
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 9 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 10 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000011756
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 6 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 6 DEPENDS connected 0:B@spatial.sa 2:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 8 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 9 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
7 8 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
1 2 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 4 THREATENS started  VALUE: true
3 9 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
3 9 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
9 10 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
2 3 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
2 8 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
8 9 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
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
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
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
        (probabilistic  0.2955  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2955  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.2955  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.2955  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.8127  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.4476  (assign (category room_0__a1) meetingroom)
                        0.4077  (assign (category room_0__a1) office)
                        0.1447  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.2955  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8127  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8127  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.0773  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.2955  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.0281  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0281  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-category-room_0__a1-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_2__b place_1__b), EXECUTED
(move_direct robot_0__c place_4__b place_2__b place_1__b), EXECUTABLE
(move_direct robot_0__c place_5__b place_4__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(move_direct robot_0__c place_3__b place_1__b place_0__b), EXECUTABLE
(move robot_0__c place_0__b place_3__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_0__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_0__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0__a1-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
4: SUCCEEDED move 0:C@spatial.sa 2:B@spatial.sa 1:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 4:B@spatial.sa 2:B@spatial.sa 1:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 5:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
7: PENDING move 0:C@spatial.sa 1:B@spatial.sa 5:B@spatial.sa
8: PENDING move_direct 0:C@spatial.sa 3:B@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
9: PENDING move 0:C@spatial.sa 0:B@spatial.sa 3:B@spatial.sa
10: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 0:B@spatial.sa
11: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 0:B@spatial.sa visualobject4
12: PENDING goal 
links:
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
3 4 THREATENS started  VALUE: true
3 11 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
3 11 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
2 3 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
2 10 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
9 10 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 10 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
9 11 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
9 11 DEPENDS in-room 0:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
1 2 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
11 12 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
10 11 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 4 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 8 DEPENDS connected 1:B@spatial.sa 0:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS label visualobject4 VALUE: magazine
0 3 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject4 VALUE: true
0 3 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: committed  VALUE: false
0 9 DEPENDS connected 0:B@spatial.sa 3:B@spatial.sa VALUE: true
0 9 DEPENDS done  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.799999986683
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 11 DEPENDS label visualobject4 VALUE: magazine
0 11 DEPENDS done  VALUE: false
0 11 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 11 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS connected 1:B@spatial.sa 2:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 10 DEPENDS done  VALUE: false
0 10 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 10 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 7 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 12 DEPENDS label visualobject4 VALUE: magazine
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (is-in robot_0__c) place_2__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) placeholder)
        (= (placestatus place_4__b) placeholder)
        (= (placestatus place_5__b) placeholder)
        (= (placestatus place_6__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room4) place_4__b)
        (= (virtual-place room5) place_6__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_5__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_6__b)
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
        (probabilistic  0.2758  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.2758  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.7144  (assign (category room_0__a1) meetingroom)
                        0.2062  (assign (category room_0__a1) office)
                        0.0794  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8493  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8493  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2758  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2758  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0808  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  )
        (probabilistic  0.0262  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0262  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-category-room_0__a1-meetingroom robot_0__c), EXECUTED
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_3__b place_2__b), EXECUTED
(move_direct robot_0__c place_6__b place_3__b place_2__b), EXECUTED
(move robot_0__c place_2__b place_6__b), EXECUTED
(move_direct robot_0__c place_4__b place_2__b place_1__b), EXECUTABLE
(move_direct robot_0__c place_5__b place_4__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_1__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-category-room_0__a1-meetingroom 0:C@spatial.sa
2: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
3: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
4: SUCCEEDED move 0:C@spatial.sa 3:B@spatial.sa 2:B@spatial.sa
5: SUCCEEDED move_direct 0:C@spatial.sa 6:B@spatial.sa 3:B@spatial.sa 2:B@spatial.sa
6: SUCCEEDED move 0:C@spatial.sa 2:B@spatial.sa 6:B@spatial.sa
7: PENDING move_direct 0:C@spatial.sa 4:B@spatial.sa 2:B@spatial.sa 1:B@spatial.sa
8: PENDING move_direct 0:C@spatial.sa 5:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
9: PENDING move 0:C@spatial.sa 1:B@spatial.sa 5:B@spatial.sa
10: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa
11: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa visualobject4
12: PENDING goal 
links:
11 12 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
2 10 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
2 3 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
10 11 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
3 11 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
3 11 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
3 4 THREATENS started  VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
9 10 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
9 10 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
9 11 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
9 11 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
8 9 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 6:B@spatial.sa
0 11 DEPENDS label visualobject4 VALUE: magazine
0 11 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 11 DEPENDS done  VALUE: false
0 11 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 12 DEPENDS label visualobject4 VALUE: magazine
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: committed  VALUE: false
0 10 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 10 DEPENDS done  VALUE: false
0 10 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 6 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS label visualobject4 VALUE: magazine
0 3 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual visualobject4 VALUE: true
0 3 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 5 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 5 DEPENDS connected 2:B@spatial.sa 3:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 8 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 7 DEPENDS connected 2:B@spatial.sa 1:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 4 DEPENDS connected 2:B@spatial.sa 3:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
0 9 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 9 DEPENDS done  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.8
0 2 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 2 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
1 2 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
1 4 THREATENS started  VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (is-in robot_0__c) place_2__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) placeholder)
        (= (placestatus place_5__b) placeholder)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room4) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_5__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_7__b)
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
        (probabilistic  0.2548  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.1911  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.2548  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.1911  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9228  (assign (category room_0__a1) meetingroom)
                        0.0600  (assign (category room_0__a1) office)
                        0.0172  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8886  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.6665  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.0846  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move_direct robot_0__c place_7__b place_2__b place_6__b), EXECUTED
(move_direct robot_0__c place_2__b place_7__b place_6__b), EXECUTABLE
(move_direct robot_0__c place_4__b place_2__b place_1__b), EXECUTABLE
(move_direct robot_0__c place_5__b place_4__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_1__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED move_direct 0:C@spatial.sa 7:B@spatial.sa 2:B@spatial.sa 6:B@spatial.sa
4: PENDING move_direct 0:C@spatial.sa 2:B@spatial.sa 7:B@spatial.sa 6:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 4:B@spatial.sa 2:B@spatial.sa 1:B@spatial.sa
6: PENDING move_direct 0:C@spatial.sa 5:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
7: PENDING move 0:C@spatial.sa 1:B@spatial.sa 5:B@spatial.sa
8: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa
9: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa visualobject4
10: PENDING goal 
links:
8 9 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
9 10 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
1 3 THREATENS started  VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 8 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
2 3 THREATENS started  VALUE: true
2 9 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 9 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS connected 2:B@spatial.sa 1:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 9 DEPENDS label visualobject4 VALUE: magazine
0 9 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 9 DEPENDS done  VALUE: false
0 9 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 10 DEPENDS label visualobject4 VALUE: magazine
0 6 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000025836
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 8 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 4 DEPENDS connected 6:B@spatial.sa 2:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS connected 6:B@spatial.sa 7:B@spatial.sa VALUE: true
0 3 DEPENDS connected 2:B@spatial.sa 6:B@spatial.sa VALUE: true
0 3 DEPENDS connected 6:B@spatial.sa 7:B@spatial.sa VALUE: true
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 2:B@spatial.sa
0 7 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
7 8 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 8 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
7 9 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
7 9 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) placeholder)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room4) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_5__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
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
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.3000  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.3000  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9397  (assign (category room_0__a1) meetingroom)
                        0.0404  (assign (category room_0__a1) office)
                        0.0199  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.3000  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.0844  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  )
        (probabilistic  0.0243  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0243  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_7__b place_4__b), EXECUTED
(move robot_0__c place_4__b place_7__b), FAILED
(move_direct robot_0__c place_5__b place_4__b place_1__b), EXECUTABLE
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_1__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED move 0:C@spatial.sa 7:B@spatial.sa 4:B@spatial.sa
4: FAILED move 0:C@spatial.sa 4:B@spatial.sa 7:B@spatial.sa
5: PENDING move_direct 0:C@spatial.sa 5:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
6: PENDING move 0:C@spatial.sa 1:B@spatial.sa 5:B@spatial.sa
7: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa
8: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa visualobject4
9: PENDING goal 
links:
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
6 7 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
6 7 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
6 8 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
6 8 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 4 DEPENDS connected 4:B@spatial.sa 7:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 3 DEPENDS connected 4:B@spatial.sa 7:B@spatial.sa VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 8 DEPENDS label visualobject4 VALUE: magazine
0 8 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 8 DEPENDS done  VALUE: false
0 8 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 5 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 5 DEPENDS connected 4:B@spatial.sa 1:B@spatial.sa VALUE: true
0 5 DEPENDS done  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000038058
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 6 DEPENDS connected 1:B@spatial.sa 5:B@spatial.sa VALUE: true
0 6 DEPENDS done  VALUE: false
0 7 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 9 DEPENDS label visualobject4 VALUE: magazine
2 3 THREATENS started  VALUE: true
2 8 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 8 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
8 9 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
7 8 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
1 3 THREATENS started  VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 7 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) placeholder)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
        (= (search_cost book on visualobject4) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room0) 200.0000)
        (= (search_cost cerealbox in room1) 200.0000)
        (= (search_cost cerealbox in room2) 200.0000)
        (= (search_cost cerealbox in room3) 200.0000)
        (= (search_cost cerealbox in room4) 200.0000)
        (= (search_cost cerealbox in room5) 200.0000)
        (= (search_cost cerealbox in room6) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
        (= (search_cost cerealbox on visualobject4) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room0) unknown)
        (= (search_cost container in room1) unknown)
        (= (search_cost container in room2) unknown)
        (= (search_cost container in room3) unknown)
        (= (search_cost container in room4) unknown)
        (= (search_cost container in room5) unknown)
        (= (search_cost container in room6) unknown)
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
        (= (search_cost magazine on visualobject4) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room0) unknown)
        (= (search_cost table in room1) unknown)
        (= (search_cost table in room2) unknown)
        (= (search_cost table in room3) unknown)
        (= (search_cost table in room4) unknown)
        (= (search_cost table in room5) unknown)
        (= (search_cost table in room6) unknown)
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room3) place_5__b)
        (= (virtual-place room4) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_5__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_6__b)
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
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9682  (assign (category room_0__a1) meetingroom)
                        0.0100  (assign (category room_0__a1) office)
                        0.0219  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0847  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.0843  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  )
        (probabilistic  0.0244  (assign (leads_to_room place_5__b meetingroom) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0244  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_4__b place_7__b), EXECUTED
(move_direct robot_0__c place_5__b place_4__b place_1__b), FAILED
(move robot_0__c place_1__b place_5__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_1__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 7:B@spatial.sa
4: FAILED move_direct 0:C@spatial.sa place_5__b 4:B@spatial.sa 1:B@spatial.sa
5: PENDING move 0:C@spatial.sa 1:B@spatial.sa place_5__b
6: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa
7: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa visualobject4
8: PENDING goal 
links:
1 6 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 3 THREATENS started  VALUE: true
6 7 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
5 6 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
5 7 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
5 7 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
2 3 THREATENS started  VALUE: true
2 7 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 7 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: place_5__b
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000036938
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 6 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 5 DEPENDS connected 1:B@spatial.sa place_5__b VALUE: true
0 5 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS connected 7:B@spatial.sa 4:B@spatial.sa VALUE: true
0 4 DEPENDS connected 1:B@spatial.sa place_5__b VALUE: true
0 4 DEPENDS connected 4:B@spatial.sa 1:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
0 8 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 7 DEPENDS label visualobject4 VALUE: magazine
0 7 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
7 8 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b place_8__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room7 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) placeholder)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
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
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
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
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
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
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room4) place_4__b)
        (= (virtual-place room5) place_6__b)
        (= (virtual-place room6) place_7__b)
        (= (virtual-place room7) place_8__b)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_1__b place_8__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_6__b)
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
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9846  (assign (category room_0__a1) meetingroom)
                        0.0113  (assign (category room_0__a1) office)
                        0.0042  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.1875  (assign (leads_to_room place_8__b meetingroom) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0847  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.6731  (assign (leads_to_room place_8__b corridor) true))
        (probabilistic  )
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move_direct robot_0__c place_8__b place_4__b place_1__b), EXECUTED
(move robot_0__c place_1__b place_8__b), EXECUTABLE
(create_cones_in_room robot_0__c magazine room_0__a1 place_1__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_1__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED move_direct 0:C@spatial.sa 8:B@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
4: PENDING move 0:C@spatial.sa 1:B@spatial.sa 8:B@spatial.sa
5: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa
6: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 1:B@spatial.sa visualobject4
7: PENDING goal 
links:
2 3 THREATENS started  VALUE: true
2 6 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 6 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 5 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 3 THREATENS started  VALUE: true
6 7 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.799999987892
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 6 DEPENDS label visualobject4 VALUE: magazine
0 6 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 7 DEPENDS label visualobject4 VALUE: magazine
0 5 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS connected 1:B@spatial.sa 8:B@spatial.sa VALUE: true
0 3 DEPENDS connected 4:B@spatial.sa 1:B@spatial.sa VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 4 DEPENDS connected 1:B@spatial.sa 8:B@spatial.sa VALUE: true
0 4 DEPENDS done  VALUE: false
5 6 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 8:B@spatial.sa
4 5 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
4 6 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
4 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b place_8__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room7 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_8__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
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
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
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
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
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
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room4) place_4__b)
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
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_4__b place_8__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_6__b)
        (connected place_8__b place_4__b)
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
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.1875  (assign (leads_to_room place_8__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9836  (assign (category room_0__a1) meetingroom)
                        0.0116  (assign (category room_0__a1) office)
                        0.0048  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.1875  (assign (leads_to_room place_8__b meetingroom) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0847  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.6731  (assign (leads_to_room place_8__b corridor) true))
        (probabilistic  )
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(move robot_0__c place_4__b place_8__b), EXECUTED
(create_cones_in_room robot_0__c magazine room_0__a1 place_4__b), EXECUTABLE
(search_for_object_in_room robot_0__c magazine room_0__a1 place_4__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 8:B@spatial.sa
4: PENDING create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 4:B@spatial.sa
5: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 4:B@spatial.sa visualobject4
6: PENDING goal 
links:
5 6 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
2 3 THREATENS started  VALUE: true
2 5 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 5 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
4 5 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
1 4 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 3 THREATENS started  VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 4 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 5 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
0 5 DEPENDS label visualobject4 VALUE: magazine
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 5 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000024239
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 6 DEPENDS label visualobject4 VALUE: magazine
0 3 DEPENDS connected 8:B@spatial.sa 4:B@spatial.sa VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 8:B@spatial.sa
0 3 DEPENDS done  VALUE: false
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b place_8__b - place
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room7 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
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
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
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
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
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
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
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
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room4) place_4__b)
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
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_4__b place_8__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_6__b)
        (connected place_8__b place_4__b)
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
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.1875  (assign (leads_to_room place_8__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.9958  (assign (category room_0__a1) meetingroom)
                        0.0029  (assign (category room_0__a1) office)
                        0.0013  (assign (category room_0__a1) corridor)
        )
        (probabilistic  0.1875  (assign (leads_to_room place_8__b meetingroom) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0847  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.6731  (assign (leads_to_room place_8__b corridor) true))
        (probabilistic  )
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(create_cones_in_room robot_0__c magazine room_0__a1 place_4__b), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0__a1 place_4__b visualobject4), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa visualobject4 magazine in 0:A1@coma
3: SUCCEEDED create_cones_in_room 0:C@spatial.sa magazine 0:A1@coma 4:B@spatial.sa
4: PENDING search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 4:B@spatial.sa visualobject4
5: PENDING goal 
links:
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 3 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS label visualobject4 VALUE: magazine
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000023942
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 3 DEPENDS in-room 4:B@spatial.sa VALUE: 0:A1@coma
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS in-room 4:B@spatial.sa VALUE: 0:A1@coma
0 4 DEPENDS label visualobject4 VALUE: magazine
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 4 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label visualobject4 VALUE: magazine
0 2 DEPENDS related-to visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists visualobject4 MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual visualobject4 VALUE: true
0 2 DEPENDS relation visualobject4 MODALITY: committed  VALUE: false
2 4 DEPENDS related-to visualobject4 MODALITY: poss 0:A1@coma VALUE: true
2 4 DEPENDS relation visualobject4 MODALITY: poss in VALUE: true
2 3 THREATENS started  VALUE: true
4 5 DEPENDS related-to visualobject4 MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS cones_created magazine in 0:A1@coma VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  corridor meetingroom office - category
           place_0__b place_1__b place_2__b place_3__b place_4__b place_6__b place_7__b place_8__b - place
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l - conegroup
           visualobject0 visualobject1 visualobject2 visualobject3 visualobject4 - visualobject
           robot_0__c - robot
           person0 person1 - person
           book cerealbox magazine table - label
           room0 room1 room2 room3 room4 room5 room6 room7 room_0__a1 - room
)

(:init  (= (associated-with person0) dummy-room)
        (= (associated-with person1) room_0__a1)
        (= (cg-label conegroup_0__l) magazine)
        (= (cg-label conegroup_1__l) magazine)
        (= (cg-label conegroup_2__l) magazine)
        (= (cg-label conegroup_3__l) magazine)
        (= (cg-label conegroup_4__l) magazine)
        (= (cg-label conegroup_5__l) magazine)
        (= (cg-label conegroup_6__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_1__b)
        (= (cg-place conegroup_2__l) place_2__b)
        (= (cg-place conegroup_3__l) place_4__b)
        (= (cg-place conegroup_4__l) place_6__b)
        (= (cg-place conegroup_5__l) place_6__b)
        (= (cg-place conegroup_6__l) place_7__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
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
        (= (entity-exists conegroup_4__l) true)
        (= (entity-exists conegroup_5__l) true)
        (= (entity-exists conegroup_6__l) true)
        (= (entity-exists place_0__b) true)
        (= (entity-exists place_1__b) true)
        (= (entity-exists place_2__b) true)
        (= (entity-exists place_3__b) true)
        (= (entity-exists place_4__b) true)
        (= (entity-exists place_6__b) true)
        (= (entity-exists place_7__b) true)
        (= (entity-exists place_8__b) true)
        (= (entity-exists robot_0__c) true)
        (= (entity-exists room_0__a1) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) container)
        (= (label visualobject1) book)
        (= (label visualobject2) cerealbox)
        (= (label visualobject3) table)
        (= (label visualobject4) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0025)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.8000)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1800)
        (= (p-visible conegroup_0__l) 0.0678)
        (= (p-visible conegroup_1__l) 0.3010)
        (= (p-visible conegroup_2__l) 0.0955)
        (= (p-visible conegroup_3__l) 0.1127)
        (= (p-visible conegroup_4__l) 0.0674)
        (= (p-visible conegroup_5__l) 0.1665)
        (= (p-visible conegroup_6__l) 0.1891)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (placestatus place_8__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject3) unknown)
        (= (related-to visualobject4) unknown)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room0) 200.0000)
        (= (search_cost book in room1) 200.0000)
        (= (search_cost book in room2) 200.0000)
        (= (search_cost book in room3) 200.0000)
        (= (search_cost book in room4) 200.0000)
        (= (search_cost book in room5) 200.0000)
        (= (search_cost book in room6) 200.0000)
        (= (search_cost book in room7) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject3) unknown)
        (= (search_cost book in visualobject4) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) unknown)
        (= (search_cost book on visualobject3) 30.0000)
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
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject3) unknown)
        (= (search_cost cerealbox in visualobject4) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) unknown)
        (= (search_cost cerealbox on visualobject3) 30.0000)
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
        (= (search_cost container in room_0__a1) unknown)
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
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject3) unknown)
        (= (search_cost magazine in visualobject4) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) unknown)
        (= (search_cost magazine on visualobject3) 30.0000)
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
        (= (search_cost table in room_0__a1) unknown)
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
        (= (virtual-place room0) place_2__b)
        (= (virtual-place room1) place_1__b)
        (= (virtual-place room2) place_3__b)
        (= (virtual-place room4) place_4__b)
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
        (connected place_2__b place_4__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_6__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_6__b)
        (connected place_4__b place_7__b)
        (connected place_4__b place_8__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_3__b)
        (connected place_6__b place_4__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_4__b)
        (connected place_7__b place_6__b)
        (connected place_8__b place_4__b)
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
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.7900  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.9958  (assign (category room_0__a1) meetingroom)
                        0.0029  (assign (category room_0__a1) office)
                        0.0013  (assign (category room_0__a1) corridor)
        )
        (probabilistic  )
        (probabilistic  )
        (probabilistic  0.0242  (assign (leads_to_room place_7__b office) true))
        (probabilistic  0.1875  (assign (leads_to_room place_8__b office) true))
        (probabilistic  0.0242  (assign (leads_to_room place_7__b meetingroom) true))
        (probabilistic  0.1875  (assign (leads_to_room place_8__b meetingroom) true))
        (probabilistic  0.2847  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8952  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.8538  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.8328  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.2734  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.0847  (assign (leads_to_room place_7__b corridor) true))
        (probabilistic  0.2513  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.6731  (assign (leads_to_room place_8__b corridor) true))
        (probabilistic  )
        (probabilistic  0.3078  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.3078  (assign (leads_to_room place_1__b meetingroom) true))
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
(__commit-object_existence_room-0 robot_0__c magazine in room_0__a1 meetingroom), EXECUTED
(__commit-sample_object_location-0 robot_0__c visualobject4 magazine in room_0__a1), EXECUTED
(search_for_object_in_room robot_0__c magazine room_0__a1 place_4__b visualobject4), UNSUCCESSFUL
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-object_existence_room-0 0:C@spatial.sa magazine in 0:A1@coma meetingroom
2: SUCCEEDED __commit-sample_object_location-0 0:C@spatial.sa 0:Q@vision.sa magazine in 0:A1@coma
3: UNSUCCESSFUL search_for_object_in_room 0:C@spatial.sa magazine 0:A1@coma 4:B@spatial.sa 0:Q@vision.sa
4: PENDING goal 
links:
2 3 DEPENDS related-to 0:Q@vision.sa MODALITY: poss 0:A1@coma VALUE: true
2 3 DEPENDS relation 0:Q@vision.sa MODALITY: poss in VALUE: true
0 1 DEPENDS started  VALUE: false
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom VALUE: 0.800000023942
0 1 DEPENDS category 0:A1@coma MODALITY: poss meetingroom VALUE: true
0 1 DEPENDS p-obj_exists magazine in 0:A1@coma meetingroom MODALITY: defined  VALUE: true
0 1 DEPENDS obj_exists_general magazine MODALITY: committed  VALUE: false
0 1 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: committed  VALUE: false
0 4 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 2 DEPENDS started  VALUE: false
0 2 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 2 DEPENDS related-to 0:Q@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS entity-exists 0:Q@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS is-virtual 0:Q@vision.sa VALUE: true
0 2 DEPENDS relation 0:Q@vision.sa MODALITY: committed  VALUE: false
0 3 DEPENDS in-room 4:B@spatial.sa VALUE: 0:A1@coma
0 3 DEPENDS label 0:Q@vision.sa VALUE: magazine
0 3 DEPENDS cg-relation 0:L@binder VALUE: in
0 3 DEPENDS cg-label 0:L@binder VALUE: magazine
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS search_cost magazine in 0:A1@coma VALUE: 200.0
0 3 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS cg-related-to 0:L@binder VALUE: 0:A1@coma
3 4 DEPENDS related-to 0:Q@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 2 DEPENDS obj_exists magazine in 0:A1@coma MODALITY: poss true VALUE: true
1 3 THREATENS started  VALUE: true
END_POPLAN
