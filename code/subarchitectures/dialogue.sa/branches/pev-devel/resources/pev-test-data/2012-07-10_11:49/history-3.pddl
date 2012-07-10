(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual person1)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.3016  (assign (category room_0__a1) meetingroom)
                        0.6977  (assign (category room_0__a1) corridor)
                        0.0007  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-contains-a-person-prior-room_0__a1-true robot_0__c), EXECUTED
(__commit-person_in_room-0 robot_0__c person1 place_0__b room_0__a1), EXECUTED
(look-for-people robot_0__c place_0__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_0__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_0__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_0__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-contains-a-person-prior-room_0__a1-true 0:C@spatial.sa
3: SUCCEEDED __commit-person_in_room-0 0:C@spatial.sa 0:S@vision.sa 0:B@spatial.sa 0:A1@coma
4: SUCCEEDED look-for-people 0:C@spatial.sa 0:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 0:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 0:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 0:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 0:B@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 0:B@spatial.sa VALUE: true
3 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 0:B@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 0:B@spatial.sa VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
2 3 DEPENDS contains-a-person-prior 0:A1@coma MODALITY: poss true VALUE: true
2 4 THREATENS started  VALUE: true
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 4 THREATENS started  VALUE: true
1 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS associated-with 0:S@vision.sa VALUE: 0:A1@coma
0 3 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS started  VALUE: false
0 3 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 3 DEPENDS is-virtual 0:S@vision.sa VALUE: true
0 2 DEPENDS contains-a-person-prior 0:A1@coma MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 0:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 4 DEPENDS in-room 0:B@spatial.sa VALUE: 0:A1@coma
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.3016  (assign (category room_0__a1) meetingroom)
                        0.6977  (assign (category room_0__a1) corridor)
                        0.0007  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.1237  (assign (is-in person1) place_7__b)
                        0.1237  (assign (is-in person1) place_4__b)
                        0.1237  (assign (is-in person1) place_3__b)
                        0.0225  (assign (is-in person1) place_0__b)
                        0.1237  (assign (is-in person1) place_5__b)
                        0.1237  (assign (is-in person1) place_6__b)
                        0.1237  (assign (is-in person1) place_1__b)
                        0.1237  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8886  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-is-in-person1-place_7__b robot_0__c), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(move robot_0__c place_7__b place_0__b), EXECUTED
(look-for-people robot_0__c place_7__b room_0__a1 person1), EXECUTABLE
(engage robot_0__c person1 place_7__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_7__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_7__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-is-in-person1-place_7__b 0:C@spatial.sa
2: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
3: SUCCEEDED move 0:C@spatial.sa 7:B@spatial.sa 0:B@spatial.sa
4: PENDING look-for-people 0:C@spatial.sa 7:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 7:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 7:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 7:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
2 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
2 3 THREATENS started  VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
0 2 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 7:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS in-room 7:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS connected 0:B@spatial.sa 7:B@spatial.sa VALUE: true
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 5 DEPENDS done  VALUE: false
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
3 7 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
3 4 DEPENDS in-room 7:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
3 6 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.3275  (assign (category room_0__a1) meetingroom)
                        0.6720  (assign (category room_0__a1) corridor)
                        0.0006  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.1237  (assign (is-in person1) place_7__b)
                        0.1237  (assign (is-in person1) place_4__b)
                        0.1237  (assign (is-in person1) place_3__b)
                        0.0225  (assign (is-in person1) place_0__b)
                        0.1237  (assign (is-in person1) place_5__b)
                        0.1237  (assign (is-in person1) place_6__b)
                        0.1237  (assign (is-in person1) place_1__b)
                        0.1237  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8886  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-is-in-person1-place_7__b robot_0__c), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(look-for-people robot_0__c place_7__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_7__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_7__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_7__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-is-in-person1-place_7__b 0:C@spatial.sa
2: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
3: SUCCEEDED look-for-people 0:C@spatial.sa 7:B@spatial.sa 0:A1@coma 0:S@vision.sa
4: PENDING engage 0:C@spatial.sa 0:S@vision.sa 7:B@spatial.sa
5: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 7:B@spatial.sa 0:S@vision.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 7:B@spatial.sa 0:S@vision.sa
7: PENDING goal 
links:
0 5 DEPENDS entity-exists 0:A1@coma VALUE: true
0 5 DEPENDS label A:w@planner.sa VALUE: container
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 5 DEPENDS in-room 7:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 2 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 6 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 6 DEPENDS label B:w@planner.sa VALUE: magazine
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS in-room 7:B@spatial.sa VALUE: 0:A1@coma
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 3 DEPENDS in-room 7:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
1 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
1 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 7:B@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS engaged 0:S@vision.sa VALUE: true
2 6 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
2 3 THREATENS started  VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_7__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.3275  (assign (category room_0__a1) meetingroom)
                        0.6720  (assign (category room_0__a1) corridor)
                        0.0006  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0173  (assign (is-in person1) place_7__b)
                        0.1384  (assign (is-in person1) place_4__b)
                        0.1384  (assign (is-in person1) place_3__b)
                        0.0252  (assign (is-in person1) place_0__b)
                        0.1384  (assign (is-in person1) place_5__b)
                        0.1384  (assign (is-in person1) place_6__b)
                        0.1384  (assign (is-in person1) place_1__b)
                        0.1384  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8732  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_3__b robot_0__c), EXECUTED
(move robot_0__c place_3__b place_7__b), EXECUTED
(look-for-people robot_0__c place_3__b room_0__a1 person1), EXECUTABLE
(engage robot_0__c person1 place_3__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_3__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_3__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_3__b 0:C@spatial.sa
3: SUCCEEDED move 0:C@spatial.sa 3:B@spatial.sa 7:B@spatial.sa
4: PENDING look-for-people 0:C@spatial.sa 3:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 3:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 3:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 3:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
3 7 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
3 4 DEPENDS in-room 3:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 3 THREATENS started  VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 5 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS connected 7:B@spatial.sa 3:B@spatial.sa VALUE: true
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 7:B@spatial.sa
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
1 3 THREATENS started  VALUE: true
1 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_3__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2683  (assign (category room_0__a1) meetingroom)
                        0.7312  (assign (category room_0__a1) corridor)
                        0.0004  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0173  (assign (is-in person1) place_7__b)
                        0.1384  (assign (is-in person1) place_4__b)
                        0.1384  (assign (is-in person1) place_3__b)
                        0.0252  (assign (is-in person1) place_0__b)
                        0.1384  (assign (is-in person1) place_5__b)
                        0.1384  (assign (is-in person1) place_6__b)
                        0.1384  (assign (is-in person1) place_1__b)
                        0.1384  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8732  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_3__b robot_0__c), EXECUTED
(look-for-people robot_0__c place_3__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_3__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_3__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_3__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_3__b 0:C@spatial.sa
3: SUCCEEDED look-for-people 0:C@spatial.sa 3:B@spatial.sa 0:A1@coma 0:S@vision.sa
4: PENDING engage 0:C@spatial.sa 0:S@vision.sa 3:B@spatial.sa
5: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 3:B@spatial.sa 0:S@vision.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 3:B@spatial.sa 0:S@vision.sa
7: PENDING goal 
links:
5 6 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 3:B@spatial.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 6 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
0 4 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 3 DEPENDS in-room 3:B@spatial.sa VALUE: 0:A1@coma
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 5 DEPENDS entity-exists 0:A1@coma VALUE: true
0 5 DEPENDS in-room 3:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS label A:w@planner.sa VALUE: container
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 5 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 3:B@spatial.sa
0 6 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 6 DEPENDS label B:w@planner.sa VALUE: magazine
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
4 5 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS engaged 0:S@vision.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_0__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1265  (assign (category room_0__a1) meetingroom)
                        0.8733  (assign (category room_0__a1) corridor)
                        0.0002  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0197  (assign (is-in person1) place_7__b)
                        0.1570  (assign (is-in person1) place_4__b)
                        0.0286  (assign (is-in person1) place_3__b)
                        0.0197  (assign (is-in person1) place_0__b)
                        0.1570  (assign (is-in person1) place_5__b)
                        0.1570  (assign (is-in person1) place_6__b)
                        0.1570  (assign (is-in person1) place_1__b)
                        0.1570  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8527  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-is-in-person1-place_1__b robot_0__c), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(move robot_0__c place_1__b place_0__b), EXECUTED
(look-for-people robot_0__c place_1__b room_0__a1 person1), EXECUTABLE
(engage robot_0__c person1 place_1__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_1__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_1__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-is-in-person1-place_1__b 0:C@spatial.sa
2: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
3: SUCCEEDED move 0:C@spatial.sa 1:B@spatial.sa 0:B@spatial.sa
4: PENDING look-for-people 0:C@spatial.sa 1:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 1:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 1:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 1:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
2 3 THREATENS started  VALUE: true
2 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
1 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
1 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
1 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
3 4 DEPENDS in-room 1:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 7 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
3 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 0:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS connected 0:B@spatial.sa 1:B@spatial.sa VALUE: true
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1137  (assign (category room_0__a1) meetingroom)
                        0.8862  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0197  (assign (is-in person1) place_7__b)
                        0.1570  (assign (is-in person1) place_4__b)
                        0.0286  (assign (is-in person1) place_3__b)
                        0.0197  (assign (is-in person1) place_0__b)
                        0.1570  (assign (is-in person1) place_5__b)
                        0.1570  (assign (is-in person1) place_6__b)
                        0.1570  (assign (is-in person1) place_1__b)
                        0.1570  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8527  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_1__b robot_0__c), EXECUTED
(look-for-people robot_0__c place_1__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_1__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_1__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_1__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_1__b 0:C@spatial.sa
3: SUCCEEDED look-for-people 0:C@spatial.sa 1:B@spatial.sa 0:A1@coma 0:S@vision.sa
4: PENDING engage 0:C@spatial.sa 0:S@vision.sa 1:B@spatial.sa
5: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 1:B@spatial.sa 0:S@vision.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 1:B@spatial.sa 0:S@vision.sa
7: PENDING goal 
links:
6 7 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
2 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 1:B@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 6 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
5 6 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
0 5 DEPENDS entity-exists 0:A1@coma VALUE: true
0 5 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS label A:w@planner.sa VALUE: container
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 5 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 6 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 6 DEPENDS label B:w@planner.sa VALUE: magazine
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS in-room 1:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS in-room 1:B@spatial.sa VALUE: 0:A1@coma
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS unresponsive 0:S@vision.sa VALUE: false
4 5 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS engaged 0:S@vision.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_1__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists cerealbox in room_0__a1 office) 0.0253)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-obj_exists magazine in room_0__a1 office) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1137  (assign (category room_0__a1) meetingroom)
                        0.8862  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0227  (assign (is-in person1) place_7__b)
                        0.1809  (assign (is-in person1) place_4__b)
                        0.0329  (assign (is-in person1) place_3__b)
                        0.0227  (assign (is-in person1) place_0__b)
                        0.1809  (assign (is-in person1) place_5__b)
                        0.1809  (assign (is-in person1) place_6__b)
                        0.0227  (assign (is-in person1) place_1__b)
                        0.1809  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8245  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_4__b robot_0__c), EXECUTED
(move robot_0__c place_4__b place_1__b), EXECUTED
(look-for-people robot_0__c place_4__b room_0__a1 person1), EXECUTABLE
(engage robot_0__c person1 place_4__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_4__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_4__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_4__b 0:C@spatial.sa
3: SUCCEEDED move 0:C@spatial.sa 4:B@spatial.sa 1:B@spatial.sa
4: PENDING look-for-people 0:C@spatial.sa 4:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 4:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 4:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 4:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 1:B@spatial.sa
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS connected 1:B@spatial.sa 4:B@spatial.sa VALUE: true
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 5 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
3 7 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
3 4 DEPENDS in-room 4:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
1 3 THREATENS started  VALUE: true
1 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
2 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 3 THREATENS started  VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1158  (assign (category room_0__a1) meetingroom)
                        0.8841  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0227  (assign (is-in person1) place_7__b)
                        0.1809  (assign (is-in person1) place_4__b)
                        0.0329  (assign (is-in person1) place_3__b)
                        0.0227  (assign (is-in person1) place_0__b)
                        0.1809  (assign (is-in person1) place_5__b)
                        0.1809  (assign (is-in person1) place_6__b)
                        0.0227  (assign (is-in person1) place_1__b)
                        0.1809  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.8245  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_4__b robot_0__c), EXECUTED
(look-for-people robot_0__c place_4__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_4__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_4__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_4__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_4__b 0:C@spatial.sa
3: SUCCEEDED look-for-people 0:C@spatial.sa 4:B@spatial.sa 0:A1@coma 0:S@vision.sa
4: PENDING engage 0:C@spatial.sa 0:S@vision.sa 4:B@spatial.sa
5: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 4:B@spatial.sa 0:S@vision.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 4:B@spatial.sa 0:S@vision.sa
7: PENDING goal 
links:
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
2 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 4:B@spatial.sa VALUE: true
4 5 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS engaged 0:S@vision.sa VALUE: true
1 3 THREATENS started  VALUE: true
1 6 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS in-room 4:B@spatial.sa VALUE: 0:A1@coma
0 5 DEPENDS entity-exists 0:A1@coma VALUE: true
0 5 DEPENDS in-room 4:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS label A:w@planner.sa VALUE: container
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 5 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 4 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 4 DEPENDS done  VALUE: false
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 6 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 6 DEPENDS label B:w@planner.sa VALUE: magazine
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
6 7 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 6 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_4__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1158  (assign (category room_0__a1) meetingroom)
                        0.8841  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0266  (assign (is-in person1) place_7__b)
                        0.0266  (assign (is-in person1) place_4__b)
                        0.0387  (assign (is-in person1) place_3__b)
                        0.0266  (assign (is-in person1) place_0__b)
                        0.2125  (assign (is-in person1) place_5__b)
                        0.2125  (assign (is-in person1) place_6__b)
                        0.0266  (assign (is-in person1) place_1__b)
                        0.2125  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.7828  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(__commit-is-in-person1-place_5__b robot_0__c), EXECUTED
(move robot_0__c place_5__b place_4__b), EXECUTED
(look-for-people robot_0__c place_5__b room_0__a1 person1), EXECUTABLE
(engage robot_0__c person1 place_5__b), EXECUTABLE
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_5__b person1), EXECUTABLE
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_5__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
2: SUCCEEDED __commit-is-in-person1-place_5__b 0:C@spatial.sa
3: SUCCEEDED move 0:C@spatial.sa 5:B@spatial.sa 4:B@spatial.sa
4: PENDING look-for-people 0:C@spatial.sa 5:B@spatial.sa 0:A1@coma 0:S@vision.sa
5: PENDING engage 0:C@spatial.sa 0:S@vision.sa 5:B@spatial.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 5:B@spatial.sa 0:S@vision.sa
7: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 5:B@spatial.sa 0:S@vision.sa
8: PENDING goal 
links:
5 6 DEPENDS engaged 0:S@vision.sa VALUE: true
5 7 DEPENDS engaged 0:S@vision.sa VALUE: true
0 5 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 5 DEPENDS done  VALUE: false
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 2 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 7 DEPENDS done  VALUE: false
0 7 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 7 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 7 DEPENDS label B:w@planner.sa VALUE: magazine
0 7 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 4:B@spatial.sa
0 3 DEPENDS connected 4:B@spatial.sa 5:B@spatial.sa VALUE: true
0 4 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS in-room 5:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS entity-exists 0:A1@coma VALUE: true
0 6 DEPENDS label A:w@planner.sa VALUE: container
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 6 DEPENDS in-room 5:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
1 3 THREATENS started  VALUE: true
1 7 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
2 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
2 3 THREATENS started  VALUE: true
2 7 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
2 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
2 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
7 8 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
3 4 DEPENDS in-room 5:B@spatial.sa MODALITY: kval 0:C@spatial.sa VALUE: true
3 7 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
3 6 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
3 5 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
4 7 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
4 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
6 8 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_5__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.1610  (assign (category room_0__a1) meetingroom)
                        0.8389  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0266  (assign (is-in person1) place_7__b)
                        0.0266  (assign (is-in person1) place_4__b)
                        0.0387  (assign (is-in person1) place_3__b)
                        0.0266  (assign (is-in person1) place_0__b)
                        0.2125  (assign (is-in person1) place_5__b)
                        0.2125  (assign (is-in person1) place_6__b)
                        0.0266  (assign (is-in person1) place_1__b)
                        0.2125  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.7828  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(__commit-is-in-person1-place_5__b robot_0__c), EXECUTED
(__commit-entity-exists-visualobject__a_w-true robot_0__c), EXECUTED
(look-for-people robot_0__c place_5__b room_0__a1 person1), EXECUTED
(engage robot_0__c person1 place_5__b), EXECUTED
(ask-for-object-existence robot_0__c container visualobject__a_w in room_0__a1 place_5__b person1), UNSUCCESSFUL
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_5__b person1), EXECUTABLE
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: SUCCEEDED __commit-is-in-person1-place_5__b 0:C@spatial.sa
2: SUCCEEDED __commit-entity-exists-visualobject__a_w-true 0:C@spatial.sa
3: SUCCEEDED look-for-people 0:C@spatial.sa 5:B@spatial.sa 0:A1@coma 0:S@vision.sa
4: SUCCEEDED engage 0:C@spatial.sa 0:S@vision.sa 5:B@spatial.sa
5: UNSUCCESSFUL ask-for-object-existence 0:C@spatial.sa container A:w@planner.sa in 0:A1@coma 5:B@spatial.sa 0:S@vision.sa
6: PENDING ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 5:B@spatial.sa 0:S@vision.sa
7: PENDING goal 
links:
2 6 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
2 3 THREATENS started  VALUE: true
4 5 DEPENDS engaged 0:S@vision.sa VALUE: true
4 6 DEPENDS engaged 0:S@vision.sa VALUE: true
6 7 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 6 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 4 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
3 5 DEPENDS is-in 0:S@vision.sa MODALITY: kd 0:C@spatial.sa VALUE: true
1 5 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
1 6 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
1 3 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
1 4 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
5 6 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
5 7 DEPENDS entity-exists A:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
0 2 DEPENDS entity-exists A:w@planner.sa MODALITY: committed  VALUE: false
0 2 DEPENDS started  VALUE: false
0 4 DEPENDS unresponsive 0:S@vision.sa VALUE: false
0 4 DEPENDS done  VALUE: false
0 4 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 6 DEPENDS done  VALUE: false
0 6 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 6 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 6 DEPENDS label B:w@planner.sa VALUE: magazine
0 6 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 6 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: committed  VALUE: false
0 1 DEPENDS started  VALUE: false
0 3 DEPENDS not_fully_explored 0:A1@coma VALUE: false
0 3 DEPENDS done  VALUE: false
0 3 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 3 DEPENDS in-room 5:B@spatial.sa VALUE: 0:A1@coma
0 3 DEPENDS in-room 5:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS entity-exists 0:A1@coma VALUE: true
0 5 DEPENDS label A:w@planner.sa VALUE: container
0 5 DEPENDS done  VALUE: false
0 5 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 5 DEPENDS entity-exists 0:A1@coma MODALITY: poss true VALUE: true
0 5 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
0 5 DEPENDS in-room 5:B@spatial.sa MODALITY: poss 0:A1@coma VALUE: true
END_POPLAN
(define (problem cogxtask)

(:domain dora-avs-iros11)

(:objects  book cerealbox magazine table - label
           person0 person1 - person
           robot_0__c - robot
           corridor meetingroom office - category
           conegroup_0__l conegroup_1__l conegroup_2__l conegroup_3__l conegroup_4__l conegroup_5__l conegroup_6__l conegroup_7__l conegroup_8__l - conegroup
           place_0__b place_1__b place_2__b place_3__b place_4__b place_5__b place_6__b place_7__b - place
           room_0__a1 - room
           visualobject0 visualobject1 visualobject2 visualobject__a_w visualobject__b_w - visualobject
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
        (= (cg-label conegroup_7__l) magazine)
        (= (cg-label conegroup_8__l) magazine)
        (= (cg-place conegroup_0__l) place_0__b)
        (= (cg-place conegroup_1__l) place_0__b)
        (= (cg-place conegroup_2__l) place_1__b)
        (= (cg-place conegroup_3__l) place_1__b)
        (= (cg-place conegroup_4__l) place_2__b)
        (= (cg-place conegroup_5__l) place_3__b)
        (= (cg-place conegroup_6__l) place_4__b)
        (= (cg-place conegroup_7__l) place_5__b)
        (= (cg-place conegroup_8__l) place_6__b)
        (= (cg-related-to conegroup_0__l) room_0__a1)
        (= (cg-related-to conegroup_1__l) room_0__a1)
        (= (cg-related-to conegroup_2__l) room_0__a1)
        (= (cg-related-to conegroup_3__l) room_0__a1)
        (= (cg-related-to conegroup_4__l) room_0__a1)
        (= (cg-related-to conegroup_5__l) room_0__a1)
        (= (cg-related-to conegroup_6__l) room_0__a1)
        (= (cg-related-to conegroup_7__l) room_0__a1)
        (= (cg-related-to conegroup_8__l) room_0__a1)
        (= (cg-relation conegroup_0__l) in)
        (= (cg-relation conegroup_1__l) in)
        (= (cg-relation conegroup_2__l) in)
        (= (cg-relation conegroup_3__l) in)
        (= (cg-relation conegroup_4__l) in)
        (= (cg-relation conegroup_5__l) in)
        (= (cg-relation conegroup_6__l) in)
        (= (cg-relation conegroup_7__l) in)
        (= (cg-relation conegroup_8__l) in)
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
        (= (dora__inroom magazine office) 0.8000)
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
        (= (dora__not_inroom magazine office) 0.2000)
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
        (= (entity-exists conegroup_7__l) true)
        (= (entity-exists conegroup_8__l) true)
        (= (entity-exists person1) true)
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
        (= (entity-exists visualobject__a_w) true)
        (= (in-room place_0__b) room_0__a1)
        (= (in-room place_1__b) room_0__a1)
        (= (in-room place_2__b) room_0__a1)
        (= (in-room place_3__b) room_0__a1)
        (= (in-room place_4__b) room_0__a1)
        (= (in-room place_5__b) room_0__a1)
        (= (in-room place_6__b) room_0__a1)
        (= (in-room place_7__b) room_0__a1)
        (= (is-in robot_0__c) place_5__b)
        (= (label visualobject0) book)
        (= (label visualobject1) cerealbox)
        (= (label visualobject2) table)
        (= (label visualobject__a_w) container)
        (= (label visualobject__b_w) magazine)
        (= (p-obj_exists cerealbox in room_0__a1 corridor) 0.0519)
        (= (p-obj_exists cerealbox in room_0__a1 meetingroom) 0.0600)
        (= (p-obj_exists magazine in room_0__a1 corridor) 0.0002)
        (= (p-obj_exists magazine in room_0__a1 meetingroom) 0.1027)
        (= (p-visible conegroup_0__l) 0.0136)
        (= (p-visible conegroup_1__l) 0.0108)
        (= (p-visible conegroup_2__l) 0.0297)
        (= (p-visible conegroup_3__l) 0.0179)
        (= (p-visible conegroup_4__l) 0.8601)
        (= (p-visible conegroup_5__l) 0.0144)
        (= (p-visible conegroup_6__l) 0.0161)
        (= (p-visible conegroup_7__l) 0.0149)
        (= (p-visible conegroup_8__l) 0.0225)
        (= (placestatus place_0__b) trueplace)
        (= (placestatus place_1__b) trueplace)
        (= (placestatus place_2__b) trueplace)
        (= (placestatus place_3__b) trueplace)
        (= (placestatus place_4__b) trueplace)
        (= (placestatus place_5__b) trueplace)
        (= (placestatus place_6__b) trueplace)
        (= (placestatus place_7__b) trueplace)
        (= (related-to visualobject0) unknown)
        (= (related-to visualobject1) unknown)
        (= (related-to visualobject2) unknown)
        (= (related-to visualobject__a_w) room_0__a1)
        (= (related-to visualobject__b_w) visualobject__a_w)
        (= (relation visualobject__a_w) in)
        (= (relation visualobject__b_w) in)
        (= (roomid room_0__a1) 0)
        (= (search_cost book in dummy-room) 200.0000)
        (= (search_cost book in room_0__a1) 200.0000)
        (= (search_cost book in visualobject0) unknown)
        (= (search_cost book in visualobject1) unknown)
        (= (search_cost book in visualobject2) unknown)
        (= (search_cost book in visualobject__a_w) unknown)
        (= (search_cost book in visualobject__b_w) unknown)
        (= (search_cost book on visualobject0) unknown)
        (= (search_cost book on visualobject1) unknown)
        (= (search_cost book on visualobject2) 30.0000)
        (= (search_cost book on visualobject__a_w) unknown)
        (= (search_cost book on visualobject__b_w) unknown)
        (= (search_cost cerealbox in dummy-room) 200.0000)
        (= (search_cost cerealbox in room_0__a1) 200.0000)
        (= (search_cost cerealbox in visualobject0) unknown)
        (= (search_cost cerealbox in visualobject1) unknown)
        (= (search_cost cerealbox in visualobject2) unknown)
        (= (search_cost cerealbox in visualobject__a_w) unknown)
        (= (search_cost cerealbox in visualobject__b_w) unknown)
        (= (search_cost cerealbox on visualobject0) unknown)
        (= (search_cost cerealbox on visualobject1) unknown)
        (= (search_cost cerealbox on visualobject2) 30.0000)
        (= (search_cost cerealbox on visualobject__a_w) unknown)
        (= (search_cost cerealbox on visualobject__b_w) unknown)
        (= (search_cost container in dummy-room) unknown)
        (= (search_cost container in room_0__a1) unknown)
        (= (search_cost container in visualobject0) unknown)
        (= (search_cost container in visualobject1) unknown)
        (= (search_cost container in visualobject2) unknown)
        (= (search_cost container in visualobject__a_w) unknown)
        (= (search_cost container in visualobject__b_w) unknown)
        (= (search_cost container on visualobject0) unknown)
        (= (search_cost container on visualobject1) unknown)
        (= (search_cost container on visualobject2) unknown)
        (= (search_cost container on visualobject__a_w) unknown)
        (= (search_cost container on visualobject__b_w) unknown)
        (= (search_cost magazine in dummy-room) 200.0000)
        (= (search_cost magazine in room_0__a1) 200.0000)
        (= (search_cost magazine in visualobject0) unknown)
        (= (search_cost magazine in visualobject1) unknown)
        (= (search_cost magazine in visualobject2) unknown)
        (= (search_cost magazine in visualobject__a_w) unknown)
        (= (search_cost magazine in visualobject__b_w) unknown)
        (= (search_cost magazine on visualobject0) unknown)
        (= (search_cost magazine on visualobject1) unknown)
        (= (search_cost magazine on visualobject2) 30.0000)
        (= (search_cost magazine on visualobject__a_w) unknown)
        (= (search_cost magazine on visualobject__b_w) unknown)
        (= (search_cost table in dummy-room) unknown)
        (= (search_cost table in room_0__a1) unknown)
        (= (search_cost table in visualobject0) unknown)
        (= (search_cost table in visualobject1) unknown)
        (= (search_cost table in visualobject2) unknown)
        (= (search_cost table in visualobject__a_w) unknown)
        (= (search_cost table in visualobject__b_w) unknown)
        (= (search_cost table on visualobject0) unknown)
        (= (search_cost table on visualobject1) unknown)
        (= (search_cost table on visualobject2) unknown)
        (= (search_cost table on visualobject__a_w) unknown)
        (= (search_cost table on visualobject__b_w) unknown)
        (= (unresponsive person1) false)
        (connected place_0__b place_1__b)
        (connected place_0__b place_2__b)
        (connected place_0__b place_3__b)
        (connected place_0__b place_7__b)
        (connected place_1__b place_0__b)
        (connected place_1__b place_2__b)
        (connected place_1__b place_4__b)
        (connected place_2__b place_0__b)
        (connected place_2__b place_1__b)
        (connected place_2__b place_3__b)
        (connected place_2__b place_4__b)
        (connected place_2__b place_5__b)
        (connected place_2__b place_6__b)
        (connected place_2__b place_7__b)
        (connected place_3__b place_0__b)
        (connected place_3__b place_2__b)
        (connected place_3__b place_7__b)
        (connected place_4__b place_1__b)
        (connected place_4__b place_2__b)
        (connected place_4__b place_5__b)
        (connected place_5__b place_2__b)
        (connected place_5__b place_4__b)
        (connected place_5__b place_6__b)
        (connected place_6__b place_2__b)
        (connected place_6__b place_5__b)
        (connected place_6__b place_7__b)
        (connected place_7__b place_0__b)
        (connected place_7__b place_2__b)
        (connected place_7__b place_3__b)
        (connected place_7__b place_6__b)
        (engaged person1)
        (hyp (entity-exists visualobject__a_w) true)
        (hyp (entity-exists visualobject__b_w) true)
        (is-virtual person0)
        (is-virtual visualobject0)
        (is-virtual visualobject1)
        (is-virtual visualobject2)
        (is-virtual visualobject__a_w)
        (is-virtual visualobject__b_w)
        (not (is-visited conegroup_0__l))
        (not (is-visited conegroup_1__l))
        (not (is-visited conegroup_2__l))
        (not (is-visited conegroup_3__l))
        (not (is-visited conegroup_4__l))
        (not (is-visited conegroup_5__l))
        (not (is-visited conegroup_6__l))
        (not (is-visited conegroup_7__l))
        (not (is-visited conegroup_8__l))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b office) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b meetingroom) true))
        (probabilistic  0.3580  (assign (leads_to_room place_3__b office) true))
        (probabilistic  0.6823  (assign (leads_to_room place_1__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b meetingroom) true))
        (probabilistic  0.7319  (assign (leads_to_room place_4__b corridor) true))
        (probabilistic  0.2777  (assign (leads_to_room place_5__b corridor) true))
        (probabilistic  0.6659  (assign (leads_to_room place_2__b corridor) true))
        (probabilistic  0.1610  (assign (category room_0__a1) meetingroom)
                        0.8389  (assign (category room_0__a1) corridor)
                        0.0001  (assign (category room_0__a1) office)
        )
        (probabilistic  0.3383  (assign (leads_to_room place_4__b meetingroom) true))
        (probabilistic  0.3383  (assign (leads_to_room place_4__b office) true))
        (probabilistic  0.2000  (assign (contains-a-person-prior room_0__a1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b meetingroom) true))
        (probabilistic  0.3805  (assign (leads_to_room place_2__b meetingroom) true))
        (probabilistic  0.6094  (assign (leads_to_room place_6__b corridor) true))
        (probabilistic  0.6979  (assign (leads_to_room place_3__b corridor) true))
        (probabilistic  0.3666  (assign (leads_to_room place_1__b office) true))
        (probabilistic  0.0003  (assign (is-in person1) place_7__b)
                        0.0003  (assign (is-in person1) place_4__b)
                        0.0005  (assign (is-in person1) place_3__b)
                        0.0003  (assign (is-in person1) place_0__b)
                        0.9907  (assign (is-in person1) place_5__b)
                        0.0027  (assign (is-in person1) place_6__b)
                        0.0003  (assign (is-in person1) place_1__b)
                        0.0027  (assign (is-in person1) place_2__b)
        )
        (probabilistic  0.3805  (assign (leads_to_room place_2__b office) true))
        (probabilistic  0.9979  (assign (does-exist person1) true))
        (probabilistic  0.3395  (assign (leads_to_room place_6__b office) true))
        (probabilistic  0.1392  (assign (leads_to_room place_5__b meetingroom) true))
)

(:goal  (and  (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__b_w)))
              (exists (?o - visualobject)  (kval robot_0__c (entity-exists visualobject__a_w)))
        )
)
(:metric minimize (total-cost ))

)
END_PROBLEM
(init ), EXECUTED
(ask-for-object-existence robot_0__c magazine visualobject__b_w in visualobject__a_w place_5__b person1), UNSUCCESSFUL
(goal ), EXECUTABLE
END_PLAN
actions:
0: SUCCEEDED init 
1: UNSUCCESSFUL ask-for-object-existence 0:C@spatial.sa magazine B:w@planner.sa in A:w@planner.sa 5:B@spatial.sa 0:S@vision.sa
2: PENDING goal 
links:
0 2 DEPENDS entity-exists A:w@planner.sa VALUE: true
0 1 DEPENDS done  VALUE: false
0 1 DEPENDS entity-exists A:w@planner.sa MODALITY: poss true VALUE: true
0 1 DEPENDS is-in 0:S@vision.sa VALUE: 5:B@spatial.sa
0 1 DEPENDS is-in 0:S@vision.sa MODALITY: poss 5:B@spatial.sa VALUE: true
0 1 DEPENDS related-to B:w@planner.sa MODALITY: poss A:w@planner.sa VALUE: true
0 1 DEPENDS can-relate 0:C@spatial.sa 0:A1@coma 0:S@vision.sa VALUE: false
0 1 DEPENDS label B:w@planner.sa VALUE: magazine
0 1 DEPENDS is-in 0:C@spatial.sa VALUE: 5:B@spatial.sa
0 1 DEPENDS entity-exists A:w@planner.sa VALUE: true
0 1 DEPENDS engaged 0:S@vision.sa VALUE: true
0 1 DEPENDS related-to A:w@planner.sa MODALITY: poss 0:A1@coma VALUE: true
1 2 DEPENDS entity-exists B:w@planner.sa MODALITY: kd 0:C@spatial.sa VALUE: true
END_POPLAN
