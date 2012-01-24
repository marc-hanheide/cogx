(define (domain dora-avs-iros11)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   conegroup place room visualobject - object
   robot - agent
   human robot - movable
   place_status label category spatial_relation - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)
   (is-visited ?c - conegroup)

   (is-virtual ?o - object)

   ;; derived predicates
   (attached_to_room ?p - place ?r - room)
   (not_fully_explored ?r)
   ;; (trans_related ?o - visualobject ?r - room)
   (cones_exist  ?l - label ?r - room)
   ;; (cones-exist ?l - label ?r - room)
   ;; (obj-possibly-in-room ?o - visualobject ?r - room)

   ;;virtual predicates
   (cones_created  ?l - label ?rel - spatial_relation ?r - room)

   ;;used to early prevent instantiation of operators with undefined probability/costs
   (defined ?svar - (function number))

   (started)
   (done)
   )

  (:functions
   (is-in ?o - robot) - place
   (is-in ?o - human) - place

   ;; === Default knowledge ===

   ;; expected cost of searching for an object. Used by CP planner
   (dora__cost_inroom ?l - label) - number
   (dora__cost_inobject ?l1 ?l2 - label) - number
   (dora__cost_on ?l1 ?l2 - label) - number
   (search_cost ?l - label ?rel - spatial_relation ?r - room) - number
   ;; default probabilities. These come from Coma.
   (dora__inroom ?l - label ?c - category) - number
   ;; (dora__inobject ?l1 ?l2 - label ?c - category) - number
   ;; (dora__on ?l1 ?l2 - label ?c - category) - number

   ;; === inferred knowledge ===
   ;; The result of applying the default knowledge.
   ;; E.g. category(r1) = kitchen AND (dora__in_room cornflakes kitchen) => (obj_exists cornflakes in kitchen)
   ;; Also see the rules below
   (obj_exists ?l - label ?r - room) - boolean
   (p-obj_exists ?l - label ?rel - spatial_relation ?r - room) - number

   ;; === room properties ===
   (category ?r - room) - category
   (roomid ?r - room) - number
   (virtual-category ?r - room) - category

   ;; === place properties ===
   (placestatus ?n - place) - place_status
   (in-room ?p - place) - room

   ;; === placeholder properties ===
   ;; (leads_to_room ?p - place ?c - category) - boolean

   ;; === object properties ===
   (label ?o - visualobject) - label
   (related-to ?o - visualobject) - room
   (relation ?o - visualobject) -  spatial_relation

   ;; === conegroup properties ===
   ;; basic properties that determine what the conegroup was generated for 
   ;; (e.g. cone group for cornflakes ON table_1)
   (cg-label ?c - conegroup) - label
   (cg-relation ?c - conegroup) - spatial_relation
   (cg-related-to ?c - conegroup) - room
   (cg-place ?c - conegroup) - place
   ;; probability of seeing an object of type (label ?c) when looking
   (p-visible ?c - conegroup) - number
   ;; the ground truth. Distribution should conform to the probability above.
   ;; Assumes that an object can be viewed from more than one conegroup
   ;; (visible_from ?o - visualobject ?c - conegroup) - boolean
   ;; If an object can only be seen from one CG, the following would be possible:
   (visible_from ?o - visualobject) - conegroup

   )

  (:constants
   placeholder trueplace - place_status
   in on - spatial_relation
   )


  ;; create dummy objects
  (:init-rule objects
              :parameters(?l - label)
              :precondition (not (exists (?o - visualobject)
                                         (and (= (label ?o) ?l)
                                              (is-virtual ?o))))
              :effect (create (?o - visualobject) (and
                                                   (is-virtual ?o)
                                                   (assign (label ?o) ?l)
                                                   (assign (related-to ?o) UNKNOWN))
                              )
              )

  ;; ;; create dummy rooms
  ;; (:init-rule rooms
  ;;             :parameters(?c - category)
  ;;             :precondition (not (exists (?r - room)
  ;;                                        (and (= (virtual-category ?r) ?c)
  ;;                                             (is-virtual ?r))))
  ;;             :effect (create (?r - room) (and
  ;;                                          (is-virtual ?r)
  ;;                                          (assign (virtual-category ?r) ?c))
  ;;                             )
  ;;             )

  ;; create dummy objects
  (:init-rule cones
              :parameters(?r - room)
              :precondition (not (exists (?c - conegroup)
                                         (and (= (cg-related-to ?c) ?r)
                                              (is-virtual ?c))))
              :effect (create (?c - conegroup) (and
                                                   (is-virtual ?c)
                                                   (assign (cg-relation ?c) in)
                                                   (assign (cg-related-to ?c) ?r)
                                                   (assign (p-visible ?c) 0.3))
                              )
              )

  (:init-rule default_search_costs_for_room
              :parameters (?l - label  ?r - room)
              :precondition (= (search_cost ?l in ?r) unknown)
              :effect (assign (search_cost ?l in ?r) (dora__cost_inroom ?l))
              )

  (:init-rule reset_defined_numbers
              :parameters (?svar - (function number))
              :precondition (defined ?svar)
              :effect (not (defined ?svar)))

  (:init-rule defined_numbers
              :parameters (?svar - (function number))
              :precondition (> ?svar 0.0001)
              :effect (defined ?svar))


  (:derived (attached_to_room ?p - place ?r - room)
            (exists (?p2 - place) (and (= (in-room ?p2) ?r)
                                       (connected ?p2 ?p))))


  (:derived (not_fully_explored ?r - room)
            (exists (?p - place) (and (not (= (placestatus ?p) trueplace))
                                      (attached_to_room ?p ?r))))
              
  ;; (:derived (trans_related ?o - visualobject ?where - (either visualobject room))
  ;;           (or (poss (related-to ?o) ?where)
  ;;               (exists (?o2 - visualobject) (and (poss (related-to ?o) ?o2)
  ;;                                                (trans_related ?o2 ?where)))))

  (:derived (cones_exist ?l - label ?r - room)
            (exists (?c - conegroup) (and (= (cg-label ?c) ?l)
                                          (= (cg-related-to ?c) ?r)
                                          (= (cg-relation ?c) in))))
                                          

  ;; (:derived (obj-possibly-in-room ?o - visualobject ?r - room)
  ;;           (exists (?p - place) (and (= (in-room ?p) ?r)
  ;;                                     (in-domain (is-in ?o) ?p))))

  ;; (:derived (cones-exist ?l - label ?r - room)
  ;;           (exists (?c - cone ?p - place) (and (= (in-room ?p) ?r)
  ;;                                               (= (is-in ?c) ?p)
  ;;                                               (= (label ?c) ?l)))
  ;;           )

  ;; (:derived (not_fully_explored ?r - room)
  ;;           (exists (?p ?p2 - place) (and (= (in-room ?p) ?r)
  ;;                                         (not (= (placestatus ?p2) trueplace))
  ;;                                         (connected ?p ?p2))))



  ;; rules that model the conditional probabilities from default sa

  ;; p(?label IN ?room | category(?room) = ?cat)
  (:dtrule obj_in_room
           :parameters (?l - label ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (defined (dora__inroom ?l ?c))
                              (not (defined (p-obj_exists ?l in ?r))))
           :effect (probabilistic (dora__inroom ?l ?c) (assign (obj_exists ?l ?r) true)))


  ;; use posterior information from conceptual.sa 
  ;; force commitment to a room category to help the heuristic
  (:dtrule object_existence_room
           :parameters (?l - label ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (defined (p-obj_exists ?l in ?r)))
           :effect (probabilistic (p-obj_exists ?l in ?r) (and (assign (obj_exists ?l ?r) true)))
           )

  ;; probability of an object being at a specific location
  ;;used only by DT (?)
  (:dtrule sample_object_location
           :parameters (?o - visualobject ?l - label ?r - room)
           :precondition (and (= (label ?o) ?l)
                              (is-virtual ?o)
                              (= (obj_exists ?l ?r) true))
           :effect (probabilistic 1.0 (and (assign (related-to ?o) ?r)
                                           (assign (relation ?o) in)))
           )

  ;; probability of finding a specific object in a conegroup
  ;;used only by DT (?)
  (:dtrule sample_cone_visibility
           :parameters (?o - visualobject ?c - conegroup ?l - label ?r - room)
           :precondition (and ;; (= (cg-relation ?c) ?rel)
                              (= (cg-related-to ?c) ?r)
                              ;; (= (relation ?o) ?rel)
                              (= (related-to ?o) ?r)
                              (= (cg-label ?c) ?l)
                              (= (label ?o) ?l)
                              (not (is-visited ?c)))
           :effect (probabilistic (p-visible ?c) (assign (visible_from ?o) ?c))
           )

  ;; probability of finding a specific object in a conegroup
  ;;used only by DT (?)
  (:dtrule sample_virtual_cone_visibility
           :parameters (?o - visualobject ?c - conegroup ?r - room)
           :precondition (and ;; (= (cg-relation ?c) ?rel)
                              (= (cg-related-to ?c) ?r)
                              (is-virtual ?c)
                              ;; (= (relation ?o) ?rel)
                              (= (related-to ?o) ?r))
           :effect (probabilistic (p-visible ?c) (assign (visible_from ?o) ?c))
           )

  (:durative-action propagate_relation_knowledge
           :agent (?a - robot)
           :parameters (?o - visualobject)
           :duration (= ?duration 1.0)
           :condition (at start (kval ?a (visible_from ?o)))
           :effect (and (at end (kval ?a (related-to ?o)))
                        (at end (kval ?a (relation ?o))))
           )

  ;; (:durative-action explore_place
  ;;          :agent (?a - robot)
  ;;          :parameters (?loc - place)
  ;;          :duration (= ?duration 0.1)
  ;;          :condition (and (over all (= (is-in ?a) ?loc))
  ;;                          (at start (= (placestatus ?loc) placeholder)))
  ;;          :effect (change (placestatus ?loc) trueplace)
  ;;          )

  (:durative-action report_position
           :agent (?a - robot)
           :parameters (?o - visualobject)
           :variables (?p - place); ?h - human)
           :duration (= ?duration 1.0)
           :condition (over all (and (kval ?a (related-to ?o))
                                     ;(= (is-in ?h) ?p)
                                     (= (is-in ?a) ?p)))
           :effect (at end (position-reported ?o))
           )

   (:durative-action move
                     :agent (?a - robot)
                     :parameters (?to - place)
                     :variables (?from - place)
                     :duration (= ?duration 2)
                     :condition (and (over all (or (connected ?from ?to)
                                                   (connected ?to ?from)
                                                   ))
                                     (over all (not (done)))
                                     (at start (= (is-in ?a) ?from)))
                     :effect (and (change (is-in ?a) ?to)
                                  (change (placestatus ?to) trueplace))
                     )

   (:durative-action move_direct
                     :agent (?a - robot)
                     :parameters (?to - place)
                     :variables (?from - place ?via - place)
                     :duration (= ?duration 3)
                     :condition (and (over all (and (or (connected ?from ?via)
                                                        (connected ?via ?from))
                                                    (or (connected ?via ?to)
                                                        (connected ?to ?via))
                                                    (= (placestatus ?to) trueplace)
                                                   ))
                                     (over all (not (done)))
                                     (at start (= (is-in ?a) ?from)))
                     :effect (and (change (is-in ?a) ?to))
                     )

   ;; (:durative-action move_direct
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?to - place)
   ;;                   :variables (?from - place ?r - room)
   ;;                   :duration (= ?duration 8)
   ;;                   :condition (and 
   ;;                                   (over all (and (= (in-room ?from) ?r)
   ;;                                                  (or (= (in-room ?to) ?r)
   ;;                                                      (attached_to_room ?to ?r))))
   ;;                                   (over all (not (done)))
   ;;                                   (at start (= (is-in ?a) ?from)))
   ;;                   :effect (and (change (is-in ?a) ?to)
   ;;                                (change (placestatus ?to) trueplace))
   ;;                   )

   ;; create cones for search in a room
   ;; precondition: robot is in the specified room
   (:durative-action create_cones_in_room
                     :agent (?a - robot)
                     :parameters (?l - label ?r - room)
                     :variables (?p - place)
                     :duration (= ?duration 1)
                     :condition (and (over all (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (poss (obj_exists ?l ?r) true)
                                                    ;; (not (not_fully_explored ?r))
                                                    (not (cones_exist ?l ?r))
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l in ?r)))
                     )
   

   ;; Abstract search action for the CP planner
   ;; Searches for an object in the room
   ;; precondition: robot is in the specified room
   (:durative-action process_virtual_cone
                     :agent (?a - robot)
                     :parameters (?l - label ?r - room)
                     :variables (?p - place ?o - visualobject ?cg - conegroup)
                     :duration (= ?duration (search_cost ?l in ?r))
                     :condition (and (at start (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (= (label ?o) ?l)
                                                    (= (cg-related-to ?cg) ?r)
                                                    (is-virtual ?cg)
                                                    (cones_created ?l in ?r)
                                                    (poss (visible_from ?o) ?cg)
                                                    (not (done))))
                                     )
                     :effect (kval ?a (related-to ?o))
                     )
                     

   ;; process one conegroup
   ;; TODO: how to model precondtitions for the robot's location
   ;;       and movement between conegroups?
   ;;       Currently, precondition is to be in the same room as the conegroup
   (:durative-action process_conegroup
                     :agent (?a - robot)
                     :parameters (?c - conegroup)
                     :variables (?p - place)
                     :duration (= ?duration 15)
                     :condition (over all (and (not (done))
                                               (not (is-virtual ?c))
                                               (= (cg-place ?c) ?p)
                                               (= (is-in ?a) ?p)))
                     :effect (and )
                     )


   ;; process one conegroup
   ;; TODO: how to model precondtitions for the robot's location
   ;;       and movement between conegroups?
   ;;       Currently, precondition is to be in the same room as the conegroup
   (:durative-action look_for_person
                     :agent (?a - robot)
                     :variables (?p - place)
                     :duration (= ?duration 10)
                     :condition (over all (and (not (done))
                                               (= (is-in ?a) ?p)))
                     :effect (and )
                     )


   ;;TODO: The observation (observed (is-in ?o) ?p) is what the planner gets at the moment
   ;;      Doesn't really fit the new model and may need to be changed
   (:observe visual_object
             :agent (?a - robot)
             :parameters (?c - conegroup ?o - visualobject ?l - label ?r - room ?p - place)
             :execution (process_conegroup ?a ?c ?p)
             :precondition (and (= (label ?o) ?l)
                                (= (cg-label ?c) ?l)
                                (= (cg-related-to ?c) ?r))
                                
             :effect (and (when (= (visible_from ?o) ?c)
                            (probabilistic 0.8 (observed (related-to ?o) ?r)))
                          (when (not (= (visible_from ?o) ?c))
                            (probabilistic 0.05 (observed (related-to ?o) ?r)))
                          )
             )

   ;;TODO: The observation (observed (is-in ?o) ?p) is what the planner gets at the moment
   ;;      Doesn't really fit the new model and may need to be changed
   (:observe person
             :agent (?a - robot)
             :parameters (?p - person ?pl - place)
             :execution (look_for_person ?a ?p)
             :precondition (and )
                                
             :effect (and (when (= (is-in ?p) ?pl)
                            (probabilistic 0.7 (observed (is-in ?p) ?pl)))
                          (when (not (= (is-in ?p) ?pl))
                            (probabilistic 0.1 (observed (is-in ?p) ?pl)))
                          )
             )
             
)
