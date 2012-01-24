(define (domain dora-avs-iros11)
  (:requirements :mapl :adl :fluents :partial-observability :dynamic-objects :action-costs)

  (:types
   conegroup - object
   robot - planning_agent
   person robot - location
   label category spatial_relation place room visualobject - concept
   visualobject room - location
   place_status - object 
;; polar_reply concept - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)
   (is-visited ?c - conegroup)

   (is-virtual ?o - object)

   ;; derived predicates
   (attached_to_room ?p - place ?r - room)
   (not_fully_explored ?r)
   (trans_related ?o - visualobject ?where - (either visualobject room))
   (cones_exist  ?l - label ?rel - spatial_relation ?where - (either visualobject room))
   ;; (cones-exist ?l - label ?r - room)
   ;; (obj-possibly-in-room ?o - visualobject ?r - room)

   ;; (any-engaged)
   (engaged ?p - person)

   ;;virtual predicates
   (cones_created  ?l - label ?rel - spatial_relation ?where - (either visualobject room))

   ;;used to early prevent instantiation of operators with undefined probability/costs
   ;; (defined ?svar - (function number))

   (started)
   (done)
   )

   ;; === perceptual fluents for modelling dialogue ===
  ;; (:percepts 
  ;;  (polar-response ?svar - (function concept) ?val - (typeof ?svar) ?reply - polar_reply)
  ;;  (general-response ?svar - (function concept) ?reply - (typeof ?svar))
  ;;  )


  (:functions
   (is-in ?o - robot) - place
   (is-in ?o - person) - place

   ;; === Default knowledge ===

   ;; expected cost of searching for an object. Used by CP planner
   (dora__cost_inroom ?l - label) - number
   (dora__cost_inobject ?l1 ?l2 - label) - number
   (dora__cost_on ?l1 ?l2 - label) - number
   (search_cost ?l - label ?rel - spatial_relation ?where - (either visualobject room)) - number
   ;; default probabilities. These come from Coma.
   (dora__inroom ?l - label ?c - category) - number
   (dora__inobject ?l1 ?l2 - label ?c - category) - number
   (dora__on ?l1 ?l2 - label ?c - category) - number

   (dora__not_inroom ?l - label ?c - category) - number

   ;; === inferred knowledge ===
   ;; The result of applying the default knowledge.
   ;; E.g. category(r1) = kitchen AND (dora__in_room cornflakes kitchen) => (obj_exists cornflakes in kitchen)
   ;; Also see the rules below
   (obj_exists ?l - label ?rel - spatial_relation  ?where - (either visualobject room)) - boolean
   (p-obj_exists ?l - label ?rel - spatial_relation  ?where - (either visualobject room) ?c - category) - number

   ;; === room properties ===
   (category ?r - room) - category
   (identity ?r - room) - category
   (roomid ?r - room) - number
   (virtual-category ?r - room) - category
   (virtual-place ?r - room) - place

   ;; === place properties ===
   (placestatus ?n - place) - place_status
   (in-room ?p - place) - room
   (place-exists ?p - place) - boolean

   ;; === placeholder properties ===
   (leads_to_room ?p - place ?c - category) - boolean

   ;; === person properties ===
   (associated-with ?p - person) - room
   (does-exist ?p - person) - boolean
   (contains-a-person-prior ?r - room) - boolean

   (unresponsive ?p - person) - boolean

   ;; === object properties ===
   (label ?o - visualobject) - label
   (related-to ?o - visualobject) - location
   (relation ?o - visualobject) -  spatial_relation

   ;; === conegroup properties ===
   ;; basic properties that determine what the conegroup was generated for 
   ;; (e.g. cone group for cornflakes ON table_1)
   (cg-label ?c - conegroup) - label
   (cg-relation ?c - conegroup) - spatial_relation
   (cg-related-to ?c - conegroup) - (either visualobject room)
   (cg-place ?c - conegroup) - place
   ;; probability of seeing an object of type (label ?c) when looking
   (p-visible ?c - conegroup) - number
   ;; the ground truth. Distribution should conform to the probability above.
   ;; Assumes that an object can be viewed from more than one conegroup
   ;; (visible_from ?o - visualobject ?c - conegroup) - boolean
   ;; If an object can only be seen from one CG, the following would be possible:
   (visible_from ?o - visualobject) - conegroup


   ;; === explanation properties ===
   (entity-exists ?o - object) - boolean

   )

  (:constants
   placeholder trueplace - place_status
   in on - spatial_relation
   container - label
   ;; yes no dontknow - polar_reply
   tutor - agent
   dummy-room - room
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

  ;; create dummy persons
  (:init-rule persons
              :parameters(?r - room)
              :precondition (and (not (is-virtual ?r))
                                 (not (exists (?p - person ?pl - place)
                                              (or (and (= (in-room ?pl) ?r)
                                                       (in-domain (is-in ?p) ?pl))
                                                  (and (is-virtual ?p)
                                                       (= (associated-with ?p) ?r))))))
              :effect (create (?p - person) (and
                                             (is-virtual ?p)
                                             (assign (associated-with ?p) ?r))
                              )
              )

  ;; create dummy rooms
  (:init-rule rooms
              :parameters(?p - place)
              :precondition (and (= (placestatus ?p) placeholder)
                                 (not (exists (?r - room)
                                              (and (= (virtual-place ?r) ?p)
                                                   (is-virtual ?r)))))
              :effect (create (?r - room) (and
                                           (is-virtual ?r)
                                           (assign (virtual-place ?r) ?p))
                              )
              )
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


  (:init-rule default_search_costs_for_room
              :parameters (?l - label  ?r - room)
              :precondition (= (search_cost ?l in ?r) unknown)
              :effect (assign (search_cost ?l in ?r) (dora__cost_inroom ?l))
              )

  (:init-rule default_search_costs_for_object_in
              :parameters (?l - label ?o - visualobject)
              :precondition (= (search_cost ?l in ?o) unknown)
              :effect (assign (search_cost ?l in ?o) (dora__cost_inobject ?l (label ?o)))
              )

  (:init-rule default_search_costs_for_object_on
              :parameters (?l - label ?o - visualobject)
              :precondition (= (search_cost ?l on ?o) unknown)
              :effect (assign (search_cost ?l on ?o) (dora__cost_on ?l (label ?o)))
              )

  ;; (:init-rule reset_defined_numbers
  ;;             :parameters (?svar - (function number))
  ;;             :precondition (defined ?svar)
  ;;             :effect (not (defined ?svar)))

  ;; (:init-rule defined_numbers
  ;;             :parameters (?svar - (function number))
  ;;             :precondition (> ?svar 0.0001)
  ;;             :effect (defined ?svar))


  (:init-rule negated-probs
              :parameters (?l - label  ?c - category)
              :precondition (defined (dora__inroom ?l ?c))
              :effect (assign (dora__not_inroom ?l ?c) (- 1.0 (dora__inroom ?l ?c)))
              )

 ;; (:init-rule reset-engaged
 ;;             :effect (not (any-engaged)))

 ;; (:init-rule engaged
 ;;             :parameters (?p - person)
 ;;             :precondition (= (engaged ?p) true)
 ;;             :effect (any-engaged))

  (:derived (attached_to_room ?p - place ?r - room)
            (exists (?p2 - place) (and (= (in-room ?p2) ?r)
                                       (not (= (placestatus ?p) trueplace))
                                       (connected ?p2 ?p))))


  (:derived (not_fully_explored ?r - room)
            (exists (?p - place) (and (attached_to_room ?p ?r))))
              
  (:derived (trans_related ?o - visualobject ?where - (either visualobject room))
            (or (poss (related-to ?o) ?where)
                (exists (?o2 - visualobject) (and (poss (related-to ?o) ?o2)
                                                 (trans_related ?o2 ?where)))))


  (:derived (cones_exist ?l - label ?rel - spatial_relation ?where - (either visualobject room))
            (exists (?c - conegroup) (and (= (cg-label ?c) ?l)
                                          (= (cg-related-to ?c) ?where)
                                          (= (cg-relation ?c) ?rel))))
                                          

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
                              (not (defined (p-obj_exists ?l in ?r ?c))))
           :effect (probabilistic (dora__inroom ?l ?c) (assign (obj_exists ?l in ?r) true)))


  ;; p(?label IN ?object | label(?object) = ?l2 AND ?object IN ?room AND category(?room) = ?cat)
  (:dtrule obj_in_obj
           :parameters (?l1 ?l2 - label ?o - visualobject ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (= (label ?o) ?l2)
                              (= (related-to ?o) ?r)
                              (= (relation ?o) in)
                              (defined (dora__inobject ?l1 ?l2 ?c))
                              (not (defined (p-obj_exists ?l1 in ?o ?c))))
           :effect (probabilistic (dora__inobject ?l1 ?l2 ?c) (assign (obj_exists ?l1 in ?o) true)))


  ;; p(?label ON ?object | label(?object) = ?l2 AND ?object IN ?room AND category(?room) = ?cat)
  (:dtrule obj_on_obj
           :parameters (?l1 ?l2 - label ?o - visualobject ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (= (label ?o) ?l2)
                              (= (related-to ?o) ?r)
                              (= (relation ?o) in)
                              (defined (dora__on ?l1 ?l2 ?c))
                              (not (defined (p-obj_exists ?l1 on ?o ?c))))
           :effect (probabilistic (dora__on ?l1 ?l2 ?c) (assign (obj_exists ?l1 on ?o) true)))

  ;; (assign (obj_in_room ?l ?r)

  ;; use posterior information from conceptual.sa 
  ;; force commitment to a room category to help the heuristic
  (:dtrule object_existence_room
           :parameters (?l - label ?rel - spatial_relation ?where - room ?c - category)
           :precondition (and (= (category ?where) ?c)
                              (defined (p-obj_exists ?l ?rel ?where ?c)))
           :effect (probabilistic (p-obj_exists ?l ?rel ?where ?c) (and (assign (obj_exists ?l ?rel ?where) true)))
           )

  ;; use posterior information from conceptual.sa
  (:dtrule object_existence_object
           :parameters (?l - label ?rel - spatial_relation ?where - visualobject ?r - room ?c - category)
           :precondition (and (= (related-to ?where) ?r)
                              (= (category ?r) ?c)
                              (defined (p-obj_exists ?l ?rel ?where ?c)))
           :effect (probabilistic (p-obj_exists ?l ?rel ?where ?c) (and (assign (obj_exists ?l ?rel ?where) true)))
           )

  ;; p(?label IN ?room | category(?room) = ?cat)
  (:dtrule person_in_room
           :parameters (?p - person ?pl - place ?r - room)
           :precondition (and (= (contains-a-person-prior ?r) true)
                              (= (in-room ?pl) ?r)
                              (= (associated-with ?p) ?r)
                              (is-virtual ?p))
           :effect (probabilistic 1.0 (assign (is-in ?p) ?pl))) ;; will automatically be normalised

  ;; probability of an object being at a specific location
  ;;used only by DT (?)
  (:dtrule sample_object_location
           :parameters (?o - visualobject ?l - label ?rel - spatial_relation ?where - (either visualobject room))
           :precondition (and (= (label ?o) ?l)
                              (is-virtual ?o)
                              (= (obj_exists ?l ?rel ?where) true))
           :effect (probabilistic 1.0 (and (assign (related-to ?o) ?where)
                                           (assign (relation ?o) ?rel)))
           )

  ;; probability of finding a specific object in a conegroup
  ;;used only by DT (?)
  (:dtrule sample_cone_visibility
           :parameters (?o - visualobject ?c - conegroup ?l - label ?rel - spatial_relation ?where - (either visualobject room))
           :precondition (and (= (cg-relation ?c) ?rel)
                              (= (cg-related-to ?c) ?where)
                              (= (relation ?o) ?rel)
                              (= (related-to ?o) ?where)
                              (= (cg-label ?c) ?l)
                              (= (label ?o) ?l)
                              (not (is-visited ?c)))
           :effect (probabilistic (p-visible ?c) (assign (visible_from ?o) ?c))
           )


  ;; Assign virtual room to a placeholder
  (:dtrule room_from_placeholder
           :parameters (?p - place ?r - room ?c - category)
           :precondition (and (= (placestatus ?p) placeholder)
                              (= (virtual-place ?r) ?p)
                              (= (leads_to_room ?p ?c) true)
                              (is-virtual ?r))
           :effect (and (probabilistic 1.0 (and (assign (in-room ?p) ?r)
                                                (assign (category ?r) ?c)))
                        (increase (total-cost) 10))
           )                                                                                                                                              
   
  ;; (:dtrule room_from_placeholder
  ;;          :parameters (?p - place ?r - room ?c - category)
  ;;          :precondition (and (= (placestatus ?p) placeholder)
  ;;                             (= (virtual-category ?r) ?c)
  ;;                             (= (leads_to_room ?p ?c) true)
  ;;                             (is-virtual ?r))
  ;;          :effect (probabilistic 1.0 (and (assign (in-room ?p) ?r)
  ;;                                          (assign (category ?r) ?c))))

  ;; (:action explore_place
  ;;          :agent (?a - robot)
  ;;          :parameters (?loc - place)
  ;;          :duration (= ?duration 0.1)
  ;;          :precondition (and  (= (is-in ?a) ?loc)
  ;;                           (= (placestatus ?loc) placeholder))
  ;;          :effect (assign (placestatus ?loc) trueplace)
  ;;          )

  (:action report_position
           :agent (?a - robot)
           :parameters (?o - visualobject)
           :variables (?p - place ?h - person)
           :precondition (and (kval ?a (related-to ?o))
                              (engaged ?h)
                              (= (is-in ?h) ?p)
                              (= (is-in ?a) ?p))
           :effect (and (position-reported ?o)
                        (increase (total-cost) 1))
           )

   (:action move
            :agent (?a - robot)
            :parameters (?to - place)
            :variables (?from - place)
            :precondition (and (or (connected ?from ?to)
                                   (connected ?to ?from))
                               (not (done))
                               (= (is-in ?a) ?from))
            :effect (and (assign (is-in ?a) ?to)
                         (assign (placestatus ?to) trueplace)
                         (kval ?a (in-room ?to))
                         (increase (total-cost) 2))

            )

   (:action move_direct
            :agent (?a - robot)
            :parameters (?to - place)
            :variables (?from - place ?via - place)
            :precondition (and (or (connected ?from ?via)
                                   (connected ?via ?from))
                               (or (connected ?via ?to)
                                   (connected ?to ?via))
                               (not (done))
                               (= (is-in ?a) ?from))
            :effect (and (assign (is-in ?a) ?to)
                         (assign (placestatus ?to) trueplace)
                         (kval ?a (in-room ?to))
                         (increase (total-cost) 3))
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
   (:action create_cones_in_room
            :agent (?a - robot)
            :parameters (?l - label ?r - room)
            :variables (?p - place)
            :precondition (and (= (is-in ?a) ?p)
                               (= (in-room ?p) ?r)
                               (poss (obj_exists ?l in ?r) true)
                               (not (not_fully_explored ?r))
                               (not (done)))
            :effect (and (cones_created ?l in ?r)
                         (increase (total-cost) 5))
            )
   
   ;; create cones for search in or on another object
   ;; precondition: robot is in the same room as the specified object
   (:action create_cones_at_object
            :agent (?a - robot)
            :parameters (?l ?lsupp - label ?rel - spatial_relation ?o - visualobject ?r - room)
            :variables (?p - place)
            :precondition (and (= (is-in ?a) ?p)
                               (= (label ?o) ?lsupp)
                               (poss (obj_exists ?l ?rel ?o) true)
                               (= (in-room ?p) ?r)
                               (= (related-to ?o) ?r)
                               (not (done)))
            :effect (and (cones_created ?l ?rel ?o)
                         (increase (total-cost) 4))
            )

   ;; (:action request-put-on-robot
   ;;          :agent (?a - robot)
   ;;          :parameters (?l - label)
   ;;          :variables (?p - person ?pl - place ?r - room ?o - visualobject)
   ;;          :precondition (and (not (done))
   ;;                          (= (is-in ?a) ?pl)
   ;;                          (= (is-in ?p) ?pl)
   ;;                          (= (in-room ?pl) ?r)
   ;;                          (trans_related ?o ?r)
   ;;                          (kval ?a (related-to ?o))
   ;;                          (= (label ?o) ?l))
   ;;          :effect (and 
   ;;                   (assign (related-to ?o) ?a)
   ;;                   (assign (relation ?o) on)
   ;;                   (increase (total-cost) 10))
   ;;          )

   ;; (:action request-pick-up
   ;;          :agent (?a - robot)
   ;;          :parameters (?l - label)
   ;;          :variables (?p - person ?pl - place ?o - visualobject)
   ;;          :precondition (and (not (done))
   ;;                          (= (is-in ?a) ?pl)
   ;;                          (= (is-in ?p) ?pl)
   ;;                          (= (related-to ?o) ?a)
   ;;                          (= (relation ?o) on)
   ;;                          (= (label ?o) ?l))
   ;;          :effect (and 
   ;;                   (assign (related-to ?o) ?p)
   ;;                   ;; (assign (relation ?o) on)
   ;;                   (increase (total-cost) 10))
   ;;          )


   ;; Abstract search action for the CP planner
   ;; Searches for an object in the room
   ;; precondition: robot is in the specified room
   (:action search_for_object_in_room
            :agent (?a - robot)
            :parameters (?l - label ?r - room)
            :variables (?p - place ?o - visualobject)
            :precondition (and (= (is-in ?a) ?p)
                               (= (in-room ?p) ?r)
                               (= (label ?o) ?l)
                               (or (cones_created ?l in ?r)
                                   (cones_exist ?l in ?r))
                               (poss (related-to ?o) ?r)
                               (poss (relation ?o) in)
                               (not (done)))
            :effect (and (increase (total-cost) (search_cost ?l in ?r)))
            :sense (= (related-to ?o) ?r)
            )
                     

   ;; ;; Abstract search action for the CP planner
   ;; ;; Searches for an object IN another object
   ;; ;; precondition: robot is in the same room as the specified object
   ;; (:durative-action search_for_object_in_object
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?l - label ?o - visualobject)
   ;;                   :variables (?p - place ?r - room ?o2 - visualobject)
   ;;                   :duration (= ?duration (search_cost ?l in ?o))
   ;;                   :condition (and (at start (and (= (is-in ?a) ?p)
   ;;                                                  (= (in-room ?p) ?r)
   ;;                                                  (poss (related-to ?o) ?r)
   ;;                                                  (kval ?a (related-to ?o))
   ;;                                                  (= (label ?o2) ?l)
   ;;                                                  (cones_created ?l in ?o)
   ;;                                                  (poss (obj_exists ?l in ?o) true)
   ;;                                                  (not (done))))
   ;;                                   )
   ;;                   :effect (kval ?a (related-to ?o2))
   ;;                   )

   ;; Abstract search action for the CP planner
   ;; Searches for an object ON another object
   ;; precondition: robot is in the same room as the specified object
   (:action search_for_object_at_object
            :agent (?a - robot)
            :parameters (?l - label ?o - visualobject ?rel - spatial_relation)
            :variables (?p - place ?r - room ?o2 - visualobject)
            :precondition (and (= (is-in ?a) ?p)
                               (= (in-room ?p) ?r)
                               (= (related-to ?o) ?r)
                               (= (label ?o2) ?l)
                               (or (cones_created ?l ?rel ?o)
                                   (cones_exist ?l ?rel ?o))
                               (poss (related-to ?o2) ?o)
                               (poss (relation ?o2) ?rel)
                               (not (done)))
            :effect (and (increase (total-cost) (search_cost ?l ?rel ?o)))
            :sense (related-to ?o2)
            )
                     

   ;; process one conegroup
   ;; TODO: how to model precondtitions for the robot's location
   ;;       and movement between conegroups?
   ;;       Currently, precondition is to be in the same room as the conegroup
   (:action process_conegroup
            :agent (?a - robot)
            :parameters (?c - conegroup)
            :variables (?p - place)
            :precondition (and (not (done))
                               (= (cg-place ?c) ?p)
                               (= (is-in ?a) ?p))
            :effect (and 
                     (increase (total-cost) 15))
            )


   (:observe visual_object
             :agent (?a - robot)
             :parameters (?c - conegroup ?o - visualobject ?l - label ?where - (either visualobject room) ?p - place)
             :execution (process_conegroup ?a ?c ?p)
             :precondition (and (= (label ?o) ?l)
                                (= (cg-label ?c) ?l)
                                (= (cg-related-to ?c) ?where))
                                
             :effect (and (when (= (visible_from ?o) ?c)
                            (probabilistic 0.8 (observed (related-to ?o) ?where)))
                          (when (not (= (visible_from ?o) ?c))
                            (probabilistic 0.05 (observed (related-to ?o) ?where)))
                          )
             )


   ;; (:observe abstract_observe_object_in_room
   ;;           :agent (?a - robot)
   ;;           :parameters (?l - label ?r - room ?p - place ?o - visualobject)
   ;;           :execution (search_for_object_in_room ?a ?l ?r ?p ?o)
   ;;           :precondition (and (= (label ?o) ?l))
                                
   ;;           :effect (and (when (= (related-to ?o) ?r)
   ;;                          (probabilistic 0.9 (observed (related-to ?o) ?r))))
   ;;           )

   (:action engage
            :agent (?a - robot)
            :parameters (?h - person)
            :variables ( ?p - place)
            :precondition (and (not (done))
                               ;; (not (any-engaged))
                               (not (= (unresponsive ?h) true))
                               (= (is-in ?h) ?p)                               
                               (= (is-in ?a) ?p))
            :effect (and 
                     (engaged ?h)
                     (increase (total-cost) 1)
                     (assign (failure-cost) 5000))
            )


   ;; (:action disengage
   ;;          :agent (?a - robot)
   ;;          :parameters (?h - person)
   ;;          :variables (?p - place)
   ;;          :precondition (and (not (done))
   ;;                             (any-engaged)
   ;;                             (= (is-in ?h) ?p)                               
   ;;                             (= (is-in ?a) ?p))
   ;;          :effect (and 
   ;;                   (not (any-engaged))
   ;;                   (increase (total-cost) 1))
   ;;          )


   ;; (:action sense-category
   ;;          :agent (?a - robot)
   ;;          :parameters (?r - room)
   ;;          :variables ( ?p - place  ?c - category)
   ;;          :precondition (and (not (done))
   ;;                             (= (is-in ?a) ?p)
   ;;                             (= (in-room ?p) ?r)
   ;;                             (poss (category ?r) ?c))
   ;;          :effect (and (increase (total-cost) 5))
   ;;          :sense (category ?r)
   ;;          )

   (:action ask-for-category-polar
            :agent (?a - robot)
            :parameters (?r - room ?c - category)
            :variables (?h - person ?p - place)
            :precondition (and (not (done))
                               (engaged ?h)
                               (= (is-in ?h) ?p)                               
                               (= (is-in ?a) ?p)
                               (= (in-room ?p) ?r))
            :effect (and 
                     (increase (total-cost) 5)
                     ;; (assign (failure-cost) 100)
                     )
            )

   (:observe room-category
             :agent (?a - robot)
             :parameters (?h - person ?c - category ?p - place ?r - room)
             :execution (ask-for-category-polar ?a ?r ?c ?h ?p)
             :precondition (and
                            (engaged ?h)
                            (= (is-in ?h) ?p))
                                
             :effect (and (when (= (category ?r) ?c)
                            (probabilistic 0.95 (observed (identity ?r) ?c))))
             )


   ;; (:observe engagement
   ;;           :agent (?a - robot)
   ;;           :parameters (?h - person ?p - place)
   ;;           :execution (engage ?a ?h ?p)
   ;;           :precondition (and)
                                
   ;;           :effect (and (when (or (not (= (is-in ?h) ?p))
   ;;                                  (= (unresponsive ?h) true))
   ;;                          (observed (unresponsive ?h) true)))
   ;;           )

   (:action look-for-people
            :agent (?a - robot)
            :variables (?p - place)
            :precondition (and (not (done))
                            (= (is-in ?a) ?p))
            :effect (and 
                     (increase (total-cost) 10))
            )

   (:observe person
             :agent (?a - robot)
             :parameters (?p - person ?pl - place)
             :execution (look-for-people ?a ?pl)
             :precondition (and )
                                
             :effect (and (when (= (is-in ?p) ?pl)
                            (probabilistic 0.7 (observed (does-exist ?p) true)))
                          (when (not (= (is-in ?p) ?pl))
                            (probabilistic 0.001 (observed (does-exist ?p) true)))
                          )
             )


   ;; (:action ask-for-category-polar
   ;;          :agent (?a - robot)
   ;;          :parameters (?r - room ?c - category)
   ;;          :variables (?pl - place ?p - person)
   ;;          :precondition (and (not (done))
   ;;                          (= (is-in ?a) ?pl)
   ;;                          (= (in-room ?pl) ?r))
   ;;          :effect (and 
   ;;                   ;; (when (= (is-in ?p) ?pl) (increase (total-cost) 7))
   ;;                   ;; (when (not (= (is-in ?p) ?pl)) (increase (total-cost) 15)))
   ;;                   (increase (total-cost) 5))
   ;;          )

   ;; (:observe category-polar
   ;;           :agent (?a - robot)
   ;;           :parameters (?r - room ?c - category ?pl - place ?p - person)
   ;;           :execution (ask-for-category-polar ?a ?r ?c ?pl ?p)
   ;;           :precondition (and (= (is-in ?p) ?pl))
                                
   ;;           :effect (and (when (= (category ?r) ?c)
   ;;                          (probabilistic 0.8 (polar-response (category ?r) ?c yes)
   ;;                                         0.1 (polar-response (category ?r) ?c no)
   ;;                                         0.1 (polar-response (category ?r) ?c dontknow)))
   ;;                        (when (not (= (category ?r) ?c))
   ;;                          (probabilistic 0.9 (polar-response (category ?r) ?c no)
   ;;                                         0.1 (polar-response (category ?r) ?c dontknow)))
   ;;                        )
   ;;           )
            
)
