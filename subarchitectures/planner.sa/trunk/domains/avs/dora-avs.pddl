(define (domain dora-test-100)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   conegroup place room - object
   visualobject - movable
   robot human - agent
   place_status label category spatial_relation - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)

   (is-virtual ?o - object)

   ;; derived predicates
   (attached_to_room ?p - place ?r - room)
   (not_fully_explored ?r)
   ;; (cones-exist ?l - label ?r - room)
   ;; (obj-possibly-in-room ?o - visualobject ?r - room)

   ;;virtual predicates
   (cones_created  ?l - label ?rel - spatial_relation ?where - (either visualobject room))

   (started)
   (done)
   )

  (:functions
   (is-in ?o - agent) - place

   ;; === Default knowledge ===

   ;; expected cost of searching for an object. Used by CP planner
   (search_cost  ?l - label ?rel - spatial_relation ?where - (either visualobject room)) - number
   ;; default probabilities. These come from Coma.
   (dora__in_room ?l - label ?c - category) - number
   (dora__in_obj ?l1 ?l2 - label ?c - category) - number
   (dora__on_obj ?l1 ?l2 - label ?c - category) - number

   ;; === inferred knowledge ===
   ;; The result of applying the default knowledge.
   ;; E.g. category(r1) = kitchen AND (dora__in_room cornflakes kitchen) => (obj_exists cornflakes in kitchen)
   ;; Also see the rules below
   (obj_exists ?l - label ?rel - spatial_relation  ?where - (either visualobject room)) - boolean

   ;; === room properties ===
   (category ?r - room) - category

   ;; === place properties ===
   (placestatus ?n - place) - place_status
   (in-room ?p - place) - room

   ;; === placeholder properties ===
   (leads_to_room ?p - place ?c - category) - boolean

   ;; === object properties ===
   (label ?o - visualobject) - label
   (related-to ?o - visualobject) - (either visualobject room)
   (relation ?o - visualobject ?what - (either visualobject room)) -  spatial_relation

   ;; === conegroup properties ===
   ;; basic properties that determine what the conegroup was generated for 
   ;; (e.g. cone group for cornflakes ON table_1)
   (label ?c - conegroup) - label
   (relation ?c - conegroup) - spatial_relation
   (related-to ?c - conegroup) - (either visualobject room)
   ;; probability of seeing an object of type (label ?c) when looking
   (p-visible ?c - conegroup) - number
   ;; the ground truth. Distribution should conform to the probability above.
   ;; Assumes that an object can be viewed from more than one conegroup
   (visible_from ?o - visualobject ?c - conegroup) - boolean
   ;; If an object can only be seen from one CG, the following would be possible:
   ;;(visible_from ?o - visualobject) - conegroup

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

  ;; create dummy rooms
  (:init-rule rooms
              :parameters(?c - category)
              :precondition (not (exists (?r - room)
                                         (and (= (category ?r) ?c)
                                              (is-virtual ?r))))
              :effect (create (?r - room) (and
                                           (is-virtual ?c)
                                           (assign (category ?r) ?c))
                              )
              )

  (:derived (attached_to_room ?p - place ?r - room)
            (exists (?p2 - place) (and (= (in-room ?p2) ?r)
                                       (connected ?p2 ?p))))

  (:derived (not_fully_explored ?r - room)
            (exists (?p - place) (and (= (placestatus ?p) placeholder)
                                      (attached_to_room ?p ?r))))
              


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
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (dora__in_room ?l ?c) (assign (obj_exists ?l in ?r) true)))

  ;; p(?label IN ?object | label(?object) = ?l2 AND ?object IN ?room AND category(?room) = ?cat)
  (:dtrule obj_in_obj
           :parameters (?l1 ?l2 - label ?o - visualobject ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (= (label ?o) ?l2)
                              (= (related-to ?o) ?r)
                              (= (relation ?o ?r) in))
           :effect (probabilistic (dora__in_obj ?l1 ?l2 ?c) (assign (obj_exists ?l1 on ?o) true)))

  ;; p(?label ON ?object | label(?object) = ?l2 AND ?object IN ?room AND category(?room) = ?cat)
  (:dtrule obj_on_obj
           :parameters (?l1 ?l2 - label ?o - visualobject ?r - room ?c - category)
           :precondition (and (= (category ?r) ?c)
                              (= (label ?o) ?l2)
                              (= (related-to ?o) ?r)
                              (= (relation ?o ?r) in))
           :effect (probabilistic (dora__on_obj ?l1 ?l2 ?c) (assign (obj_exists ?l1 on ?o) true)))


  ;; probability of an object being at a specific location
  ;;used only by DT (?)
  (:dtrule sample_object_location
           :parameters (?o - visualobject ?c - conegroup ?l - label ?rel - spatial_relation ?where - (either visualobject room))
           :precondition (and (= (relation ?c) ?rel)
                              (= (related-to ?c) ?where)
                              (= (label ?c) ?l)
                              (= (label ?o) ?l)
                              (is-virtual ?o)
                              (= (obj_exists ?l ?rel ?where) true))
           :effect (probabilistic 1.0 (and (assign (related-to ?o) ?where)
                                           (assign (relation ?o ?where) ?rel)))
           )

  ;; probability of finding a specific object in a conegroup
  ;;used only by DT (?)
  (:dtrule sample_cone_visibility
           :parameters (?o - visualobject ?c - conegroup ?l - label ?rel - spatial_relation ?where - (either visualobject room))
           :precondition (and (= (relation ?c) ?rel)
                              (= (related-to ?c) ?where)
                              (= (relation ?o ?where) ?rel)
                              (= (related-to ?o) ?where)
                              (= (label ?c) ?l)
                              (= (label ?o) ?l))
           :effect (probabilistic (p-visible ?c) (assign (visible_from ?o ?c) true))
           )


  ;; Assign virtual room to a placeholder
  (:dtrule room_from_placeholder
           :parameters (?p - place ?r - room ?c - category)
           :precondition (and (= (placestatus ?p) placeholder)
                              (= (category ?r) ?c)
                              (= (leads_to_room ?p ?c) true)
                              (is-virtual ?r))
           :effect (probabilistic 1.0 (assign (in-room ?p) ?r)))

  (:durative-action explore_place
           :agent (?a - robot)
           :parameters (?loc - place)
           :duration (= ?duration 0.1)
           :condition (and (over all (= (is-in ?a) ?loc)) 
                           (at start (= (placestatus ?loc) placeholder)))
           :effect (change (placestatus ?loc) trueplace)
           )

  (:durative-action report_position
           :agent (?a - robot)
           :parameters (?o - visualobject)
           :variables (?p - place ?h - human)
           :duration (= ?duration 1.0)
           :condition (over all (and (kval ?a (related-to ?o))
                                     (= (is-in ?h) ?p)
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
                     :effect (and (change (is-in ?a) ?to))
                     )

   ;; create cones for search in a room
   ;; precondition: robot is in the specified room
   (:durative-action create_cones_in_room
                     :agent (?a - robot)
                     :parameters (?l - label ?r - room)
                     :variables (?p - place)
                     :duration (= ?duration 10)
                     :condition (and (over all (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (not (not_fully_explored ?r))
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l in ?r)))
                     )
   
   ;; create cones for search in or on another object
   ;; precondition: robot is in the same room as the specified object
   (:durative-action create_cones_at_object
                     :agent (?a - robot)
                     :parameters (?l - label ?rel - spatial_relation ?o - visualobject)
                     :variables (?r - room ?p - place)
                     :duration (= ?duration 10)
                     :condition (and (over all (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (= (related-to ?o) ?r)
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l ?rel ?o)))
                     )

   ;; Abstract search action for the CP planner
   ;; Searches for an object in the room
   ;; precondition: robot is in the specified room
   (:durative-action seach_for_object_in_room
                     :agent (?a - robot)
                     :parameters (?l - label ?r - room)
                     :variables (?p - place ?o - visualobject)
                     :duration (= ?duration (search_cost ?l in ?r))
                     :condition (and (at start (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (= (label ?o) ?l)
                                                    (cones_created ?l in ?r)
                                                    (hyp (obj_exists ?l in ?r) true)
                                                    (not (done))))
                                     )
                     :effect (kval ?a (related-to ?o))
                     )
                     

   ;; Abstract search action for the CP planner
   ;; Searches for an object IN another object
   ;; precondition: robot is in the same room as the specified object
   (:durative-action seach_for_object_in_object
                     :agent (?a - robot)
                     :parameters (?l - label ?o - visualobject)
                     :variables (?p - place ?r - room ?o2 - visualobject)
                     :duration (= ?duration (search_cost ?l in ?o))
                     :condition (and (at start (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (= (related-to ?o) ?r)
                                                    (= (label ?o2) ?l)
                                                    (cones_created ?l in ?o)
                                                    (hyp (obj_exists ?l in ?o) true)
                                                    (not (done))))
                                     )
                     :effect (kval ?a (related-to ?o2))
                     )

   ;; Abstract search action for the CP planner
   ;; Searches for an object ON another object
   ;; precondition: robot is in the same room as the specified object
   (:durative-action seach_for_object_on_object
                     :agent (?a - robot)
                     :parameters (?l - label ?o - visualobject)
                     :variables (?p - place ?r - room ?o2 - visualobject)
                     :duration (= ?duration (search_cost ?l on ?o))
                     :condition (and (at start (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (= (related-to ?o) ?r)
                                                    (= (label ?o2) ?l)
                                                    (cones_created ?l on ?o)
                                                    (hyp (obj_exists ?l on ?o) true)
                                                    (not (done))))
                                     )
                     :effect (kval ?a (related-to ?o2))
                     )
                     

   ;; process one conegroup
   ;; TODO: how to model precondtitions for the robot's location
   ;;       and movement between conegroups?
   ;;       Currently, precondition is to be in the same room as the conegroup
   (:durative-action process_conegroup
                     :agent (?a - robot)
                     :parameters (?c - conegroup)
                     :variables (?p - place)
                     :duration (= ?duration 4)
                     :condition (over all (and (not (done))
                                               (= (is-in ?a) ?p)))
                     :effect (and )
                     )


   ;;TODO: The observation (observed (is-in ?o) ?p) is what the planner gets at the moment
   ;;      Doesn't really fit the new model and may need to be changed
   (:observe visual_object
             :agent (?a - robot)
             :parameters (?c - conegroup ?o - visualobject ?l - label ?where - (either visualobject room) ?p - place)
             :execution (process_conegroup ?a ?c ?p)
             :precondition (and (= (label ?o) ?l)
                                (= (label ?c) ?l)
                                (= (related-to ?c) ?where))
                                
             :effect (and (when (= (visible_from ?o ?c) true)
                            (probabilistic 0.8 (observed (related-to ?o) ?where)))
                          (when (not (= (visible_from ?o ?c) true))
                            (probabilistic 0.1 (observed (related-to ?o) ?where)))
                          )
             )
             
)
