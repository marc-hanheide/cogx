(define (domain dora-test-100)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   cone place room - object
   virtual-place - place
   visualobject - movable
   robot - agent
   robot human - movable
   place_status label category - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)

   (is-virtual ?o - object)
   ;; derived predicates
   (cones-exist ?l - label ?r - room)
   (obj-possibly-in-room ?o - visualobject ?r - room)
   (not_fully_explored ?r)

   ;;virtual predicates
   ;;(cones_created ?l - label ?r - room)
   (started)
   (done)
   )

  (:functions
   (is-in ?o - movable) - place
   (is-in ?c - cone) - place
   (in-room ?p - place) - room
   (in-room ?c - cone) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
   (label ?c - cone) - label
   (placestatus ?n - place) - place_status

   (ex-in-room ?l - label ?r - room) - boolean
   ;;(probability ?c - cone) - number
   (place-prob ?p - place ?l - label) - number
   (p-is-in ?p - place) - number
   (dora__in ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (log-place-prob ?p - place ?l - label) - number
   (log-dora__in ?l - label ?c - category ) - number
   (log-p-category ?r - room ?c - category ) - number
   (total-p-cost) - number
   )

  (:constants
   placeholder trueplace - place_status
   )

  (:derived (obj-possibly-in-room ?o - visualobject ?r - room)
            (exists (?p - place) (and (= (in-room ?p) ?r)
                                      (in-domain (is-in ?o) ?p))))

  (:derived (cones-exist ?l - label ?r - room)
            (exists (?c - cone ?p - place) (and (= (in-room ?p) ?r)
                                                (= (is-in ?c) ?p)
                                                (= (label ?c) ?l)))
            )

  (:derived (not_fully_explored ?r - room)
            (exists (?p ?p2 - place) (and (= (in-room ?p) ?r)
                                          (not (= (placestatus ?p2) trueplace))
                                          (connected ?p ?p2))))

  (:action _sample_existence
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (dora__in ?l ?c) (assign (ex-in-room ?l ?r) true))
                                                   ;;(assign (ex-in-room ?l ?r) false))
           )

  (:action _sample_is_in
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?p - place ?o - visualobject)
           :precondition (and (= (in-room ?p) ?r)
                              (= (label ?o) ?l)
                              (in-domain (is-in ?o) ?p) 
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (place-prob ?p ?l) (assign (is-in ?o) ?p))
           )

   ;; (:durative-action spin
   ;;                   :agent (?a - robot)
   ;;                   :duration (= ?duration 0)
   ;;                   :condition (over all (done))
   ;;                   :effect (and)
   ;;                   )

  (:durative-action explore_place
           :agent (?a - robot)
           :parameters (?loc - place)
           :duration (= ?duration 0.1)
           :condition (and (over all (= (is-in ?a) ?loc)) (at start (= (placestatus ?loc) placeholder)))
           :effect (change (placestatus ?loc) trueplace)
           )

  (:durative-action report_position
           :agent (?a - robot)
           :parameters (?o - visualobject)
           :variables (?p - place ?h - human)
           :duration (= ?duration 1.0)
           :condition (over all (and (kval ?a (is-in ?o))
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
                     
   (:durative-action process_all_cones_at_place
                     :agent (?a - robot)
                     :parameters (?p - place ?l - label )
                     :variables (?o - visualobject)
                     :duration (= ?duration 10)
                     :condition (over all (and (not (done))
                                               (= (is-in ?a) ?p)
                                               (= (label ?o) ?l)))
                     :effect (and )
                     )

   (:observe visual_object
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?p - place)
             :execution (process_all_cones_at_place ?a ?p ?l ?o)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )
             
)
