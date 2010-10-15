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
   (cones_created ?l - label ?r - room)
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
   (probability ?c - cone) - number
   (p-probability ?p - place ?l - label) - number
   (virtual-probability ?p - virtual-place) - number
   (p-is-in ?p - place) - number
   (dora__in ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (log-p-probability ?p - place ?l - label) - number
   (log-virtual-probability ?p - virtual-place) - number
   (log-dora__in ?l - label ?c - category ) - number
   (log-p-category ?r - room ?c - category ) - number
   (total-p-cost) - number
   )

  (:constants
   placeholder trueplace - place_status
   kitchen office living_room - category
   ;;cornflakes table mug - label ;;oven fridge book board-game - label
   )

  (:init-rule init
              :effect (and (assign (total-p-cost) 10))
              )

  ;; (:init-rule bla
  ;;             :parameters(?c - colorname ?o - visualobject)
  ;;             :precondition (not (> (color-gain ?o ?c) 0))
  ;;             :effect (assign ((color-gain ?o ?c) 0)))

  (:init-rule objects
              :parameters(?l - label)
              :precondition (not (exists (?o - visualobject)
                                      (= (label ?o) ?l)))
              :effect (create (?o - visualobject) (and
                                                   (is-virtual ?o)
                                                   (assign (label ?o) ?l)
                                                   (assign (is-in ?o) UNKNOWN))
                              )
              )

  ;; (:init-rule categories
  ;;             :parameters(?r - room)
  ;;             :effect (assign (category ?r) UNKNOWN)
  ;;             )

  (:init-rule init-place-probs
              :parameters(?p - place ?l - label)
              :precondition (exists (?c - cone) (and (= (is-in ?c) ?p)
                                                     (= (label ?c) ?l)))
              :effect (and (assign (p-probability ?p ?l) 0))
              )

  (:init-rule places-probs
              :parameters(?p - place ?l - label ?c - cone)
              :precondition (and (= (is-in ?c) ?p)
                                 (= (label ?c) ?l))
              :effect (increase (p-probability ?p ?l) (probability ?c))
              )

  (:init-rule virtual-places
              :parameters(?r - room)
              :precondition (not (exists (?p - virtual-place)
                                         (= (in-room ?p) ?r)))
              :effect (and (create (?p - virtual-place) (and
                                                         (is-virtual ?p)
                                                         (assign (in-room ?p) ?r)
                                                         (assign (virtual-probability ?p) 0.3))
                           )
                           (create (?p - virtual-place) (and
                                                         (is-virtual ?p)
                                                         (assign (in-room ?p) ?r)
                                                         (assign (virtual-probability ?p) 0.3))
                           ))
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

  (:action sample_existence
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (dora__in ?l ?c) (assign (ex-in-room ?l ?r) true))
                                                   ;;(assign (ex-in-room ?l ?r) false))
           )

  (:action sample_is_in
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?p - place ?o - visualobject)
           :precondition (and (= (in-room ?p) ?r)
                              (= (label ?o) ?l)
                              (not (is-virtual ?p))
                              (in-domain (is-in ?o) ?p) 
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (p-probability ?p ?l) (assign (is-in ?o) ?p))
           )

  ;; (:action sample_is_in
  ;;          :agent (?a - agent)
  ;;          :parameters (?l - label ?r - room ?p - place ?c - cone ?o - visualobject)
  ;;          :precondition (and (= (in-room ?p) ?r)
  ;;                             (= (is-in ?c) ?p)
  ;;                             (= (label ?c) ?l)
  ;;                             (= (label ?o) ?l)
  ;;                             (in-domain (is-in ?o) ?p) 
  ;;                             (= (ex-in-room ?l ?r) true))
  ;;          :effect (probabilistic (probability ?c) (assign (is-in ?o) ?p))
  ;;          )

  (:action sample_is_in_virtual
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?p - virtual-place ?o - visualobject)
           :precondition (and (= (in-room ?p) ?r)
                              (= (label ?o) ?l)
                              (obj-possibly-in-room ?o ?r)
                              (not (cones-exist ?l ?r))
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (virtual-probability ?p) (assign (is-in ?o) ?p))
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
                     :effect (and (change (is-in ?a) ?to)
                                  (at start (started)))
                     )

   (:durative-action create_cones
                     :agent (?a - robot)
                     :parameters (?l - label ?r - room)
                     :variables (?p - place)
                     :duration (= ?duration 10)
                     :condition (and (over all (and (= (is-in ?a) ?p)
                                                    (= (in-room ?p) ?r)
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l ?r))
                                  (at start (started)))
                     )


   ;; (:durative-action process_cone
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?c - cone)
   ;;                   :variables (?o - visualobject ?l - label ?p - place)
   ;;                   :duration (= ?duration 4)
   ;;                   :condition (over all (and (not (done))
   ;;                                             (= (is-in ?a) ?p)
   ;;                                             (= (is-in ?c) ?p)
   ;;                                             (= (label ?c) ?l)
   ;;                                             (= (label ?o) ?l)))
   ;;                   :effect (and (at start (started)))
   ;;                   )


   (:durative-action process_virtual_place
                     :agent (?a - robot)
                     :parameters (?o - visualobject ?l - label ?r - room ?p - virtual-place)
                     :duration (= ?duration 10)
                     :condition (over all (and (not (done))
                                               (cones_created ?l ?r)
                                               (= (in-room ?p) ?r)
                                               (= (label ?o) ?l)))
                     :effect (and (at start (started)))
                     )

   (:observe visual_object_virtual
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?r - room ?p - virtual-place)
             :execution (process_virtual_place ?a ?o ?l ?r ?p)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )
                     
   (:durative-action process_all_cones_at_place
                     :agent (?a - robot)
                     :parameters (?p - place ?l - label )
                     :variables (?o - visualobject)
                     :duration (= ?duration 10)
                     :condition (over all (and (not (done))
                                               (= (is-in ?a) ?p)
                                               (= (label ?o) ?l)))
                     :effect (and (at start (started)))
                     )

   (:observe visual_object
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?p - place)
             :execution (process_all_cones_at_place ?a ?p ?l ?o)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )
             
)
