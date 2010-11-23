(define (domain dora-test-100)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   cone place room - object
   virtual-place - place
   visualobject - movable
   robot - agent
   robot human - movable
   place_status label category detection_difficulty - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)

   ;;virtual predicates
   (done)
   )

  (:functions
   (is-in ?o - movable) - place
   (in-room ?p - place) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
   (placestatus ?n - place) - place_status

   (ex-in-room ?l - label ?r - room) - boolean
   ;; (percept-prob ?l - label) - number
   (difficulty ?l - label) - detection_difficulty
   )

  (:constants
   placeholder trueplace - place_status
   easy medium hard - detection_difficulty
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
                    :effect (and (change (placestatus ?loc) trueplace))
                    )

  (:durative-action report_position
                    :agent (?a - robot)
                    :parameters (?o - visualobject)
                    :variables (?p - place ?h - human)
                    :duration (= ?duration 1.0)
                    :condition (over all (and (kval ?a (is-in ?o))
                                              (= (is-in ?h) ?p)
                                              (= (is-in ?a) ?p)))
                    :effect (and (at end (position-reported ?o)))
                    )

  (:durative-action move
                    :agent (?a - robot)
                    :parameters (?to - place)
                    :variables (?from - place)
                    :duration (= ?duration 3)
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
                    :duration (= ?duration 6)
                    :condition (over all (and (not (done))
                                              (= (is-in ?a) ?p)
                                              (= (label ?o) ?l)))
                    :effect (and )
                    )

  (:observe visual_object_easy
            :agent (?a - robot)
            :parameters (?o - visualobject ?l - label ?p - place)
            :execution (process_all_cones_at_place ?a ?p ?l ?o)
            :precondition (= (difficulty ?l) easy)
            :effect (and (when (and (= (is-in ?o) ?p))
                           (probabilistic 0.9 (observed (is-in ?o) ?p))))
            )

  (:observe visual_object_medium
            :agent (?a - robot)
            :parameters (?o - visualobject ?l - label ?p - place)
            :execution (process_all_cones_at_place ?a ?p ?l ?o)
            :precondition (= (difficulty ?l) medium)
            :effect (and (when (and (= (is-in ?o) ?p))
                           (probabilistic 0.7 (observed (is-in ?o) ?p))))
            )

  (:observe visual_object_hard
            :agent (?a - robot)
            :parameters (?o - visualobject ?l - label ?p - place)
            :execution (process_all_cones_at_place ?a ?p ?l ?o)
            :precondition (= (difficulty ?l) hard)
            :effect (and (when (and (= (is-in ?o) ?p))
                           (probabilistic 0.5 (observed (is-in ?o) ?p))))
            )
             
  )
