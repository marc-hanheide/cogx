(define (domain dora-test-100)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   place room - object
   cone - place
   visualobject - movable
   robot - agent
   robot - movable
   place_status label category - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (cones_created ?l ?r)
   (fully_explored ?r)
   (started)
   (select-locked)
   (done)
   )

  (:functions
   (is-in ?o - movable) - place
   (is-in ?c - cone) - place
   (really-is-in ?o - movable) - place
   (in-room ?p - place) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
   (label ?c - cone) - label
;   (cone-label ?c - cone) - label
   (ex-in-room ?l - label ?r - room) - boolean
   (probability ?c - cone) - number
   (p-is-in ?p - place) - number
   (dora__in ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (total-p-costs) - number
   )

  (:constants
   dummy-cone - label
   kitchen office living_room - category
   ;;cornflakes table mug - label ;;oven fridge book board-game - label
   )

  (:init-rule init
              :effect (and (assign (total-p-costs) 200))
              )

  (:init-rule objects
              :parameters(?l - label)
              :effect (create (?o - visualobject)
                              (assign (label ?o) ?l))
              )
  (:init-rule categories
              :parameters(?r - room)
              :effect (assign-probabilistic (category ?r) 
                                            0.3 kitchen
                                            0.3 office
                                            0.3 living_room)
              )

  (:init-rule places
              :effect (forall (?p - place) (assign (p-is-in ?p) 0.3))
              )

  (:init-rule cones
              :parameters(?r - room)
              :effect (and (create (?c - cone) (and
                                                (assign (in-room ?c) ?r)
                                                (assign (label ?c) dummy-cone)))
                           )
              )

  (:derived (cones-exist ?l - label ?r - room)
            (exists (?c - cone) (and (= (in-room ?c) ?r)
                                     (= (cone-label ?c) ?l)))
            )

  ;; (:derived (fully_explored ?r - room)
  ;;           (forall (?p - place) (or (not (= (in-room ?p) ?r))
  ;;                                    (

  (:action sample_existence
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (dora__in ?l ?c) (assign (ex-in-room ?l ?r) true))
                                                   ;;(assign (ex-in-room ?l ?r) false))
           )

  ;; (:action sample_is_in_place
  ;;          :agent (?a - agent)
  ;;          :parameters (?l - label ?r - room ?p - place ?o - visualobject)
  ;;          :precondition (and (= (in-room ?p) ?r)
  ;;                             (= (label ?o) ?l)
  ;;                             (= (ex-in-room ?l ?r) true))
  ;;          :effect (probabilistic (p-is-in ?p) (assign (is-in ?o) ?p))
  ;;          )

  (:action sample_is_in_cone
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
           :precondition (and (= (in-room ?c) ?r)
                              (= (label ?c) ?l)
                              (= (label ?o) ?l)
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (probability ?p) (assign (is-in ?o) ?c))
           )

  (:action sample_is_in_dummy
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
           :precondition (and (= (in-room ?c) ?r)
                              (= (label ?o) ?l)
                              (= (label ?c) dummy-cone)
                              (not (cones-exist ?l ?r))
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic 0.9 (assign (is-in ?o) ?c))
           )

   ;; (:durative-action spin
   ;;                   :agent (?a - robot)
   ;;                   :duration (= ?duration 0)
   ;;                   :condition (over all (done))
   ;;                   :effect (and)
   ;;                   )

   (:durative-action move
                     :agent (?a - robot)
                     :parameters (?to - place)
                     :variables (?from - place)
                     :duration (= ?duration 5)
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
                                                    ;;(hyp (ex-in-room ?l ?r) true)
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l ?r))
                                  (at start (started)))
                     )


   (:durative-action process_cone
                     :agent (?a - robot)
                     :parameters (?c - cone)
                     :variables (?o - visualobject ?l - label ?p - place)
                     :duration (= ?duration 4)
                     :condition (over all (and (not (done))
                                               (= (is-in ?a) ?p)
                                               (= (is-in ?c) ?p)
                                               (= (label ?c) ?l)
                                               (= (label ?o) ?l)))
                     :effect (and (at start (started)))
                     )

   (:observe visual_object_in_cone
             :agent (?a - robot)
             :parameters (?c - cone ?o - visualobject ?l - label ?p - place)
             :execution (process_cone ?a ?c ?o ?l ?p)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )


   (:durative-action process_dummy_cone
                     :agent (?a - robot)
                     :parameters (?c - cone)
                     :variables (?o - visualobject ?l - label ?r - room)
                     :duration (= ?duration 10)
                     :condition (over all (and (not (done))
                                               (cones_created ?l ?r)
                                               (= (in-room ?c) ?r)
                                               (= (label ?c) dummy-cone)
                                               (= (label ?o) ?l)))
                     :effect (and (at start (started)))
                     )

   (:observe visual_object_in_cone
             :agent (?a - robot)
             :parameters (?c - cone ?o - visualobject ?l - label ?r - room)
             :execution (process_dummy_cone ?a ?c ?o ?l ?r)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )
                     
;;    (:durative-action look-for-object
;;                      :agent (?a - robot)
;;                      :parameters (?l - label)
;;                      :variables (?o - visualobject ?p - place)
;;                      :duration (= ?duration 2)
;;                      :condition (and (over all (and (not (done))
;;                                                     (= (is-in ?a) ?p)
;;                                                     (= (label ?o) ?l))))
;;                                      ;(at start (hyp (is-in ?o) ?c)))
;;                      :effect (and ;;(at end (assign (really-is-in ?o) ?c))
;;                                   ;;(at end (kval ?a (is-in ?o)))
;;                                   (at start (started)))
;;                      )

;;    (:observe visual_object
;;              :agent (?a - robot)
;;              :parameters (?o - visualobject ?l - label ?p - place)
;;              :execution (look-for-object ?a ?l ?o ?p)
;; ;             :precondition (= (label ?o) ?l)
;;              :effect (when (= (is-in ?o) ?p)
;;                        (probabilistic 0.8 (observed (is-in ?o) ?p)))
;;              )
             
)
