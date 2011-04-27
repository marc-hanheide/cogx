(define (domain find-objects)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   place room - object
   cone - place
   dummy-cone - place
   visualobject - movable
   robot - agent
   robot - movable
   place_status label category - object
   )

  (:predicates
   (cones_created ?l - label ?r - room)
   (connected ?p1 ?p2 - place)
   (started)
;   (select-locked)
   (cones-exist ?l - label ?r - room)
   (done)
   )

  (:functions
   (is-in ?o - movable) - place
   (in-room ?p - place) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
   (cone-label ?c - cone) - label
   (ex-in-room ?l - label ?r - room) - boolean
   (p-is-in ?c - cone) - number
   (dora__in ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (total-p-costs) - number
   )

  (:constants 
   dummy-cone - label
  )

  ;; (:init-rule objects
  ;;             :effect (forall (?l - label) (create (?o - visualobject)
  ;;                                                  (assign (label ?o) ?l)))
  ;;             )
  (:init-rule objects
              :parameters(?l - label)
              :effect (create (?o - visualobject)
                              (assign (label ?o) ?l))
              )

  (:init-rule cones
              :parameters(?r - room)
              :effect (and
                       ;; (create (?c - cone) (and
                       ;;                      (assign (in-room ?c) ?r)
                       ;;                      (assign (cone-label ?c) dummy-cone)))
                       (create (?c - cone) (and
                                            (assign (in-room ?c) ?r)
                                            (assign (cone-label ?c) dummy-cone)))
                       )
              )

  (:derived (cones-exist ?l - label ?r - room)
            (exists (?c - cone) (and (= (in-room ?c) ?r)
                                     (= (cone-label ?c) ?l)))
            )

  (:action sample_existence
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (dora__in ?l ?c) (assign (ex-in-room ?l ?r) true))
                               ;;(assign (ex-in-room ?l ?r) false))
           )

  (:action sample_is_in
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
           :precondition (and (= (in-room ?c) ?r)
                              (= (label ?o) ?l)
                              (= (cone-label ?c) ?l)
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (p-is-in ?c) (assign (is-in ?o) ?c))
           )

  (:action sample_is_in_dummy
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
           :precondition (and (= (in-room ?c) ?r)
                              (= (label ?o) ?l)
                              (= (cone-label ?c) dummy-cone)
                              (not (cones-exist ?l ?r))
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic 0.9 (assign (is-in ?o) ?c))
           )


   ;; (:durative-action select_category
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?r - room ?c - category)
   ;;                   :duration (= ?duration (* (total-p-costs) (- 1 (p-category ?r ?c))))
   ;;                   :condition (over all (not (started)))
   ;;                   ;;:condition (at start (not (kval ?a (category ?r))))
   ;;                   :effect (and (at end (commit (category ?r) ?c))
   ;;                                (at end (decrease (total-p-costs) ?duration)))
   ;;                   )

   ;; (:durative-action select_ex_in_room
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?l - label ?r - room ?c - category)
   ;;                   :duration (= ?duration (* (total-p-costs) (- 1 (p-ex-in-room ?l ?c))))
   ;;                   :condition (and (over all (not (started)))
   ;;                                   (at start (hyp (category ?r) ?c)))
   ;;                   :effect (and (at end (commit (ex-in-room ?l ?r) true))
   ;;                                (at end (decrease (total-p-costs) ?duration)))
   ;;                   )

   ;; (:durative-action select_is_in
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
   ;;                   :duration (= ?duration (* (total-p-costs) (- 1 (p-is-in ?c))))
   ;;                   :condition (and (over all (not (started)))
   ;;                                   (at start (and (hyp (ex-in-room ?l ?r) true)
   ;;                                                  (= (label ?o) ?l)
   ;;                                                  (= (room ?c) ?r))))
   ;;                   :effect (and (at end (commit (is-in ?o) ?c))
   ;;                                (at end (decrease (total-p-costs) ?duration)))
                                 
   ;;                   )

   (:durative-action move
                     :agent (?a - robot)
                     :parameters (?from - place ?to - place)
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
                                                    (hyp (ex-in-room ?l ?r) true)
                                                    (not (done))))
                                     )
                     :effect (and (at end (cones_created ?l ?r))
                                  (at start (started)))
                     )

   ;; (:observe new_cone
   ;;           :agent (?a - robot)
   ;;           :parameters (?o - visualobject ?l - label ?r - room ?p - place)
   ;;           :execution (create_cones ?a ?l ?r ?p)
   ;;           :effect (when (= (ex-in-room ?l ?r) true)
   ;;                     (probabilistic 0.9 (observed (is-in ?o) dummy-cone)))
   ;;           )

   (:durative-action look_at_object_in_dummy_cone
                     :agent (?a - robot)
                     :parameters (?o - visualobject ?l - label ?r - room ?c - cone)
                     :duration (= ?duration 1)
                     :condition (over all (and (not (done))
                                               (cones_created ?l ?r)
                                               (= (label ?o) ?l)
                                               (= (in-room ?c) ?r)
                                               (= (cone-label ?c) dummy-cone)));(over all (= (is-in ?a) ?c))
                                     ;(at start (hyp (is-in ?o) ?c)))
                     :effect (and ;;(at end (assign (really-is-in ?o) ?c))
                                  ;;(at end (kval ?a (is-in ?o)))
                                  (at start (started)))
                     )

   (:observe visual_object_in_dummy
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?r - room ?c - cone)
             :execution (look_at_object_in_dummy_cone ?a ?o ?l ?r ?c )
             :effect (when (= (is-in ?o) ?c)
                       (probabilistic 0.8 (observed (is-in ?o) ?c)))
             )
                     
   (:durative-action look_at_object
                     :agent (?a - robot)
                     :parameters (?o - visualobject ?l - label ?c - cone)
                     :duration (= ?duration 1)
                     :condition (over all (and (not (done))
                                               (= (cone-label ?c) ?l)
                                               (= (label ?o) ?l)))
                                        ;(over all (= (is-in ?a) ?c))
                                     ;(at start (hyp (is-in ?o) ?c)))
                     :effect (and ;;(at end (assign (really-is-in ?o) ?c))
                                  ;;(at end (kval ?a (is-in ?o)))
                                  (at start (started)))
                     )

   (:observe visual_object
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?c - cone)
             :execution (look_at_object ?a ?o ?l ?c)
             :effect (when (= (is-in ?o) ?c)
                       (probabilistic 0.8 (observed (is-in ?o) ?c)))
             )
             
)
