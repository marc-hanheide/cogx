(define (domain dora-test-100)
  (:requirements :mapl :adl :fluents :durative-actions :partial-observability :dynamic-objects)

  (:types
   place room - object
;   cone - place
   visualobject - movable
   robot - agent
   robot - movable
   place_status label category - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
;   (cones_created ?l ?r)
   (started)
   (select-locked)
   (done)
   )

  (:functions
   (is-in ?o - movable) - place
   (really-is-in ?o - movable) - place
   (in-room ?p - place) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
;   (cone-label ?c - cone) - label
   (ex-in-room ?l - label ?r - room) - boolean
   (p-is-in ?p - place) - number
   (dora__in ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (total-p-costs) - number
   )

  (:constants
   kitchen office living_room - category
   cornflakes table mug - label ;;oven fridge book board-game - label
   )

  (:init-rule init
              :effect (and (assign (total-p-costs) 200)
                           ;; (assign (p-ex-in-room cornflakes kitchen) 0.8)
                           ;; (assign (p-ex-in-room table kitchen) 0.9)
                           ;; (assign (p-ex-in-room mug kitchen) 0.8)
                           ;; ;; (assign (p-ex-in-room oven kitchen) 0.8)
                           ;; ;; (assign (p-ex-in-room fridge kitchen) 0.7)
                           ;; ;; (assign (p-ex-in-room book kitchen) 0.2)
                           ;; ;; (assign (p-ex-in-room board-game kitchen) 0.1)

                           ;; (assign (p-ex-in-room cornflakes office) 0.1)
                           ;; (assign (p-ex-in-room table office) 0.9)
                           ;; (assign (p-ex-in-room mug office) 0.9)
                           ;; ;; (assign (p-ex-in-room oven office) 0.0)
                           ;; ;; (assign (p-ex-in-room fridge office) 0.1)
                           ;; ;; (assign (p-ex-in-room book office) 0.8)
                           ;; ;; (assign (p-ex-in-room board-game office) 0.2)

                           ;; (assign (p-ex-in-room cornflakes living-room) 0.1)
                           ;; (assign (p-ex-in-room table living-room) 0.7)
                           ;; (assign (p-ex-in-room mug living-room) 0.6)
                           )
              )

  (:init-rule objects
              :parameters(?l - label)
              :effect (create (?o - visualobject)
                              (assign (label ?o) ?l))
              )

  ;; (:init-rule cones
  ;;             :effect (forall (?r - room) (and
  ;;                                          (create (?c - cone) (and
  ;;                                                               (assign (in-room ?c) ?r)
  ;;                                                               (assign (cone-label ?c) dummy-cone)))
  ;;                                          ))
  ;;             )

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
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (p-is-in ?p) (assign (is-in ?o) ?p))
           )

  ;; (:action sample_is_in_dummy
  ;;          :agent (?a - agent)
  ;;          :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
  ;;          :precondition (and (= (in-room ?c) ?r)
  ;;                             (= (label ?o) ?l)
  ;;                             (= (cone-label ?c) dummy-cone)
  ;;                             (not (cones-exist ?l ?r))
  ;;                             (= (ex-in-room ?l ?r) true))
  ;;          :effect (probabilistic 0.9 (assign (is-in ?o) ?c))
  ;;          )

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
                     
   (:durative-action look-for-object
                     :agent (?a - robot)
                     :parameters (?l - label)
                     :variables (?o - visualobject ?p - place)
                     :duration (= ?duration 2)
                     :condition (and (over all (and (not (done))
                                                    (= (is-in ?a) ?p)
                                                    (= (label ?o) ?l))))
                                     ;(at start (hyp (is-in ?o) ?c)))
                     :effect (and ;;(at end (assign (really-is-in ?o) ?c))
                                  ;;(at end (kval ?a (is-in ?o)))
                                  (at start (started)))
                     )

   (:observe visual_object
             :agent (?a - robot)
             :parameters (?o - visualobject ?l - label ?p - place)
             :execution (look-for-object ?a ?l ?o ?p)
;             :precondition (= (label ?o) ?l)
             :effect (when (= (is-in ?o) ?p)
                       (probabilistic 0.8 (observed (is-in ?o) ?p)))
             )
             
)
