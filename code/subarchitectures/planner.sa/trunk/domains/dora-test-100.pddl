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
   (p-ex-in-room ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (total-p-costs) - number
   )

  (:constants
   kitchen office living-room - category
   cornflakes table mug - label ;;oven fridge book board-game - label
   )

  (:init-rule init
              :effect (and (assign (total-p-costs) 200)
                           (assign (p-ex-in-room cornflakes kitchen) 0.8)
                           (assign (p-ex-in-room table kitchen) 0.9)
                           (assign (p-ex-in-room mug kitchen) 0.8)
                           ;; (assign (p-ex-in-room oven kitchen) 0.8)
                           ;; (assign (p-ex-in-room fridge kitchen) 0.7)
                           ;; (assign (p-ex-in-room book kitchen) 0.2)
                           ;; (assign (p-ex-in-room board-game kitchen) 0.1)

                           (assign (p-ex-in-room cornflakes office) 0.1)
                           (assign (p-ex-in-room table office) 0.9)
                           (assign (p-ex-in-room mug office) 0.9)
                           ;; (assign (p-ex-in-room oven office) 0.0)
                           ;; (assign (p-ex-in-room fridge office) 0.1)
                           ;; (assign (p-ex-in-room book office) 0.8)
                           ;; (assign (p-ex-in-room board-game office) 0.2)

                           (assign (p-ex-in-room cornflakes living-room) 0.1)
                           (assign (p-ex-in-room table living-room) 0.7)
                           (assign (p-ex-in-room mug living-room) 0.6)
                           )
              )

  (:init-rule objects
              :effect (forall (?l - label) (create (?o - visualobject)
                                                   (assign (label ?o) ?l)))
              )

  (:init-rule categories
              :effect (forall (?r - room) (assign-probabilistic (category ?r) 
                                                                0.3 kitchen
                                                                0.3 office
                                                                0.3 living-room))
              )

  (:init-rule places
              :effect (forall (?p - place) (assign (p-is-in ?p) 0.3))
              )

  (:action sample_existence
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (p-ex-in-room ?l ?c) (assign (ex-in-room ?l ?r) true) 
                                                       (assign (ex-in-room ?l ?r) false))
           )

  (:action sample_is_in
           :agent (?a - agent)
           :parameters (?l - label ?r - room ?p - place ?o - visualobject)
           :precondition (and (= (in-room ?p) ?r)
                              (= (label ?o) ?l)
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (p-is-in ?p) (assign (is-in ?o) ?p))
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
                                     (at start (= (is-in ?a) ?from)))
                     :effect (and (change (is-in ?a) ?to)
                                  (at start (started)))
                     )
                     
   (:durative-action look-for-object
                     :agent (?a - robot)
                     :parameters (?l - label)
                     :variables (?o - visualobject ?p - place)
                     :duration (= ?duration 1)
                     :condition (and (over all (and (= (is-in ?a) ?p)
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
