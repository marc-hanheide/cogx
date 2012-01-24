(define (domain find-objects)
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
   (started)
   (select-locked)
   )

  (:functions
   (is-in ?o - movable) - place
   (really-is-in ?o - movable) - place
   (room ?p - place) - room
   (category ?r - room) - category
   (label ?o - visualobject) - label
   (cone-label ?c - cone) - label
   (ex-in-room ?l ?r) - boolean
   (p-is-in ?c - cone) - number
   (p-ex-in-room ?l - label ?c - category ) - number
   (p-category ?r - room ?c - category ) - number
   (total-p-costs) - number
   )

  (:init-rule objects
              :effect (forall (?l - label) (create (?o - visualobject)
                                                   (assign (label ?o) ?l)))
              )

  (:dtrule sample_existence
           :parameters (?l - label ?r - room ?c - category)
           :precondition (= (category ?r) ?c)
           :effect (probabilistic (p-ex-in-room ?l ?c) (assign (ex-in-room ?l ?r) true) )
                                                       ;; (assign (ex-in-room ?l ?r) false))
           )

  (:dtrule sample_is_in
           :parameters (?l - label ?r - room ?c - cone ?o - visualobject)
           :precondition (and (= (room ?c) ?r)
                              (= (label ?o) ?l)
                              (= (ex-in-room ?l ?r) true))
           :effect (probabilistic (p-is-in ?c) (assign (is-in ?o) ?c))
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
                     
   (:durative-action look_at_object
                     :agent (?a - robot)
                     :parameters (?o - visualobject ?c - cone)
                     :duration (= ?duration 1)
                     :condition (and );(over all (= (is-in ?a) ?c))
                                     ;(at start (hyp (is-in ?o) ?c)))
                     :effect (and ;;(at end (assign (really-is-in ?o) ?c))
                                  ;;(at end (kval ?a (is-in ?o)))
                                  (at start (started)))
                     )

   (:observe visual_object
             :agent (?a - robot)
             :parameters (?o - visualobject ?c - cone)
             :execution (look_at_object ?a ?o ?c)
             :effect (when (= (is-in ?o) ?c)
                       (probabilistic 0.8 (observed (is-in ?o) ?c)))
             )
             
)
