;; Net benefit version of the IPC-2002 Zenotravel domain.

;; Refuel action taken out so that it is not always possible to
;; satisfy all goals. See "sequential" version for some more comments.

(define (domain zeno-travel)

(:requirements :action-costs :goal-utilities :numeric-fluents :typing)

(:types locatable city - object
        aircraft person - locatable)

(:predicates (at ?l - locatable ?c - city)
             (in ?p - person ?a - aircraft))

(:functions (fuel-level ?a - aircraft) - number
            (total-cost) - number
            (fly-cost ?c1 - city ?c2 - city) - number
            (zoom-cost ?c1 - city ?c2 - city) - number)

(:action board
         :parameters (?p - person ?a - aircraft ?c - city)
         :precondition (and (at ?p ?c)
                            (at ?a ?c))
         :effect (and (not (at ?p ?c))
                      (in ?p ?a)))

(:action debark
         :parameters (?p - person ?a - aircraft ?c - city)
         :precondition (and (in ?p ?a)
                            (at ?a ?c))
         :effect (and (not (in ?p ?a))
                      (at ?p ?c)))

(:action fly 
         :parameters (?a - aircraft ?c1 ?c2 - city)
         :precondition (and (at ?a ?c1)
                            (>= (fuel-level ?a) (fly-fuel ?c1 ?c2)))
         :effect (and (not (at ?a ?c1))
                      (at ?a ?c2)
                      (decrease (fuel-level ?a) (fly-fuel ?c1 ?c2))
                      (increase (total-cost) (fly-cost ?c1 ?c2))))

(:action zoom
         :parameters (?a - aircraft ?c1 ?c2 - city)
         :precondition (and (at ?a ?c1)
                            (>= (fuel-level ?a) (zoom-fuel ?c1 ?c2)))
         :effect (and (not (at ?a ?c1))
                      (at ?a ?c2)
                      (decrease (fuel-level ?a) (zoom-fuel ?c1 ?c2))
                      (increase (total-cost) (zoom-cost ?c1 ?c2))))

)
