;; Net benefit version of the IPC-2002 Zenotravel domain.

;; Refuel action taken out so that it is not always possible to
;; satisfy all goals. See "sequential" version for some more comments.

(define (domain zeno-travel)

(:requirements :action-costs :fluents :goal-utilities :typing)

(:types locatable city - object
        aircraft person - locatable)

(:functions (at ?l - locatable) - city
            (in ?p - person) - aircraft
            (fuel-level ?a - aircraft) - number
            (total-cost) - number
            (fly-cost ?c1 - city ?c2 - city) - number
            (zoom-cost ?c1 - city ?c2 - city) - number)

(:action board
         :parameters (?p - person ?a - aircraft)
         :precondition (= (at ?p) (at ?a))
         :effect (and (assign (at ?p) undefined)
                      (assign (in ?p) ?a)))

(:action debark
         :parameters (?p - person ?a - aircraft)
         :precondition (= (in ?p) ?a)
         :effect (and (assign (in ?p) undefined)
                      (assign (at ?p) (at ?a))))

(:action fly 
         :parameters (?a - aircraft ?c - city)
         :precondition (>= (fuel-level ?a) (fly-fuel (at ?a) ?c))
         :effect (and (assign (at ?a) ?c)
                      (decrease (fuel-level ?a) (fly-fuel (at ?a) ?c))
                      (increase (total-cost) (fly-cost (at ?a) ?c2))))

(:action zoom 
         :parameters (?a - aircraft ?c - city)
         :precondition (>= (fuel-level ?a) (zoom-fuel (at ?a) ?c))
         :effect (and (assign (at ?a) ?c)
                      (decrease (fuel-level ?a) (zoom-fuel (at ?a) ?c))
                      (increase (total-cost) (zoom-cost (at ?a) ?c2))))

)
