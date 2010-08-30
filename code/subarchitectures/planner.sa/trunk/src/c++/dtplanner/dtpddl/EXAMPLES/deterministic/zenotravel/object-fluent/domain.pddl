;; Action costs version of the IPC-2002 Zenotravel domain.

;; The encoding is somewhat strange because the costs of
;; flying/zooming from city to city differs for different pairs of
;; cities, but the fuel usage does not. But since this is only
;; intended as a test domain, this shouldn't be much of an issue.

(define (domain zeno-travel)

(:requirements :action-costs :object-fluents :typing)

(:types locatable city fuel-amount - object
        aircraft person - locatable)

(:functions (at ?l - locatable) - city
            (in ?p - person) - aircraft
            (fuel-level ?a - aircraft) - fuel-amount
            (next ?n - fuel-amount) - fuel-amount
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
         :parameters (?a - aircraft ?c - city ?n - fuel-amount)
         :precondition (= (next ?n) (fuel-level ?a))
         :effect (and (assign (at ?a) ?c)
                      (assign (fuel-level ?a) ?n)
                      (increase (total-cost) (fly-cost (at ?a) ?c))))

(:action zoom
         :parameters (?a - aircraft ?c - city ?n - fuel-amount)
         :precondition (= (next (next ?n)) (fuel-level ?a))
         :effect (and (assign (at ?a) ?c)
                      (assign (fuel-level ?a) ?n)
                      (increase (total-cost) (zoom-cost (at ?a) ?c))))

(:action refuel
         :parameters (?a - aircraft ?n - fuel-amount)
         :precondition (= (next (fuel-level ?a)) ?n)
         :effect (and (assign (fuel-level ?a) ?n)
                      (increase (total-cost) 50)))

)
