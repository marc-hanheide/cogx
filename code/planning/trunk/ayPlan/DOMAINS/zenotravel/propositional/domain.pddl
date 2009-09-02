;; Action costs version of the IPC-2002 Zenotravel domain.

;; The encoding is somewhat strange because the costs of
;; flying/zooming from city to city differs for different pairs of
;; cities, but the fuel usage does not. But since this is only
;; intended as a test domain, this shouldn't be much of an issue.

(define (domain zeno-travel)

(:requirements :action-costs :typing)

(:types locatable city fuel-amount - object
        aircraft person - locatable)

(:predicates (at ?l - locatable ?c - city)
             (in ?p - person ?a - aircraft)
	     (fuel-level ?a - aircraft ?n - fuel-amount)
	     (next ?n1 ?n2 - fuel-amount))

(:functions (total-cost) - number
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
         :parameters (?a - aircraft ?c1 ?c2 - city ?n1 ?n2 - fuel-amount)
         :precondition (and (at ?a ?c1)
                            (fuel-level ?a ?n2)
                            (next ?n1 ?n2))
         :effect (and (not (at ?a ?c1))
                      (at ?a ?c2)
                      (not (fuel-level ?a ?n2))
                      (fuel-level ?a ?n1)
                      (increase (total-cost) (fly-cost ?c1 ?c2))))

(:action zoom
         :parameters (?a - aircraft ?c1 ?c2 - city ?n1 ?n2 ?n3 - fuel-amount)
         :precondition (and (at ?a ?c1)
                            (fuel-level ?a ?n3)
                            (next ?n1 ?n2)
                            (next ?n2 ?n3))
         :effect (and (not (at ?a ?c1))
                      (at ?a ?c2)
                      (not (fuel-level ?a ?n3))
                      (fuel-level ?a ?n1)
                      (increase (total-cost) (zoom-cost ?c1 ?c2))))

(:action refuel
         :parameters (?a - aircraft ?c - city ?n1 ?n2 - fuel-amount)
         :precondition (and (fuel-level ?a ?n1)
                            (next ?n1 ?n2)
                            (at ?a ?c))
         :effect (and (not (fuel-level ?a ?n1))
                      (fuel-level ?a ?n2)
                      (increase (total-cost) 50)))

)
