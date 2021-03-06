;; Logistics domain, PDDL 1.2 version.

(define (domain logistics)

(:requirements :typing :action-costs :goal-utilities) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object)
  
(:predicates  (in-city ?l - location ?c - city)
              (at ?obj - thing ?l - location)
              (in ?p - package ?veh - vehicle))

(:functions (total-cost) - number)
  
(:action drive
         :parameters    (?t - truck ?from ?to - location ?c - city)
         :precondition  (and (at ?t ?from)
                             (in-city ?from ?c)
                             (in-city ?to ?c))
         :effect        (and (not (at ?t ?from))
                             (at ?t ?to)
		 (increase (total-cost) 2))
)

(:action fly
         :parameters    (?a - airplane ?from ?to - airport)
         :precondition  (at ?a ?from)
         :effect        (and (not (at ?a ?from))
                             (at ?a ?to)
		(increase (total-cost) 10))
)

(:action load
         :parameters    (?v - vehicle ?p - package ?l - location)
         :precondition  (and (at ?v ?l)
                             (at ?p ?l))
         :effect        (and (not (at ?p ?l))
                             (in ?p ?v)
		(increase (total-cost) 1))
)

(:action unload
         :parameters    (?v - vehicle ?p - package ?l - location)
         :precondition  (and (at ?v ?l)
                             (in ?p ?v))
         :effect        (and (not (in ?p ?v))
                             (at ?p ?l)
		(increase (total-cost) 1))
)

)
