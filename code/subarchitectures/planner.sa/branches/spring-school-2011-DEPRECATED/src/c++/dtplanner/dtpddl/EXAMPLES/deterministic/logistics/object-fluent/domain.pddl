;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :object-fluents :action-costs) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object)
  
(:functions
	(total-cost) - number
	(city-of ?l - location) - city
              (location-of ?t - thing) - (either location vehicle))

(:action drive
         :parameters    (?t - truck ?to - location)
         :precondition  (= (city-of (location-of ?t)) (city-of ?to))
         :effect    (and    (assign (location-of ?t) ?to)
		(increase (total-cost) 2)))

(:action fly
         :parameters    (?a - airplane ?to - airport)
         :effect     (and  (assign (location-of ?a) ?to)
		(increase (total-cost) 3)))

(:action load
         :parameters    (?p - package ?v - vehicle)
         :precondition  (= (location-of ?p) (location-of ?v))
         :effect     (and  (assign (location-of ?p) ?v)
		   (increase (total-cost) 2)))

(:action unload
         :parameters    (?p - package ?v - vehicle)
         :precondition  (= (location-of ?p) ?v)
         :effect     (and   (assign (location-of ?p) (location-of ?v))
		(increase (total-cost) 2)))

)

;; EOF
