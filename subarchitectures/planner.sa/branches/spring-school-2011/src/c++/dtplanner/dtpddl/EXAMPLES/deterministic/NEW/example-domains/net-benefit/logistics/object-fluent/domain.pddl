;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :object-fluents :action-costs :goal-utilities) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object)
  
(:functions
	(total-cost) - number
	(city-of ?l - location) - city
              (location-of ?t - thing) - (either location vehicle))

(:durative-action drive
         :parameters    (?t - truck ?to - location)
         :duration (= ?duration 4)
         :condition  (over all (= (city-of (location-of ?t)) (city-of ?to)))
         :effect   (and  (change (location-of ?t) ?to)
		(increase (total-cost) 2)))

(:durative-action fly
         :parameters    (?a - airplane ?to - airport)
         :duration (= ?duration 8)
         :condition ()
         :effect  (and   (change (location-of ?a) ?to)
		(increase (total-cost) 2)))

(:durative-action load
         :parameters    (?p - package ?v - vehicle)
         :duration (= ?duration 2)
         :condition  (over all (= (location-of ?p) (location-of ?v)))
         :effect   (and  (change (location-of ?p) ?v)
		(increase (total-cost) 2)))

(:durative-action unload
         :parameters    (?p - package ?v - vehicle)
         :duration (= ?duration 2)
         :condition  (at start (= (location-of ?p) ?v))
         :effect   (and  (change (location-of ?p) (location-of ?v))
		(increase (total-cost) 2)))
)
