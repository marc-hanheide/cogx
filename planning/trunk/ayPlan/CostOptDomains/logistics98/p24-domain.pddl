(define (domain logistics-strips)
(:requirements :typing :action-costs)
 (:types block)
 
  (:predicates 	(OBJ ?obj - block )
	       	(TRUCK ?truck - block )
               	(LOCATION ?loc - block )
		(AIRPLANE ?airplane - block )
                (CITY ?city - block )
                (AIRPORT ?airport - block )
		(at ?obj - block  ?loc - block )
		(in ?obj1 - block  ?obj2 - block )
		(in-city ?obj - block  ?city - block ))
 
 (:functions
     (total-cost) - number
  )
  


(:action LOAD-TRUCK
  :parameters
   (?obj - block 
    ?truck - block 
    ?loc - block )
  :precondition
   (and (OBJ ?obj) (TRUCK ?truck) (LOCATION ?loc)
   (at ?truck ?loc) (at ?obj ?loc))
  :effect
   (and (not (at ?obj ?loc)) (in ?obj ?truck)
   (increase (total-cost) 1))
   )

(:action LOAD-AIRPLANE
  :parameters
   (?obj - block 
    ?airplane - block 
    ?loc - block )
  :precondition
   (and (OBJ ?obj) (AIRPLANE ?airplane) (LOCATION ?loc)
   (at ?obj ?loc) (at ?airplane ?loc))
  :effect
   (and (not (at ?obj ?loc)) (in ?obj ?airplane)
   (increase (total-cost) 1))
   )

(:action UNLOAD-TRUCK
  :parameters
   (?obj - block 
    ?truck - block 
    ?loc - block )
  :precondition
   (and (OBJ ?obj) (TRUCK ?truck) (LOCATION ?loc)
        (at ?truck ?loc) (in ?obj ?truck))
  :effect
   (and (not (in ?obj ?truck)) (at ?obj ?loc)
   (increase (total-cost) 1))
   )

(:action UNLOAD-AIRPLANE
  :parameters
   (?obj - block 
    ?airplane - block 
    ?loc - block )
  :precondition
   (and (OBJ ?obj) (AIRPLANE ?airplane) (LOCATION ?loc)
        (in ?obj ?airplane) (at ?airplane ?loc))
  :effect
   (and (not (in ?obj ?airplane)) (at ?obj ?loc)
   (increase (total-cost) 1))
   )

(:action DRIVE-TRUCK
  :parameters
   (?truck - block 
    ?loc-from - block 
    ?loc-to - block 
    ?city - block )
  :precondition
   (and (TRUCK ?truck) ;(LOCATION ?loc-from) (LOCATION ?loc-to) (CITY ?city)
   (at ?truck ?loc-from)
   (in-city ?loc-from ?city)
   (in-city ?loc-to ?city))
  :effect
   (and (not (at ?truck ?loc-from)) (at ?truck ?loc-to)
   (increase (total-cost) 1))
   )

(:action FLY-AIRPLANE
  :parameters
   (?airplane - block 
    ?loc-from - block 
    ?loc-to - block )
  :precondition
   (and (AIRPLANE ?airplane) (AIRPORT ?loc-from) (AIRPORT ?loc-to)
	(at ?airplane ?loc-from))
  :effect
   (and (not (at ?airplane ?loc-from)) (at ?airplane ?loc-to)
   (increase (total-cost) 1))
   )
)
