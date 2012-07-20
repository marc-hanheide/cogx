(define (domain logistics_conf)
(:requirements :adl :object-fluents)
  (:types
	AIRPORT	- LOCATION
	AIRPLANE TRUCK - VEHICLE
	VEHICLE LOCATION - OBJ_LOCATION
	VEHICLE OBJ - LOCATABLE
	OBJ CITY)

  (:functions 	
      (pos ?obj - LOCATABLE) - OBJ_LOCATION
      (in_city ?loc - OBJ_LOCATION) - CITY
)

(:action LOAD_TRUCK_LOC
  :parameters
   (?obj - OBJ
    ?truck - TRUCK
    ?loc - LOCATION)
  :precondition
   (and
	(= (pos ?truck) ?loc)
	(= (pos ?obj) ?loc) 
   )
  :effect
   (and (assign (pos ?obj) ?truck))
   
)


(:action LOAD_AIRPLANE
  :parameters
   (?obj - OBJ ?airplane - AIRPLANE ?loc - AIRPORT) 
  :precondition (and
   (= (pos ?airplane) ?loc)
   (= (pos ?obj) ?loc)) 
  :effect
   (and (assign (pos ?obj) ?airplane)))

(:action UNLOAD_TRUCK_LOC
  :parameters
   (?obj - OBJ
    ?truck - TRUCK
    ?loc - LOCATION)
  :precondition
   (and 
	(= (pos ?truck) ?loc) 
	(= (pos ?obj) ?truck) 
   )
  :effect
   (and (assign (pos ?obj) ?loc)))


(:action UNLOAD_AIRPLANE
  :parameters
   (?obj - OBJ
    ?airplane - AIRPLANE
    ?loc - AIRPORT)
  :precondition (and
   (= (pos ?airplane) ?loc)
   (= (pos ?obj) ?airplane))
  :effect
   (and (assign (pos ?obj) ?loc)))

(:action DRIVE_TRUCK
  :parameters
   (?truck - TRUCK
    ?loc1 - LOCATION
    ?loc2 - LOCATION
    ?city - CITY)
  :precondition
   (and (= (pos ?truck) ?loc1)
        (= (in_city ?loc1) ?city)
        (= (in_city ?loc2) ?city)
	;(in_city ?truck : ?city)
	;(not (at_ta ?truck ?loc2))
   )
  :effect
   (and (assign (pos ?truck) ?loc2)))

(:action FLY_AIRPLANE
  :parameters
   (?airplane - AIRPLANE
    ?loc1 - AIRPORT
    ?loc2 - AIRPORT)
  :precondition
  (and  
	(= (pos ?airplane) ?loc1)
  )
  :effect
   (and (assign (pos ?airplane) ?loc2 )))


)
