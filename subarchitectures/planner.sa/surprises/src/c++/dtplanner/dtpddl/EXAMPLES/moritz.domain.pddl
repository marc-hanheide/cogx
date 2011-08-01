(define (domain dt-cogx)

  (:requirements :quantified-preconditions :equality :negative-preconditions :conditional-effects :typing :adl :disjunctive-preconditions :partial-observability )

  (:types  room - object
	   place_status - object
	   feature - object
	   agent - object
	   robot - movable
	   robot - agent
	   boolean - object
	   place - object
	   subgoal - object
	   movable - object
	   planning_agent - agent
	   place_name - object

	   )

  (:functions (reward) - int )

  (:predicates  
   (rubbish)
   (committed-is-in ?r - movable)
		(kval-placestatus ?a - agent ?n - place)
		(placestatus ?n - place ?value - place_status)
		(i_in-domain-is-in ?r - movable ?v - place)
		(kd-is-in ?a - agent ?r - movable)
		(kval-name ?a - agent ?p - place)
		(i_in-domain-name ?p - place ?v - place_name)
		(committed-name ?p - place)
		(in-domain-placestatus ?n - place ?v - place_status)
		(kd-placestatus ?a - agent ?n - place)
		(is-in ?r - movable ?value - place)
		(kd-name ?a - agent ?p - place)
		(in-domain-is-in ?r - movable ?v - place)
		(connected ?n1 - place ?n2 - place)
		(name ?p - place ?value - place_name)
		(i_in-domain-placestatus ?n - place ?v - place_status)
		(in-domain-name ?p - place ?v - place_name)
		(kval-is-in ?a - agent ?r - movable)
		(committed-placestatus ?n - place)
		(visited ?r - robot ?p - place)
		)

  (:percepts  (observed-name ?p - place ?v - place_name)
	      (observed-is-in ?r - movable ?v - place)
	      (observed-placestatus ?n - place ?v - place_status)
	      )


  (:constants  dora - robot
	       mars earth - place_name
	       place_0_pt_spatial_data - place
	       placeholder trueplace - place_status
	       )

  (:action  explore_place
	    :parameters  (?a - robot ?loc - place)
	    :precondition  (and  (placestatus ?loc placeholder)
				 (is-in ?a ?loc)
				 )
	    :effect  (and  (not (placestatus ?loc placeholder))
			   (placestatus ?loc trueplace)
			   )
	    )

  (:action  look-for-object
	    :parameters  (?a - robot ?loc - place)
	    :precondition  (and  (is-in ?a ?loc))
	    :effect  (visited dora ?loc)
	    )

  (:action  ask-for-placename
	    :parameters  (?a - robot ?loc - place)
	    :precondition  (and  (is-in ?a ?loc))
	    )

  (:action  stupid
	    :parameters  (?a - robot ?loc - place)
	    :precondition  (and  (is-in ?a ?loc))
	    )

  (:action  commit-name-place_0_pt_spatial_data
	    :parameters  (?val - place_name)
	    :precondition  (not (committed-name place_0_pt_spatial_data))
	    :effect  (and  (committed-name place_0_pt_spatial_data)
			   (when  (name place_0_pt_spatial_data ?val)
			     (increase (reward ) 100)
			     )
			   )
	    )

  (:observe  placename
	     :parameters  (?a - robot ?loc - place ?n - place_name)
	     :execution  (ask-for-placename ?a ?loc)
	     :effect  (and  (when  (name ?loc ?n)
			      (probabilistic  0.7  (observed-name ?loc ?n))
			      )
			    (when  (not (name ?loc ?n))
			      (probabilistic  0.1  (observed-name ?loc ?n))
			      )
			    )
	     )

  )
