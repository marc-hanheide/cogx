(define (domain Rover)
(:requirements :typing :fluents :goal-utilities :object-fluents)

(:types rover waypoint store store_state camera mode lander objective - object)

(:constants empty full - store_state)

(:predicates        
	(can_traverse ?r - rover ?x - waypoint ?y - waypoint)
	(equipped_for_soil_analysis ?r - rover)
	(equipped_for_rock_analysis ?r - rover)
	(equipped_for_imaging ?r - rover)
	(have_rock_analysis ?r - rover ?w - waypoint)
	(have_soil_analysis ?r - rover ?w - waypoint)
	(calibrated ?c - camera ?r - rover) 
	(supports ?c - camera ?m - mode)
	(available ?r - rover)
	(visible ?w - waypoint ?p - waypoint)
	(have_image ?r - rover ?o - objective ?m - mode)
	(communicated_soil_data ?w - waypoint)
	(communicated_rock_data ?w - waypoint)
	(communicated_image_data ?o - objective ?m - mode)
	(at_soil_sample ?w - waypoint)
	(at_rock_sample ?w - waypoint)
	(visible_from ?o - objective ?w - waypoint)
	(store_of ?s - store ?r - rover)
	(on_board ?i - camera ?r - rover)
	(channel_free ?l - lander)
)

(:functions
	(traverse_cost ?r - rover ?f - waypoint ?t - waypoint)
	(total-cost) - number
	(at ?x - (either rover lander)) - waypoint
	(calibration_target ?i - camera) - objective
	(sstate ?s - store) - store_state)
	

	
(:action navigate
:parameters (?x - rover ?y - waypoint ?z - waypoint) 
:precondition (and (can_traverse ?x ?y ?z)
                          (available ?x)
	            (= (at ?x) ?y) 
                          (visible ?y ?z))
:effect (and (assign (at ?x) ?z)
                (increase (total-cost)  (traverse_cost ?x ?y ?z)))
)

(:action sample_soil
:parameters (?x - rover ?s - store ?p - waypoint)
:precondition (and (= (at ?x) ?p)
	            (at_soil_sample ?p)
	            (equipped_for_soil_analysis ?x)
	            (store_of ?s ?x)
	            (= (sstate ?s) empty))
:effect (and (assign (sstate ?s) full)
	   (have_soil_analysis ?x ?p)
	   (not (at_soil_sample ?p))
	   (increase (total-cost) 2))
)

(:action sample_rock
:parameters (?x - rover ?s - store ?p - waypoint)
:precondition (and (= (at ?x) ?p)
	 	(at_rock_sample ?p)
		(equipped_for_rock_analysis ?x)
		(store_of ?s ?x)
		(= (sstate ?s) empty))
:effect (and (assign (sstate ?s) full)
	(have_rock_analysis ?x ?p)
	(not (at_rock_sample ?p))
	(increase (total-cost) 2))
)

(:action drop
:parameters (?x - rover ?y - store)
:precondition (and (store_of ?y ?x)
		(= (sstate ?y) full))
:effect (and (assign (sstate ?y) empty)
	(increase (total-cost) 2))
)

(:action calibrate
 :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
 :precondition (and (equipped_for_imaging ?r)
		(= (calibration_target ?i) ?t)
		(= (at ?r) ?w)
		(visible_from ?t ?w)
		(on_board ?i ?r))
 :effect (and (calibrated ?i ?r)
	(increase (total-cost) 2))
)


(:action take_image
 :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
 :precondition (and (calibrated ?i ?r)
		(on_board ?i ?r)
		(equipped_for_imaging ?r)
		(supports ?i ?m)
		(visible_from ?o ?p)
		(= (at ?r) ?p))
 :effect (and (have_image ?r ?o ?m)
	(not (calibrated ?i ?r))
	(increase (total-cost) 2))
)


(:action communicate_soil_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :precondition (and (= (at ?r) ?x)
		(= (at ?l) ?y)
		(have_soil_analysis ?r ?p) 
		(visible ?x ?y)
		(available ?r)
		(channel_free ?l))
 :effect (and (not (available ?r))
		(not (channel_free ?l))
		(channel_free ?l)
		(available ?r)
		(communicated_soil_data ?p)
		(increase (total-cost) 2))
)

(:action communicate_rock_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :precondition (and (= (at ?r) ?x)
		(= (at ?l) ?y)
		(have_rock_analysis ?r ?p)
		(visible ?x ?y)
		(available ?r)
		(channel_free ?l))
 :effect (and (not (available ?r))
	(not (channel_free ?l))
	(channel_free ?l)
	(available ?r)
	(communicated_rock_data ?p)
	(increase (total-cost) 3))
)


(:action communicate_image_data
 :parameters (?r - rover ?l - lander ?o - objective
		?m - mode ?x - waypoint ?y - waypoint)
 :precondition (and (= (at ?r) ?x)
		(= (at ?l) ?y)
		(have_image ?r ?o ?m)
		(visible ?x ?y)
		(available ?r)
		(channel_free ?l))
 :effect (and (not (available ?r))
	(not (channel_free ?l))
	(channel_free ?l)
	(communicated_image_data ?o ?m)
	(available ?r)
	(increase (total-cost) 2))
)
)


;; EOF
