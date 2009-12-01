(define (domain satellite)
(:requirements :equality :strips :typing :action-costs)
 (:types block)

(:predicates
	 (on_board ?i - block  ?s - block ) (supports ?i - block  ?m - block ) (pointing ?s - block  ?d - block ) (power_avail ?s - block ) (power_on ?i - block ) (calibrated ?i - block ) (have_image ?d - block  ?m - block ) (calibration_target ?i - block  ?d - block )(satellite ?x - block ) (direction ?x - block ) (instrument ?x - block ) (mode ?x - block ) )
	 
	  (:functions
     (total-cost) - number
  )
	 
(:action turn_to
 :parameters ( ?s - block  ?d_new - block  ?d_prev - block )
 :precondition
	(and (satellite ?s) (direction ?d_new) (direction ?d_prev)  (pointing ?s ?d_prev))
 :effect
	(and (pointing ?s ?d_new) (not (pointing ?s ?d_prev))(increase (total-cost) 1)
	)
	)

(:action switch_on
 :parameters ( ?i - block  ?s - block )
 :precondition
	(and (instrument ?i) (satellite ?s)  (on_board ?i ?s) (power_avail ?s))
 :effect
	(and (power_on ?i) (not (calibrated ?i)) (not (power_avail ?s))(increase (total-cost) 1)
	)
	)

(:action switch_off
 :parameters ( ?i - block  ?s - block )
 :precondition
	(and (instrument ?i) (satellite ?s)  (on_board ?i ?s) (power_on ?i))
 :effect
	(and (power_avail ?s) (not (power_on ?i))(increase (total-cost) 1)
	)
	)

(:action calibrate
 :parameters ( ?s - block  ?i - block  ?d - block )
 :precondition
	(and (satellite ?s) (instrument ?i) (direction ?d)  (on_board ?i ?s) (calibration_target ?i ?d) (pointing ?s ?d) (power_on ?i))
 :effect
	 (and (calibrated ?i) (increase (total-cost) 1)
	 )
	 )

(:action take_image
 :parameters ( ?s - block  ?d - block  ?i - block  ?m - block )
 :precondition
	(and (satellite ?s) (direction ?d) (instrument ?i) (mode ?m)  (calibrated ?i) (on_board ?i ?s) (supports ?i ?m) (power_on ?i) (pointing ?s ?d) (power_on ?i))
 :effect
	 (and (have_image ?d ?m)(increase (total-cost) 1)
	 )
	 )

)

