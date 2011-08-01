;; Satellite domain, PDDL 3.1 version

(define (domain satellite-object-fluents)

(:requirements :typing :equality :object-fluents :action-costs)

(:types satellite direction instrument mode - object)

(:predicates
	(on_board ?i - instrument ?s - satellite)
	(supports ?i - instrument ?m - mode)
	(power_avail ?s - satellite)
	(power_on ?i - instrument)
	(calibrated ?i - instrument)
	(have_image ?d - direction ?m - mode))

(:functions (total-cost) - number
	 (pointing ?s - satellite) - direction
	 (calibration_target ?i - instrument) - direction)
	 


(:action turn_to
   :parameters (?s - satellite ?d_new - direction)
   :effect (and (assign (pointing ?s) ?d_new)
	     (increase (total-cost) 2))
)


(:action switch_on
   :parameters (?i - instrument ?s - satellite) 
   :precondition (and (on_board ?i ?s) 
                      	(power_avail ?s))
   :effect (and (power_on ?i)
                   (not (calibrated ?i))
                   (not (power_avail ?s))
                   (increase (total-cost) 1))
)

(:action switch_off
   :parameters (?i - instrument ?s - satellite)
   :precondition (and (on_board ?i ?s)
                      (power_on ?i))
   :effect (and (not (power_on ?i))
                    (power_avail ?s)
                    (increase (total-cost) 1))
)

(:action calibrate
   :parameters (?s - satellite ?i - instrument ?d - direction)
   :precondition (and (on_board ?i ?s)
		 (= (calibration_target ?i) ?d)
                             (= (pointing ?s) ?d)
                             (power_on ?i))
   :effect (and (calibrated ?i)
                    (increase (total-cost) 2))
)

(:action take_image
   :parameters (?s - satellite ?d - direction ?i - instrument ?m - mode)
   :precondition (and (calibrated ?i)
                      (on_board ?i ?s)
                      (supports ?i ?m)
                      (power_on ?i)
                      (= (pointing ?s) ?d))
   :effect (and (have_image ?d ?m)
                   (increase (total-cost) 3))
)
)


;; EOF
