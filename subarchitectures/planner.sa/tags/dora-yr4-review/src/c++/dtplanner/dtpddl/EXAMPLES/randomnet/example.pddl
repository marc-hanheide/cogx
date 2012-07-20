
  (:action open
	   :parameters (?x - switching-device)

	   :precondition (and (closed ?x)
			      (or (not (powered ?ls1)) (not (powered ?ls2)) ) 

			      )

	   :effect (when (chosen ?x) (assign (reward) 5))
	   )

  (:action close
	   :parameters (?x - switching-device)

	   :precondition (and (biased ?x))

	   :effect (and (closed ?x))
	   )