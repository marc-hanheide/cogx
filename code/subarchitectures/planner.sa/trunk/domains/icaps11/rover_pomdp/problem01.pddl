(define (scenario roverprob1)
  (:domain POMDPRover)
  (:common
   (:objects
	general - lander
	spirit - rover
	spiritstore - store
	w0 w1 w2 - waypoint
	r1 r2 - rock
	)
   (:init
	(visible w1 w0)
	(visible w1 w2)
	(visible w2 w0)
	(visible w2 w1)
	(visible w0 w1)
	(visible w0 w2)
	(can_traverse spirit w0 w1)
	(can_traverse spirit w0 w2)
	(can_traverse spirit w1 w2)
	(can_traverse spirit w1 w0)
	(can_traverse spirit w2 w0)
	(can_traverse spirit w2 w1)
	(at_rock r1 w1)
	(at_rock r2 w2)
	(at_lander general w0)
	(at spirit w0)
	(has_spectrometer spirit)
	(has_microscope spirit)
	(has_rat spirit)
	(empty spiritstore)
	(available spirit)
	(store_of spiritstore spirit)
	(channel_free general)
	(probabilistic 0.4 (assign (rock_type r1) carbonate)
                   0.6 (assign (rock_type r1) basalt))
	(probabilistic 0.2 (assign (rock_type r2) carbonate)
                   0.8 (assign (rock_type r2) basalt))
    ))

  (:agent spirit
          (:goal (communicated_rock_data carbonate))

          (:metric minimize (total-time))
          )
  )