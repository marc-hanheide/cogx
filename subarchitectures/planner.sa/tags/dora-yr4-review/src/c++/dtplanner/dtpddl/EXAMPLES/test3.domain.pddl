
(define (domain birmingham)
  
  (:requirements 
;;    IPC6 elements -- Planning off the ground
   :typing ;; Keywords :: ":types"
   :strips 
   :equality
   :fluents 
   
;;    Uncertainty track at IPC6
   :probabilistic-effects
   
;;    PDDL syntactic sugar
;;    Keywords :: 
   :universal-effects  
   :conditional-effects  
   
;;    DTP-ELEMENT
;;    Keywords :: ":percepts", ":observe", and ":execution"
   :partial-observability 

    )

  (:types 
   	coin - object
   )


  (:predicates 

   (biased ?x - coin)
   (chosen ?x - coin)

	       )

  (:functions (reward) - number )

  
  (:percepts

   (o-heads ?x - coin)

	       )

  (:action flip
	   :parameters (?x - coin)

	   :precondition (and (chosen ?x) (biased ?x))

	   :effect (assign (reward) 5)
	   )

  (:action flip2
	   :parameters (?x - coin)

;; 	   :precondition (and (chosen ?x) (biased ?x))
	   :precondition (and (not (and (or (not (chosen ?x)) (not (chosen ?x))) (or (not (chosen ?x)) (not (chosen ?x))))) (or (chosen ?x) (chosen ?x)) (biased ?x))

	   :effect (increase (reward) 5)
	   )

  (:action gamble
	   :parameters (?x - coin)

	   :precondition ()

	   :effect (chosen ?x)
	   )


  (:observe coin-status
	   :parameters (?x - coin)

	   :execution (flip ?x)

	   :precondition (not (biased ?x))

	   :effect (probabilistic 0.5 (o-heads ?x)
		    )
	   )

  (:observe coin-status-biased
	   :parameters (?x - coin)

	   :execution (or (flip ?x) (flip ?x))

	   :precondition (biased ?x)

	   :effect (probabilistic 0.8 (o-heads ?x)
		    )
	   )


  )