
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
   (gambled)
   (biased ?x - coin)
   (chosen ?x - coin)

	       )

  (:functions (reward) - number )

  
  (:percepts

   (o-heads ?x - coin)

	       )

  (:action flip
	   :parameters (?x - coin)

	   :precondition (and (biased ?x))

	   :effect (when (chosen ?x) (assign (reward) 5))
	   )

;;   (:action flip2
;; 	   :parameters (?x - coin)

;; 	   :precondition (and (biased ?x))

;; 	   :effect (when (chosen ?x) (assign (reward) 10))
;; 	   )

;;   (:action flip3
;; 	   :parameters (?x - coin)

;; 	   :precondition (and (biased ?x))

;; 	   :effect (when (chosen ?x) (assign (reward) 1))
;; 	   )

  (:action gamble
	   :parameters (?x - coin)

	   :precondition (not (gambled))

	   :effect (and (gambled) (chosen ?x) )
	   )


  (:observe coin-status
	   :parameters (?x - coin)

	   :execution (flip ?x)

	   :precondition (not (biased ?x))

	   :effect (probabilistic 0.01 (o-heads ?x)
		    )
	   )

  (:observe coin-status-biased
	   :parameters (?x - coin)
	   
	   :execution (or (flip ?x) (flip ?x))

	   :precondition (biased ?x)

	   :effect (probabilistic 0.9 (o-heads ?x)
		    )
	   )


  )