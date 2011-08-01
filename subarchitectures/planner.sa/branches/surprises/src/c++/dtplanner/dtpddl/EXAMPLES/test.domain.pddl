


(define (domain  dora_the_explorer)
  
  (:requirements 
   ;; IPC6 elements -- Planning off the ground
   :typing ;; Keywords :: ":types"
   :strips 
   :equality
   :fluents 
   
   ;; Uncertainty track at IPC6
   :probabilistic-effects
   
   ;; PDDL syntactic sugar
   ;; Keywords :: 
   :universal-effects  
   :conditional-effects  
   
   ;; DTP-ELEMENT
   ;; Keywords :: ":percepts", ":observe", and ":execution"
   :partial-observability 

    )

  (:types 
   place label widget feature  model-slot - object
   model - (either widget feature) 
   )

  (:constants 
   biscuit 
   something 
   )

  (:predicates (something ?x - object ?y - nothing)
	       (ekse ?x - object ?y - nothing)
	       (ekse ?x - (either object) ?y - (either nothing or-soemthing))

	       )

  (:derived (my_own_predicate-1 ?m - model ?s - model-slot) 
	    (forall (?a - Car) (or (some ?a ?s) (other ?m ?s))))

  (:derived (my_own_predicate-2 ?m - model ?s - model-slot) 
	    (or (some ?a ?s) (other ?m ?s)))
2
  (:action foreground_model
	   :parameters (?m - model ?s - model-slot)
	   
	   :precondition (if (forall (?a - Car)
				 (forall (?c - Car) 
					 (not (or 
					       (cool ?c)
					       (cool ?a) 
					       (and (some ?m ?s) (other ?m ?s)))))
				 )
(forall (?a - Car)
				 (exists (?c - Car) 
					 (not (or 
					       (cool ?c)
					       (cool ?a) 
					       (and (some ?m ?s) (other ?m ?s)))))
				 )
			     )
	   )
	   

)