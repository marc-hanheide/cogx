(define (domain  dora_the_explorer2)
  
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

)