(domain (george-june)

	(:types
	 Concept - object  
	 Visual-Object - object
	 Model - object
	 )

	(:predicates

	 (made-commit ?x - Visual-Object ?c - Concept)

	 (been-used-for-learning-sample ?c - Concept ?x - Visual-Object)
   
	 (learnable-concept ?c - Concept)
   
	 (model-part-of-concept ?m - Model ?c - Concept)
	 )
 
	(:functions
	 (attributed-label ?x -VisualObject ?c - Concept) - Model
	 (true-positive ?c - Concept) - number
	 (true-negative ?c - Concept) - number
	 (tutor-answers-polar ?c - Concept ?x - Visual-Object) - Model
	 )

	(:s-functions 
	 (total-reward) - number

	 (REAL-has-property ?x - Visual-Object ?c - Concept) - Model
	 )

	(:o-functions 

	 (OBSERVE-has-property ?x - Visual-Object ?c - Concept) - Model

	 )

	(:action learn-from-sample 
	 
		 (:parameters ?x - Visual-Object ?c - Concept ?m - Model)
		 (:agent ?a - robot)
		 (:precondition 
		  (and 
		   (not (been-used-for-learning-sample ?c ?x))
		   (kval ?a (attributed-label ?x ?c)
			 (= (attributed-label ?x ?c ) ?m)
			 (learnable-concept ?c)
			 (model-part-of-concept ?m ?c)
			 )
		   )
	 
		  (:effect 
		   ;; Learning action
		   (and 
		    (been-used-for-learning-sample ?c ?x)
		    ) 
		   )
		  )
		 )

	(:observe object-color
		  (:parameters ?x - Visual-Object ?c - Concept ?m - Model)

		  (:execution 
		   (learn-from-sample ?x ?c ?m)
		   )

		  (:precondition )

		  (:effects
	   
		   (when (= (REAL-has-property ?x ?c) ?m)
		     (probability (true-positive ?c) 
				  (and (assign (OBSERVE-has-property ?x ?c) ?m ))
				  )
		     )
		   (when (not (= (REAL-has-property ?x ?c) ?m))
		     (probability (false-positive ?c) 
				  (and (assign (OBSERVE-has-property ?x ?c) ?m ))
				  )
		     )
	   
		   )
	  
		  )


	(:action commit-to

		 (:parameters ?x - Visual-Object ?c - Concept)
	  
		 (:precondition
		  (and
		   (not (made-commit ?x ?c))			 
		   )	 
		  )

		 (:effects
		  (and
		   (made-commit ?x ?c)
		   (when (= (OBSERVE-has-property ?x ?c) (REAL-has-property ?x ?c))
		     (increase (total-reward) 1000))
		   )
	 
		  )
		 )
	 
	 
	(:action aks-for-property-polar	;; is the color of this object red?
	 
		 (:parameters ?x - Visual-Object ?c - concept ?m - Model)
		 (:agent ?a - robot)
		 (:precondition 
		  (and 
		   (not (been-used-for-learning-sample ?x ?c))
		   (model-needs-learning ?m)
		   (model-part-of-concept ?m ?c )
		   (not (kval ?a (attributed-label ?x ?c)))
		   )
		  )
	 
		 (:effect 
		  ;; Clarification action
		  (and
		   (probability (tutor-answers-polar ?c)
				(probability ( ;; probability that ?m is the correct value of ?c for ?x ) 
					      (kval ?a (attributed-label ?x ?c)
						    ))
					     )
	  
				)
		   )
		  )
		 )


	(:action aks-for-property-general ;; what is the color of this object?
	 
		 (:parameters ?x - Visual-Object ?c - concept)
		 (:agent ?a - robot)
		 (:precondition 
		  (and 
		   (not (been-used-for-learning-sample ?x ?c))
		   (exists ?m - Model (and (model-needs-learning ?m) (model-part-of-concept ?m ?c)))
		   (not (kval ?a (attributed-label ?x ?c)))
		   )
		  )
	 
		 (:effect 

		  (and
		   (probability (tutor-answers-general ?c)
				(kval ?a (attributed-label ?x ?c))
				)
		   )
	  
		  )
		 )
	)
