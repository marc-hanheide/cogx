(define (domain cogx)

	(:requirements :mapl :adl :object-fluents :action-costs)
	
	(:types 
		place place_id - object
		robot - agent
	)
	
    (:predicates
		(connected ?n1 - place ?n2 - place)
		(visited ?p - place)
    )	

	(:functions
        (is-in ?r - robot) - place
        (placeid ?p - place) - place_id
 	)
	
;;; actions ;;;

	(:action move
	 :agent (?a - robot)
	 :parameters (?to - place)
	 :variables (?from - place)
	 :precondition (and
		(= (is-in ?a) ?from)
		(or (connected ?from ?to)
		    (connected ?to ?from))
		)
	 :effect (and
		  (increase (total-cost) 10)

;; 		  (when 
;; 		    (increase (total-cost) 10))
		  
;; 		  (when (not (or (connected ?from ?to)
;; 			    (connected ?to ?from))
;; 		    (increase (total-cost) 10)))
		  
		(assign (is-in ?a) ?to)
	))

	(:action expensive-move
	 :agent (?a - robot)
	 :parameters (?to - place)
	 :variables (?from - place)
	 :precondition (and
		(= (is-in ?a) ?from)
		)
	 :effect (and
		  (increase (total-cost) 10000)

;; 		  (when 
;; 		    (increase (total-cost) 10))
		  
;; 		  (when (not (or (connected ?from ?to)
;; 			    (connected ?to ?from))
;; 		    (increase (total-cost) 10)))
		  
		(assign (is-in ?a) ?to)
	))

;; 	(:action look-for-object-cheap
;; 	 :agent (?a - robot)
;; 	 :parameters (?to - place)
;; 	 :precondition (and
;; 			(not (visited ?to))
;; 			(= (is-in ?a) ?to)

			
;; ;; 			(exists (?pp - place) 
;; ;; 				(and 
;; ;; 				 (not (visited ?pp))
;; ;; 				 (or (connected ?to ?pp)
;; ;; 				     (connected ?pp ?to))) 
;; ;; 				 )

;; ;; 			(forall (?pp - place) 
;; ;; 				(or 
;; ;; 				 (not (visited ?pp))
;; ;; 				 (not (or (connected ?to ?pp)
;; ;; 					  (connected ?pp ?to))) 
;; ;; 				 ))
;; 			)
;; 	 :effect (and
;; 		  (increase (total-cost) 10000)
;; 		  (visited ?to)	
;; 		  (update (visited ?to) true) ))

	
	(:action look-for-object
	 :agent (?a - robot)
	 :parameters (?to - place)
	 :precondition (and
			(not (visited ?to))
			(= (is-in ?a) ?to)

			)
	 :effect (and
;; 		  (increase (total-cost) 1020)
		  (visited ?to)	
		  (update (visited ?to) true) ))

        (:action look-for-people
	           :agent (?a - robot)
	           :variables (?p - place)
	           :precondition (= (is-in ?a) ?p)
	           :effect (and
	                    (explored ?p)
	                    (update (explored ?p) true)
	                    )
	           )

         (:action ask-for-your-name
	           :agent (?a - robot)
	           :parameters (?p - person)
	           :variables (?loc - place)
	           :precondition (and
	                          (= (is-in ?a) ?loc)
	                          (= (is-in ?p) ?loc)
	                          )
	           :effect
	           (kval ?a (name ?p))
	           )
	
	  (:action commit-name
	           :agent (?a - robot)
	           :parameters (?p - person ?n - person_name)
	           :precondition (and
	                          (in-domain (name ?p) ?n)
	                          )
	           :replan (kval ?a (name ?p))
	           :effect
	           (assign (name ?p) ?n)
	           )


)

;; (metric minimize (total-costs))

;; goal: (forall (?p - place) (visited ?p))
