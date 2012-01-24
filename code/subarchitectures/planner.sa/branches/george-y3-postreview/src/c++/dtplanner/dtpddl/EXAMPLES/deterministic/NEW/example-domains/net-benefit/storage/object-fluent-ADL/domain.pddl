; IPC5 Domain: Storage Propositional
; Authors: Alfonso Gerevini and Alessandro Saetti 
; Adapted for PDDL 3.1 by Ioannis Refanidis

(define (domain Storage-NetBenefit-OF-ADL)

(:requirements :typing  :object-fluents :equality :action-costs :goal-utilities :preferences :adl)

(:types hoist surface place - object
	container depot - place
	area crate - surface
	storearea transitarea - area)

(:constants 
		no-place - place	
		no-crate - crate
		no-area  - storearea
)
		
(:predicates
	     (connected ?a1 ?a2 - area)					; static predicate
         (inside ?s - storearea ?p - place)			; static predicate
         (clear ?s - storearea)
)
             
(:functions
		(at ?h - hoist) - area			
		(on ?c - crate) - storearea

	    (lifting ?h - hoist) - crate 	; meaning what is lifted by ?h
	    			; (= (lifting ?h) no-crate) means that ?h is available
	    (in ?c - crate) - place	; similar semantics to 'inside' predicate, but dynamic.
	    			; (= (in ?c) no-place) means that ?c is lifted by some hoist.
	    (total-cost) - number
) 

(:action lift
 :parameters (?h - hoist ?s - storearea ?c - crate  ?p - place)
 :precondition (and (= (on ?c) ?s) (inside ?s ?p) (= (in ?c) ?p) (connected ?s (at ?h)) (= (lifting ?h) no-crate) )
 :effect (and (clear ?s) (assign (on ?c) no-area) (assign (lifting ?h) ?c) (assign (in ?c) no-place) (increase (total-cost) 1)))
				
 
(:action drop
 :parameters (?h - hoist ?c - crate ?s - storearea ?p - place)
 :precondition (and (connected ?s (at ?h))  (= (lifting ?h) ?c) (= (on ?c) no-area) (clear ?s) (inside ?s ?p))
 :effect (and (assign (lifting ?h) no-crate) (assign (on ?c) ?s) (not (clear ?s)) (assign (in ?c) ?p)  (increase (total-cost) 1)))      

 	      
(:action move
 :parameters (?h - hoist ?from ?to - storearea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to) (clear ?to) )
 :effect (and (assign (at ?h) ?to) (clear ?from) (not (clear ?to)) (increase (total-cost) 2) ))

 
(:action go-out
 :parameters (?h - hoist ?from - storearea ?to - transitarea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to))
 :effect (and (assign (at ?h) ?to) (clear ?from)  (increase (total-cost) 3) ))

 
(:action go-in
 :parameters (?h - hoist ?from - transitarea ?to - storearea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to) (clear ?to)  )
 :effect (and  (assign (at ?h) ?to) (not (clear ?to))   (increase (total-cost) 3) ))
)

