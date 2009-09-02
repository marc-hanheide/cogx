; IPC5 Domain: Storage Propositional
; Authors: Alfonso Gerevini and Alessandro Saetti 
; Adapted for PDDL 3.1 by Ioannis Refanidis

(define (domain Storage-Sequential-ObjectFluents)

(:requirements :typing  :object-fluents :equality)

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
) 

(:action lift
 :parameters (?h - hoist ?s - storearea ?c - crate  ?p - place)
 :precondition (and (= (on ?c) ?s) (inside ?s ?p) (= (in ?c) ?p) (connected ?s (at ?h)) (= (lifting ?h) no-crate) )
 :effect (and (clear ?s) (assign (on ?c) no-area) (assign (lifting ?h) ?c) (assign (in ?c) no-place) ))
				
 
(:action drop
 :parameters (?h - hoist ?c - crate ?s - storearea ?p - place)
 :precondition (and (connected ?s (at ?h))  (= (lifting ?h) ?c) (= (on ?c) no-area) (clear ?s) (inside ?s ?p))
 :effect (and (assign (lifting ?h) no-crate) (assign (on ?c) ?s) (not (clear ?s)) (assign (in ?c) ?p)  ))   

 	      
(:action move
 :parameters (?h - hoist ?from ?to - storearea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to) (clear ?to) )
 :effect (and (assign (at ?h) ?to) (clear ?from) (not (clear ?to)) ))

 
(:action go-out
 :parameters (?h - hoist ?from - storearea ?to - transitarea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to))
 :effect (and (assign (at ?h) ?to) (clear ?from)  ))

 
(:action go-in
 :parameters (?h - hoist ?from - transitarea ?to - storearea)
 :precondition (and (= (at ?h) ?from) (connected ?from ?to) (clear ?to)  )
 :effect (and  (assign (at ?h) ?to) (not (clear ?to))   ))
)

