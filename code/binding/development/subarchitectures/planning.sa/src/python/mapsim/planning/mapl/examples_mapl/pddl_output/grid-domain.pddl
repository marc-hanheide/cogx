(define (domain pml)
(:requirements :typing)

(:types 
   agentlike number gridcontent - object
   agent static_agent - agentlike
   self_agent - agent
   movable - gridcontent
   boolean - object
)

(:constants
   empty obstacle - gridcontent
   n0 - number
   true false - boolean
   agt0 - self_agent
)

(:predicates
  (timeout ?agt - agentlike ?t - number)
  (succ ?small ?big - number)
  (pos ?obj - movable ?x ?y - number)
  (occupant ?x ?y - number ?occupant - (either agentlike gridcontent))
  (touching ?o1 ?o2 - movable)
; automatically generated belief state variables
  (k_occupant ?a - agent ?x ?y - number)  
  (k_pos ?a - agent ?obj - movable)
)

;;; sensors ;;;




;;; actions ;;;

(:action __activate__
 :parameters (?a - agent ?t ?tn - number)
 :precondition (and 
	(timeout ?a ?t) (succ ?tn ?t))
 :effect (and
	(not (timeout ?a ?t)) (timeout ?a ?tn)) 
)

(:action push
 :parameters (?a - agent ?o - movable ?ox ?oy ?nx ?ny - number)
 :precondition (and
	(timeout ?a n0)
	(occupant ?ox ?oy ?o) (occupant ?nx ?ny empty)
	(or (and (succ ?nx ?ox) (= ?oy ?ny)) 
            (and (succ ?ox ?nx) (= ?oy ?ny))
            (and (succ ?ny ?oy) (= ?ox ?nx)) 
            (and (succ ?oy ?ny) (= ?ox ?nx)))) 
 :effect (and
	(occupant ?ox ?oy empty) (not (occupant ?ox ?oy ?o))
	(occupant ?nx ?ny ?o) (not (occupant ?nx ?ny empty))
	(pos ?o ?nx ?ny) (not (pos ?o ?ox ?oy))
	(forall (?o2 - movable)
		(when (touching ?o ?o2)
		      (and (not (touching ?o ?o2)) 
			   (not (touching ?o2 ?o)))))
))

(:action fine_push
 :parameters (?a - agent ?o1 ?o2 - movable ?x1 ?y1 ?x2 ?y2 - number)
 :precondition (and 
	(timeout ?a n0)
	(pos ?o1 ?x1 ?y1) (pos ?o2 ?x2 ?y2)
	(or (and (succ ?x1 ?x2) (= ?y2 ?y1)) 
            (and (succ ?x2 ?x1) (= ?y2 ?y1))
            (and (succ ?y1 ?y2) (= ?x2 ?x1)) 
            (and (succ ?y2 ?y1) (= ?x2 ?x1))))
 :effect (and
	(touching ?o1 ?o2)
	(touching ?o2 ?o1)
))

)