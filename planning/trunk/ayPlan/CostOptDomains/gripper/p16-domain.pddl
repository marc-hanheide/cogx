(define (domain gripper-strips)
(:requirements :typing :action-costs)
(:types block)
   (:predicates (room ?r - block )
		(ball ?b - block )
		(gripper ?g - block )
		(at-robby ?r - block )
		(at ?b - block  ?r - block )
		(free ?g - block )
		(carry ?o - block  ?g - block ))

 (:functions
     (total-cost) - number
  )
  
  
   (:action move
       :parameters  (?from - block  ?to - block )
       :precondition (and  (room ?from) (room ?to) (at-robby ?from))
       :effect (and  (at-robby ?to)
		     (not (at-robby ?from))
		     (increase (total-cost) 1))
		     )



   (:action pick
       :parameters (?obj - block  ?room - block  ?gripper - block )
       :precondition  (and  (ball ?obj) (room ?room) (gripper ?gripper)
			    (at ?obj ?room) (at-robby ?room) (free ?gripper))
       :effect (and (carry ?obj ?gripper)
		    (not (at ?obj ?room)) 
		    (not (free ?gripper))
		    (increase (total-cost) 1))
		    )


   (:action drop
       :parameters  (?obj - block   ?room - block  ?gripper - block )
       :precondition  (and  (ball ?obj) (room ?room) (gripper ?gripper)
			    (carry ?obj ?gripper) (at-robby ?room))
       :effect (and (at ?obj ?room)
		    (free ?gripper)
		    (not (carry ?obj ?gripper))
		    (increase (total-cost) 1))
		    )
		    
		    )


