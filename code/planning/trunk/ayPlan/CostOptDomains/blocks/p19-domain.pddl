;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS)
  (:requirements :typing :action-costs)
 (:types block)

  (:predicates (on ?x - block  ?y - block )
	       (ontable ?x - block )
	       (clear ?x - block )
	       (handempty)
	       (holding ?x - block )
	       )

 (:functions
     (total-cost) - number
  )

  (:action pick-up
	     :parameters (?x - block )
	     :precondition (and (clear ?x) (ontable ?x) (handempty))
	     :effect
	     (and (not (ontable ?x))
		   (not (clear ?x))
		   (not (handempty))
		   (holding ?x)
		   (increase (total-cost) 1)
	     )
    )

  (:action put-down
	     :parameters (?x - block )
	     :precondition (holding ?x)
	     :effect
	     (and (not (holding ?x))
		   (clear ?x)
		   (handempty)
		   (ontable ?x)
		   (increase (total-cost) 1)
		 )
   )
		   
  (:action stack
	     :parameters (?x - block  ?y - block )
	     :precondition (and (holding ?x) (clear ?y))
	     :effect
	     (and (not (holding ?x))
		   (not (clear ?y))
		   (clear ?x)
		   (handempty)
		   (on ?x ?y)
		   (increase (total-cost) 1)
		  )
  )
  (:action unstack
	     :parameters (?x - block  ?y - block )
	     :precondition (and (on ?x ?y) (clear ?x) (handempty))
	     :effect
	     (and (holding ?x)
		   (clear ?y)
		   (not (clear ?x))
		   (not (handempty))
		   (not (on ?x ?y))
		   (increase (total-cost) 1)
		 )
  )
)
