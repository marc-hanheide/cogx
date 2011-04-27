;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS)
  (:requirements :strips :typing :action-costs :goal-utilities)
  (:types block)
  (:predicates (on ?x - block ?y - block)
	       (on-table ?x - block)
	       (clear ?x - block)
	       (handempty)
	       (holding ?x - block)
	       )

  (:functions (total-cost) - number)

  (:action pick-up
	     :parameters (?x - block)
	     :precondition (and (clear ?x) (on-table ?x) (handempty))
	     :effect
	     (and (not (on-table ?x))
		   (not (clear ?x))
		   (not (handempty))
		   (holding ?x)
		(increase (total-cost) 2)))

  (:action put-down
	     :parameters (?x - block)
	     :precondition (holding ?x)
	     :effect
	     (and (not (holding ?x))
		   (clear ?x)
		   (handempty)
		   (on-table ?x)
		(increase (total-cost) 5)))
  (:action stack
	     :parameters (?x - block ?y - block)
	     :precondition (and (holding ?x) (clear ?y))
	     :effect
	     (and (not (holding ?x))
		   (not (clear ?y))
		   (clear ?x)
		   (handempty)
		   (on ?x ?y)
		(increase (total-cost) 3)))

  (:action unstack
	     :parameters (?x - block ?y - block)
	     :precondition (and (on ?x ?y) (clear ?x) (handempty))
	     :effect
	     (and (holding ?x)
		   (clear ?y)
		   (not (clear ?x))
		   (not (handempty))
		   (not (on ?x ?y))
		   (increase (total-cost) 2)))
)
