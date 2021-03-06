;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS-object-fluents)
  (:requirements :typing :equality :action-costs :object-fluents) 
  (:types block)
  
  (:constants no-block - block)
  
  (:predicates (on-table ?x - block))

  (:functions
		(total-cost) - number
		(in-hand) - block
		(on-block ?x - block) - block) ;;what is in top of block ?x

  (:action pick-up
	     :parameters (?x - block)
	     :precondition (and (= (on-block ?x) no-block)
			(on-table ?x) (= (in-hand) no-block))
	     :effect
	     (and (not (on-table ?x))
		   (assign (in-hand) ?x)
		   (increase (total-cost) 2)))

  (:action put-down
	     :parameters (?x - block)
	     :precondition (= (in-hand) ?x)
	     :effect
	     (and (assign (in-hand) no-block)
		   (on-table ?x)
		   (increase (total-cost) 2)))
  
  (:action stack
	     :parameters (?x - block ?y - block)
	     :precondition (and (= (in-hand) ?x) (= (on-block ?y) no-block))
	     :effect
	     (and (assign (in-hand) no-block)
	   	  (assign (on-block ?y) ?x)
		  (increase (total-cost) 3)))

  (:action unstack
	     :parameters (?x - block ?y - block)
	     :precondition (and (= (on-block ?y) ?x)
			(= (on-block ?x) no-block) (= (in-hand) no-block))
	     :effect
	     (and (assign (in-hand) ?x)
		  (assign (on-block ?y) no-block)
		  (increase (total-cost) 3)))
)


;; EOF
