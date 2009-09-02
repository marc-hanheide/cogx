;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS-object-fluents)
  (:requirements :typing :equality :durative-actions :action-costs
	         :goal-utilities :object-fluents)

  (:types block)
  
  (:constants no-block - block)
  
  (:predicates (on-table ?x - block))

  (:functions
		(total-cost) - number
		(in-hand) - block
		(on-block ?x - block) - block) ;;what is in top of block ?x

  (:durative-action pick-up
	     :parameters (?x - block)
	     :duration (= ?duration 3)
	     :condition (and (over all (= (on-block ?x) no-block))
			(at start (on-table ?x))
			(at start (= (in-hand) no-block)))
	     :effect
	     (and (at start (not (on-table ?x)))
		(change (in-hand) ?x)
		 (increase (total-cost) 2)))

  (:durative-action put-down
	     :parameters (?x - block)
	     :duration (= ?duration 3)
	     :condition (at start (= (in-hand) ?x))
	     :effect
	     (and (change (in-hand) no-block)
		   (at end (on-table ?x))
		 (increase (total-cost) 2)))
  
  (:durative-action stack
	     :parameters (?x - block ?y - block)
	     :duration (= ?duration 2)
	     :condition (and (at start (= (in-hand) ?x))
			(at start (= (on-block ?y) no-block)))
	     :effect
	     (and (change (in-hand) no-block)
	   	(change (on-block ?y) ?x)
		 (increase (total-cost) 3)))

  (:durative-action unstack
	     :parameters (?x - block ?y - block)
	     :duration (= ?duration 2)
	     :condition (and (at start (= (on-block ?y) ?x))
			(over all (= (on-block ?x) no-block))
			(at start (= (in-hand) no-block)))
	     :effect
	     (and (change (in-hand) ?x)
		(change (on-block ?y) no-block)
		 (increase (total-cost) 3)))
)


;; EOF
