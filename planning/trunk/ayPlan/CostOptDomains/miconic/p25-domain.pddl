(define (domain miconic)
(:requirements :typing :action-costs)
 (:types block)

  

(:predicates 
(origin ?person - block  ?floor - block  )
;; entry of ?person is ?floor
;; inertia

(floor ?floor - block )
(passenger ?passenger - block )

(destin ?person - block  ?floor - block  )
;; exit of ?person is ?floor
;; inertia

(above ?floor1 - block  ?floor2 - block  )
;; ?floor2 is located above of ?floor1

(boarded ?person - block  )
;; true if ?person has boarded the lift

(served ?person - block  )
;; true if ?person has alighted as her destination

(lift-at ?floor - block  )
;; current position of the lift is at ?floor
)


 
 (:functions
     (total-cost) - number
  )



;;stop and allow boarding

(:action board
  :parameters (?f - block  ?p - block )
  :precondition (and (floor ?f) (passenger ?p)(lift-at ?f) (origin ?p ?f))
  :effect (and (boarded ?p)
  (increase (total-cost) 1)
  )
  
  )

(:action depart
  :parameters (?f - block   ?p - block )
  :precondition (and (floor ?f) (passenger ?p) (lift-at ?f) (destin ?p ?f)
		     (boarded ?p))
  :effect (and (not (boarded ?p))
	       (served ?p)
	       (increase (total-cost) 1))
	       )
;;drive up

(:action up
  :parameters (?f1 - block  ?f2 - block )
  :precondition (and (floor ?f1) (floor ?f2) (lift-at ?f1) (above ?f1 ?f2))
  :effect (and (lift-at ?f2) (not (lift-at ?f1))
  (increase (total-cost) 1))
  )


;;drive down

(:action down
  :parameters (?f1 - block  ?f2 - block )
  :precondition (and (floor ?f1) (floor ?f2) (lift-at ?f1) (above ?f2 ?f1))
  :effect (and (lift-at ?f2) (not (lift-at ?f1))
  (increase (total-cost) 1)
  )
  )
)




