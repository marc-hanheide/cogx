;; Action costs made evil.

(define (domain ytilanoitisoporp)

(:requirements :action-costs :typing)


(:types boolean - object)

(:predicates (on ?b - boolean)
             (off ?b - boolean)
	     (related ?b1 ?b2 - boolean))



(:functions (total-cost) - number
            (switch-cost ?b - boolean) - number )



(:action switch-on
         :parameters (?b1 ?b2 - boolean)
         :precondition (and (off ?b1) (related ?b1 ?b2) )
         :effect (and (not (off ?b1)) 
		      (off ?b2)
		      (not (on ?b2))
                      (on ?b1)
		      (increase (total-cost) (switch-cost ?b1))
		      )
)

(:action switch-off
         :parameters (?b - boolean)
         :precondition (and (on ?b) )
         :effect (and (not (on ?b))
                      (off ?b)
		      (increase (total-cost) (switch-cost ?b))
		      )
)

(:action super-switch
         :parameters (?b - boolean)
         :precondition (and (on ?b) )
         :effect (and (on ?b)
                      (not (off ?b))
		      (increase (total-cost) 100000))
)


)
