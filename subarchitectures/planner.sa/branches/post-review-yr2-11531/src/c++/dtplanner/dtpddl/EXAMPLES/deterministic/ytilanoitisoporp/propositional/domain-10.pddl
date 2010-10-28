(define (domain ytilanoitisoporp)
(:requirements :action-costs :typing)
(:types boolean - object)
(:predicates (on ?b - boolean)
(off ?b - boolean)
(related ?b1 ?b2 - boolean)
(unrelated ?b - boolean))
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
(:action unrelated-switch-on
:parameters (?b - boolean)
:precondition (and (off ?b) (unrelated ?b) )
:effect (and (not (off ?b)) 
(on ?b)
(increase (total-cost) (switch-cost ?b))
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
:parameters ()
:precondition ()
:effect (and 
(on o1 )
(not (off o1 ))
(on o2 )
(not (off o2 ))
(not (on o3 ))
(off o3 )
(on o4 )
(not (off o4 ))
(not (on o5 ))
(off o5 )
(on o6 )
(not (off o6 ))
(not (on o7 ))
(off o7 )
(on o8 )
(not (off o8 ))
(on o9 )
(not (off o9 ))
(not (on o10 ))
(off o10 )
(increase (total-cost) 10000))) 
)
