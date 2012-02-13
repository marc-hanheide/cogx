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
(on o3 )
(not (off o3 ))
(on o4 )
(not (off o4 ))
(on o5 )
(not (off o5 ))
(not (on o6 ))
(off o6 )
(not (on o7 ))
(off o7 )
(not (on o8 ))
(off o8 )
(on o9 )
(not (off o9 ))
(on o10 )
(not (off o10 ))
(on o11 )
(not (off o11 ))
(not (on o12 ))
(off o12 )
(not (on o13 ))
(off o13 )
(not (on o14 ))
(off o14 )
(on o15 )
(not (off o15 ))
(not (on o16 ))
(off o16 )
(not (on o17 ))
(off o17 )
(on o18 )
(not (off o18 ))
(increase (total-cost) 100000))) 
)