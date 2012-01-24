(define (problem ytilanoitisoporp-1211325368 )
(:domain ytilanoitisoporp)
(:objects 
o1 - boolean
o2 - boolean
o3 - boolean
o4 - boolean
o5 - boolean
o6 - boolean
o7 - boolean
o8 - boolean
o9 - boolean
o10 - boolean
)
(:init 
(off o1 )
(= (switch-cost o1 ) 682 )
(related o1 o2 )
(off o2 )
(= (switch-cost o2 ) 283 )
(related o2 o3 )
(off o3 )
(= (switch-cost o3 ) 760 )
(related o3 o4 )
(off o4 )
(= (switch-cost o4 ) 671 )
(related o4 o5 )
(off o5 )
(= (switch-cost o5 ) 228 )
(related o5 o6 )
(off o6 )
(= (switch-cost o6 ) 710 )
(related o6 o7 )
(off o7 )
(= (switch-cost o7 ) 32 )
(related o7 o8 )
(off o8 )
(= (switch-cost o8 ) 11 )
(related o8 o9 )
(off o9 )
(= (switch-cost o9 ) 193 )
(related o9 o10 )
(off o10 )
(= (switch-cost o10 ) 286 )
(unrelated o10 )
(= (total-cost) 0)
)
(:goal 
(and 
(on o1 )
(on o2 )
(off o3 )
(on o4 )
(off o5 )
(on o6 )
(off o7 )
(on o8 )
(on o9 )
(off o10 )
)
)
 (:metric minimize (total-cost))
)
