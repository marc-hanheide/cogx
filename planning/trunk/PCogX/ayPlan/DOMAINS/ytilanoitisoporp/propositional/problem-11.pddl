(define (problem ytilanoitisoporp-1231945671 )
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
o11 - boolean
)
(:init 
(off o1 )
(= (switch-cost o1 ) 8 )
(related o1 o2 )
(off o2 )
(= (switch-cost o2 ) 6 )
(related o2 o3 )
(off o3 )
(= (switch-cost o3 ) 5 )
(related o3 o4 )
(off o4 )
(= (switch-cost o4 ) 3 )
(related o4 o5 )
(off o5 )
(= (switch-cost o5 ) 0 )
(related o5 o6 )
(off o6 )
(= (switch-cost o6 ) 5 )
(related o6 o7 )
(off o7 )
(= (switch-cost o7 ) 5 )
(related o7 o8 )
(off o8 )
(= (switch-cost o8 ) 2 )
(related o8 o9 )
(off o9 )
(= (switch-cost o9 ) 5 )
(related o9 o10 )
(off o10 )
(= (switch-cost o10 ) 2 )
(related o10 o11 )
(off o11 )
(= (switch-cost o11 ) 9 )
(unrelated o11 )
(= (total-cost) 0)
)
(:goal 
(and 
(off o1 )
(on o2 )
(on o3 )
(on o4 )
(off o5 )
(on o6 )
(on o7 )
(on o8 )
(off o9 )
(on o10 )
(on o11 )
)
)
 (:metric minimize (total-cost))
)
