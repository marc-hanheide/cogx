(define (problem ytilanoitisoporp-1231945674 )
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
o12 - boolean
o13 - boolean
o14 - boolean
)
(:init 
(off o1 )
(= (switch-cost o1 ) 4 )
(related o1 o2 )
(off o2 )
(= (switch-cost o2 ) 6 )
(related o2 o3 )
(off o3 )
(= (switch-cost o3 ) 1 )
(related o3 o4 )
(off o4 )
(= (switch-cost o4 ) 7 )
(related o4 o5 )
(off o5 )
(= (switch-cost o5 ) 1 )
(related o5 o6 )
(off o6 )
(= (switch-cost o6 ) 0 )
(related o6 o7 )
(off o7 )
(= (switch-cost o7 ) 5 )
(related o7 o8 )
(off o8 )
(= (switch-cost o8 ) 1 )
(related o8 o9 )
(off o9 )
(= (switch-cost o9 ) 3 )
(related o9 o10 )
(off o10 )
(= (switch-cost o10 ) 6 )
(related o10 o11 )
(off o11 )
(= (switch-cost o11 ) 3 )
(related o11 o12 )
(off o12 )
(= (switch-cost o12 ) 1 )
(related o12 o13 )
(off o13 )
(= (switch-cost o13 ) 1 )
(related o13 o14 )
(off o14 )
(= (switch-cost o14 ) 0 )
(unrelated o14 )
(= (total-cost) 0)
)
(:goal 
(and 
(on o1 )
(on o2 )
(on o3 )
(on o4 )
(on o5 )
(off o6 )
(on o7 )
(on o8 )
(off o9 )
(off o10 )
(off o11 )
(off o12 )
(on o13 )
(off o14 )
)
)
 (:metric minimize (total-cost))
)
