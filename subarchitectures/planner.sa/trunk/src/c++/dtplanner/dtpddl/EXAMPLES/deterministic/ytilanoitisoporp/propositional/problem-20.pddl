(define (problem ytilanoitisoporp-1231946114 )
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
o15 - boolean
o16 - boolean
o17 - boolean
o18 - boolean
o19 - boolean
o20 - boolean
)
(:init 
(off o1 )
(= (switch-cost o1 ) 7 )
(related o1 o2 )
(off o2 )
(= (switch-cost o2 ) 2 )
(related o2 o3 )
(off o3 )
(= (switch-cost o3 ) 1 )
(related o3 o4 )
(off o4 )
(= (switch-cost o4 ) 5 )
(related o4 o5 )
(off o5 )
(= (switch-cost o5 ) 1 )
(related o5 o6 )
(off o6 )
(= (switch-cost o6 ) 4 )
(related o6 o7 )
(off o7 )
(= (switch-cost o7 ) 0 )
(related o7 o8 )
(off o8 )
(= (switch-cost o8 ) 4 )
(related o8 o9 )
(off o9 )
(= (switch-cost o9 ) 4 )
(related o9 o10 )
(off o10 )
(= (switch-cost o10 ) 5 )
(related o10 o11 )
(off o11 )
(= (switch-cost o11 ) 7 )
(related o11 o12 )
(off o12 )
(= (switch-cost o12 ) 8 )
(related o12 o13 )
(off o13 )
(= (switch-cost o13 ) 7 )
(related o13 o14 )
(off o14 )
(= (switch-cost o14 ) 1 )
(related o14 o15 )
(off o15 )
(= (switch-cost o15 ) 2 )
(related o15 o16 )
(off o16 )
(= (switch-cost o16 ) 4 )
(related o16 o17 )
(off o17 )
(= (switch-cost o17 ) 5 )
(related o17 o18 )
(off o18 )
(= (switch-cost o18 ) 9 )
(related o18 o19 )
(off o19 )
(= (switch-cost o19 ) 2 )
(related o19 o20 )
(off o20 )
(= (switch-cost o20 ) 6 )
(unrelated o20 )
(= (total-cost) 0)
)
(:goal 
(and 
(on o1 )
(on o2 )
(on o3 )
(on o4 )
(off o5 )
(off o6 )
(on o7 )
(off o8 )
(on o9 )
(off o10 )
(off o11 )
(on o12 )
(off o13 )
(on o14 )
(off o15 )
(on o16 )
(on o17 )
(off o18 )
(off o19 )
(on o20 )
)
)
 (:metric minimize (total-cost))
)
