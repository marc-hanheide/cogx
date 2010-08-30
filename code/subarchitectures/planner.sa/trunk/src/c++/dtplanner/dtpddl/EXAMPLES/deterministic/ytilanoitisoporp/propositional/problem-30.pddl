(define (problem ytilanoitisoporp-1231945698 )
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
o21 - boolean
o22 - boolean
o23 - boolean
o24 - boolean
o25 - boolean
o26 - boolean
o27 - boolean
o28 - boolean
o29 - boolean
o30 - boolean
)
(:init 
(off o1 )
(= (switch-cost o1 ) 3 )
(related o1 o2 )
(off o2 )
(= (switch-cost o2 ) 2 )
(related o2 o3 )
(off o3 )
(= (switch-cost o3 ) 5 )
(related o3 o4 )
(off o4 )
(= (switch-cost o4 ) 6 )
(related o4 o5 )
(off o5 )
(= (switch-cost o5 ) 7 )
(related o5 o6 )
(off o6 )
(= (switch-cost o6 ) 8 )
(related o6 o7 )
(off o7 )
(= (switch-cost o7 ) 0 )
(related o7 o8 )
(off o8 )
(= (switch-cost o8 ) 9 )
(related o8 o9 )
(off o9 )
(= (switch-cost o9 ) 8 )
(related o9 o10 )
(off o10 )
(= (switch-cost o10 ) 5 )
(related o10 o11 )
(off o11 )
(= (switch-cost o11 ) 9 )
(related o11 o12 )
(off o12 )
(= (switch-cost o12 ) 9 )
(related o12 o13 )
(off o13 )
(= (switch-cost o13 ) 7 )
(related o13 o14 )
(off o14 )
(= (switch-cost o14 ) 4 )
(related o14 o15 )
(off o15 )
(= (switch-cost o15 ) 6 )
(related o15 o16 )
(off o16 )
(= (switch-cost o16 ) 5 )
(related o16 o17 )
(off o17 )
(= (switch-cost o17 ) 1 )
(related o17 o18 )
(off o18 )
(= (switch-cost o18 ) 7 )
(related o18 o19 )
(off o19 )
(= (switch-cost o19 ) 4 )
(related o19 o20 )
(off o20 )
(= (switch-cost o20 ) 0 )
(related o20 o21 )
(off o21 )
(= (switch-cost o21 ) 3 )
(related o21 o22 )
(off o22 )
(= (switch-cost o22 ) 5 )
(related o22 o23 )
(off o23 )
(= (switch-cost o23 ) 9 )
(related o23 o24 )
(off o24 )
(= (switch-cost o24 ) 6 )
(related o24 o25 )
(off o25 )
(= (switch-cost o25 ) 7 )
(related o25 o26 )
(off o26 )
(= (switch-cost o26 ) 6 )
(related o26 o27 )
(off o27 )
(= (switch-cost o27 ) 6 )
(related o27 o28 )
(off o28 )
(= (switch-cost o28 ) 9 )
(related o28 o29 )
(off o29 )
(= (switch-cost o29 ) 6 )
(related o29 o30 )
(off o30 )
(= (switch-cost o30 ) 1 )
(unrelated o30 )
(= (total-cost) 0)
)
(:goal 
(and 
(on o1 )
(off o2 )
(off o3 )
(off o4 )
(on o5 )
(off o6 )
(on o7 )
(on o8 )
(on o9 )
(on o10 )
(on o11 )
(off o12 )
(off o13 )
(off o14 )
(on o15 )
(off o16 )
(on o17 )
(off o18 )
(off o19 )
(on o20 )
(off o21 )
(on o22 )
(on o23 )
(on o24 )
(on o25 )
(off o26 )
(on o27 )
(on o28 )
(on o29 )
(on o30 )
)
)
 (:metric minimize (total-cost))
)
