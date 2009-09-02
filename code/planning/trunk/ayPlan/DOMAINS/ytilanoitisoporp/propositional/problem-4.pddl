(define (problem ytilanoitisoporp-1211257576 )
  (:domain ytilanoitisoporp)
  (:objects 
   o1 - boolean
   o2 - boolean
   o3 - boolean
   o4 - boolean
   )
  (:init 
   (off o1 )
   (related o1 o2 )
   (off o2 )
   (related o2 o3 )
   (off o3 )
   (related o3 o4 )
   (off o4 )
   (= (switch-cost o1 ) 41 )
   (= (switch-cost o2 ) 815 )
   (= (switch-cost o3 ) 137 )
   (= (switch-cost o4 ) 609 )
   (= (total-cost) 0)
   )
  (:goal 
   (and 
    (off o1 )
    (on o2 )
    (off o3 )
    (off o4 )
    )
   )
  (:metric minimize (total-cost))
  )
