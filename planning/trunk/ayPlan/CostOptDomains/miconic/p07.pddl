


(define (problem mixed-f4-p2-u0-v0-g0-a0-n0-A0-B0-N0-F0-r1)
   (:domain miconic)
   (:objects p0 p1
             f0 f1 f2 f3 - block )


(:init
(= (total-cost) 0)
(passenger p0)
(passenger p1)
(floor f0)
(floor f1)
(floor f2)
(floor f3)

(above f0 f1)
(above f0 f2)
(above f0 f3)

(above f1 f2)
(above f1 f3)

(above f2 f3)



(origin p0 f0)
(destin p0 f1)

(origin p1 f3)
(destin p1 f0)






(lift-at f0)
)


(:goal (and 
(served p0)
(served p1)
))
 (:metric minimize (total-cost))
)



