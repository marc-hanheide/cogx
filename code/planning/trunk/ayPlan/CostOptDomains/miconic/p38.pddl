(define (problem mixed-f16-p8-u0-v0-g0-a0-n0-A0-B0-N0-F0-r2)
   (:domain miconic)
   (:objects p0 p1 p2 p3 p4 p5 p6 p7
             f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 
             f10 f11 f12 f13 f14 f15 - block )


(:init
(= (total-cost) 0)
(passenger p0)
(passenger p1)
(passenger p2)
(passenger p3)
(passenger p4)
(passenger p5)
(passenger p6)
(passenger p7)
(floor f0)
(floor f1)
(floor f2)
(floor f3)
(floor f4)
(floor f5)
(floor f6)
(floor f7)
(floor f8)
(floor f9)
(floor f10)
(floor f11)
(floor f12)
(floor f13)
(floor f14)
(floor f15)

(above f0 f1)
(above f0 f2)
(above f0 f3)
(above f0 f4)
(above f0 f5)
(above f0 f6)
(above f0 f7)
(above f0 f8)
(above f0 f9)
(above f0 f10)
(above f0 f11)
(above f0 f12)
(above f0 f13)
(above f0 f14)
(above f0 f15)

(above f1 f2)
(above f1 f3)
(above f1 f4)
(above f1 f5)
(above f1 f6)
(above f1 f7)
(above f1 f8)
(above f1 f9)
(above f1 f10)
(above f1 f11)
(above f1 f12)
(above f1 f13)
(above f1 f14)
(above f1 f15)

(above f2 f3)
(above f2 f4)
(above f2 f5)
(above f2 f6)
(above f2 f7)
(above f2 f8)
(above f2 f9)
(above f2 f10)
(above f2 f11)
(above f2 f12)
(above f2 f13)
(above f2 f14)
(above f2 f15)

(above f3 f4)
(above f3 f5)
(above f3 f6)
(above f3 f7)
(above f3 f8)
(above f3 f9)
(above f3 f10)
(above f3 f11)
(above f3 f12)
(above f3 f13)
(above f3 f14)
(above f3 f15)

(above f4 f5)
(above f4 f6)
(above f4 f7)
(above f4 f8)
(above f4 f9)
(above f4 f10)
(above f4 f11)
(above f4 f12)
(above f4 f13)
(above f4 f14)
(above f4 f15)

(above f5 f6)
(above f5 f7)
(above f5 f8)
(above f5 f9)
(above f5 f10)
(above f5 f11)
(above f5 f12)
(above f5 f13)
(above f5 f14)
(above f5 f15)

(above f6 f7)
(above f6 f8)
(above f6 f9)
(above f6 f10)
(above f6 f11)
(above f6 f12)
(above f6 f13)
(above f6 f14)
(above f6 f15)

(above f7 f8)
(above f7 f9)
(above f7 f10)
(above f7 f11)
(above f7 f12)
(above f7 f13)
(above f7 f14)
(above f7 f15)

(above f8 f9)
(above f8 f10)
(above f8 f11)
(above f8 f12)
(above f8 f13)
(above f8 f14)
(above f8 f15)

(above f9 f10)
(above f9 f11)
(above f9 f12)
(above f9 f13)
(above f9 f14)
(above f9 f15)

(above f10 f11)
(above f10 f12)
(above f10 f13)
(above f10 f14)
(above f10 f15)

(above f11 f12)
(above f11 f13)
(above f11 f14)
(above f11 f15)

(above f12 f13)
(above f12 f14)
(above f12 f15)

(above f13 f14)
(above f13 f15)

(above f14 f15)



(origin p0 f11)
(destin p0 f0)

(origin p1 f4)
(destin p1 f10)

(origin p2 f9)
(destin p2 f14)

(origin p3 f6)
(destin p3 f14)

(origin p4 f15)
(destin p4 f7)

(origin p5 f10)
(destin p5 f11)

(origin p6 f15)
(destin p6 f2)

(origin p7 f2)
(destin p7 f14)






(lift-at f0)
)


(:goal (and 
(served p0)
(served p1)
(served p2)
(served p3)
(served p4)
(served p5)
(served p6)
(served p7)
))
 (:metric minimize (total-cost))
)



