(define (problem zeno-example)

(:domain zeno-travel)

(:objects plane1 - aircraft
          plane2 - aircraft
          person1 - person
          person2 - person
          person3 - person
          person4 - person
          city0 - city
          city1 - city
          city2 - city
          city3 - city)

(:init (at plane1 city1)
       (= (fuel-level plane1) 100)
       (at plane2 city2)
       (= (fuel-level plane2) 100)
       (at person1 city3)
       (at person2 city0)
       (at person3 city0)
       (at person4 city1)
       (next fl0 fl1)
       (next fl1 fl2)
       (next fl2 fl3)
       (next fl3 fl4)
       (next fl4 fl5)
       (next fl5 fl6)

       (= (fly-fuel city0 city0) 0)
       (= (fly-fuel city1 city1) 0)
       (= (fly-fuel city2 city2) 0)
       (= (fly-fuel city3 city3) 0)
       (= (fly-fuel city0 city1) 30)
       (= (fly-fuel city1 city0) 30)
       (= (fly-fuel city0 city2) 15)
       (= (fly-fuel city2 city0) 15)
       (= (fly-fuel city0 city3) 19)
       (= (fly-fuel city3 city0) 19)
       (= (fly-fuel city1 city2) 12)
       (= (fly-fuel city2 city1) 12)
       (= (fly-fuel city1 city3) 18)
       (= (fly-fuel city3 city1) 18)
       (= (fly-fuel city2 city3) 20)
       (= (fly-fuel city3 city2) 20)

       (= (zoom-fuel city0 city0) 0)
       (= (zoom-fuel city1 city1) 0)
       (= (zoom-fuel city2 city2) 0)
       (= (zoom-fuel city3 city3) 0)
       (= (zoom-fuel city0 city1) 55)
       (= (zoom-fuel city1 city0) 55)
       (= (zoom-fuel city0 city2) 33)
       (= (zoom-fuel city2 city0) 33)
       (= (zoom-fuel city0 city3) 44)
       (= (zoom-fuel city3 city0) 44)
       (= (zoom-fuel city1 city2) 30)
       (= (zoom-fuel city2 city1) 30)
       (= (zoom-fuel city1 city3) 42)
       (= (zoom-fuel city3 city1) 42)
       (= (zoom-fuel city2 city3) 50)
       (= (zoom-fuel city3 city2) 50)

       (= (fly-cost city0 city0) 0)
       (= (fly-cost city1 city1) 0)
       (= (fly-cost city2 city2) 0)
       (= (fly-cost city3 city3) 0)
       (= (fly-cost city0 city1) 140)
       (= (fly-cost city1 city0) 140)
       (= (fly-cost city0 city2) 70)
       (= (fly-cost city2 city0) 70)
       (= (fly-cost city0 city3) 131)
       (= (fly-cost city3 city0) 131)
       (= (fly-cost city1 city2) 124)
       (= (fly-cost city2 city1) 124)
       (= (fly-cost city1 city3) 88)
       (= (fly-cost city3 city1) 88)
       (= (fly-cost city2 city3) 201)
       (= (fly-cost city3 city2) 201)

       (= (zoom-cost city0 city0) 0)
       (= (zoom-cost city1 city1) 0)
       (= (zoom-cost city2 city2) 0)
       (= (zoom-cost city3 city3) 0)
       (= (zoom-cost city0 city1) 88)
       (= (zoom-cost city1 city0) 88)
       (= (zoom-cost city0 city2) 44)
       (= (zoom-cost city2 city0) 44)
       (= (zoom-cost city0 city3) 100)
       (= (zoom-cost city3 city0) 100)
       (= (zoom-cost city1 city2) 94)
       (= (zoom-cost city2 city1) 94)
       (= (zoom-cost city1 city3) 55)
       (= (zoom-cost city3 city1) 55)
       (= (zoom-cost city2 city3) 130)
       (= (zoom-cost city3 city2) 130)

       (= (total-cost) 0))

(:goal (and (preference g0 (at person1 city2))
            (preference g1 (at person2 city3))
            (preference g2 (at person3 city3))
            (preference g3 (at person4 city3))))

(:metric maximize
         (- 600
            (+ (* (is-violated g0) 175)
               (* (is-violated g1) 75)
               (* (is-violated g2) 125)
               (* (is-violated g3) 225))
            (total-cost)))

)
