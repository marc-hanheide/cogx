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
          city3 - city
          fl0 - fuel-amount
          fl1 - fuel-amount
          fl2 - fuel-amount
          fl3 - fuel-amount
          fl4 - fuel-amount
          fl5 - fuel-amount
          fl6 - fuel-amount)

(:init (= (at plane1) city1)
       (= (fuel-level plane1) fl5)
       (= (at plane2) city2)
       (= (fuel-level plane2) fl0)
       (= (at person1) city3)
       (= (at person2) city0)
       (= (at person3) city0)
       (= (at person4) city1)
       (= (next fl0) fl1)
       (= (next fl1) fl2)
       (= (next fl2) fl3)
       (= (next fl3) fl4)
       (= (next fl4) fl5)
       (= (next fl5) fl6)

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

(:goal (and (= (at person1) city2)
            (= (at person2) city3)
            (= (at person3) city3)
            (= (at person4) city3)))

(:metric minimize (total-cost))

)
