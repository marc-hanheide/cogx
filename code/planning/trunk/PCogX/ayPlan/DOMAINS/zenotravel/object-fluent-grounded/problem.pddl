(define (problem zeno-example)

(:domain zeno-travel)

(:init (= (at-plane1) city1)
       (= (fuel-level-plane1) fl5)
       (= (at-plane2) city2)
       (= (fuel-level-plane2) fl0)
       (= (at-person1) city3)
       (= (at-person2) city0)
       (= (at-person3) city0)
       (= (at-person4) city1)

       (= (total-cost) 0))

(:goal (and (= (at-person1) city2)
            (= (at-person2) city3)
            (= (at-person3) city3)
            (= (at-person4) city3)))

(:metric minimize (total-cost))

)
