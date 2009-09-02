(define (problem zeno-example)

(:domain zeno-travel)

(:init (at-plane1-city1)
       (= (fuel-level-plane1) 100)
       (at-plane2-city2)
       (= (fuel-level-plane2) 100)
       (at-person1-city3)
       (at-person2-city0)
       (at-person3-city0)
       (at-person4-city1)
       (= (total-cost) 0))

(:goal (and (preference g0 (at-person1-city2))
            (preference g1 (at-person2-city3))
            (preference g2 (at-person3-city3))
            (preference g3 (at-person4-city3))))

(:metric maximize
         (- 600
            (+ (* (is-violated g0) 175)
               (* (is-violated g1) 75)
               (* (is-violated g2) 125)
               (* (is-violated g3) 225))
            (total-cost)))

)
