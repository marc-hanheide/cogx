(define (scenario dora-test-world)

  (:domain dora-test-100)

  (:common
   (:objects  dora - robot
              human - human
              p1a p2a p1b p2b - place
              ra rb - room
              kitchen office living_room - category
              kitchen_pantry dining_room bathroom - category
              cornflakes table mug oven fridge book board-game - label
              obj1 obj2 obj3 obj4 obj5 obj6 obj7 - visualobject
              )

   (:init  (= (is-in dora)  p1a)

           (= (in-room p1a) ra)
           (= (in-room p2a) ra)
           (= (in-room p1b) rb)
           (= (in-room p2b) rb)
           (connected p1a p2a)
           (connected p2a p1b)
           (connected p1b p2b)
           (= (placestatus p2b) placeholder)

           (= (label obj1) cornflakes)
           (= (label obj2) table)
           (= (label obj3) mug)
           (= (label obj4) oven)
           (= (label obj5) fridge)
           (= (label obj6) book)
           (= (label obj7) board-game)

           (probabilistic  0.2000  (and  (assign (category ra) living_room)
                                         (probabilistic  0.6000  (and  (assign (ex-in-room mug ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj3) p1a)
                                                                                       0.4000  (assign (is-in obj3) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.3000  (assign (ex-in-room book ra) true))
                                         (probabilistic  0.7000  (and  (assign (ex-in-room table ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj2) p1a)
                                                                                       0.4000  (assign (is-in obj2) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (and  (assign (ex-in-room cornflakes ra) true)
                                                                       (probabilistic  0.5000  (assign (is-in obj1) p1a)
                                                                                       0.5000  (assign (is-in obj1) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.7000  (assign (ex-in-room board-game ra) true))
                                         )
                           0.7000  (and  (assign (category ra) office)
                                         (probabilistic  0.9000  (and  (assign (ex-in-room mug ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj3) p1a)
                                                                                       0.4000  (assign (is-in obj3) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.8000  (assign (ex-in-room book ra) true))
                                         (probabilistic  0.9000  (and  (assign (ex-in-room table ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj2) p1a)
                                                                                       0.4000  (assign (is-in obj2) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (and  (assign (ex-in-room cornflakes ra) true)
                                                                       (probabilistic  0.5000  (assign (is-in obj1) p1a)
                                                                                       0.5000  (assign (is-in obj1) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (assign (ex-in-room fridge ra) true))
                                         (probabilistic  0.2000  (assign (ex-in-room board-game ra) true))
                                         )
                           0.1000  (and  (assign (category ra) kitchen)
                                         (probabilistic  0.8000  (and  (assign (ex-in-room mug ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj3) p1a)
                                                                                       0.4000  (assign (is-in obj3) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.2000  (assign (ex-in-room book ra) true))
                                         (probabilistic  0.9000  (and  (assign (ex-in-room table ra) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj2) p1a)
                                                                                       0.4000  (assign (is-in obj2) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.8000  (and  (assign (ex-in-room cornflakes ra) true)
                                                                       (probabilistic  0.5000  (assign (is-in obj1) p1a)
                                                                                       0.5000  (assign (is-in obj1) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.7000  (assign (ex-in-room fridge ra) true))
                                         (probabilistic  0.8000  (and  (assign (ex-in-room oven ra) true)
                                                                       (probabilistic  0.1000  (assign (is-in obj4) p1a)
                                                                                       0.9000  (assign (is-in obj4) p2a)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (assign (ex-in-room board-game ra) true))
                                         )
                           )
           (probabilistic  0.1000  (assign (category rb) bathroom)
                           0.2000  (and  (assign (category rb) living_room)
                                         (probabilistic  0.7000  (and  (assign (ex-in-room table rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj2) p2b)
                                                                                       0.3000  (assign (is-in obj2) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.3000  (assign (ex-in-room book rb) true))
                                         (probabilistic  0.6000  (and  (assign (ex-in-room mug rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj3) p2b)
                                                                                       0.3000  (assign (is-in obj3) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.7000  (assign (ex-in-room board-game rb) true))
                                         (probabilistic  0.1000  (and  (assign (ex-in-room cornflakes rb) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj1) p2b)
                                                                                       0.4000  (assign (is-in obj1) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         )
                           0.2000  (and  (assign (category rb) office)
                                         (probabilistic  0.9000  (and  (assign (ex-in-room table rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj2) p2b)
                                                                                       0.3000  (assign (is-in obj2) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.8000  (assign (ex-in-room book rb) true))
                                         (probabilistic  0.9000  (and  (assign (ex-in-room mug rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj3) p2b)
                                                                                       0.3000  (assign (is-in obj3) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (assign (ex-in-room fridge rb) true))
                                         (probabilistic  0.1000  (and  (assign (ex-in-room cornflakes rb) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj1) p2b)
                                                                                       0.4000  (assign (is-in obj1) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.2000  (assign (ex-in-room board-game rb) true))
                                         )
                           0.5000  (and  (assign (category rb) kitchen)
                                         (probabilistic  0.9000  (and  (assign (ex-in-room table rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj2) p2b)
                                                                                       0.3000  (assign (is-in obj2) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.2000  (assign (ex-in-room book rb) true))
                                         (probabilistic  0.8000  (and  (assign (ex-in-room mug rb) true)
                                                                       (probabilistic  0.7000  (assign (is-in obj3) p2b)
                                                                                       0.3000  (assign (is-in obj3) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.7000  (assign (ex-in-room fridge rb) true))
                                         (probabilistic  0.8000  (and  (assign (ex-in-room cornflakes rb) true)
                                                                       (probabilistic  0.6000  (assign (is-in obj1) p2b)
                                                                                       0.4000  (assign (is-in obj1) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.8000  (and  (assign (ex-in-room oven rb) true)
                                                                       (probabilistic  0.2000  (assign (is-in obj4) p2b)
                                                                                       0.8000  (assign (is-in obj4) p1b)
                                                                                       )
                                                                       )
                                                         )
                                         (probabilistic  0.1000  (assign (ex-in-room board-game rb) true))
                                         )
                           )
           )
   )

  (:agent dora
          (:goal  (and (kval dora (is-in obj1))
                       )
                  )

          )
  )