;; Action costs version of the IPC-2002 Zenotravel domain.

;; The encoding is somewhat strange because the costs of
;; flying/zooming from city to city differs for different pairs of
;; cities, but the fuel usage does not. But since this is only
;; intended as a test domain, this shouldn't be much of an issue.

(define (domain zeno-travel)

(:requirements :action-costs :object-fluents)

(:constants
    plane1
    plane2
    person1
    person2
    person3
    person4
    city0
    city1
    city2
    city3
    fl0
    fl1
    fl2
    fl3
    fl4
    fl5
    fl6
)

(:functions
     (at-plane1) - object
     (at-plane2) - object
     (at-person1) - object
     (at-person2) - object
     (at-person3) - object
     (at-person4) - object
     (in-person1) - object
     (in-person2) - object
     (in-person3) - object
     (in-person4) - object
     (fuel-level-plane1) - object
     (fuel-level-plane2) - object
     (total-cost) - number
)

(:action board-person1-plane1-city0
         :parameters ()
         :precondition (and (= (at-person1) city0)
                            (= (at-plane1) city0))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane1)))

(:action board-person1-plane1-city1
         :parameters ()
         :precondition (and (= (at-person1) city1)
                            (= (at-plane1) city1))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane1)))

(:action board-person1-plane1-city2
         :parameters ()
         :precondition (and (= (at-person1) city2)
                            (= (at-plane1) city2))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane1)))

(:action board-person1-plane1-city3
         :parameters ()
         :precondition (and (= (at-person1) city3)
                            (= (at-plane1) city3))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane1)))

(:action board-person1-plane2-city0
         :parameters ()
         :precondition (and (= (at-person1) city0)
                            (= (at-plane2) city0))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane2)))

(:action board-person1-plane2-city1
         :parameters ()
         :precondition (and (= (at-person1) city1)
                            (= (at-plane2) city1))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane2)))

(:action board-person1-plane2-city2
         :parameters ()
         :precondition (and (= (at-person1) city2)
                            (= (at-plane2) city2))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane2)))

(:action board-person1-plane2-city3
         :parameters ()
         :precondition (and (= (at-person1) city3)
                            (= (at-plane2) city3))
         :effect (and (assign (at-person1) undefined)
                      (assign (in-person1) plane2)))

(:action board-person2-plane1-city0
         :parameters ()
         :precondition (and (= (at-person2) city0)
                            (= (at-plane1) city0))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane1)))

(:action board-person2-plane1-city1
         :parameters ()
         :precondition (and (= (at-person2) city1)
                            (= (at-plane1) city1))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane1)))

(:action board-person2-plane1-city2
         :parameters ()
         :precondition (and (= (at-person2) city2)
                            (= (at-plane1) city2))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane1)))

(:action board-person2-plane1-city3
         :parameters ()
         :precondition (and (= (at-person2) city3)
                            (= (at-plane1) city3))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane1)))

(:action board-person2-plane2-city0
         :parameters ()
         :precondition (and (= (at-person2) city0)
                            (= (at-plane2) city0))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane2)))

(:action board-person2-plane2-city1
         :parameters ()
         :precondition (and (= (at-person2) city1)
                            (= (at-plane2) city1))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane2)))

(:action board-person2-plane2-city2
         :parameters ()
         :precondition (and (= (at-person2) city2)
                            (= (at-plane2) city2))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane2)))

(:action board-person2-plane2-city3
         :parameters ()
         :precondition (and (= (at-person2) city3)
                            (= (at-plane2) city3))
         :effect (and (assign (at-person2) undefined)
                      (assign (in-person2) plane2)))

(:action board-person3-plane1-city0
         :parameters ()
         :precondition (and (= (at-person3) city0)
                            (= (at-plane1) city0))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane1)))

(:action board-person3-plane1-city1
         :parameters ()
         :precondition (and (= (at-person3) city1)
                            (= (at-plane1) city1))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane1)))

(:action board-person3-plane1-city2
         :parameters ()
         :precondition (and (= (at-person3) city2)
                            (= (at-plane1) city2))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane1)))

(:action board-person3-plane1-city3
         :parameters ()
         :precondition (and (= (at-person3) city3)
                            (= (at-plane1) city3))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane1)))

(:action board-person3-plane2-city0
         :parameters ()
         :precondition (and (= (at-person3) city0)
                            (= (at-plane2) city0))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane2)))

(:action board-person3-plane2-city1
         :parameters ()
         :precondition (and (= (at-person3) city1)
                            (= (at-plane2) city1))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane2)))

(:action board-person3-plane2-city2
         :parameters ()
         :precondition (and (= (at-person3) city2)
                            (= (at-plane2) city2))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane2)))

(:action board-person3-plane2-city3
         :parameters ()
         :precondition (and (= (at-person3) city3)
                            (= (at-plane2) city3))
         :effect (and (assign (at-person3) undefined)
                      (assign (in-person3) plane2)))

(:action board-person4-plane1-city0
         :parameters ()
         :precondition (and (= (at-person4) city0)
                            (= (at-plane1) city0))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane1)))

(:action board-person4-plane1-city1
         :parameters ()
         :precondition (and (= (at-person4) city1)
                            (= (at-plane1) city1))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane1)))

(:action board-person4-plane1-city2
         :parameters ()
         :precondition (and (= (at-person4) city2)
                            (= (at-plane1) city2))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane1)))

(:action board-person4-plane1-city3
         :parameters ()
         :precondition (and (= (at-person4) city3)
                            (= (at-plane1) city3))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane1)))

(:action board-person4-plane2-city0
         :parameters ()
         :precondition (and (= (at-person4) city0)
                            (= (at-plane2) city0))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane2)))

(:action board-person4-plane2-city1
         :parameters ()
         :precondition (and (= (at-person4) city1)
                            (= (at-plane2) city1))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane2)))

(:action board-person4-plane2-city2
         :parameters ()
         :precondition (and (= (at-person4) city2)
                            (= (at-plane2) city2))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane2)))

(:action board-person4-plane2-city3
         :parameters ()
         :precondition (and (= (at-person4) city3)
                            (= (at-plane2) city3))
         :effect (and (assign (at-person4) undefined)
                      (assign (in-person4) plane2)))

(:action debark-person1-plane1-city0
         :parameters ()
         :precondition (and (= (in-person1) plane1)
                            (= (at-plane1) city0))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city0)))

(:action debark-person1-plane1-city1
         :parameters ()
         :precondition (and (= (in-person1) plane1)
                            (= (at-plane1) city1))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city1)))

(:action debark-person1-plane1-city2
         :parameters ()
         :precondition (and (= (in-person1) plane1)
                            (= (at-plane1) city2))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city2)))

(:action debark-person1-plane1-city3
         :parameters ()
         :precondition (and (= (in-person1) plane1)
                            (= (at-plane1) city3))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city3)))

(:action debark-person1-plane2-city0
         :parameters ()
         :precondition (and (= (in-person1) plane2)
                            (= (at-plane2) city0))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city0)))

(:action debark-person1-plane2-city1
         :parameters ()
         :precondition (and (= (in-person1) plane2)
                            (= (at-plane2) city1))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city1)))

(:action debark-person1-plane2-city2
         :parameters ()
         :precondition (and (= (in-person1) plane2)
                            (= (at-plane2) city2))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city2)))

(:action debark-person1-plane2-city3
         :parameters ()
         :precondition (and (= (in-person1) plane2)
                            (= (at-plane2) city3))
         :effect (and (assign (in-person1) undefined)
                      (assign (at-person1) city3)))

(:action debark-person2-plane1-city0
         :parameters ()
         :precondition (and (= (in-person2) plane1)
                            (= (at-plane1) city0))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city0)))

(:action debark-person2-plane1-city1
         :parameters ()
         :precondition (and (= (in-person2) plane1)
                            (= (at-plane1) city1))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city1)))

(:action debark-person2-plane1-city2
         :parameters ()
         :precondition (and (= (in-person2) plane1)
                            (= (at-plane1) city2))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city2)))

(:action debark-person2-plane1-city3
         :parameters ()
         :precondition (and (= (in-person2) plane1)
                            (= (at-plane1) city3))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city3)))

(:action debark-person2-plane2-city0
         :parameters ()
         :precondition (and (= (in-person2) plane2)
                            (= (at-plane2) city0))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city0)))

(:action debark-person2-plane2-city1
         :parameters ()
         :precondition (and (= (in-person2) plane2)
                            (= (at-plane2) city1))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city1)))

(:action debark-person2-plane2-city2
         :parameters ()
         :precondition (and (= (in-person2) plane2)
                            (= (at-plane2) city2))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city2)))

(:action debark-person2-plane2-city3
         :parameters ()
         :precondition (and (= (in-person2) plane2)
                            (= (at-plane2) city3))
         :effect (and (assign (in-person2) undefined)
                      (assign (at-person2) city3)))

(:action debark-person3-plane1-city0
         :parameters ()
         :precondition (and (= (in-person3) plane1)
                            (= (at-plane1) city0))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city0)))

(:action debark-person3-plane1-city1
         :parameters ()
         :precondition (and (= (in-person3) plane1)
                            (= (at-plane1) city1))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city1)))

(:action debark-person3-plane1-city2
         :parameters ()
         :precondition (and (= (in-person3) plane1)
                            (= (at-plane1) city2))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city2)))

(:action debark-person3-plane1-city3
         :parameters ()
         :precondition (and (= (in-person3) plane1)
                            (= (at-plane1) city3))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city3)))

(:action debark-person3-plane2-city0
         :parameters ()
         :precondition (and (= (in-person3) plane2)
                            (= (at-plane2) city0))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city0)))

(:action debark-person3-plane2-city1
         :parameters ()
         :precondition (and (= (in-person3) plane2)
                            (= (at-plane2) city1))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city1)))

(:action debark-person3-plane2-city2
         :parameters ()
         :precondition (and (= (in-person3) plane2)
                            (= (at-plane2) city2))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city2)))

(:action debark-person3-plane2-city3
         :parameters ()
         :precondition (and (= (in-person3) plane2)
                            (= (at-plane2) city3))
         :effect (and (assign (in-person3) undefined)
                      (assign (at-person3) city3)))

(:action debark-person4-plane1-city0
         :parameters ()
         :precondition (and (= (in-person4) plane1)
                            (= (at-plane1) city0))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city0)))

(:action debark-person4-plane1-city1
         :parameters ()
         :precondition (and (= (in-person4) plane1)
                            (= (at-plane1) city1))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city1)))

(:action debark-person4-plane1-city2
         :parameters ()
         :precondition (and (= (in-person4) plane1)
                            (= (at-plane1) city2))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city2)))

(:action debark-person4-plane1-city3
         :parameters ()
         :precondition (and (= (in-person4) plane1)
                            (= (at-plane1) city3))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city3)))

(:action debark-person4-plane2-city0
         :parameters ()
         :precondition (and (= (in-person4) plane2)
                            (= (at-plane2) city0))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city0)))

(:action debark-person4-plane2-city1
         :parameters ()
         :precondition (and (= (in-person4) plane2)
                            (= (at-plane2) city1))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city1)))

(:action debark-person4-plane2-city2
         :parameters ()
         :precondition (and (= (in-person4) plane2)
                            (= (at-plane2) city2))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city2)))

(:action debark-person4-plane2-city3
         :parameters ()
         :precondition (and (= (in-person4) plane2)
                            (= (at-plane2) city3))
         :effect (and (assign (in-person4) undefined)
                      (assign (at-person4) city3)))

(:action fly-plane1-city0-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 131)))

(:action fly-plane1-city1-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 88)))

(:action fly-plane1-city2-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl1))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 131)))

(:action fly-plane2-city1-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 88)))

(:action fly-plane2-city2-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl1))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city1-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city2-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city0)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city1)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city2)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl2))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl3))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl4))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl5))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (= (fuel-level-plane1) fl6))
         :effect (and (assign (at-plane1) city3)
                      (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city1-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city2-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city0)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city1)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city2)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl2))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl3))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl4))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl5))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (= (fuel-level-plane2) fl6))
         :effect (and (assign (at-plane2) city3)
                      (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 0)))

(:action refuel-plane1-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl0)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl1)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl2)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl3)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl4)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl5)
                            (= (at-plane1) city0))
         :effect (and (assign (fuel-level-plane1) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl0)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl1)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl2)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl3)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl4)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl5)
                            (= (at-plane1) city1))
         :effect (and (assign (fuel-level-plane1) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl0)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl1)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl2)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl3)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl4)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl5)
                            (= (at-plane1) city2))
         :effect (and (assign (fuel-level-plane1) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl0)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl1)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl2)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl3)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl4)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane1) fl5)
                            (= (at-plane1) city3))
         :effect (and (assign (fuel-level-plane1) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl0)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl1)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl2)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl3)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl4)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl5)
                            (= (at-plane2) city0))
         :effect (and (assign (fuel-level-plane2) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl0)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl1)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl2)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl3)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl4)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl5)
                            (= (at-plane2) city1))
         :effect (and (assign (fuel-level-plane2) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl0)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl1)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl2)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl3)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl4)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl5)
                            (= (at-plane2) city2))
         :effect (and (assign (fuel-level-plane2) fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl0-fl1
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl0)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl1-fl2
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl1)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl2-fl3
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl2)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl3-fl4
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl3)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl4-fl5
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl4)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl5-fl6
         :parameters ()
         :precondition (and (= (fuel-level-plane2) fl5)
                            (= (at-plane2) city3))
         :effect (and (assign (fuel-level-plane2) fl6)
                      (increase (total-cost) 50)))

)

