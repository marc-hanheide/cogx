;; Action costs version of the IPC-2002 Zenotravel domain.

;; The encoding is somewhat strange because the costs of
;; flying/zooming from city to city differs for different pairs of
;; cities, but the fuel usage does not. But since this is only
;; intended as a test domain, this shouldn't be much of an issue.

(define (domain zeno-travel)

(:requirements :action-costs)

(:predicates
     (at-plane1-city0)
     (at-plane1-city1)
     (at-plane1-city2)
     (at-plane1-city3)
     (at-plane2-city0)
     (at-plane2-city1)
     (at-plane2-city2)
     (at-plane2-city3)
     (at-person1-city0)
     (at-person1-city1)
     (at-person1-city2)
     (at-person1-city3)
     (at-person2-city0)
     (at-person2-city1)
     (at-person2-city2)
     (at-person2-city3)
     (at-person3-city0)
     (at-person3-city1)
     (at-person3-city2)
     (at-person3-city3)
     (at-person4-city0)
     (at-person4-city1)
     (at-person4-city2)
     (at-person4-city3)
     (in-person1-plane1)
     (in-person1-plane2)
     (in-person2-plane1)
     (in-person2-plane2)
     (in-person3-plane1)
     (in-person3-plane2)
     (in-person4-plane1)
     (in-person4-plane2)
     (fuel-level-plane1-fl0)
     (fuel-level-plane1-fl1)
     (fuel-level-plane1-fl2)
     (fuel-level-plane1-fl3)
     (fuel-level-plane1-fl4)
     (fuel-level-plane1-fl5)
     (fuel-level-plane1-fl6)
     (fuel-level-plane2-fl0)
     (fuel-level-plane2-fl1)
     (fuel-level-plane2-fl2)
     (fuel-level-plane2-fl3)
     (fuel-level-plane2-fl4)
     (fuel-level-plane2-fl5)
     (fuel-level-plane2-fl6)
)

(:functions (total-cost) - number)

(:action board-person1-plane1-city0
         :parameters ()
         :precondition (and (at-person1-city0)
                            (at-plane1-city0))
         :effect (and (not (at-person1-city0))
                      (in-person1-plane1)))

(:action board-person1-plane1-city1
         :parameters ()
         :precondition (and (at-person1-city1)
                            (at-plane1-city1))
         :effect (and (not (at-person1-city1))
                      (in-person1-plane1)))

(:action board-person1-plane1-city2
         :parameters ()
         :precondition (and (at-person1-city2)
                            (at-plane1-city2))
         :effect (and (not (at-person1-city2))
                      (in-person1-plane1)))

(:action board-person1-plane1-city3
         :parameters ()
         :precondition (and (at-person1-city3)
                            (at-plane1-city3))
         :effect (and (not (at-person1-city3))
                      (in-person1-plane1)))

(:action board-person1-plane2-city0
         :parameters ()
         :precondition (and (at-person1-city0)
                            (at-plane2-city0))
         :effect (and (not (at-person1-city0))
                      (in-person1-plane2)))

(:action board-person1-plane2-city1
         :parameters ()
         :precondition (and (at-person1-city1)
                            (at-plane2-city1))
         :effect (and (not (at-person1-city1))
                      (in-person1-plane2)))

(:action board-person1-plane2-city2
         :parameters ()
         :precondition (and (at-person1-city2)
                            (at-plane2-city2))
         :effect (and (not (at-person1-city2))
                      (in-person1-plane2)))

(:action board-person1-plane2-city3
         :parameters ()
         :precondition (and (at-person1-city3)
                            (at-plane2-city3))
         :effect (and (not (at-person1-city3))
                      (in-person1-plane2)))

(:action board-person2-plane1-city0
         :parameters ()
         :precondition (and (at-person2-city0)
                            (at-plane1-city0))
         :effect (and (not (at-person2-city0))
                      (in-person2-plane1)))

(:action board-person2-plane1-city1
         :parameters ()
         :precondition (and (at-person2-city1)
                            (at-plane1-city1))
         :effect (and (not (at-person2-city1))
                      (in-person2-plane1)))

(:action board-person2-plane1-city2
         :parameters ()
         :precondition (and (at-person2-city2)
                            (at-plane1-city2))
         :effect (and (not (at-person2-city2))
                      (in-person2-plane1)))

(:action board-person2-plane1-city3
         :parameters ()
         :precondition (and (at-person2-city3)
                            (at-plane1-city3))
         :effect (and (not (at-person2-city3))
                      (in-person2-plane1)))

(:action board-person2-plane2-city0
         :parameters ()
         :precondition (and (at-person2-city0)
                            (at-plane2-city0))
         :effect (and (not (at-person2-city0))
                      (in-person2-plane2)))

(:action board-person2-plane2-city1
         :parameters ()
         :precondition (and (at-person2-city1)
                            (at-plane2-city1))
         :effect (and (not (at-person2-city1))
                      (in-person2-plane2)))

(:action board-person2-plane2-city2
         :parameters ()
         :precondition (and (at-person2-city2)
                            (at-plane2-city2))
         :effect (and (not (at-person2-city2))
                      (in-person2-plane2)))

(:action board-person2-plane2-city3
         :parameters ()
         :precondition (and (at-person2-city3)
                            (at-plane2-city3))
         :effect (and (not (at-person2-city3))
                      (in-person2-plane2)))

(:action board-person3-plane1-city0
         :parameters ()
         :precondition (and (at-person3-city0)
                            (at-plane1-city0))
         :effect (and (not (at-person3-city0))
                      (in-person3-plane1)))

(:action board-person3-plane1-city1
         :parameters ()
         :precondition (and (at-person3-city1)
                            (at-plane1-city1))
         :effect (and (not (at-person3-city1))
                      (in-person3-plane1)))

(:action board-person3-plane1-city2
         :parameters ()
         :precondition (and (at-person3-city2)
                            (at-plane1-city2))
         :effect (and (not (at-person3-city2))
                      (in-person3-plane1)))

(:action board-person3-plane1-city3
         :parameters ()
         :precondition (and (at-person3-city3)
                            (at-plane1-city3))
         :effect (and (not (at-person3-city3))
                      (in-person3-plane1)))

(:action board-person3-plane2-city0
         :parameters ()
         :precondition (and (at-person3-city0)
                            (at-plane2-city0))
         :effect (and (not (at-person3-city0))
                      (in-person3-plane2)))

(:action board-person3-plane2-city1
         :parameters ()
         :precondition (and (at-person3-city1)
                            (at-plane2-city1))
         :effect (and (not (at-person3-city1))
                      (in-person3-plane2)))

(:action board-person3-plane2-city2
         :parameters ()
         :precondition (and (at-person3-city2)
                            (at-plane2-city2))
         :effect (and (not (at-person3-city2))
                      (in-person3-plane2)))

(:action board-person3-plane2-city3
         :parameters ()
         :precondition (and (at-person3-city3)
                            (at-plane2-city3))
         :effect (and (not (at-person3-city3))
                      (in-person3-plane2)))

(:action board-person4-plane1-city0
         :parameters ()
         :precondition (and (at-person4-city0)
                            (at-plane1-city0))
         :effect (and (not (at-person4-city0))
                      (in-person4-plane1)))

(:action board-person4-plane1-city1
         :parameters ()
         :precondition (and (at-person4-city1)
                            (at-plane1-city1))
         :effect (and (not (at-person4-city1))
                      (in-person4-plane1)))

(:action board-person4-plane1-city2
         :parameters ()
         :precondition (and (at-person4-city2)
                            (at-plane1-city2))
         :effect (and (not (at-person4-city2))
                      (in-person4-plane1)))

(:action board-person4-plane1-city3
         :parameters ()
         :precondition (and (at-person4-city3)
                            (at-plane1-city3))
         :effect (and (not (at-person4-city3))
                      (in-person4-plane1)))

(:action board-person4-plane2-city0
         :parameters ()
         :precondition (and (at-person4-city0)
                            (at-plane2-city0))
         :effect (and (not (at-person4-city0))
                      (in-person4-plane2)))

(:action board-person4-plane2-city1
         :parameters ()
         :precondition (and (at-person4-city1)
                            (at-plane2-city1))
         :effect (and (not (at-person4-city1))
                      (in-person4-plane2)))

(:action board-person4-plane2-city2
         :parameters ()
         :precondition (and (at-person4-city2)
                            (at-plane2-city2))
         :effect (and (not (at-person4-city2))
                      (in-person4-plane2)))

(:action board-person4-plane2-city3
         :parameters ()
         :precondition (and (at-person4-city3)
                            (at-plane2-city3))
         :effect (and (not (at-person4-city3))
                      (in-person4-plane2)))

(:action debark-person1-plane1-city0
         :parameters ()
         :precondition (and (in-person1-plane1)
                            (at-plane1-city0))
         :effect (and (not (in-person1-plane1))
                      (at-person1-city0)))

(:action debark-person1-plane1-city1
         :parameters ()
         :precondition (and (in-person1-plane1)
                            (at-plane1-city1))
         :effect (and (not (in-person1-plane1))
                      (at-person1-city1)))

(:action debark-person1-plane1-city2
         :parameters ()
         :precondition (and (in-person1-plane1)
                            (at-plane1-city2))
         :effect (and (not (in-person1-plane1))
                      (at-person1-city2)))

(:action debark-person1-plane1-city3
         :parameters ()
         :precondition (and (in-person1-plane1)
                            (at-plane1-city3))
         :effect (and (not (in-person1-plane1))
                      (at-person1-city3)))

(:action debark-person1-plane2-city0
         :parameters ()
         :precondition (and (in-person1-plane2)
                            (at-plane2-city0))
         :effect (and (not (in-person1-plane2))
                      (at-person1-city0)))

(:action debark-person1-plane2-city1
         :parameters ()
         :precondition (and (in-person1-plane2)
                            (at-plane2-city1))
         :effect (and (not (in-person1-plane2))
                      (at-person1-city1)))

(:action debark-person1-plane2-city2
         :parameters ()
         :precondition (and (in-person1-plane2)
                            (at-plane2-city2))
         :effect (and (not (in-person1-plane2))
                      (at-person1-city2)))

(:action debark-person1-plane2-city3
         :parameters ()
         :precondition (and (in-person1-plane2)
                            (at-plane2-city3))
         :effect (and (not (in-person1-plane2))
                      (at-person1-city3)))

(:action debark-person2-plane1-city0
         :parameters ()
         :precondition (and (in-person2-plane1)
                            (at-plane1-city0))
         :effect (and (not (in-person2-plane1))
                      (at-person2-city0)))

(:action debark-person2-plane1-city1
         :parameters ()
         :precondition (and (in-person2-plane1)
                            (at-plane1-city1))
         :effect (and (not (in-person2-plane1))
                      (at-person2-city1)))

(:action debark-person2-plane1-city2
         :parameters ()
         :precondition (and (in-person2-plane1)
                            (at-plane1-city2))
         :effect (and (not (in-person2-plane1))
                      (at-person2-city2)))

(:action debark-person2-plane1-city3
         :parameters ()
         :precondition (and (in-person2-plane1)
                            (at-plane1-city3))
         :effect (and (not (in-person2-plane1))
                      (at-person2-city3)))

(:action debark-person2-plane2-city0
         :parameters ()
         :precondition (and (in-person2-plane2)
                            (at-plane2-city0))
         :effect (and (not (in-person2-plane2))
                      (at-person2-city0)))

(:action debark-person2-plane2-city1
         :parameters ()
         :precondition (and (in-person2-plane2)
                            (at-plane2-city1))
         :effect (and (not (in-person2-plane2))
                      (at-person2-city1)))

(:action debark-person2-plane2-city2
         :parameters ()
         :precondition (and (in-person2-plane2)
                            (at-plane2-city2))
         :effect (and (not (in-person2-plane2))
                      (at-person2-city2)))

(:action debark-person2-plane2-city3
         :parameters ()
         :precondition (and (in-person2-plane2)
                            (at-plane2-city3))
         :effect (and (not (in-person2-plane2))
                      (at-person2-city3)))

(:action debark-person3-plane1-city0
         :parameters ()
         :precondition (and (in-person3-plane1)
                            (at-plane1-city0))
         :effect (and (not (in-person3-plane1))
                      (at-person3-city0)))

(:action debark-person3-plane1-city1
         :parameters ()
         :precondition (and (in-person3-plane1)
                            (at-plane1-city1))
         :effect (and (not (in-person3-plane1))
                      (at-person3-city1)))

(:action debark-person3-plane1-city2
         :parameters ()
         :precondition (and (in-person3-plane1)
                            (at-plane1-city2))
         :effect (and (not (in-person3-plane1))
                      (at-person3-city2)))

(:action debark-person3-plane1-city3
         :parameters ()
         :precondition (and (in-person3-plane1)
                            (at-plane1-city3))
         :effect (and (not (in-person3-plane1))
                      (at-person3-city3)))

(:action debark-person3-plane2-city0
         :parameters ()
         :precondition (and (in-person3-plane2)
                            (at-plane2-city0))
         :effect (and (not (in-person3-plane2))
                      (at-person3-city0)))

(:action debark-person3-plane2-city1
         :parameters ()
         :precondition (and (in-person3-plane2)
                            (at-plane2-city1))
         :effect (and (not (in-person3-plane2))
                      (at-person3-city1)))

(:action debark-person3-plane2-city2
         :parameters ()
         :precondition (and (in-person3-plane2)
                            (at-plane2-city2))
         :effect (and (not (in-person3-plane2))
                      (at-person3-city2)))

(:action debark-person3-plane2-city3
         :parameters ()
         :precondition (and (in-person3-plane2)
                            (at-plane2-city3))
         :effect (and (not (in-person3-plane2))
                      (at-person3-city3)))

(:action debark-person4-plane1-city0
         :parameters ()
         :precondition (and (in-person4-plane1)
                            (at-plane1-city0))
         :effect (and (not (in-person4-plane1))
                      (at-person4-city0)))

(:action debark-person4-plane1-city1
         :parameters ()
         :precondition (and (in-person4-plane1)
                            (at-plane1-city1))
         :effect (and (not (in-person4-plane1))
                      (at-person4-city1)))

(:action debark-person4-plane1-city2
         :parameters ()
         :precondition (and (in-person4-plane1)
                            (at-plane1-city2))
         :effect (and (not (in-person4-plane1))
                      (at-person4-city2)))

(:action debark-person4-plane1-city3
         :parameters ()
         :precondition (and (in-person4-plane1)
                            (at-plane1-city3))
         :effect (and (not (in-person4-plane1))
                      (at-person4-city3)))

(:action debark-person4-plane2-city0
         :parameters ()
         :precondition (and (in-person4-plane2)
                            (at-plane2-city0))
         :effect (and (not (in-person4-plane2))
                      (at-person4-city0)))

(:action debark-person4-plane2-city1
         :parameters ()
         :precondition (and (in-person4-plane2)
                            (at-plane2-city1))
         :effect (and (not (in-person4-plane2))
                      (at-person4-city1)))

(:action debark-person4-plane2-city2
         :parameters ()
         :precondition (and (in-person4-plane2)
                            (at-plane2-city2))
         :effect (and (not (in-person4-plane2))
                      (at-person4-city2)))

(:action debark-person4-plane2-city3
         :parameters ()
         :precondition (and (in-person4-plane2)
                            (at-plane2-city3))
         :effect (and (not (in-person4-plane2))
                      (at-person4-city3)))

(:action fly-plane1-city0-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 131)))

(:action fly-plane1-city0-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 131)))

(:action fly-plane1-city1-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 88)))

(:action fly-plane1-city1-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 88)))

(:action fly-plane1-city2-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 201)))

(:action fly-plane1-city2-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl1))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane1-city3-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 131)))

(:action fly-plane2-city0-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 131)))

(:action fly-plane2-city1-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 88)))

(:action fly-plane2-city1-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 88)))

(:action fly-plane2-city2-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 201)))

(:action fly-plane2-city2-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city0-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city0-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city1-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city1-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city2-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city2-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city3-fl0-fl1
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl1))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action fly-plane2-city3-city3-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city0-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city0)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city0))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city1-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city1-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city1)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city1))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city2-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city2-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city2)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city2))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city0)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city1)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city2)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl2))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl3))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl4))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl5))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city3-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane1-city3)
                            (fuel-level-plane1-fl6))
         :effect (and (not (at-plane1-city3))
                      (at-plane1-city3)
                      (not (fuel-level-plane1-fl6))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city0-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city0)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city0))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city1-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city1-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city1)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city1))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city2-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city2-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city2)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city2))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city0-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city0-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city0)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city1-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city1-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city1)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city2-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city2-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city2)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city3-fl0-fl1-fl2
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl2))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl1-fl2-fl3
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl3))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl2-fl3-fl4
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl4))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl3-fl4-fl5
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl5))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city3-city3-fl4-fl5-fl6
         :parameters ()
         :precondition (and (at-plane2-city3)
                            (fuel-level-plane2-fl6))
         :effect (and (not (at-plane2-city3))
                      (at-plane2-city3)
                      (not (fuel-level-plane2-fl6))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 0)))

(:action refuel-plane1-city0-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane1-fl0)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl0))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane1-fl1)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane1-fl2)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane1-fl3)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane1-fl4)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city0-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane1-fl5)
                            (at-plane1-city0))
         :effect (and (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane1-fl0)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl0))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane1-fl1)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane1-fl2)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane1-fl3)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane1-fl4)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city1-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane1-fl5)
                            (at-plane1-city1))
         :effect (and (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane1-fl0)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl0))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane1-fl1)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane1-fl2)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane1-fl3)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane1-fl4)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city2-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane1-fl5)
                            (at-plane1-city2))
         :effect (and (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane1-fl0)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl0))
                      (fuel-level-plane1-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane1-fl1)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl1))
                      (fuel-level-plane1-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane1-fl2)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl2))
                      (fuel-level-plane1-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane1-fl3)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl3))
                      (fuel-level-plane1-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane1-fl4)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl4))
                      (fuel-level-plane1-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane1-city3-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane1-fl5)
                            (at-plane1-city3))
         :effect (and (not (fuel-level-plane1-fl5))
                      (fuel-level-plane1-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane2-fl0)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl0))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane2-fl1)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane2-fl2)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane2-fl3)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane2-fl4)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city0-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane2-fl5)
                            (at-plane2-city0))
         :effect (and (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane2-fl0)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl0))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane2-fl1)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane2-fl2)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane2-fl3)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane2-fl4)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city1-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane2-fl5)
                            (at-plane2-city1))
         :effect (and (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane2-fl0)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl0))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane2-fl1)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane2-fl2)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane2-fl3)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane2-fl4)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city2-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane2-fl5)
                            (at-plane2-city2))
         :effect (and (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl6)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl0-fl1
         :parameters ()
         :precondition (and (fuel-level-plane2-fl0)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl0))
                      (fuel-level-plane2-fl1)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl1-fl2
         :parameters ()
         :precondition (and (fuel-level-plane2-fl1)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl1))
                      (fuel-level-plane2-fl2)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl2-fl3
         :parameters ()
         :precondition (and (fuel-level-plane2-fl2)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl2))
                      (fuel-level-plane2-fl3)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl3-fl4
         :parameters ()
         :precondition (and (fuel-level-plane2-fl3)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl3))
                      (fuel-level-plane2-fl4)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl4-fl5
         :parameters ()
         :precondition (and (fuel-level-plane2-fl4)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl4))
                      (fuel-level-plane2-fl5)
                      (increase (total-cost) 50)))

(:action refuel-plane2-city3-fl5-fl6
         :parameters ()
         :precondition (and (fuel-level-plane2-fl5)
                            (at-plane2-city3))
         :effect (and (not (fuel-level-plane2-fl5))
                      (fuel-level-plane2-fl6)
                      (increase (total-cost) 50)))

)
