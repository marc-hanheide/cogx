;; Net benefit version of the IPC-2002 Zenotravel domain.

;; Refuel action taken out so that it is not always possible to
;; satisfy all goals. See "sequential" version for some more comments.

(define (domain zeno-travel)

(:requirements :action-costs :fluents :goal-utilities)

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
     (fuel-level-plane1) - number
     (fuel-level-plane2) - number
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

(:action fly-plane1-city0-city0
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city0-city1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 30))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 30)
                      (increase (total-cost) 140)))

(:action fly-plane1-city0-city2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 15))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 15)
                      (increase (total-cost) 70)))

(:action fly-plane1-city0-city3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 19))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 19)
                      (increase (total-cost) 131)))

(:action fly-plane1-city1-city0
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 30))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 30)
                      (increase (total-cost) 140)))

(:action fly-plane1-city1-city1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city1-city2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 12))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 12)
                      (increase (total-cost) 124)))

(:action fly-plane1-city1-city3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 18))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 18)
                      (increase (total-cost) 88)))

(:action fly-plane1-city2-city0
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 15))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 15)
                      (increase (total-cost) 70)))

(:action fly-plane1-city2-city1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 12))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 12)
                      (increase (total-cost) 124)))

(:action fly-plane1-city2-city2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action fly-plane1-city2-city3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 20))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 20)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city0
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 19))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 19)
                      (increase (total-cost) 131)))

(:action fly-plane1-city3-city1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 18))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 18)
                      (increase (total-cost) 88)))

(:action fly-plane1-city3-city2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 20))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 20)
                      (increase (total-cost) 201)))

(:action fly-plane1-city3-city3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city0
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city0-city1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 30))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 30)
                      (increase (total-cost) 140)))

(:action fly-plane2-city0-city2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 15))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 15)
                      (increase (total-cost) 70)))

(:action fly-plane2-city0-city3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 19))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 19)
                      (increase (total-cost) 131)))

(:action fly-plane2-city1-city0
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 30))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 30)
                      (increase (total-cost) 140)))

(:action fly-plane2-city1-city1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city1-city2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 12))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 12)
                      (increase (total-cost) 124)))

(:action fly-plane2-city1-city3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 18))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 18)
                      (increase (total-cost) 88)))

(:action fly-plane2-city2-city0
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 15))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 15)
                      (increase (total-cost) 70)))

(:action fly-plane2-city2-city1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 12))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 12)
                      (increase (total-cost) 124)))

(:action fly-plane2-city2-city2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action fly-plane2-city2-city3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 20))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 20)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city0
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 19))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 19)
                      (increase (total-cost) 131)))

(:action fly-plane2-city3-city1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 18))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 18)
                      (increase (total-cost) 88)))

(:action fly-plane2-city3-city2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 20))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 20)
                      (increase (total-cost) 201)))

(:action fly-plane2-city3-city3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city0
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city0-city1
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 55))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 55)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city0-city2
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 33))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 33)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city0-city3
         :parameters ()
         :precondition (and (= (at-plane1) city0)
                            (>= (fuel-level-plane1) 44))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 44)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city1-city0
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 55))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 55)
                      (increase (total-cost) 88)))

(:action zoom-plane1-city1-city1
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city1-city2
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 30))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 30)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city1-city3
         :parameters ()
         :precondition (and (= (at-plane1) city1)
                            (>= (fuel-level-plane1) 42))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 42)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city2-city0
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 33))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 33)
                      (increase (total-cost) 44)))

(:action zoom-plane1-city2-city1
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 30))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 30)
                      (increase (total-cost) 94)))

(:action zoom-plane1-city2-city2
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane1-city2-city3
         :parameters ()
         :precondition (and (= (at-plane1) city2)
                            (>= (fuel-level-plane1) 50))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 50)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city0
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 44))
         :effect (and (assign (at-plane1) city0)
                      (decrease (fuel-level-plane1) 44)
                      (increase (total-cost) 100)))

(:action zoom-plane1-city3-city1
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 42))
         :effect (and (assign (at-plane1) city1)
                      (decrease (fuel-level-plane1) 42)
                      (increase (total-cost) 55)))

(:action zoom-plane1-city3-city2
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 50))
         :effect (and (assign (at-plane1) city2)
                      (decrease (fuel-level-plane1) 50)
                      (increase (total-cost) 130)))

(:action zoom-plane1-city3-city3
         :parameters ()
         :precondition (and (= (at-plane1) city3)
                            (>= (fuel-level-plane1) 0))
         :effect (and (assign (at-plane1) city3)
                      (decrease (fuel-level-plane1) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city0
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city0-city1
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 55))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 55)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city0-city2
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 33))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 33)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city0-city3
         :parameters ()
         :precondition (and (= (at-plane2) city0)
                            (>= (fuel-level-plane2) 44))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 44)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city1-city0
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 55))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 55)
                      (increase (total-cost) 88)))

(:action zoom-plane2-city1-city1
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city1-city2
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 30))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 30)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city1-city3
         :parameters ()
         :precondition (and (= (at-plane2) city1)
                            (>= (fuel-level-plane2) 42))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 42)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city2-city0
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 33))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 33)
                      (increase (total-cost) 44)))

(:action zoom-plane2-city2-city1
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 30))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 30)
                      (increase (total-cost) 94)))

(:action zoom-plane2-city2-city2
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))

(:action zoom-plane2-city2-city3
         :parameters ()
         :precondition (and (= (at-plane2) city2)
                            (>= (fuel-level-plane2) 50))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 50)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city0
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 44))
         :effect (and (assign (at-plane2) city0)
                      (decrease (fuel-level-plane2) 44)
                      (increase (total-cost) 100)))

(:action zoom-plane2-city3-city1
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 42))
         :effect (and (assign (at-plane2) city1)
                      (decrease (fuel-level-plane2) 42)
                      (increase (total-cost) 55)))

(:action zoom-plane2-city3-city2
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 50))
         :effect (and (assign (at-plane2) city2)
                      (decrease (fuel-level-plane2) 50)
                      (increase (total-cost) 130)))

(:action zoom-plane2-city3-city3
         :parameters ()
         :precondition (and (= (at-plane2) city3)
                            (>= (fuel-level-plane2) 0))
         :effect (and (assign (at-plane2) city3)
                      (decrease (fuel-level-plane2) 0)
                      (increase (total-cost) 0)))


)
