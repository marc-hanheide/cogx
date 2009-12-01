(define (problem ZTRAVEL-07)
(:domain zeno-travel)
(:objects
	plane1 - aircraft
	plane2 - aircraft
	person1 - person
	person2 - person
	person3 - person
	person4 - person
	person5 - person
	person6 - person
	city0 - city
	city1 - city
	city2 - city
	city3 - city
	fl0 - flevel
	fl1 - flevel
	fl2 - flevel
	fl3 - flevel
	fl4 - flevel
	fl5 - flevel
	fl6 - flevel
	)
(:init
    (= (total-cost) 0)
	(at plane1 city2)
	(fuel-level plane1 fl1)
	(at plane2 city1)
	(fuel-level plane2 fl1)
	(at person1 city3)
	(at person2 city3)
	(at person3 city3)
	(at person4 city1)
	(at person5 city3)
	(at person6 city0)
	(next fl0 fl1)
	(next fl1 fl2)
	(next fl2 fl3)
	(next fl3 fl4)
	(next fl4 fl5)
	(next fl5 fl6)
)
(:goal (and
	(at plane2 city1)
	(at person1 city2)
	(at person3 city3)
	(at person4 city3)
	(at person5 city2)
	(at person6 city2)
	))
  (:metric minimize (total-cost))
)
