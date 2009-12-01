(define (problem ZTRAVEL-06)
(:domain zeno-travel)
(:objects
	plane1 - aircraft
	plane2 - aircraft
	person1 - person
	person2 - person
	person3 - person
	person4 - person
	person5 - person
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
	(fuel-level plane1 fl5)
	(at plane2 city1)
	(fuel-level plane2 fl3)
	(at person1 city0)
	(at person2 city0)
	(at person3 city3)
	(at person4 city1)
	(at person5 city2)
	(next fl0 fl1)
	(next fl1 fl2)
	(next fl2 fl3)
	(next fl3 fl4)
	(next fl4 fl5)
	(next fl5 fl6)
)
(:goal (and
	(at person1 city3)
	(at person2 city1)
	(at person3 city3)
	(at person4 city3)
	(at person5 city1)
	))
  (:metric minimize (total-cost))
)
