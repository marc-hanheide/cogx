(define (problem logistics-prob35)
(:domain logistics_conf)

(:objects package5 package4 package3 package2 package1 - obj 
          city8 city7 city6 city5 city4 city3 city2 city1 - city
          truck15 truck14 truck13 truck12 truck11 truck10 truck9 truck8 truck7 truck6
          truck5 truck4 truck3 truck2 truck1 - truck
          plane1 - airplane
          city8-2 city8-1 city7-2 city7-1 city6-2 city6-1 city5-2 city5-1 city4-2 city4-1 city3-2 city3-1 city2-2 city2-1 city1-2 city1-1 - location
          city8-3 city7-3 city6-3 city5-3 city4-3 city3-3 city2-3 city1-3 - airport)
   (:init
          (= (in_city city8-3) city8)
          (= (in_city city8-2) city8)
          (= (in_city city8-1) city8)
          (= (in_city city7-3) city7)
          (= (in_city city7-2) city7)
          (= (in_city city7-1) city7)
          (= (in_city city6-3) city6)
          (= (in_city city6-2) city6)
          (= (in_city city6-1) city6)
          (= (in_city city5-3) city5)
          (= (in_city city5-2) city5)
          (= (in_city city5-1) city5)
          (= (in_city city4-3) city4)
          (= (in_city city4-2) city4)
          (= (in_city city4-1) city4)
          (= (in_city city3-3) city3)
          (= (in_city city3-2) city3)
          (= (in_city city3-1) city3)
          (= (in_city city2-3) city2)
          (= (in_city city2-2) city2)
          (= (in_city city2-1) city2)
          (= (in_city city1-3) city1)
          (= (in_city city1-2) city1)
          (= (in_city city1-1) city1)
          (= (pos plane1) city3-3)
          (= (pos truck15) city8-2)
          (= (pos truck14) city7-2)
          (= (pos truck13) city6-1)
          (= (pos truck12) city5-2)
          (= (pos truck11) city4-1)
          (= (pos truck10) city3-2)
          (= (pos truck9) city2-1)
          (= (pos truck8) city1-2)
          (= (pos truck7) city6-1)
          (= (pos truck6) city3-2)
          (= (pos truck5) city1-3)
          (= (pos truck4) city4-3)
          (= (pos truck3) city1-3)
          (= (pos truck2) city7-1)
          (= (pos truck1) city8-1)

          (= (pos package1) city2-1)
          (= (pos package2) city5-1)
          (= (pos package3) city1-1)
          (= (pos package4) city5-1)
          (= (pos package5) city3-1)

)


(:goal (and (= (pos package5) city6-3)
            (= (pos package4) city5-3)
            (= (pos package3) city8-3)
            (= (pos package2) city4-3)
            (= (pos package1) city6-3)))


)


