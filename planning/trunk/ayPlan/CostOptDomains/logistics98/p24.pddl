(define (problem strips-log-x-24)
   (:domain logistics-strips)
   (:objects package11 package10 package9 package8 package7
             package6 package5 package4 package3 package2 package1 city9
             city8 city7 city6 city5 city4 city3 city2 city1 truck54
             truck53 truck52 truck51 truck50 truck49 truck48 truck47
             truck46 truck45 truck44 truck43 truck42 truck41 truck40
             truck39 truck38 truck37 truck36 truck35 truck34 truck33
             truck32 truck31 truck30 truck29 truck28 truck27 truck26
             truck25 truck24 truck23 truck22 truck21 truck20 truck19
             truck18 truck17 truck16 truck15 truck14 truck13 truck12
             truck11 truck10 truck9 truck8 truck7 truck6 truck5 truck4
             truck3 truck2 truck1 plane8 plane7 plane6 plane5 plane4 plane3
             plane2 plane1 city9-11 city9-10 city9-9 city9-8 city9-7
             city9-6 city9-5 city9-4 city9-3 city9-2 city9-1 city8-11
             city8-10 city8-9 city8-8 city8-7 city8-6 city8-5 city8-4
             city8-3 city8-2 city8-1 city7-11 city7-10 city7-9 city7-8
             city7-7 city7-6 city7-5 city7-4 city7-3 city7-2 city7-1
             city6-11 city6-10 city6-9 city6-8 city6-7 city6-6 city6-5
             city6-4 city6-3 city6-2 city6-1 city5-11 city5-10 city5-9
             city5-8 city5-7 city5-6 city5-5 city5-4 city5-3 city5-2
             city5-1 city4-11 city4-10 city4-9 city4-8 city4-7 city4-6
             city4-5 city4-4 city4-3 city4-2 city4-1 city3-11 city3-10
             city3-9 city3-8 city3-7 city3-6 city3-5 city3-4 city3-3
             city3-2 city3-1 city2-11 city2-10 city2-9 city2-8 city2-7
             city2-6 city2-5 city2-4 city2-3 city2-2 city2-1 city1-11
             city1-10 city1-9 city1-8 city1-7 city1-6 city1-5 city1-4
             city1-3 city1-2 city1-1 city9-12 city8-12 city7-12 city6-12
             city5-12 city4-12 city3-12 city2-12 city1-12 - block )
   (:init (obj package11) (= (total-cost) 0)
          (obj package10)
          (obj package9)
          (obj package8)
          (obj package7)
          (obj package6)
          (obj package5)
          (obj package4)
          (obj package3)
          (obj package2)
          (obj package1)
          (city city9)
          (city city8)
          (city city7)
          (city city6)
          (city city5)
          (city city4)
          (city city3)
          (city city2)
          (city city1)
          (truck truck54)
          (truck truck53)
          (truck truck52)
          (truck truck51)
          (truck truck50)
          (truck truck49)
          (truck truck48)
          (truck truck47)
          (truck truck46)
          (truck truck45)
          (truck truck44)
          (truck truck43)
          (truck truck42)
          (truck truck41)
          (truck truck40)
          (truck truck39)
          (truck truck38)
          (truck truck37)
          (truck truck36)
          (truck truck35)
          (truck truck34)
          (truck truck33)
          (truck truck32)
          (truck truck31)
          (truck truck30)
          (truck truck29)
          (truck truck28)
          (truck truck27)
          (truck truck26)
          (truck truck25)
          (truck truck24)
          (truck truck23)
          (truck truck22)
          (truck truck21)
          (truck truck20)
          (truck truck19)
          (truck truck18)
          (truck truck17)
          (truck truck16)
          (truck truck15)
          (truck truck14)
          (truck truck13)
          (truck truck12)
          (truck truck11)
          (truck truck10)
          (truck truck9)
          (truck truck8)
          (truck truck7)
          (truck truck6)
          (truck truck5)
          (truck truck4)
          (truck truck3)
          (truck truck2)
          (truck truck1)
          (airplane plane8)
          (airplane plane7)
          (airplane plane6)
          (airplane plane5)
          (airplane plane4)
          (airplane plane3)
          (airplane plane2)
          (airplane plane1)
          (location city9-11)
          (location city9-10)
          (location city9-9)
          (location city9-8)
          (location city9-7)
          (location city9-6)
          (location city9-5)
          (location city9-4)
          (location city9-3)
          (location city9-2)
          (location city9-1)
          (location city8-11)
          (location city8-10)
          (location city8-9)
          (location city8-8)
          (location city8-7)
          (location city8-6)
          (location city8-5)
          (location city8-4)
          (location city8-3)
          (location city8-2)
          (location city8-1)
          (location city7-11)
          (location city7-10)
          (location city7-9)
          (location city7-8)
          (location city7-7)
          (location city7-6)
          (location city7-5)
          (location city7-4)
          (location city7-3)
          (location city7-2)
          (location city7-1)
          (location city6-11)
          (location city6-10)
          (location city6-9)
          (location city6-8)
          (location city6-7)
          (location city6-6)
          (location city6-5)
          (location city6-4)
          (location city6-3)
          (location city6-2)
          (location city6-1)
          (location city5-11)
          (location city5-10)
          (location city5-9)
          (location city5-8)
          (location city5-7)
          (location city5-6)
          (location city5-5)
          (location city5-4)
          (location city5-3)
          (location city5-2)
          (location city5-1)
          (location city4-11)
          (location city4-10)
          (location city4-9)
          (location city4-8)
          (location city4-7)
          (location city4-6)
          (location city4-5)
          (location city4-4)
          (location city4-3)
          (location city4-2)
          (location city4-1)
          (location city3-11)
          (location city3-10)
          (location city3-9)
          (location city3-8)
          (location city3-7)
          (location city3-6)
          (location city3-5)
          (location city3-4)
          (location city3-3)
          (location city3-2)
          (location city3-1)
          (location city2-11)
          (location city2-10)
          (location city2-9)
          (location city2-8)
          (location city2-7)
          (location city2-6)
          (location city2-5)
          (location city2-4)
          (location city2-3)
          (location city2-2)
          (location city2-1)
          (location city1-11)
          (location city1-10)
          (location city1-9)
          (location city1-8)
          (location city1-7)
          (location city1-6)
          (location city1-5)
          (location city1-4)
          (location city1-3)
          (location city1-2)
          (location city1-1)
          (airport city9-12)
          (location city9-12)
          (airport city8-12)
          (location city8-12)
          (airport city7-12)
          (location city7-12)
          (airport city6-12)
          (location city6-12)
          (airport city5-12)
          (location city5-12)
          (airport city4-12)
          (location city4-12)
          (airport city3-12)
          (location city3-12)
          (airport city2-12)
          (location city2-12)
          (airport city1-12)
          (location city1-12)
          (in-city city9-12 city9)
          (in-city city9-11 city9)
          (in-city city9-10 city9)
          (in-city city9-9 city9)
          (in-city city9-8 city9)
          (in-city city9-7 city9)
          (in-city city9-6 city9)
          (in-city city9-5 city9)
          (in-city city9-4 city9)
          (in-city city9-3 city9)
          (in-city city9-2 city9)
          (in-city city9-1 city9)
          (in-city city8-12 city8)
          (in-city city8-11 city8)
          (in-city city8-10 city8)
          (in-city city8-9 city8)
          (in-city city8-8 city8)
          (in-city city8-7 city8)
          (in-city city8-6 city8)
          (in-city city8-5 city8)
          (in-city city8-4 city8)
          (in-city city8-3 city8)
          (in-city city8-2 city8)
          (in-city city8-1 city8)
          (in-city city7-12 city7)
          (in-city city7-11 city7)
          (in-city city7-10 city7)
          (in-city city7-9 city7)
          (in-city city7-8 city7)
          (in-city city7-7 city7)
          (in-city city7-6 city7)
          (in-city city7-5 city7)
          (in-city city7-4 city7)
          (in-city city7-3 city7)
          (in-city city7-2 city7)
          (in-city city7-1 city7)
          (in-city city6-12 city6)
          (in-city city6-11 city6)
          (in-city city6-10 city6)
          (in-city city6-9 city6)
          (in-city city6-8 city6)
          (in-city city6-7 city6)
          (in-city city6-6 city6)
          (in-city city6-5 city6)
          (in-city city6-4 city6)
          (in-city city6-3 city6)
          (in-city city6-2 city6)
          (in-city city6-1 city6)
          (in-city city5-12 city5)
          (in-city city5-11 city5)
          (in-city city5-10 city5)
          (in-city city5-9 city5)
          (in-city city5-8 city5)
          (in-city city5-7 city5)
          (in-city city5-6 city5)
          (in-city city5-5 city5)
          (in-city city5-4 city5)
          (in-city city5-3 city5)
          (in-city city5-2 city5)
          (in-city city5-1 city5)
          (in-city city4-12 city4)
          (in-city city4-11 city4)
          (in-city city4-10 city4)
          (in-city city4-9 city4)
          (in-city city4-8 city4)
          (in-city city4-7 city4)
          (in-city city4-6 city4)
          (in-city city4-5 city4)
          (in-city city4-4 city4)
          (in-city city4-3 city4)
          (in-city city4-2 city4)
          (in-city city4-1 city4)
          (in-city city3-12 city3)
          (in-city city3-11 city3)
          (in-city city3-10 city3)
          (in-city city3-9 city3)
          (in-city city3-8 city3)
          (in-city city3-7 city3)
          (in-city city3-6 city3)
          (in-city city3-5 city3)
          (in-city city3-4 city3)
          (in-city city3-3 city3)
          (in-city city3-2 city3)
          (in-city city3-1 city3)
          (in-city city2-12 city2)
          (in-city city2-11 city2)
          (in-city city2-10 city2)
          (in-city city2-9 city2)
          (in-city city2-8 city2)
          (in-city city2-7 city2)
          (in-city city2-6 city2)
          (in-city city2-5 city2)
          (in-city city2-4 city2)
          (in-city city2-3 city2)
          (in-city city2-2 city2)
          (in-city city2-1 city2)
          (in-city city1-12 city1)
          (in-city city1-11 city1)
          (in-city city1-10 city1)
          (in-city city1-9 city1)
          (in-city city1-8 city1)
          (in-city city1-7 city1)
          (in-city city1-6 city1)
          (in-city city1-5 city1)
          (in-city city1-4 city1)
          (in-city city1-3 city1)
          (in-city city1-2 city1)
          (in-city city1-1 city1)
          (at plane8 city6-12)
          (at plane7 city6-12)
          (at plane6 city9-12)
          (at plane5 city3-12)
          (at plane4 city9-12)
          (at plane3 city2-12)
          (at plane2 city7-12)
          (at plane1 city6-12)
          (at truck54 city9-4)
          (at truck53 city8-5)
          (at truck52 city7-11)
          (at truck51 city6-1)
          (at truck50 city5-4)
          (at truck49 city4-5)
          (at truck48 city3-2)
          (at truck47 city2-8)
          (at truck46 city1-9)
          (at truck45 city5-4)
          (at truck44 city7-6)
          (at truck43 city8-5)
          (at truck42 city3-2)
          (at truck41 city6-8)
          (at truck40 city2-3)
          (at truck39 city2-10)
          (at truck38 city2-12)
          (at truck37 city2-5)
          (at truck36 city5-7)
          (at truck35 city1-3)
          (at truck34 city1-11)
          (at truck33 city6-10)
          (at truck32 city1-2)
          (at truck31 city3-3)
          (at truck30 city5-12)
          (at truck29 city2-4)
          (at truck28 city3-3)
          (at truck27 city2-3)
          (at truck26 city3-3)
          (at truck25 city4-1)
          (at truck24 city1-12)
          (at truck23 city1-1)
          (at truck22 city7-11)
          (at truck21 city4-5)
          (at truck20 city8-6)
          (at truck19 city1-4)
          (at truck18 city4-7)
          (at truck17 city4-10)
          (at truck16 city1-11)
          (at truck15 city9-7)
          (at truck14 city3-3)
          (at truck13 city1-4)
          (at truck12 city4-8)
          (at truck11 city8-9)
          (at truck10 city6-7)
          (at truck9 city6-10)
          (at truck8 city1-1)
          (at truck7 city4-3)
          (at truck6 city4-5)
          (at truck5 city4-5)
          (at truck4 city9-8)
          (at truck3 city4-8)
          (at truck2 city1-5)
          (at truck1 city9-7)
          (at package11 city5-4)
          (at package10 city2-5)
          (at package9 city6-5)
          (at package8 city1-5)
          (at package7 city3-9)
          (at package6 city6-9)
          (at package5 city4-12)
          (at package4 city7-1)
          (at package3 city6-6)
          (at package2 city2-7)
          (at package1 city8-1))
   (:goal (and (at package11 city8-8)
               (at package10 city3-2)
               (at package9 city1-11)
               (at package8 city3-4)))
                        (:metric minimize (total-cost))
               )
