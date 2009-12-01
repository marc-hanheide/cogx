(define (problem strips-log-x-21)
   (:domain logistics-strips)
   (:objects package11 package10 package9 package8 package7
             package6 package5 package4 package3 package2 package1 city14
             city13 city12 city11 city10 city9 city8 city7 city6 city5 city4
             city3 city2 city1 truck40 truck39 truck38 truck37 truck36
             truck35 truck34 truck33 truck32 truck31 truck30 truck29
             truck28 truck27 truck26 truck25 truck24 truck23 truck22
             truck21 truck20 truck19 truck18 truck17 truck16 truck15
             truck14 truck13 truck12 truck11 truck10 truck9 truck8 truck7
             truck6 truck5 truck4 truck3 truck2 truck1 plane13 plane12
             plane11 plane10 plane9 plane8 plane7 plane6 plane5 plane4
             plane3 plane2 plane1 city14-8 city14-7 city14-6 city14-5
             city14-4 city14-3 city14-2 city14-1 city13-8 city13-7 city13-6
             city13-5 city13-4 city13-3 city13-2 city13-1 city12-8 city12-7
             city12-6 city12-5 city12-4 city12-3 city12-2 city12-1 city11-8
             city11-7 city11-6 city11-5 city11-4 city11-3 city11-2 city11-1
             city10-8 city10-7 city10-6 city10-5 city10-4 city10-3 city10-2
             city10-1 city9-8 city9-7 city9-6 city9-5 city9-4 city9-3
             city9-2 city9-1 city8-8 city8-7 city8-6 city8-5 city8-4
             city8-3 city8-2 city8-1 city7-8 city7-7 city7-6 city7-5
             city7-4 city7-3 city7-2 city7-1 city6-8 city6-7 city6-6
             city6-5 city6-4 city6-3 city6-2 city6-1 city5-8 city5-7
             city5-6 city5-5 city5-4 city5-3 city5-2 city5-1 city4-8
             city4-7 city4-6 city4-5 city4-4 city4-3 city4-2 city4-1
             city3-8 city3-7 city3-6 city3-5 city3-4 city3-3 city3-2
             city3-1 city2-8 city2-7 city2-6 city2-5 city2-4 city2-3
             city2-2 city2-1 city1-8 city1-7 city1-6 city1-5 city1-4
             city1-3 city1-2 city1-1 city14-9 city13-9 city12-9 city11-9
             city10-9 city9-9 city8-9 city7-9 city6-9 city5-9 city4-9
             city3-9 city2-9 city1-9 - block )
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
          (city city14)
          (city city13)
          (city city12)
          (city city11)
          (city city10)
          (city city9)
          (city city8)
          (city city7)
          (city city6)
          (city city5)
          (city city4)
          (city city3)
          (city city2)
          (city city1)
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
          (airplane plane13)
          (airplane plane12)
          (airplane plane11)
          (airplane plane10)
          (airplane plane9)
          (airplane plane8)
          (airplane plane7)
          (airplane plane6)
          (airplane plane5)
          (airplane plane4)
          (airplane plane3)
          (airplane plane2)
          (airplane plane1)
          (location city14-8)
          (location city14-7)
          (location city14-6)
          (location city14-5)
          (location city14-4)
          (location city14-3)
          (location city14-2)
          (location city14-1)
          (location city13-8)
          (location city13-7)
          (location city13-6)
          (location city13-5)
          (location city13-4)
          (location city13-3)
          (location city13-2)
          (location city13-1)
          (location city12-8)
          (location city12-7)
          (location city12-6)
          (location city12-5)
          (location city12-4)
          (location city12-3)
          (location city12-2)
          (location city12-1)
          (location city11-8)
          (location city11-7)
          (location city11-6)
          (location city11-5)
          (location city11-4)
          (location city11-3)
          (location city11-2)
          (location city11-1)
          (location city10-8)
          (location city10-7)
          (location city10-6)
          (location city10-5)
          (location city10-4)
          (location city10-3)
          (location city10-2)
          (location city10-1)
          (location city9-8)
          (location city9-7)
          (location city9-6)
          (location city9-5)
          (location city9-4)
          (location city9-3)
          (location city9-2)
          (location city9-1)
          (location city8-8)
          (location city8-7)
          (location city8-6)
          (location city8-5)
          (location city8-4)
          (location city8-3)
          (location city8-2)
          (location city8-1)
          (location city7-8)
          (location city7-7)
          (location city7-6)
          (location city7-5)
          (location city7-4)
          (location city7-3)
          (location city7-2)
          (location city7-1)
          (location city6-8)
          (location city6-7)
          (location city6-6)
          (location city6-5)
          (location city6-4)
          (location city6-3)
          (location city6-2)
          (location city6-1)
          (location city5-8)
          (location city5-7)
          (location city5-6)
          (location city5-5)
          (location city5-4)
          (location city5-3)
          (location city5-2)
          (location city5-1)
          (location city4-8)
          (location city4-7)
          (location city4-6)
          (location city4-5)
          (location city4-4)
          (location city4-3)
          (location city4-2)
          (location city4-1)
          (location city3-8)
          (location city3-7)
          (location city3-6)
          (location city3-5)
          (location city3-4)
          (location city3-3)
          (location city3-2)
          (location city3-1)
          (location city2-8)
          (location city2-7)
          (location city2-6)
          (location city2-5)
          (location city2-4)
          (location city2-3)
          (location city2-2)
          (location city2-1)
          (location city1-8)
          (location city1-7)
          (location city1-6)
          (location city1-5)
          (location city1-4)
          (location city1-3)
          (location city1-2)
          (location city1-1)
          (airport city14-9)
          (location city14-9)
          (airport city13-9)
          (location city13-9)
          (airport city12-9)
          (location city12-9)
          (airport city11-9)
          (location city11-9)
          (airport city10-9)
          (location city10-9)
          (airport city9-9)
          (location city9-9)
          (airport city8-9)
          (location city8-9)
          (airport city7-9)
          (location city7-9)
          (airport city6-9)
          (location city6-9)
          (airport city5-9)
          (location city5-9)
          (airport city4-9)
          (location city4-9)
          (airport city3-9)
          (location city3-9)
          (airport city2-9)
          (location city2-9)
          (airport city1-9)
          (location city1-9)
          (in-city city14-9 city14)
          (in-city city14-8 city14)
          (in-city city14-7 city14)
          (in-city city14-6 city14)
          (in-city city14-5 city14)
          (in-city city14-4 city14)
          (in-city city14-3 city14)
          (in-city city14-2 city14)
          (in-city city14-1 city14)
          (in-city city13-9 city13)
          (in-city city13-8 city13)
          (in-city city13-7 city13)
          (in-city city13-6 city13)
          (in-city city13-5 city13)
          (in-city city13-4 city13)
          (in-city city13-3 city13)
          (in-city city13-2 city13)
          (in-city city13-1 city13)
          (in-city city12-9 city12)
          (in-city city12-8 city12)
          (in-city city12-7 city12)
          (in-city city12-6 city12)
          (in-city city12-5 city12)
          (in-city city12-4 city12)
          (in-city city12-3 city12)
          (in-city city12-2 city12)
          (in-city city12-1 city12)
          (in-city city11-9 city11)
          (in-city city11-8 city11)
          (in-city city11-7 city11)
          (in-city city11-6 city11)
          (in-city city11-5 city11)
          (in-city city11-4 city11)
          (in-city city11-3 city11)
          (in-city city11-2 city11)
          (in-city city11-1 city11)
          (in-city city10-9 city10)
          (in-city city10-8 city10)
          (in-city city10-7 city10)
          (in-city city10-6 city10)
          (in-city city10-5 city10)
          (in-city city10-4 city10)
          (in-city city10-3 city10)
          (in-city city10-2 city10)
          (in-city city10-1 city10)
          (in-city city9-9 city9)
          (in-city city9-8 city9)
          (in-city city9-7 city9)
          (in-city city9-6 city9)
          (in-city city9-5 city9)
          (in-city city9-4 city9)
          (in-city city9-3 city9)
          (in-city city9-2 city9)
          (in-city city9-1 city9)
          (in-city city8-9 city8)
          (in-city city8-8 city8)
          (in-city city8-7 city8)
          (in-city city8-6 city8)
          (in-city city8-5 city8)
          (in-city city8-4 city8)
          (in-city city8-3 city8)
          (in-city city8-2 city8)
          (in-city city8-1 city8)
          (in-city city7-9 city7)
          (in-city city7-8 city7)
          (in-city city7-7 city7)
          (in-city city7-6 city7)
          (in-city city7-5 city7)
          (in-city city7-4 city7)
          (in-city city7-3 city7)
          (in-city city7-2 city7)
          (in-city city7-1 city7)
          (in-city city6-9 city6)
          (in-city city6-8 city6)
          (in-city city6-7 city6)
          (in-city city6-6 city6)
          (in-city city6-5 city6)
          (in-city city6-4 city6)
          (in-city city6-3 city6)
          (in-city city6-2 city6)
          (in-city city6-1 city6)
          (in-city city5-9 city5)
          (in-city city5-8 city5)
          (in-city city5-7 city5)
          (in-city city5-6 city5)
          (in-city city5-5 city5)
          (in-city city5-4 city5)
          (in-city city5-3 city5)
          (in-city city5-2 city5)
          (in-city city5-1 city5)
          (in-city city4-9 city4)
          (in-city city4-8 city4)
          (in-city city4-7 city4)
          (in-city city4-6 city4)
          (in-city city4-5 city4)
          (in-city city4-4 city4)
          (in-city city4-3 city4)
          (in-city city4-2 city4)
          (in-city city4-1 city4)
          (in-city city3-9 city3)
          (in-city city3-8 city3)
          (in-city city3-7 city3)
          (in-city city3-6 city3)
          (in-city city3-5 city3)
          (in-city city3-4 city3)
          (in-city city3-3 city3)
          (in-city city3-2 city3)
          (in-city city3-1 city3)
          (in-city city2-9 city2)
          (in-city city2-8 city2)
          (in-city city2-7 city2)
          (in-city city2-6 city2)
          (in-city city2-5 city2)
          (in-city city2-4 city2)
          (in-city city2-3 city2)
          (in-city city2-2 city2)
          (in-city city2-1 city2)
          (in-city city1-9 city1)
          (in-city city1-8 city1)
          (in-city city1-7 city1)
          (in-city city1-6 city1)
          (in-city city1-5 city1)
          (in-city city1-4 city1)
          (in-city city1-3 city1)
          (in-city city1-2 city1)
          (in-city city1-1 city1)
          (at plane13 city8-9)
          (at plane12 city4-9)
          (at plane11 city2-9)
          (at plane10 city5-9)
          (at plane9 city2-9)
          (at plane8 city7-9)
          (at plane7 city5-9)
          (at plane6 city3-9)
          (at plane5 city11-9)
          (at plane4 city7-9)
          (at plane3 city6-9)
          (at plane2 city11-9)
          (at plane1 city2-9)
          (at truck40 city14-3)
          (at truck39 city13-7)
          (at truck38 city12-7)
          (at truck37 city11-5)
          (at truck36 city10-1)
          (at truck35 city9-3)
          (at truck34 city8-8)
          (at truck33 city7-1)
          (at truck32 city6-3)
          (at truck31 city5-7)
          (at truck30 city4-4)
          (at truck29 city3-1)
          (at truck28 city2-7)
          (at truck27 city1-5)
          (at truck26 city4-6)
          (at truck25 city9-1)
          (at truck24 city9-7)
          (at truck23 city4-4)
          (at truck22 city3-3)
          (at truck21 city7-8)
          (at truck20 city2-6)
          (at truck19 city13-3)
          (at truck18 city5-5)
          (at truck17 city4-5)
          (at truck16 city7-1)
          (at truck15 city7-8)
          (at truck14 city4-9)
          (at truck13 city2-1)
          (at truck12 city2-8)
          (at truck11 city6-2)
          (at truck10 city8-3)
          (at truck9 city3-1)
          (at truck8 city11-7)
          (at truck7 city8-5)
          (at truck6 city4-7)
          (at truck5 city1-3)
          (at truck4 city1-3)
          (at truck3 city2-6)
          (at truck2 city2-9)
          (at truck1 city12-9)
          (at package11 city3-8)
          (at package10 city3-1)
          (at package9 city2-5)
          (at package8 city6-6)
          (at package7 city9-8)
          (at package6 city1-6)
          (at package5 city1-1)
          (at package4 city8-8)
          (at package3 city5-8)
          (at package2 city10-1)
          (at package1 city12-1))
   (:goal (and (at package11 city9-8)
               (at package10 city12-9)
               (at package9 city2-3)
               (at package8 city10-7)
               (at package7 city2-2)
               (at package6 city12-6)
               (at package5 city14-8)
               (at package4 city13-2)
               (at package3 city2-8)
               (at package2 city6-2)
               (at package1 city11-6)))
                        (:metric minimize (total-cost))
               )
