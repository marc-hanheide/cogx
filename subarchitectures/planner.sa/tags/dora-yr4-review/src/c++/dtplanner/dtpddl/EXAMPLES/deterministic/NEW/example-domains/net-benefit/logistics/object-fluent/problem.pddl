(define (problem logistics-4-0)

(:domain logistics-object-fluents)

(:objects  apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 apt2 - airport
           pos1 pos2 - location
           cit1 cit2 - city)

(:init  (= (total-cost) 0)
        (= (location-of apn1) apt2)
        (= (location-of tru1) pos1)
        (= (location-of tru2) pos2)
        (= (location-of obj11) pos1)
        (= (location-of obj12) pos1)
        (= (location-of obj13) pos1)
        (= (location-of obj21) pos2)
        (= (location-of obj22) pos2)
        (= (location-of obj23) pos2)
        (= (city-of apt1) cit1)
        (= (city-of apt2) cit2)
        (= (city-of pos1) cit1)
        (= (city-of pos2) cit2))

(:goal  (and (preference g0 (= (location-of obj11) apt1))
             (preference g1 (= (location-of obj13) apt1))
             (preference g2 (= (location-of obj21) pos1))
             (preference g3 (= (location-of obj23) pos1))))

(:metric maximize
        (- 34.5
        (+ (* (is-violated g0) 3)
            (* (is-violated g1) 6.5)
            (* (is-violated g2) 9)
            (* (is-violated g3) 16)
            (total-cost))))
)
