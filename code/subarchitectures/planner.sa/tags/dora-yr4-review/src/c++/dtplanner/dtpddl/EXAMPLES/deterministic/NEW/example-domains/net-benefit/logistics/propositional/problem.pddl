(define (problem logistics-4-0)

(:domain logistics)

(:objects  apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 apt2 - airport
           pos1 pos2 - location
           cit1 cit2 - city)

(:init  (at apn1 apt2)
        (at tru1 pos1)
        (at tru2 pos2)
        (at obj11 pos1)
        (at obj12 pos1)
        (at obj13 pos1)
        (at obj21 pos2)
        (at obj22 pos2)
        (at obj23 pos2)
        (in-city apt1 cit1)
        (in-city apt2 cit2)
        (in-city pos1 cit1)
        (in-city pos2 cit2)
        (= (total-cost) 0))

(:goal  (and
	(preference g0 (at obj11 apt1))
             (preference g1 (at obj13 apt1))
             (preference g2 (at obj21 pos1))
             (preference g3 (at obj23 pos1))))

(:metric minimize
        (+ (* (is-violated g0) 334.3)
            (* (is-violated g1) 76.5)
            (* (is-violated g2) 177.9)
            (* (is-violated g3) 150.0)
            (total-cost))))
)
