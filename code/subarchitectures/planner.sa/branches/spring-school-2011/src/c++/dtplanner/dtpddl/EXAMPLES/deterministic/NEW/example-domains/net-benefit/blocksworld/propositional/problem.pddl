(define (problem BLOCKS-4-0)

(:domain BLOCKS)

(:objects D B A C - block)

(:INIT 	(CLEAR C) 
	(CLEAR A) 
	(CLEAR B) 
	(CLEAR D) 
	(ON-TABLE C) 
	(ON-TABLE A)
	(ON-TABLE B) 
	(ON-TABLE D) 
	(HANDEMPTY))

(:goal (and
	(preference g0 (ON D C))
	(preference g1 (ON C B))
	(ON B A)))

(:metric maximize
	(- 8
        (+ (* (is-violated g0) 3)
            (* (is-violated g1) 5)
            (total-cost))))
)
