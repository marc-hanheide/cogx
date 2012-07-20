(define (problem BLOCKS-4-0)

(:domain BLOCKS-object-fluents)

(:objects D B A C - block)

(:init	(= (total-cost) 0)
	(= (on-block C) no-block) 
	(= (on-block A) no-block) 
	(= (on-block B) no-block) 
	(= (on-block D) no-block)
	(on-table C) 
	(on-table A)
	(on-table B) 
	(on-table D)
	(= (in-hand) no-block))

(:goal (and
	(preference g0 (= (on-block C) D))
	(preference g1 (= (on-block B) C))
	(preference g2 (= (on-block A) B))))

(:metric maximize
         (- 12
            (+ (* (is-violated g0) 4)
               (* (is-violated g1) 3)
               (* (is-violated g2) 5)
               (total-cost)))))
