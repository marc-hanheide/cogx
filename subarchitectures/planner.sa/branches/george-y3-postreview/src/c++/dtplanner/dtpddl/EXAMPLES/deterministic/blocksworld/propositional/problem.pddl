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
	(HANDEMPTY)
	(GON D C) 
	(GON C B) 
	(GON B A)
	(= (total-cost ) 0)
)

(:metric maximise (reward))
)
