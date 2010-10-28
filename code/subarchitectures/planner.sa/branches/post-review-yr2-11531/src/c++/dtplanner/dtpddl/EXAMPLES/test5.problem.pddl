
(define (problem birmingham_1)
  (:domain birmingham)
  (:objects c1 c2 - coin)
  (:init 
         (probabilistic 0.5 (biased c1))
         (probabilistic 0.5 (biased c2))
  )

 (:metric maximize (reward) )  
  
)
