
(define (problem birmingham_1)
  (:domain birmingham)
  (:objects c - coin)
  (:init 
         (probabilistic 0.8 (biased c))
  )

;;  (:metric maximize (reward) ) 
 (:metric 
  minimise 
  (total-cost) )  
  
)
