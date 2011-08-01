(define (problem birmingham_1)
  (:domain birmingham)
  (:objects c - coin)
  (:init 
         (probabilistic 8/9 (not-flattire))
         (goal-location n0)
  )

  (:metric maximize (reward))   
)
