(define (problem BLOCKS-6-2)
(:domain BLOCKS)
(:objects E F B D C A  - block )
(:INIT  (= (total-cost) 0)  (= (total-cost) 0) (CLEAR A) (ONTABLE C) (ON A D) (ON D B) (ON B F) (ON F E) (ON E C)
 (HANDEMPTY))
(:goal (AND (ON E F) (ON F A) (ON A B) (ON B C) (ON C D)))
(:metric minimize (total-cost))
)
