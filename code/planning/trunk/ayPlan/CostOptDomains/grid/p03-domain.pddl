(define (domain grid)
(:requirements :typing :action-costs)
 (:types block)

(:predicates (conn ?x - block  ?y - block )
             (key-shape ?k - block  ?s - block )
             (lock-shape ?x - block  ?s - block )
             (at ?r - block  ?x - block  )
	     (at-robot ?x - block )
             (place ?p - block )
             (key ?k - block )
             (shape ?s - block )
             (locked ?x - block )
             (holding ?k - block )
             (open ?x - block )
             (arm-empty ))

 (:functions
     (total-cost) - number
  )

(:action unlock
:parameters (?curpos - block  ?lockpos - block  ?key - block  ?shape - block )
:precondition (and (place ?curpos) (place ?lockpos) (key ?key) (shape ?shape)
          (conn ?curpos ?lockpos) (key-shape ?key ?shape)
                   (lock-shape ?lockpos ?shape) (at-robot ?curpos) 
                   (locked ?lockpos) (holding ?key))
:effect (and  (open ?lockpos) (not (locked ?lockpos))
(increase (total-cost) 1))
)


(:action move
:parameters (?curpos - block  ?nextpos - block )
:precondition (and (place ?curpos) (place ?nextpos)
               (at-robot ?curpos) (conn ?curpos ?nextpos) (open ?nextpos))
:effect (and (at-robot ?nextpos) (not (at-robot ?curpos))
(increase (total-cost) 1))
)

(:action pickup
:parameters (?curpos - block  ?key - block )
:precondition (and (place ?curpos) (key ?key) 
                  (at-robot ?curpos) (at ?key ?curpos) (arm-empty ))
:effect (and (holding ?key)
   (not (at ?key ?curpos)) (not (arm-empty ))
   (increase (total-cost) 1))
   )


(:action pickup-and-loose
:parameters (?curpos - block  ?newkey - block  ?oldkey - block )
:precondition (and (place ?curpos) (key ?newkey) (key ?oldkey)
                  (at-robot ?curpos) (holding ?oldkey) (at ?newkey ?curpos))
:effect (and (holding ?newkey) (at ?oldkey ?curpos)
        (not (holding ?oldkey)) (not (at ?newkey ?curpos))
        (increase (total-cost) 1))
        )

(:action putdown
:parameters (?curpos - block  ?key - block )
:precondition (and (place ?curpos) (key ?key) 
                  (at-robot ?curpos) (holding ?key))
:effect (and (arm-empty ) (at ?key ?curpos) (not (holding ?key))
(increase (total-cost) 1))


)



)


	

