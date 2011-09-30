(rules (domain dora-avs-iros11)

(:init-rule unknown-container
 :precondition (not (exists (?cont - visualobject) (and
       (= (label ?cont) container)
    )))
 :effect (create (?cont - visualobject) (and 
     (assign (label ?cont) container)
    ))
)

(:action unknown-container-in-room
 :parameters (?r - room ?cont - visualobject)
 :precondition (and 
    (= (label ?cont) container))
 :effect (and 
     (assign (related-to ?cont ) ?r) (assign (relation ?cont ) in)
    )
)

(:action object_possibly_in_container
 :parameters (?o ?cont - visualobject ?r - room)
 :precondition (and (= (label ?cont) container) 
     (= (related-to ?cont) ?r) (= (relation ?cont ) in)
    )
 :effect (and 
     (assign (related-to ?o) ?cont) (assign (relation ?o) in)
    )
)

)