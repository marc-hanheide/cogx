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


(:action create_cones_in_room
         :agent (?a - robot)
         :parameters (?l - label ?r - room)
         :variables (?p - place)
         :precondition (and (= (is-in ?a) ?p)
                            (= (in-room ?p) ?r)
                            (not (not_fully_explored ?r))
                            (not (done)))
         :effect (and (cones_created ?l in ?r)
                      (increase (total-cost) 5))
         )

(:action search_for_object_in_room
         :agent (?a - robot)
         :parameters (?l - label ?r - room)
         :variables (?p - place ?o - visualobject)
         :precondition (and (= (is-in ?a) ?p)
                            (= (in-room ?p) ?r)
                            (= (label ?o) ?l)
                            (or (cones_created ?l in ?r)
                                (cones_exist ?l in ?r))
                            (not (done)))
         :effect (and (increase (total-cost) (search_cost ?l in ?r))
                      (when (and (poss (related-to ?o) ?r)
                                 (poss (relation ?o) in))
                        (kd ?a (related-to ?o))))
         )



)
