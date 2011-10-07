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
                    (not (started))
    (= (label ?cont) container))
 :effect (and 
     (assign (related-to ?cont ) ?r) (assign (relation ?cont ) in)
    )
)

(:action object_possibly_in_container
 :parameters (?o ?cont - visualobject ?l - label ?r - room)
 :precondition (and (= (label ?cont) container) 
                    (= (label ?o) ?l)
                    (poss (obj_exists ?l in ?r) true)
                    (= (related-to ?cont) ?r) (= (relation ?cont ) in)
                    (not (started))
    )
 :effect (and 
          (assign (probability) 0.9)
          (poss (related-to ?o) ?cont)
          (poss (relation ?o) in)
    )
)

(:action obj_not_in_room
         :parameters (?l - label ?r - room ?c - category)
         :precondition (and (poss (category ?r) ?c)
                            (defined (dora__inroom ?l ?c)))
         :effect (and (assign (probability) (dora__not_inroom ?l ?c))
                      (poss (obj_exists ?l in ?r) false)))

(:action sample_object_unknown
         :parameters (?o - visualobject ?l - label ?rel - spatial_relation ?where - (either visualobject room))
         :precondition (and (= (label ?o) ?l)
                            (is-virtual ?o)
                            (not (exists (?rel2 - spatial_relation ?where2 - (either visualobject room))
                                         (poss (obj_exists ?l ?rel ?where) true)))
                            (poss (obj_exists ?l ?rel ?where) false))
         :effect (and (poss (related-to ?o) unknown-room)
                      (poss (relation ?o) unknown-relation))
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
