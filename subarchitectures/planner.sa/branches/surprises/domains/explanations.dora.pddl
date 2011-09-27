(:ow-rule unknown_cupboard
 :parameters (?r - room)
 :precondition (and (not (exists (?cb - cupboard) (= (obj_exists ?cb in ?r) true))))
 :effect (create (?cb - cupboard) (assign (obj_exists ?cb in ?r) true))
)

(:ow-rule object_possibly_in_container
 :parameters (?cb - cupboard ?l - label ?r - room)
 :precondition (and (= (obj_exists ?cb in ?r) true) (obj-possibly-in-room ?l ?r))
 :effect (assign (obj_exists ?l in ?cb) true))
)

