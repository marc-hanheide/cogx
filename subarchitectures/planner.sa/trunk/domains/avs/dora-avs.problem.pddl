(define (problem test)

(:domain dora-avs-iros11)

(:objects  dora - robot
           p1a p2a p3a p4a p1b p2b p3b p1c p2c p3c - place
           ra rb rc - room
           kitchen office living_room corridor - category
           cereal_box book mug table dishwasher cupboard bookcase - label
           h1 - human
           ;; kitchen_pantry dining_room bathroom - category
           ;;cereal_box table mug - label oven fridge book board-game 
           ;; cereal_box - label
)

(:init  (= (is-in dora)  p1a)
        (= (is-in h1)  p2b)

        (= (in-room p1a) ra)
        (= (in-room p2a) ra)
        (= (in-room p3a) ra)
        (= (in-room p4a) ra)
        (= (in-room p1b) rb)
        (= (in-room p2b) rb)
        (= (in-room p3b) rb)
        (= (in-room p1c) rc)
        (= (in-room p2c) rc)
        (= (in-room p3c) rb)

        (connected p1a p2a)
        (connected p2a p3a)
        (connected p3a p4a)
        (connected p1a p1b)
        (connected p1b p2b)
        (connected p2b p3b)
        (connected p3b p1b)
        (connected p3a p1c)
        (connected p1c p2c)
        (connected p2c p3c)
        (connected p3c p1c)

        (assign-probabilistic (category ra) 
                              0.1 kitchen
                              0.7 corridor
                              0.2 living_room)

        (assign-probabilistic (category rb) 
                              0.6 kitchen
                              0.2 office
                              0.2 living_room)

        (assign-probabilistic (category rc) 
                              0.2 kitchen
                              0.3 office
                              0.5 living_room)

        (= (default_room_search_cost cereal_box) 200)
        (= (default_room_search_cost mug) 200)
        (= (default_room_search_cost book) 200)
        (= (default_room_search_cost table) 50)
        (= (default_room_search_cost cupboard) 50)
        (= (default_room_search_cost bookcase) 50)
        (= (default_room_search_cost dishwasher) 50)

        (= (default_search_cost cereal_box on table) 30)
        (= (default_search_cost mug on table) 30)
        (= (default_search_cost book on table) 30)

        (= (default_search_cost cereal_box in dishwasher) 50)
        (= (default_search_cost mug in dishwasher) 50)
        (= (default_search_cost book in dishwasher) 50)

        (= (default_search_cost cereal_box on dishwasher) 30)
        (= (default_search_cost mug on dishwasher) 30)
        (= (default_search_cost book on dishwasher) 30)

        (= (default_search_cost cereal_box in cupboard) 40)
        (= (default_search_cost mug in cupboard) 40)
        (= (default_search_cost book in cupboard) 40)

        (= (default_search_cost cereal_box in bookcase) 40)
        (= (default_search_cost mug in bookcase) 40)
        (= (default_search_cost book in bookcase) 40)

        (= (dora__in_room cereal_box kitchen) 0.7)
        (= (dora__in_room mug kitchen) 0.8)
        (= (dora__in_room table kitchen) 0.9)
        (= (dora__in_room dishwasher kitchen) 0.7)
        (= (dora__in_room cupboard kitchen) 0.9)
        (= (dora__in_room bookcase kitchen) 0.1)

        (= (dora__on_obj cereal_box table kitchen) 0.7)
        (= (dora__in_obj cereal_box cupboard kitchen) 0.7)
        (= (dora__in_obj cereal_box bookcase kitchen) 0.1)
        (= (dora__on_obj mug table kitchen) 0.7)
        (= (dora__in_obj mug cupboard kitchen) 0.9)
        (= (dora__in_obj mug dishwasher kitchen) 0.7)
        (= (dora__on_obj book table kitchen) 0.1)
        (= (dora__in_obj book bookcase kitchen) 0.9)


        (= (dora__in_room cereal_box office) 0.1)
        (= (dora__in_room mug office) 0.7)
        (= (dora__in_room book office) 0.8)
        (= (dora__in_room table office) 0.9)
        (= (dora__in_room cupboard office) 0.1)
        (= (dora__in_room bookcase office) 0.7)

        (= (dora__on_obj cereal_box table office) 0.1)
        (= (dora__in_obj cereal_box cupboard office) 0.3)
        (= (dora__in_obj cereal_box bookcase office) 0.05)
        (= (dora__on_obj mug table office) 0.7)
        (= (dora__in_obj mug cupboard office) 0.9)
        (= (dora__in_obj mug bookcase office) 0.2)
        (= (dora__on_obj book table office) 0.6)
        (= (dora__in_obj book cupboard office) 0.1)
        (= (dora__in_obj book bookcase office) 0.9)

        (= (dora__in_room cereal_box living_room) 0.2)
        (= (dora__in_room mug living_room) 0.7)
        (= (dora__in_room book living_room) 0.5)
        (= (dora__in_room table living_room) 0.7)
        (= (dora__in_room cupboard living_room) 0.1)
        (= (dora__in_room bookcase living_room) 0.8)

        (= (dora__on_obj cereal_box table living_room) 0.2)
        (= (dora__in_obj cereal_box cupboard living_room) 0.3)
        (= (dora__in_obj cereal_box bookcase living_room) 0.05)
        (= (dora__on_obj mug table living_room) 0.5)
        (= (dora__in_obj mug cupboard living_room) 0.9)
        (= (dora__in_obj mug bookcase living_room) 0.2)
        (= (dora__on_obj book table living_room) 0.7)
        (= (dora__in_obj book bookcase living_room) 0.9)

        (= (dora__in_room cereal_box corridor) 0.1)
        (= (dora__in_room mug corridor) 0.2)
        (= (dora__in_room book corridor) 0.2)
        (= (dora__in_room table corridor) 0.2)
        (= (dora__in_room cupboard corridor) 0.4)
        (= (dora__in_room bookcase corridor) 0.5)

        (= (dora__on_obj cereal_box table corridor) 0.1)
        (= (dora__in_obj cereal_box cupboard corridor) 0.2)
        (= (dora__on_obj mug table corridor) 0.3)
        (= (dora__in_obj mug cupboard corridor) 0.9)
        (= (dora__in_obj mug bookcase corridor) 0.05)
        (= (dora__on_obj book table corridor) 0.5)
        (= (dora__in_obj book bookcase corridor) 0.9)

        ;; (= (p-ex-in-room cereal_box kitchen) 0.8)
        ;; (= (p-ex-in-room table kitchen) 0.9)
        ;; (= (p-ex-in-room mug kitchen) 0.8)
        ;; (= (p-ex-in-room oven kitchen) 0.8)
        ;; (= (p-ex-in-room fridge kitchen) 0.7)
        ;; (= (p-ex-in-room book kitchen) 0.2)
        ;; (= (p-ex-in-room board-game kitchen) 0.1)

        ;; (= (p-ex-in-room cereal_box office) 0.1)
        ;; (= (p-ex-in-room table office) 0.9)
        ;; (= (p-ex-in-room mug office) 0.9)
        ;; (= (p-ex-in-room oven office) 0.0)
        ;; (= (p-ex-in-room fridge office) 0.1)
        ;; (= (p-ex-in-room book office) 0.8)
        ;; (= (p-ex-in-room board-game office) 0.2)

        ;; (= (p-ex-in-room cereal_box living-room) 0.1)
        ;; (= (p-ex-in-room table living-room) 0.7)
        ;; (= (p-ex-in-room mug living-room) 0.6)
        ;; (= (p-ex-in-room oven living-room) 0.0)
        ;; (= (p-ex-in-room fridge living-room) 0.0)
        ;; (= (p-ex-in-room book living-room) 0.3)
        ;; (= (p-ex-in-room board-game living-room) 0.7)


)

;; (:goal  (and (exists (?o - visualobject) (and (= (label ?o) cereal_box)
;;                                               (position-reported ?o))))

;; (:goal  (and (exists (?o - visualobject) (and (= (label ?o) mug)
;;                                               (trans_related ?o rc)
;;                                               (position-reported ?o)))))

(:goal  (and (exists (?o - visualobject ?r - room) (and (= (label ?o) mug)
                                                        (poss (category ?r) office)
                                                        (trans_related ?o ?r)
                                                        (position-reported ?o)))))

)