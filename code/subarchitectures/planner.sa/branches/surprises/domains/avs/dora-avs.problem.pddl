(define (problem test)

(:domain dora-avs-iros11)

(:objects  dora - robot
           p1a p2a p3a p4a p1b p2b p3b p1c p2c p3c - place
           f1a - place
           cinrb1 cinrb2 cinrb3 - conegroup
           tinrb1 tinrb2 tinrb3 - conegroup
           cont1 cont2 cont3 - conegroup
           ra rb rc - room
           kitchen office living_room corridor - category
           cereal_box book mug table dishwasher cupboard bookcase - label
           h1 - human
           ;; kitchen_pantry dining_room bathroom - category
           ;;cereal_box table mug - label oven fridge book board-game 
           ;; cereal_box - label
           table01 - visualobject
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

        ;; (poss (in-room p1a) ra)
        ;; (poss (in-room p2a) ra)
        ;; (poss (in-room p3a) ra)
        ;; (poss (in-room p4a) ra)
        ;; (poss (in-room p1b) rb)
        ;; (poss (in-room p2b) rb)
        ;; (poss (in-room p3b) rb)
        ;; (poss (in-room p1c) rc)
        ;; (poss (in-room p2c) rc)
        ;; (poss (in-room p3c) rb)

        (= (placestatus p1a) trueplace)
        (= (placestatus p2a) trueplace)
        (= (placestatus p3a) trueplace)
        (= (placestatus p4a) trueplace)
        (= (placestatus p1b) trueplace)
        (= (placestatus p2b) trueplace)
        (= (placestatus p3b) trueplace)
        (= (placestatus p1c) trueplace)
        (= (placestatus p2c) trueplace)
        (= (placestatus p3c) trueplace)

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

        (= (placestatus f1a) placeholder)
        (connected p1a f1a)
        (probabilistic 0.3  (assign (leads_to_room f1a office) true))
        (probabilistic 0.2  (assign (leads_to_room f1a kitchen) true))
        (probabilistic 0.1  (assign (leads_to_room f1a living_room) true))

        (assign-probabilistic (category ra)
                              0.7 kitchen
                              0.1 corridor
                              0.2 living_room)

        ;; (assign-probabilistic (category rb)
        ;;                       ;; 0.6 kitchen
        ;;                       0.2 office
        ;;                       0.2 living_room)
        (= (category rb) kitchen)
        ;; (poss (category rb) kitchen)

        (assign-probabilistic (category rc)
                              ;; 0.2 kitchen
                              0.3 office
                              0.5 living_room)

        (= (label table01) table)
        (= (related-to table01) rb)
        (= (relation table01 rb) in)
        ;; (poss (related-to table01) rb)
        ;; (poss (relation table01 rb) in)

        (= (cg-label cinrb1) cereal_box)
        (= (cg-label cinrb2) cereal_box)
        (= (cg-label cinrb3) cereal_box)
        (= (cg-place cinrb1) p1b)
        (= (cg-place cinrb2) p2b)
        (= (cg-place cinrb3) p3b)
        (= (cg-relation cinrb1) in)
        (= (cg-relation cinrb2) in)
        (= (cg-relation cinrb3) in)
        (= (cg-related-to cinrb1) rb)
        (= (cg-related-to cinrb2) rb)
        (= (cg-related-to cinrb3) rb)
        (= (p-visible cinrb1) 0.3)
        (= (p-visible cinrb2) 0.3)
        (= (p-visible cinrb3) 0.2)

        (= (cg-label tinrb1) table)
        (= (cg-label tinrb2) table)
        (= (cg-label tinrb3) table)
        (= (cg-place tinrb1) p1b)
        (= (cg-place tinrb2) p2b)
        (= (cg-place tinrb3) p3b)
        (= (cg-relation tinrb1) in)
        (= (cg-relation tinrb2) in)
        (= (cg-relation tinrb3) in)
        (= (cg-related-to tinrb1) rb)
        (= (cg-related-to tinrb2) rb)
        (= (cg-related-to tinrb3) rb)
        (= (p-visible tinrb1) 0.3)
        (= (p-visible tinrb2) 0.3)
        (= (p-visible tinrb3) 0.2)

        (= (cg-label cont1) cereal_box)
        (= (cg-label cont2) cereal_box)
        (= (cg-label cont3) cereal_box)
        (= (cg-place cont1) p1b)
        (= (cg-place cont2) p2b)
        (= (cg-place cont3) p2b)
        (= (cg-relation cont1) on)
        (= (cg-relation cont2) on)
        (= (cg-relation cont3) on)
        (= (cg-related-to cont1) table01)
        (= (cg-related-to cont2) table01)
        (= (cg-related-to cont3) table01)
        (= (p-visible cont1) 0.3)
        (= (p-visible cont2) 0.3)
        (= (p-visible cont3) 0.2)


        ;; (= (default_room_search_cost cereal_box) 200)
        ;; (= (default_room_search_cost mug) 200)
        ;; (= (default_room_search_cost book) 200)
        ;; (= (default_room_search_cost table) 50)
        ;; (= (default_room_search_cost cupboard) 50)
        ;; (= (default_room_search_cost bookcase) 50)
        ;; (= (default_room_search_cost dishwasher) 50)

        ;; (= (default_search_cost cereal_box on table) 30)
        ;; (= (default_search_cost mug on table) 30)
        ;; (= (default_search_cost book on table) 30)

        ;; (= (default_search_cost cereal_box in dishwasher) 50)
        ;; (= (default_search_cost mug in dishwasher) 50)
        ;; (= (default_search_cost book in dishwasher) 50)

        ;; (= (default_search_cost cereal_box on dishwasher) 30)
        ;; (= (default_search_cost mug on dishwasher) 30)
        ;; (= (default_search_cost book on dishwasher) 30)

        ;; (= (default_search_cost cereal_box in cupboard) 40)
        ;; (= (default_search_cost mug in cupboard) 40)
        ;; (= (default_search_cost book in cupboard) 40)

        ;; (= (default_search_cost cereal_box in bookcase) 40)
        ;; (= (default_search_cost mug in bookcase) 40)
        ;; (= (default_search_cost book in bookcase) 40)

        ;; (= (dora__in_room cereal_box kitchen) 0.1)
        ;; (= (dora__in_room mug kitchen) 0.8)
        ;; (= (dora__in_room table kitchen) 0.9)
        ;; (= (dora__in_room dishwasher kitchen) 0.7)
        ;; (= (dora__in_room cupboard kitchen) 0.9)
        ;; (= (dora__in_room bookcase kitchen) 0.1)

        ;; (= (dora__on_obj cereal_box table kitchen) 0.7)
        ;; (= (dora__in_obj cereal_box cupboard kitchen) 0.7)
        ;; (= (dora__in_obj cereal_box bookcase kitchen) 0.1)
        ;; (= (dora__on_obj mug table kitchen) 0.7)
        ;; (= (dora__in_obj mug cupboard kitchen) 0.9)
        ;; (= (dora__in_obj mug dishwasher kitchen) 0.7)
        ;; (= (dora__on_obj book table kitchen) 0.1)
        ;; (= (dora__in_obj book bookcase kitchen) 0.9)


        ;; (= (dora__in_room cereal_box office) 0.1)
        ;; (= (dora__in_room mug office) 0.7)
        ;; (= (dora__in_room book office) 0.8)
        ;; (= (dora__in_room table office) 0.9)
        ;; (= (dora__in_room cupboard office) 0.1)
        ;; (= (dora__in_room bookcase office) 0.7)

        ;; (= (dora__on_obj cereal_box table office) 0.1)
        ;; (= (dora__in_obj cereal_box cupboard office) 0.3)
        ;; (= (dora__in_obj cereal_box bookcase office) 0.05)
        ;; (= (dora__on_obj mug table office) 0.7)
        ;; (= (dora__in_obj mug cupboard office) 0.9)
        ;; (= (dora__in_obj mug bookcase office) 0.2)
        ;; (= (dora__on_obj book table office) 0.6)
        ;; (= (dora__in_obj book cupboard office) 0.1)
        ;; (= (dora__in_obj book bookcase office) 0.9)

        ;; (= (dora__in_room cereal_box living_room) 0.2)
        ;; (= (dora__in_room mug living_room) 0.7)
        ;; (= (dora__in_room book living_room) 0.5)
        ;; (= (dora__in_room table living_room) 0.7)
        ;; (= (dora__in_room cupboard living_room) 0.1)
        ;; (= (dora__in_room bookcase living_room) 0.8)

        ;; (= (dora__on_obj cereal_box table living_room) 0.2)
        ;; (= (dora__in_obj cereal_box cupboard living_room) 0.3)
        ;; (= (dora__in_obj cereal_box bookcase living_room) 0.05)
        ;; (= (dora__on_obj mug table living_room) 0.5)
        ;; (= (dora__in_obj mug cupboard living_room) 0.9)
        ;; (= (dora__in_obj mug bookcase living_room) 0.2)
        ;; (= (dora__on_obj book table living_room) 0.7)
        ;; (= (dora__in_obj book bookcase living_room) 0.9)

        ;; (= (dora__in_room cereal_box corridor) 0.1)
        ;; (= (dora__in_room mug corridor) 0.2)
        ;; (= (dora__in_room book corridor) 0.2)
        ;; (= (dora__in_room table corridor) 0.2)
        ;; (= (dora__in_room cupboard corridor) 0.4)
        ;; (= (dora__in_room bookcase corridor) 0.5)

        ;; (= (dora__on_obj cereal_box table corridor) 0.1)
        ;; (= (dora__in_obj cereal_box cupboard corridor) 0.2)
        ;; (= (dora__on_obj mug table corridor) 0.3)
        ;; (= (dora__in_obj mug cupboard corridor) 0.9)
        ;; (= (dora__in_obj mug bookcase corridor) 0.05)
        ;; (= (dora__on_obj book table corridor) 0.5)
        ;; (= (dora__in_obj book bookcase corridor) 0.9)

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

(:goal  (and (exists (?o - visualobject) (and (= (label ?o) cereal_box)
                                              (position-reported ?o)))))

;; (:goal  (and (exists (?o - visualobject) (and (= (label ?o) mug)
;;                                               (trans_related ?o rc)
;;                                               (position-reported ?o)))))

;; (:goal  (and (exists (?o - visualobject ?r - room) (and (= (label ?o) mug)
;;                                                         (poss (category ?r) office)
;;                                                         (trans_related ?o ?r)
;;                                                         (position-reported ?o)))))

)