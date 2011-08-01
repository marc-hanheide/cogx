(define (scenario dora-test)

(:domain dora-test-100)

(:common
 (:objects  dora - robot
            human - human
            p1a p2a p1b p2b - place
            ra rb - room
            kitchen office living_room - category
            kitchen_pantry dining_room bathroom - category
            cornflakes table mug oven fridge book board-game - label
            obj1 obj2 obj3 obj4 obj5 obj6 obj7 - visualobject
            ;; table mug cereal_box - label
            ;;c1a c2a c3a;; c4a c5a c6a
            ;;c1b c2b c3b;; c4b c5b - cone
            ;;- cone
            ;;           o-cf o-tbl o-mug o-oven - visualobject
            )

 (:init  (= (is-in dora)  p1a)
         (= (total-p-cost ) 200)

         (= (in-room p1a) ra)
         (= (in-room p2a) ra)
         (= (in-room p1b) rb)
         (= (in-room p2b) rb)
         (connected p1a p2a)
         (connected p2a p1b)
         (connected p1b p2b)
         (= (placestatus p2b) placeholder)

         (= (label obj1) cornflakes)
         (= (label obj2) table)
         (= (label obj3) mug)
         (= (label obj4) oven)
         (= (label obj5) fridge)
         (= (label obj6) book)
         (= (label obj7) board-game)

         ;;         (= (in-room c1a) ra)
         ;;         (= (in-room c2a) ra)
         ;;         (= (in-room c3a) ra)
         ;;         (connected c1a p1a)
         ;;         (connected c2a p1a)
         ;;         (connected c3a p1b)
         ;; ;        (connected c4a p1b)
         ;; ;        (connected c5a p1b)
         ;; ;        (connected c6a p1b)
         ;;         (= (in-room c1b) rb)
         ;;         (= (in-room c2b) rb)
         ;;         (= (in-room c3b) rb)
         ;;         (= (cone-label c1b) cereal_box)
         ;;         (= (cone-label c2b) cereal_box)
         ;;         (= (cone-label c3b) cereal_box)
         ;; ;        (= (room c4b) rb)
         ;; ;        (= (room c5b) rb)
         ;;         (connected c1b p1b)
         ;;         (connected c2b p1b)
         ;;         (connected c3b p2b)
         ;;  ;       (connected c4b p2b)
         ;;  ;       (connected c5b p2b)

         ;; (= (label o-cf) cornflakes)
         ;; (= (label o-tbl) table)
         ;; (= (label o-mug) mug)
                                        ;        (= (label o-oven) oven)

         (assign-probabilistic (category ra)
                               0.1 kitchen
                               0.7 office
                               0.2 living_room)

         (assign-probabilistic (category rb)
                               0.5 kitchen
                               0.1 bathroom
                               0.2 office
                               0.2 living_room)

         (= (dora__in cornflakes kitchen) 0.8)
         (= (dora__in table kitchen) 0.9)
         (= (dora__in mug kitchen) 0.8)
         (= (dora__in oven kitchen) 0.8)
         (= (dora__in fridge kitchen) 0.7)
         (= (dora__in book kitchen) 0.2)
         (= (dora__in board-game kitchen) 0.1)

         (= (dora__in cornflakes office) 0.1)
         (= (dora__in table office) 0.9)
         (= (dora__in mug office) 0.9)
         (= (dora__in oven office) 0.0)
         (= (dora__in fridge office) 0.1)
         (= (dora__in book office) 0.8)
         (= (dora__in board-game office) 0.2)

         (= (dora__in cornflakes living_room) 0.1)
         (= (dora__in table living_room) 0.7)
         (= (dora__in mug living_room) 0.6)
         (= (dora__in oven living_room) 0.0)
         (= (dora__in fridge living_room) 0.0)
         (= (dora__in book living_room) 0.3)
         (= (dora__in board-game living_room) 0.7)

         (= (place-prob p1a cornflakes) 0.5)
         (= (place-prob p2a cornflakes) 0.5)
         (= (place-prob p1a table) 0.6)
         (= (place-prob p2a table) 0.4)
         (= (place-prob p1a mug) 0.6)
         (= (place-prob p2a mug) 0.4)
         (= (place-prob p1a oven) 0.1)
         (= (place-prob p2a oven) 0.9)

         (= (place-prob p1b cornflakes) 0.4)
         (= (place-prob p2b cornflakes) 0.6)
         (= (place-prob p1b table) 0.3)
         (= (place-prob p2b table) 0.7)
         (= (place-prob p1b mug) 0.3)
         (= (place-prob p2b mug) 0.7)
         (= (place-prob p1b oven) 0.8)
         (= (place-prob p2b oven) 0.2)

         ;; (= (p-is-in c1a) 0.5)
         ;; (= (p-is-in c2a) 0.3)
         ;; (= (p-is-in c3a) 0.2)
         ;; (= (p-is-in c4a) 0.1)
         ;; (= (p-is-in c5a) 0.1)
         ;; (= (p-is-in c6a) 0.1)

         ;; (= (p-is-in c1b) 0.6)
         ;; (= (p-is-in c2b) 0.2)
         ;; (= (p-is-in c3b) 0.15)
         ;; (= (p-is-in c4b) 0.15)
         ;; (= (p-is-in c5b) 0.1)

         ))

(:agent dora
        (:init
         (= (is-in obj1) UNKNOWN)
         (= (is-in obj2) UNKNOWN)
         (= (is-in obj3) UNKNOWN)
         (= (is-in obj4) UNKNOWN)
         (= (is-in obj5) UNKNOWN)
         (= (is-in obj6) UNKNOWN)
         (= (is-in obj7) UNKNOWN)
         )
        (:goal  (and (exists (?o - visualobject) (and (= (label ?o) cornflakes)
                                                      (kval dora (is-in ?o)))))

                ))

)