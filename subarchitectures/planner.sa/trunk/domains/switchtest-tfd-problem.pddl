(define (problem test)

(:domain find-objects)

(:objects  dora - robot
           p1a p2a  p1b p2b - place
           ra rb - room
           kitchen office living_room - category
           ;;cornflakes table mug - label oven fridge book board-game 
           cereal_box - label
           c1a c2a c3a;; c4a c5a c6a
           c1b c2b c3b;; c4b c5b - cone
           - cone
;;           o-cf o-tbl o-mug o-oven - visualobject
)

(:init  (= (is-in dora)  p1a)
        (= (total-p-costs) 200)

        (= (in-room p1a) ra)
        (= (in-room p2a) ra)
        (= (in-room p1b) rb)
        (= (in-room p2b) rb)
        (connected p1a p2a)
        (connected p2a p1b)
        (connected p1b p2b)

        (= (in-room c1a) ra)
        (= (in-room c2a) ra)
        (= (in-room c3a) ra)
;        (= (room c4a) ra)
;        (= (room c5a) ra)
;        (= (room c6a) ra)
        (connected c1a p1a)
        (connected c2a p1a)
        (connected c3a p1b)
;        (connected c4a p1b)
;        (connected c5a p1b)
;        (connected c6a p1b)
        (= (in-room c1b) rb)
        (= (in-room c2b) rb)
        (= (in-room c3b) rb)
        (= (cone-label c1b) cereal_box)
        (= (cone-label c2b) cereal_box)
        (= (cone-label c3b) cereal_box)
;        (= (room c4b) rb)
;        (= (room c5b) rb)
        (connected c1b p1b)
        (connected c2b p1b)
        (connected c3b p2b)
 ;       (connected c4b p2b)
 ;       (connected c5b p2b)

        ;; (= (label o-cf) cornflakes)
        ;; (= (label o-tbl) table)
        ;; (= (label o-mug) mug)
;        (= (label o-oven) oven)

        (assign-probabilistic (category ra) 
                              0.1 kitchen
                              0.7 office
                              0.2 living_room)

        (assign-probabilistic (category rb) 
                              0.6 kitchen
                              0.2 office
                              0.2 living_room)

        ;; (= (p-ex-in-room cornflakes kitchen) 0.8)
        ;; (= (p-ex-in-room table kitchen) 0.9)
        ;; (= (p-ex-in-room mug kitchen) 0.8)
        ;; (= (p-ex-in-room oven kitchen) 0.8)
        ;; (= (p-ex-in-room fridge kitchen) 0.7)
        ;; (= (p-ex-in-room book kitchen) 0.2)
        ;; (= (p-ex-in-room board-game kitchen) 0.1)

        ;; (= (p-ex-in-room cornflakes office) 0.1)
        ;; (= (p-ex-in-room table office) 0.9)
        ;; (= (p-ex-in-room mug office) 0.9)
        ;; (= (p-ex-in-room oven office) 0.0)
        ;; (= (p-ex-in-room fridge office) 0.1)
        ;; (= (p-ex-in-room book office) 0.8)
        ;; (= (p-ex-in-room board-game office) 0.2)

        ;; (= (p-ex-in-room cornflakes living-room) 0.1)
        ;; (= (p-ex-in-room table living-room) 0.7)
        ;; (= (p-ex-in-room mug living-room) 0.6)
        ;; (= (p-ex-in-room oven living-room) 0.0)
        ;; (= (p-ex-in-room fridge living-room) 0.0)
        ;; (= (p-ex-in-room book living-room) 0.3)
        ;; (= (p-ex-in-room board-game living-room) 0.7)

        (= (p-is-in c1a) 0.5)
        (= (p-is-in c2a) 0.3)
        (= (p-is-in c3a) 0.2)
        ;; (= (p-is-in c4a) 0.1)
        ;; (= (p-is-in c5a) 0.1)
        ;; (= (p-is-in c6a) 0.1)

        (= (p-is-in c1b) 0.6)
        (= (p-is-in c2b) 0.2)
        (= (p-is-in c3b) 0.15)
        ;; (= (p-is-in c4b) 0.15)
        ;; (= (p-is-in c5b) 0.1)

)

(:goal  (and (exists (?o - visualobject) (and (= (label ?o) cereal_box)
                                              (kval dora (is-in ?o)))))

)

)