(define (domain dt-dora-test-100)

(:requirements :quantified-preconditions :numeric-fluents :equality :action-costs :negative-preconditions :conditional-effects :typing :adl :disjunctive-preconditions :partial-observability)

(:types  category - object
         planning_agent - agent
         room - object
         place_status - object
         feature - object
         agent - object
         boolean - object
         place - object
         robot - movable
         robot - agent
         subgoal - object
         movable - object
         label - object
         visualobject - movable
         
)

(:predicates  (committed-category ?r - room)
              (kval-really-is-in ?a - agent ?o - movable)
              (hyp-category ?r - room ?v - category)
              (committed-is-in ?o - movable)
              (committed-in-room ?p - place)
              (commit-ex-in-room ?l - label ?r - room ?v - boolean)
              (label ?o - visualobject ?value - label)
              (category ?r - room ?value - category)
              (kd-is-in ?a - agent ?o - movable)
              (kval-category ?a - agent ?r - room)
              (committed-really-is-in ?o - movable)
              (kd-in-room ?a - agent ?p - place)
              (kval-ex-in-room ?a - agent ?l - label ?r - room)
              (select-locked )
              (kval-label ?a - agent ?o - visualobject)
              (committed-ex-in-room ?l - label ?r - room)
              (kd-ex-in-room ?a - agent ?l - label ?r - room)
              (i_in-domain-ex-in-room ?l - label ?r - room ?v - boolean)
              (is-in ?o - movable ?value - place)
              (not-instantiated ?o - object)
              (i_in-domain-really-is-in ?o - movable ?v - place)
              (i_in-domain-is-in ?o - movable ?v - place)
              (in-domain-label ?o - visualobject ?v - label)
              (hyp-label ?o - visualobject ?v - label)
              (started )
              (in-domain-category ?r - room ?v - category)
              (ex-in-room ?l - label ?r - room ?value - boolean)
              (commit-in-room ?p - place ?v - room)
              (unused ?o - object)
              (hyp-is-in ?o - movable ?v - place)
              (in-domain-is-in ?o - movable ?v - place)
              (in-domain-in-room ?p - place ?v - room)
              (connected ?p1 - place ?p2 - place)
              (commit-category ?r - room ?v - category)
              (hyp-really-is-in ?o - movable ?v - place)
              (hyp-ex-in-room ?l - label ?r - room ?v - boolean)
              (committed-label ?o - visualobject)
              (kval-in-room ?a - agent ?p - place)
              (commit-is-in ?o - movable ?v - place)
              (i_in-domain-category ?r - room ?v - category)
              (in-domain-really-is-in ?o - movable ?v - place)
              (i_in-domain-label ?o - visualobject ?v - label)
              (kd-really-is-in ?a - agent ?o - movable)
              (kd-label ?a - agent ?o - visualobject)
              (i_in-domain-in-room ?p - place ?v - room)
              (in-domain-ex-in-room ?l - label ?r - room ?v - boolean)
              (in-room ?p - place ?value - room)
              (kd-category ?a - agent ?r - room)
              (kval-is-in ?a - agent ?o - movable)
              (commit-really-is-in ?o - movable ?v - place)
              (hyp-in-room ?p - place ?v - room)
              (commit-label ?o - visualobject ?v - label)
              (really-is-in ?o - movable ?value - place)
	      (been-there ?value - place)
	      (thingi ?value - place)
	      (looked-3 ?p - place)
	      (looked-2 ?p - place)
	      (looked-1 ?p - place)
	      (commitment)
)

(:percepts  (observed-really-is-in ?o - movable ?v - place)
            (observed-category ?r - room ?v - category)
            (observed-in-room ?p - place ?v - room)
            (observed-label ?o - visualobject ?v - label)
            (observed-is-in ?o - movable ?v - place)
            (observed-ex-in-room ?l - label ?r - room ?v - boolean)
)

(:functions  (p-is-in ?p - place) - number
             (total-p-costs ) - number
             (p-category ?r - room ?c - category) - number
             (reward ) - number
             (p-ex-in-room ?l - label ?c - category) - number
)

(:constants  visualobject2 - visualobject
             room__k_a - room
             mug table cornflakes - label
             false true - boolean
             living-room office kitchen - category
)

;; (:action  thing
;; 	  :parameters  (?a - label)
;; 	  :effect (and (when (and (thingi mug) (thingi table) (thingi cornflakes)) (assign (reward) 10000))
;; 		       (thingi ?a))
;; )

(:action  move
          :parameters  (?a - robot ?to - place ?from - place)
          :precondition  (and  (not (commitment)) 
			       (or  (connected ?from ?to)
                                    (connected ?to ?from)
                               )
                               (is-in ?a ?from)
                         )
          :effect  (and  
;; 		    (when (been-there ?to) (assign (reward ) -40.0))
;; 		    (when (not (been-there ?to)) (assign (reward ) -20.0))
		    (been-there ?to)
		    (not (is-in ?a ?from))
		    (is-in ?a ?to)
		    (started )
;; 		    (assign (reward ) -2.0000)
                   )
)


(:action spin
:precondition  (and  (commitment)))

;; (:action  look-for-object
;;           :parameters  (?a - robot ?l - label ?o - visualobject ?p - place)
;;           :precondition  (and  (not (commitment))
;; 			       (is-in ?a ?p)
;;                                (label ?o ?l)
;;                          )
;;           :effect  (and  (started )
;;                          (assign (reward ) -1.0000)
;;                    )
;; )


(:action  look-for-object-1
          :parameters  (?a - robot ?l - label ?o - visualobject ?p - place)
          :precondition  (and  (not (commitment)) (not (looked-1 ?p)) 
			       (is-in ?a ?p)
                               (label ?o ?l)
                         )
          :effect  (and  (looked-1 ?p) (started )
                         (assign (reward ) 0.0000)
                   )
)

;; (:action  look-for-object-2
;;           :parameters  (?a - robot ?l - label ?o - visualobject ?p - place)
;;           :precondition  (and  (not (commitment)) (not (looked-2 ?p)) 
;; 			       (is-in ?a ?p)
;;                                (label ?o ?l)
;;                          )
;;           :effect  (and  (looked-2 ?p) (started )
;;                          (assign (reward ) 0.0000)
;;                    )
;; )

;; (:action  look-for-object-3
;;           :parameters  (?a - robot ?l - label ?o - visualobject ?p - place)
;;           :precondition  (and  (not (looked-3 ?p)) 
;; 			       (is-in ?a ?p)
;;                                (label ?o ?l)
;;                          )
;;           :effect  (and  (looked-3 ?p) (started )
;;                          (assign (reward ) -1.0000)
;;                    )
;; )

(:action  commit-is-in-visualobject2
          :parameters  (?val - place)
          :precondition  (and  (not  (commitment))
                         )
          :effect  (and  (commitment) 
                         (when  (is-in visualobject2 ?val)
                                (assign (reward ) 5000)
                         )
                         (when  (not (is-in visualobject2 ?val))
                                (assign (reward ) -5000)
                         )
                   )
)

;; (:action  disconfirm-ex-in-room-cornflakes-room__k_a
;;           :precondition  (and  (not  (commitment))
;;                          )
;;           :effect  (and  (commitment) 
;;                          (when  (not (ex-in-room cornflakes room__k_a true))
;;                                 (assign (reward ) 5000.0000)
;;                          )
;;                          (when  (ex-in-room cornflakes room__k_a true)
;;                                 (assign (reward ) -5000.0000)
;;                          )
;;                    )
;; )

;; (:action  disconfirm-category-room__k_a
;;           :precondition  (and (not  (commitment))
;;                          )
;;           :effect  (and  (commitment) 
;;                          (when  (not (category room__k_a kitchen))
;;                                 (assign (reward ) 5000.0000)
;;                          )

;;                          (when  (category room__k_a kitchen)
;;                                 (assign (reward ) -5000.0000)
;;                          )
;;                    )
;; )

;; (:observe  visual_object
;;            :parameters  (?a - robot ?o - visualobject ?l - label ?p - place)
;;            :execution  (look-for-object ?a ?l ?o ?p)
;;            :precondition  (and  )
;;            :effect  (and (when  (is-in ?o ?p)
;;                            (probabilistic  0.8000  (observed-is-in ?o ?p))
;; 			   )
;; 			 (when  (not (is-in ?o ?p))
;; 			   (probabilistic  0.0100  (observed-is-in ?o ?p))
;; 			   )
;; 			 )
;; )

(:observe  visual_object-1
           :parameters  (?a - robot ?o - visualobject ?l - label ?p - place)
           :execution  (look-for-object-1 ?a ?l ?o ?p)
           :precondition  (and  )
           :effect  (and (when  (is-in ?o ?p)
                           (probabilistic  0.8000  (observed-is-in ?o ?p))
			   )
			 (when  (not (is-in ?o ?p))
			   (probabilistic  0.0100  (observed-is-in ?o ?p))
			   )
			 )
)

;; (:observe  visual_object-2
;;            :parameters  (?a - robot ?o - visualobject ?l - label ?p - place)
;;            :execution  (look-for-object-2 ?a ?l ?o ?p)
;;            :precondition  (and  )
;;            :effect  (and (when  (is-in ?o ?p)
;;                            (probabilistic  0.8000  (observed-is-in ?o ?p))
;; 			   )
;; 			 (when  (not (is-in ?o ?p))
;; 			   (probabilistic  0.0100  (observed-is-in ?o ?p))
;; 			   )
;; 			 )
;; )

)
