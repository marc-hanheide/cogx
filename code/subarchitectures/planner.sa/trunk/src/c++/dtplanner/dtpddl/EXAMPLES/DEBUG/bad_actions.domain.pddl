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

(:action  move
          :parameters  (?a - robot ?to - place ?from - place)
          :precondition  (and  (or  (connected ?from ?to)
                                    (connected ?to ?from)
                               )
                               (is-in ?a ?from)
                         )
          :effect  (and  (not (is-in ?a ?from))
                         (is-in ?a ?to)
                         (started )
                         (assign (reward ) -2.0000)
                   )
)

(:action  look-for-object
          :parameters  (?a - robot ?l - label ?o - visualobject ?p - place)
          :precondition  (and  (is-in ?a ?p)
                               (label ?o ?l)
                         )
          :effect  (and  (started )
                         (assign (reward ) -1.0000)
                   )
)

(:action  commit-is-in-visualobject2
          :parameters  (?val - place)
          :precondition  (and  (not (committed-is-in visualobject2))
                               (not (committed-ex-in-room cornflakes room__k_a))
                               (not (committed-category room__k_a))
                         )
          :effect  (and  (committed-is-in visualobject2)
                         (when  (is-in visualobject2 ?val)
                                (assign (reward ) 100)
                         )
                         (when  (not (is-in visualobject2 ?val))
                                (assign (reward ) -200)
                         )
                   )
)

(:action  disconfirm-ex-in-room-cornflakes-room__k_a
          :precondition  (and  (not (committed-ex-in-room cornflakes room__k_a))
                               (not (committed-is-in visualobject2))
                         )
          :effect  (and  (committed-ex-in-room cornflakes room__k_a)
                         (when  (not (ex-in-room cornflakes room__k_a true))
                                (assign (reward ) 50.0000)
                         )
                         (when  (ex-in-room cornflakes room__k_a true)
                                (assign (reward ) -500.0000)
                         )
                   )
)

(:action  disconfirm-category-room__k_a
          :precondition  (and  (not (committed-category room__k_a))
                               (not (committed-is-in visualobject2))
                         )
          :effect  (and  (committed-category room__k_a)
                         (when  (not (category room__k_a kitchen))
                                (assign (reward ) 50.0000)
                         )
                         (when  (category room__k_a kitchen)
                                (assign (reward ) -500.0000)
                         )
                   )
)

(:observe  visual_object
           :parameters  (?a - robot ?o - visualobject ?l - label ?p - place)
           :execution  (look-for-object ?a ?l ?o ?p)
           :precondition  (and  )
           :effect  (when  (is-in ?o ?p)
                           (probabilistic  0.8000  (observed-is-in ?o ?p))
                    )
)

)
