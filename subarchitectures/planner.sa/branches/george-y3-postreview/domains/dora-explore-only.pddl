(define (domain dora-avs-iros11)
  (:requirements :mapl :adl :fluents :partial-observability :dynamic-objects :action-costs)

  (:types
   conegroup - object
   robot - planning_agent
   person robot - location
   label category spatial_relation place room visualobject - concept
   visualobject room - location
   place_status - object 
;; polar_reply concept - object
   )

  (:predicates
   (connected ?p1 ?p2 - place)
   (position-reported ?o - visualobject)
   (is-visited ?c - conegroup)

   (is-virtual ?o - object)

   ;; derived predicates
   (attached_to_room ?p - place ?r - room)
   ;; (cones-exist ?l - label ?r - room)
   ;; (obj-possibly-in-room ?o - visualobject ?r - room)

   ;; (any-engaged)
   (engaged ?p - person)

   (started)
   (done)
   )

   ;; === perceptual fluents for modelling dialogue ===
  ;; (:percepts 
  ;;  (polar-response ?svar - (function concept) ?val - (typeof ?svar) ?reply - polar_reply)
  ;;  (general-response ?svar - (function concept) ?reply - (typeof ?svar))
  ;;  )


  (:functions
   (is-in ?o - robot) - place
   (is-in ?o - person) - place


   ;; expected cost of searching for an object. Used by CP planner
   (dora__cost_inroom ?l - label) - number
   (dora__cost_inobject ?l1 ?l2 - label) - number
   (dora__cost_on ?l1 ?l2 - label) - number
   (search_cost ?l - label ?rel - spatial_relation ?where - (either visualobject room)) - number
   ;; default probabilities. These come from Coma.
   (dora__inroom ?l - label ?c - category) - number
   (dora__inobject ?l1 ?l2 - label ?c - category) - number
   (dora__on ?l1 ?l2 - label ?c - category) - number

   (dora__not_inroom ?l - label ?c - category) - number

   ;; === room properties ===
   (category ?r - room) - category
   (identity ?r - room) - category
   (roomid ?r - room) - number
   (virtual-category ?r - room) - category
   (virtual-place ?r - room) - place

   ;; === place properties ===
   (placestatus ?n - place) - place_status
   (in-room ?p - place) - room
   (place-exists ?p - place) - boolean

   ;; === person properties ===
   (associated-with ?p - person) - room
   (does-exist ?p - person) - boolean
   (contains-a-person-prior ?r - room) - boolean

   (unresponsive ?p - person) - boolean

   ;; === object properties ===
   (label ?o - visualobject) - label
   (related-to ?o - visualobject) - location
   (relation ?o - visualobject) -  spatial_relation

   ;; === conegroup properties ===
   ;; basic properties that determine what the conegroup was generated for 
   ;; (e.g. cone group for cornflakes ON table_1)
   (cg-label ?c - conegroup) - label
   (cg-relation ?c - conegroup) - spatial_relation
   (cg-related-to ?c - conegroup) - (either visualobject room)
   (cg-place ?c - conegroup) - place
   ;; probability of seeing an object of type (label ?c) when looking
   (p-visible ?c - conegroup) - number
   ;; the ground truth. Distribution should conform to the probability above.
   ;; Assumes that an object can be viewed from more than one conegroup
   ;; (visible_from ?o - visualobject ?c - conegroup) - boolean
   ;; If an object can only be seen from one CG, the following would be possible:
   (visible_from ?o - visualobject) - conegroup


   ;; === explanation properties ===
   (entity-exists ?o - object) - boolean

   )

  (:constants
   placeholder trueplace - place_status
   in on - spatial_relation
   container - label
   ;; yes no dontknow - polar_reply
   tutor - agent
   dummy-room - room
   )

  ;; create dummy persons
  (:init-rule persons
              :parameters(?r - room)
              :precondition (and (not (is-virtual ?r))
                                 (not (exists (?p - person ?pl - place)
                                              (or (and (= (in-room ?pl) ?r)
                                                       (in-domain (is-in ?p) ?pl))
                                                  (and (is-virtual ?p)
                                                       (= (associated-with ?p) ?r))))))
              :effect (create (?p - person) (and
                                             (is-virtual ?p)
                                             (assign (associated-with ?p) ?r))
                              )
              )

  ;; p(?label IN ?room | category(?room) = ?cat)
  (:dtrule person_in_room
           :parameters (?p - person ?pl - place ?r - room)
           :precondition (and (= (contains-a-person-prior ?r) true)
                              (= (in-room ?pl) ?r)
                              (= (associated-with ?p) ?r)
                              (is-virtual ?p))
           :effect (probabilistic 1.0 (assign (is-in ?p) ?pl))) ;; will automatically be normalised

 ;; (:init-rule reset-engaged
 ;;             :effect (not (any-engaged)))

 ;; (:init-rule engaged
 ;;             :parameters (?p - person)
 ;;             :precondition (= (engaged ?p) true)
 ;;             :effect (any-engaged))


  ;; (:derived (obj-possibly-in-room ?o - visualobject ?r - room)
  ;;           (exists (?p - place) (and (= (in-room ?p) ?r)
  ;;                                     (in-domain (is-in ?o) ?p))))

  ;; (:derived (cones-exist ?l - label ?r - room)
  ;;           (exists (?c - cone ?p - place) (and (= (in-room ?p) ?r)
  ;;                                               (= (is-in ?c) ?p)
  ;;                                               (= (label ?c) ?l)))
  ;;           )

  ;; (:derived (not_fully_explored ?r - room)
  ;;           (exists (?p ?p2 - place) (and (= (in-room ?p) ?r)
  ;;                                         (not (= (placestatus ?p2) trueplace))
  ;;                                         (connected ?p ?p2))))


  ;; (:dtrule room_from_placeholder
  ;;          :parameters (?p - place ?r - room ?c - category)
  ;;          :precondition (and (= (placestatus ?p) placeholder)
  ;;                             (= (virtual-category ?r) ?c)
  ;;                             (= (leads_to_room ?p ?c) true)
  ;;                             (is-virtual ?r))
  ;;          :effect (probabilistic 1.0 (and (assign (in-room ?p) ?r)
  ;;                                          (assign (category ?r) ?c))))

  ;; (:action explore_place
  ;;          :agent (?a - robot)
  ;;          :parameters (?loc - place)
  ;;          :duration (= ?duration 0.1)
  ;;          :precondition (and  (= (is-in ?a) ?loc)
  ;;                           (= (placestatus ?loc) placeholder))
  ;;          :effect (assign (placestatus ?loc) trueplace)
  ;;          )


   (:action move
            :agent (?a - robot)
            :parameters (?to - place)
            :variables (?from - place)
            :precondition (and (or (connected ?from ?to)
                                   (connected ?to ?from))
                               (not (done))
                               (= (is-in ?a) ?from))
            :effect (and (assign (is-in ?a) ?to)
                         (assign (placestatus ?to) trueplace)
                         (kval ?a (in-room ?to))
                         (increase (total-cost) 2))

            )

   (:action move_direct
            :agent (?a - robot)
            :parameters (?to - place)
            :variables (?from - place ?via - place)
            :precondition (and (or (connected ?from ?via)
                                   (connected ?via ?from))
                               (or (connected ?via ?to)
                                   (connected ?to ?via))
                               (not (done))
                               (= (is-in ?a) ?from))
            :effect (and (assign (is-in ?a) ?to)
                         (assign (placestatus ?to) trueplace)
                         (kval ?a (in-room ?to))
                         (increase (total-cost) 3))
            )

   ;; (:durative-action move_direct
   ;;                   :agent (?a - robot)
   ;;                   :parameters (?to - place)
   ;;                   :variables (?from - place ?r - room)
   ;;                   :duration (= ?duration 8)
   ;;                   :condition (and 
   ;;                                   (over all (and (= (in-room ?from) ?r)
   ;;                                                  (or (= (in-room ?to) ?r)
   ;;                                                      (attached_to_room ?to ?r))))
   ;;                                   (over all (not (done)))
   ;;                                   (at start (= (is-in ?a) ?from)))
   ;;                   :effect (and (change (is-in ?a) ?to)
   ;;                                (change (placestatus ?to) trueplace))
   ;;                   )

   (:action look-for-people
            :agent (?a - robot)
            :variables (?p - place)
            :precondition (and (not (done))
                            (= (is-in ?a) ?p))
            :effect (and 
                     (increase (total-cost) 10))
            )

   (:observe person
             :agent (?a - robot)
             :parameters (?p - person ?pl - place)
             :execution (look-for-people ?a ?pl)
             :precondition (and )
                                
             :effect (and (when (= (is-in ?p) ?pl)
                            (probabilistic 0.7 (observed (does-exist ?p) true)))
                          (when (not (= (is-in ?p) ?pl))
                            (probabilistic 0.001 (observed (does-exist ?p) true)))
                          )
             )

)
