;;;  Original Authors: Michael Littman and David Weissman  ;;;
;;;  Modified: Blai Bonet for IPC 2006 ;;;
;;;  Modified: Charles Gretton for CogX 2009 ;;;

(define (domain tire)
  (:requirements 
      :partial-observability ;; Not in IPC-5 tireworld
      :fluents ;; Not in IPC-5  tireworld
      :universal-effects  ;; Not in IPC-5 tireworld
      :conditional-effects  ;; Not in IPC-5 tireworld

      :typing 
      :strips 
      :equality 
      :probabilistic-effects)

  (:types location)

  (:predicates 
   (vehicle-at ?loc - location) 

   (spare-in ?loc - location) 

   (road ?from - location ?to - location) 

   (goal-location ?loc) ;; Not in IPC-5 tireworld

   (not-flattire) 

   (hasspare)
   )

  ;; Note, here all the state-predicates are repeated except for
  ;; "(not-flattire)". A repeated state predicate in ":percepts" is
  ;; fully observable.
  (:percepts
   
   ;; Do we know if we have a flat tire?
   (observe-not-flattire) ;; Not in IPC-5 tireworld

   ;; Fully observable -- i.e. follows state variable.
   (vehicle-at ?loc - location) 

   ;; Fully observable -- i.e. follows state variable.
   (spare-in ?loc - location) 

   ;; Fully observable -- i.e. follows state variable.
   (road ?from - location ?to - location) 

   ;; Fully observable -- i.e. follows state variable.
   (goal-location ?loc)

   ;; Fully observable -- i.e. follows state variable.
   (hasspare)
   )

  (:action move-car
    :parameters 
    (?from - location ?to - location)
    
    :precondition 
    (and 
     (vehicle-at ?from) 
     (road ?from ?to) 
     (not-flattire))

    :effect 
    (and 
     (vehicle-at ?to) 
     (not (vehicle-at ?from)) 
     (probabilistic 2/5 (not (not-flattire)))

     ;; Following was not in IPC-5 tireworld
     (forall 
      (?loc - location) 
      (when (and (goal-location ?loc)
                 (= ?to ?loc))
             (increase (reward) 1000)))
     )
  )
  (:action loadtire
    :parameters (?loc - location)
    :precondition (and (vehicle-at ?loc) 
                       (spare-in ?loc))
    :effect (and (hasspare) (not (spare-in ?loc)))
   )
  (:action changetire
    :precondition (hasspare)
    :effect (probabilistic 1/2 (and (not (hasspare)) 
                                    (not-flattire)))
  )

  ;; Following perception was not in IPC-5 tireworld
  (:observe tire-status-after-move    
   :parameters 
   (?from - location ?to - location)
        
   :execution
   (move-car ?from ?to)

   :precondition 
   ()
   
   :effect 
   (and (when (not-flattire) 
          (probabilistic 7/8 (observe-not-flattire)
                         1/8 (not (observe-not-flattire))))
        (when (not (not-flattire)) 
          (probabilistic 7/8 (not (observe-not-flattire))
                         1/8 (observe-not-flattire))))
   
   )
)
