;;;  DRAFT DTPDDL for Dora2.0 (Vienna version, Febuary 2010)
;;;
;;;  Based on the CogX MAPL description of Dora for the 2009 CogX
;;;  demonstrations. The domain on which this is based was authored by
;;;  Michael Brenner and Moritz G�belbecker, 2009.


(define (domain dora_the_explorer)  

  (:requirements 
   ;; IPC6 elements -- Planning off the ground
   :typing ;; Keywords :: ":types"
   :strips 
   :equality
   :fluents 
   
   ;; Uncertainty track at IPC6
   :probabilistic-effects
   
   ;; PDDL syntactic sugar
   ;; Keywords :: 
   :universal-effects  
   :conditional-effects  
   
   ;; DTP-ELEMENT
   ;; Keywords :: ":percepts", ":observe", and ":execution"
   :partial-observability 

    )

  
  (:types 
   room label widget feature  model-slot - object
   model - (either widget feature person) 
   )

  (:constants 
   
   ;; "Empty" means that no ML is running in whatever model-slot is
   ;; occupied by the feature "Empty".
   Empty - feature
   
   )
  
  
  (:predicates 

   ;; Has a room been explored?
   (explored ?p - room) ;; Dora-1.0 -- 2009


   ;; (see also function symbol "foregrounded_mode", and action
   ;; symbols "foreground_model" and "background_model")
   ;; 
   ;; A "model" is something that we typically associate with Machine
   ;; Learning (ML), or an ML-based algorithm. We suppose that Dora2.0
   ;; has a limited capacity with regards to the number of "models" that
   ;; can be "foregrounded" at any given time. Hence we provide a
   ;; limited number of "model-slots". There are some "models", such as
   ;; face-detection "models", that may be able to be foregrounded all
   ;; the time -- i.e., perhaps because there is very low resource
   ;; consumption wrt using that "model". In this case, we shall suppose
   ;; a "model-slot" is occupied by that "model", and that planning
   ;; cannot decide to use that "model-slot" for some other purpose.
   (deletable ?s - model-slot)
   

   (connected ?p1 - room ?p2 - room) ;; Dora-1.0 -- 2009


   ;; The location at which a widget is located.
   (widget-location ?o - widget ?p - room) ;; Dora-2.0

   ;; The location at which a feature (shelf/poster/etc) is located.
   (featured-at ?model - feature ?location - room) ;; Dora-2.0
   )


  ;; An "s-functions" is a function that characterises the world
  ;; state. This is distinct from an "o-functions" that would
  ;; characterise aspects of the robots observation status.
  (:s-functions
   
   ;; The natural langauge lable of a room.
   (labelled ?r - room) - label ;; Dora-2.0

   ;; Location of the robot
   (located) - room ;; Dora-1.0 -- 2009 [agent free]
   
   ;; Reward that has been received so far...
   (reward) - double
   

   ;; (see also predicate symbol "deletable", and action symbols
   ;; "foreground_model" and "background_model")
   ;;
   ;; A function from model-slots to models. There are a limited
   ;; number of slots, and each is associated with an ML-based
   ;; object/face/etc detection component.
   (foregrounded-model ?s - model-slot ) - model


   ;; Probabilistic information about how the models behave.
   (probability__observe_widget_model_at_label__if_false ?l - label ?m - widget) - double
   (probability__observe_widget_model_at_label__if_true ?l - label ?m - widget) - double
   )

  
   (probability__observe_feature_at_room_with_label__if_true ?loc - room ?m - feature ?l - label) - double
   (probability__observe_feature_at_room_with_label__if_feature_false ?loc - room ?m - feature ?l - label) - double
   

  
  (:percepts
   
   (observed_model_at_room ?n - room ?m - model)

   )



  ;; This action is suppose to cause vision to foreground a model, and
  ;; perhaps the robot will also perform some useful sensing. This
  ;; goes along with the Michael-Z idea of having vision running
  ;; continuously in some thread/process.
  (:action foreground_model
	   :parameters (?m - model ?s - model-slot)
	   :precondition (and (= (foregrounded-model ?s) Empty)

			      ;; Test we haven't already foregrounded the model
			      (forall (?s2 - model-slot) 
				      (not (= (foregrounded-model ?s2) ?m)))

			      ;; Test that the slot is empty, otherwise
			      ;; we should empty it before
			      ;; foregrounding something here.
			      (forall (?m2 - model) 
				      (or (not (= (foregrounded-model ?s) ?m2))
					  (= ?m2 Empty)
					  )
				      )
			      )

			      
	   :effect (assign (foregrounded-model ?s) ?m)
	   )

  ;; Sometimes a model cannot be backgrounded. This could be the case
  ;; if the model, were, for example a face detector -- i.e., is is
  ;; cheap to run in real-time, and there is no need to stop running
  ;; it.
  (:action background_model
	   :parameters (?s - model-slot)
	   :precondition (and (deletable ?s))
	   :effect (assign (foregrounded-model ?s) Empty)
	   )


  ;; Attempt to get some reward. Only worth doing if you are certain,
  ;; because reward is impossible to achieve if this action fails even
  ;; once.
  (:action commit__widget_location 
	   :parameters (?w - widget ?loc - room)

	   :precondition (forall (?loc2 - room) 
				 (not (absolute_belief__widget_location ?w ?loc2)))

	   :effect (and (absolute_belief__widget_location ?w ?loc)
			(when (widget-location ?w ?loc) 
			  (increase (reward) 1000.0)))
	   )
  
  
  ;; Dora-1.0 -- 2009 
  (:action explore-room
	   :parameters (?loc - room)
	   :precondition (and
			  (= (located) ?loc)
			  )
	   :effect (and (explored ?loc))
	   )

  ;;  Dora-1.0 -- 2009 
  (:action move-to-explored-room
	   :parameters (?to - room ?from - room)
	   :precondition 
	   (and
	    (= (located) ?from)
	    (explored ?to)
	    )
	   :effect 
	   (and
	    (assign (located) ?to)
	    )
	   )

  ;;  Dora-1.0 -- 2009 
  (:action move-to-connected-room
	   :parameters (?to - room ?from - room)
	   :precondition 
	   (and
	    (= (located) ?from)
	    (connected ?from ?to)
	    )
	   :effect 
	   (and
	    (assign (located) ?to)
	    )
	   )



  
  (:observe reset_model_observations__on_state
	    :parameters 
	    (?loc - room ?m - model)
	    
	    :execution
	    ()

	    :precondition
	    (and (observed_model_at_room ?loc ?m)
		 (forall (?s - model-slot) (not (= (foregrounded-model ?s) ?m))) )

	    
	    :effect 
	    (and (not (observed_model_at_room ?loc ?m)))
	    )

  
  (:observe reset_model_observations__on_execution
	    :parameters 
	    (?loc - room ?m - model)
	    
	    :execution
	    (not (explore-room ?loc))

	    :precondition
	    (and (observed_model_at_room ?loc ?m))
	    
	    :effect 
	    (and (not (observed_model_at_room ?loc ?m)))
	    )


  (:observe model_widget
	    :parameters 
	    (?location - room ?l - label ?model - widget)
	    
	    :execution
	    (explore-room ?location)
	    
	    :precondition 
	    (and 
	     (exists (?s - slot) (= (foregrounded-model ?s) ?model))

	     (= (labelled ?location) ?l)
	     )
   
	    :effect 
	    (and 
	     (when (widget-location ?model ?location) 
	       (probabilistic 
		(probability__observe_widget_model_at_label__if_true ?l ?model) 
		(observed_model_at_room ?location ?model)
		)
	       )

	     (when (not (widget-location ?model ?location) )
	       (probabilistic 
		(probability__observe_widget_model_at_label__if_false ?l ?model) 
		(observed_model_at_room ?location ?model)
		)
	       )
	     )
	    )


  
  (:observe model_feature
	    :parameters 
	    (?location - room ?l - label ?model - feature)
	    
	    :execution
	    (explore_room ?location)
	    
	    :precondition 
	    (and 
		 (exists (?s - slot) (= (foregrounded-model ?s) ?model))
		 
		 )
   
	    :effect 
	    (and 
	     
	     (when (and (featured-at ?model ?location)
			(= (labelled ?location) ?l))
	       (probabilistic 
		(probability__observe_feature_at_room_with_label__if_true ?location ?model ?l) 
		(observed_model_at_room ?location ?model)
		)
	       )
	     
	     (when (and (not (featured-at ?model ?location))
			(= (labelled ?location) ?l))
	       (probabilistic 
		(probability__observe_feature_at_room_with_label__if_feature_false ?location ?model ?l) 
		(observed_model_at_room ?location ?model)
		)
	       )
	    )
	    )
)