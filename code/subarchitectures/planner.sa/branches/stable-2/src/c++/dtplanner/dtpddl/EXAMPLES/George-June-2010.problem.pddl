(define (problem george-bham-may)

	 (:domain george-june)

	 (:objects

	  visual-object1 visual-object2 - Visual-Object	;; visual object that are on the scene

	  shape1 shape2 shape3 cup tea-box color1 color2 color3 unknown - Model	;; based on Model statuses epistemic objects
 
	  color shape appearance - Concept ;; ?? another kind of epistemic objects, model status proprety or hardcoded
	  )

	 (:init 

	  ;; Defeault knowledge

 
	  (model-part-of-concept color1 color)
	  (model-part-of-concept color2 color)
	  (model-part-of-concept shape1 shape)
	  ;; .......

	  ;;Labels
 
	  ;;
	  (learnable-concept color) ;;? harcoded or based on an epistemic object
	  (learnable-concept shape)
	  (learnable-concept appearance)
 
	  ;;Appearances
	  
	  (= (true-positive color) .7)
	 )

	 (:metric maximise (total-reward))

	 (:goal (forall (?x - Visual-Object) 
			(and (been-used-for-learning-sample color ?x)
			     (been-used-for-learning-sample shape ?x)
			     (been-used-for-learning-sample appearance ?x)
			     )))
	 )

;; Step 1 -- Vision posts a belief about an object to binder. Posts a
;; belief with a type-field "Visual-Object". This has features "color"
;; (i.e., discrete category -- e.g., Color1, Color2, Color3), "shape",
;; "appearance" (i.e., SIFT).
