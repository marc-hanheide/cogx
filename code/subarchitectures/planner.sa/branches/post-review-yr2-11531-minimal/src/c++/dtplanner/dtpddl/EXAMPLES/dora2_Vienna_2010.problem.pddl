;;;  DRAFT DTPDDL for Dora2.0 (Vienna version, Febuary 2010)
;;;
;;;  Based on the CogX MAPL description of Dora for the 2009 CogX
;;;  demonstrations. The domain on which this is based was authored by
;;;  Michael Brenner and Moritz Göbelbecker, 2009.


(define (problem dora_the_explorer__Vienna_2_2010)

  (:domain dora_the_explorer)

  (:objects 
	
   Hall R1 R2 R3 - place
	
   Library Kitchen Office Hall-Label - label

   Corn-flakes - widget

   Bookshelf Desktop Chef - feature

   S1 - model-slot
   
   )


  (:init (connected Hall R1) (connected R1 Hall)
	 (connected Hall R2) (connected R2 Hall)
	 (connected Hall R3) (connected R3 Hall)

	 (= (foregrounded-model S1) Empty)

	 (= (reward) 0)

	 (deletable S1)
	 
	 (assign (located) Hall)

	 (labelled Hall Hall-Label)

	 ;; Code for generating the initial belief-state.
	 ;;
	 ;; We don't actually know the true label of a place, only
	 ;; have probabilistic information about that. So, all we know
	 ;; here is that all the places have a distinct label.
	 (probabilistic 1/8 (and (assign (labelled R1) Library) 
				 (assign (labelled R2) Kitchen) 
				 (assign (labelled R3) Office) 
				 (widget-location Corn-flakes R2)
				 (probabilistic 0.9 (featured-at Bookshelf R1) )
				 (probabilistic 0.3 (featured-at Bookshelf R3) )
				 (probabilistic 0.8 (featured-at Chef R2)
						0.1 (featured-at Chef R1) 
						0.1 (featured-at Chef R3) )
				 (probabilistic 0.9 (featured-at Desktop R3) )
				 (probabilistic 0.3 (featured-at Desktop R1) )
				 (probabilistic 0.1 (featured-at Desktop R2) )
				 (probabilistic 0.8 (widget-location Corn-flakes R2)
						0.2 (widget-location Corn-flakes R3) )
				 )

			1/8 (and (assign (labelled R1) Library) 
				 (assign (labelled R2) Office) 
				 (assign (labelled R3) Kitchen) 
				 (widget-location Corn-flakes R3)
				 (probabilistic 0.9 (featured-at Bookshelf R1) )
				 (probabilistic 0.3 (featured-at Bookshelf R2) )
				 (probabilistic 0.8 (featured-at Chef R3) 
						0.1 (featured-at Chef R1) 
						0.1 (featured-at Chef R2) )
				 (probabilistic 0.9 (featured-at Desktop R2) )
				 (probabilistic 0.3 (featured-at Desktop R1) )
				 (probabilistic 0.1 (featured-at Desktop R3) )
				 (probabilistic 0.8 (widget-location Corn-flakes R3)
						0.2 (widget-location Corn-flakes R2) )
				 )

			1/4 (and 
			     (assign (labelled R1) Kitchen) 
			     (assign (labelled R2) Library) 
			     (assign (labelled R3) Office) 
			     (widget-location Corn-flakes R1)
			     (probabilistic 0.9 (featured-at Bookshelf R2) )
			     (probabilistic 0.3 (featured-at Bookshelf R3) )
			     (probabilistic 0.8 (featured-at Chef R1) 
					    0.1 (featured-at Chef R2) 
					    0.1 (featured-at Chef R3) )
			     (probabilistic 0.9 (featured-at Desktop R3) )
			     (probabilistic 0.3 (featured-at Desktop R2) )
			     (probabilistic 0.1 (featured-at Desktop R1) )
			     (probabilistic 0.8 (widget-location Corn-flakes R1)
					    0.2 (widget-location Corn-flakes R3) )
			     )

			1/4 (and (assign (labelled R1) Kitchen) 
				 (assign (labelled R2) Office) 
				 (assign (labelled R3) Library) 
				 (widget-location Corn-flakes R1)
				 (probabilistic 0.9 (featured-at Bookshelf R3) )
				 (probabilistic 0.3 (featured-at Bookshelf R2) )
				 (probabilistic 0.8 (featured-at Chef R1) 
						0.1 (featured-at Chef R3) 
						0.1 (featured-at Chef R2) )
				 (probabilistic 0.9 (featured-at Desktop R2) )
				 (probabilistic 0.3 (featured-at Desktop R3) )
				 (probabilistic 0.1 (featured-at Desktop R1) )
				 (probabilistic 0.8 (widget-location Corn-flakes R1)
						0.2 (widget-location Corn-flakes R2)
						)
				 )
				 
			1/8 (and (assign (labelled R1) Office) 
				 (assign (labelled R2) Kitchen) 
				 (assign (labelled R3) Library) 
				 (widget-location Corn-flakes R2)
				 (probabilistic 0.9 (featured-at Bookshelf R3) )
				 (probabilistic 0.3 (featured-at Bookshelf R1) )
				 (probabilistic 0.8 (featured-at Chef R2) 
						0.1 (featured-at Chef R3) 
						0.1 (featured-at Chef R1) )
				 (probabilistic 0.9 (featured-at Desktop R1) )
				 (probabilistic 0.3 (featured-at Desktop R3) )
				 (probabilistic 0.1 (featured-at Desktop R2) )
				 (probabilistic 0.8 (widget-location Corn-flakes R2)
						0.2 (widget-location Corn-flakes R1)
						)
				 )

			1/8 (and (assign (labelled R1) Office) 
				 (assign (labelled R2) Library) 
				 (assign (labelled R3) Kitchen) 
				 (widget-location Corn-flakes R3)
				 (probabilistic 0.9 (featured-at Bookshelf R2) )
				 (probabilistic 0.3 (featured-at Bookshelf R1) )
				 (probabilistic 0.8 (featured-at Chef R3) 
						0.1 (featured-at Chef R2) 
						0.1 (featured-at Chef R1) )
				 (probabilistic 0.9 (featured-at Desktop R1) )
				 (probabilistic 0.3 (featured-at Desktop R2) )
				 (probabilistic 0.1 (featured-at Desktop R3) )
				 (probabilistic 0.8 (widget-location Corn-flakes R3)
						0.2 (widget-location Corn-flakes R1)
						)
				 )
			)

	 

	 

	 (assign (probability__observe_widget_model_at_label__if_true Library Corn-flakes) .7)
	 (assign (probability__observe_widget_model_at_label__if_true Kitchen Corn-flakes) .7)
	 (assign (probability__observe_widget_model_at_label__if_true Office Corn-flakes) .7)

	 (assign (probability__observe_widget_model_at_label__if_false Library Corn-flakes) .1)
	 (assign (probability__observe_widget_model_at_label__if_false Kitchen Corn-flakes) .1)
	 (assign (probability__observe_widget_model_at_label__if_false Office Corn-flakes) .1)



	 
	 ;; I shall include these zeroed entries in the CPT only once,
	 ;; and omit them below. We shall assume zero, as is common
	 ;; for PDDL, where no entry is given.
	 (assign (probability__observe_feature_at_place_with_label__if_true Hall Bookshelf Library) 0.0 )
	 (assign (probability__observe_feature_at_place_with_label__if_true Hall Bookshelf Kitchen) 0.0 )
	 (assign (probability__observe_feature_at_place_with_label__if_true Hall Bookshelf Office) 0.0 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Bookshelf Library) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Bookshelf Office) 0.3 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Bookshelf Library) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Bookshelf Office) 0.3 )

	 ;; Perhaps place-3 is cluttered, and so we decide that
	 ;; bookshelf detection is not very useful here.
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Bookshelf Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Bookshelf Office) 0.1 )


	 ;; Perhaps you won't see a desktop in a kitchen, ever, because it would be obscured.
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Desktop Library) 0.3 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Desktop Office) 0.9 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Desktop Library) 0.3 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Desktop Office) 0.9 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Desktop Library) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Desktop Office) 0.9 )


	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Chef Library) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Chef Kitchen) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R1 Chef Office) 0.9 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Chef Library) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Chef Kitchen) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R2 Chef Office) 0.9 )

	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Chef Library) 0.7 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Chef Kitchen) 0.9 )
	 (assign (probability__observe_feature_at_place_with_label__if_true R3 Chef Office) 0.7 )





	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Bookshelf Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Bookshelf Office) 0.1 )
	 
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Bookshelf Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Bookshelf Office) 0.1 )

	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Bookshelf Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Bookshelf Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Bookshelf Office) 0.1 )


	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Desktop Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Desktop Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Desktop Office) 0.1 )

	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Desktop Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Desktop Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Desktop Office) 0.1 )

	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Desktop Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Desktop Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Desktop Office) 0.1 )


	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Chef Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Chef Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R1 Chef Office) 0.1 )

	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Chef Library) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Chef Kitchen) 0.1 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R2 Chef Office) 0.1 )

	 ;; You can accidentally see chefs quite easily in R3. Again, lots of clutter.
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Chef Library) 0.2 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Chef Kitchen) 0.3 )
	 (assign (probability__observe_feature_at_place_with_label__if_feature_false R3 Chef Office) 0.2 )

  )

  (:metric maximize (reward))   
)
