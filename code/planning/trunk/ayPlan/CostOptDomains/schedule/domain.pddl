;; Schedule World
;;

(define (domain schedule)
  (:requirements :adl :typing)
 (:types block)

  (:types temperature-type
	  ashape
	  surface
	  machine
	  part
	  colour
	  width
	  anorient)

  (:constants cold hot - temperature-type
	      cylindrical - ashape 
	      polisher roller lathe grinder punch drill-press
	      spray-painter immersion-painter - machine
              polished rough smooth - surface)

  (:predicates (temperature ?obj - block  - part ?temp - block  - temperature-type)
	       (busy ?machine - block  - machine)
	       (scheduled ?obj - block  - part)
	       (objscheduled)
	       (surface-condition ?obj - block  - part ?surface-cond - block  - surface)
	       (shape ?obj - block  - part ?shape - block  - ashape)
	       (painted ?obj - block  - part ?colour - block  - colour)
	       (has-hole ?obj - block  - part ?width - block  - width ?orientation - block  - anorient)
	       (has-bit ?machine - block  - machine ?width - block  - width)
	       (can-orient ?machine - block  - machine ?orientation - block  - anorient)
	       (has-paint ?machine - block  - machine ?colour - block  - colour))

  (:action do-polish
	   :parameters (?x - block  - part)
	   :precondition (and (not (busy polisher))
			      (not (scheduled ?x))
			      (temperature ?x cold))
	   :effect (and (busy polisher)
			(scheduled ?x)
			(surface-condition ?x polished)
			(when (not (objscheduled))
			  (objscheduled))
			(forall (?oldsurface - surface)
				(when (and (surface-condition ?x ?oldsurface)
                                           (not (= ?oldsurface polished)))
				  (not (surface-condition ?x ?oldsurface))))))

  (:action do-roll
	   :parameters (?x - block  - part)
	   :precondition (and (not (busy roller))
			      (not (scheduled ?x)))
	   :effect (and
		    (busy roller)
		    (scheduled ?x)
		    (temperature ?x hot)
		    (shape ?x cylindrical)
		    (when (not (objscheduled))
		      (objscheduled))
		    (forall (?oldsurface - surface)
			    (when (surface-condition ?x ?oldsurface)
			      (not (surface-condition ?x ?oldsurface))))
		    (forall (?oldpaint - colour)
			    (when (painted ?x ?oldpaint)
			      (not (painted ?x ?oldpaint))))
		    (forall (?oldwidth - width ?oldorient - anorient)
			    (when (has-hole ?x ?oldwidth ?oldorient)
			      (not (has-hole ?x ?oldwidth ?oldorient))))
		    (forall (?oldshape - ashape)
			    (when (and (shape ?x ?oldshape)
                                       (not (= ?oldshape cylindrical)))
			      (not (shape ?x ?oldshape))))
		    (forall (?oldtemp - temperature-type)
			    (when (and (temperature ?x ?oldtemp)
                                       (not (= ?oldtemp hot)))
			      (not (temperature ?x ?oldtemp))))))

  (:action do-lathe
	   :parameters (?x - block  - part) 
	   :precondition (and (not (busy lathe))
			      (not (scheduled ?x)))
	   :effect (and 
		    (busy lathe)
		    (scheduled ?x)
		    (surface-condition ?x rough)
		    (shape ?x cylindrical)
		    (when (not (objscheduled))
		      (objscheduled))
		    (forall (?oldshape - ashape)
			    (when (and (shape ?x ?oldshape)
                                       (not (= ?oldshape cylindrical)))
			      (not (shape ?x ?oldshape))))
		    (forall (?oldsurface - surface)
			    (when (and (surface-condition ?x ?oldsurface)
                                       (not (= ?oldsurface rough)))
			      (not (surface-condition ?x ?oldsurface))))
		    (forall (?oldpaint - colour)
			    (when (painted ?x ?oldpaint)
			    (not (painted ?x ?oldpaint))))))

  (:action do-grind
	   :parameters (?x - block  - part) 
	   :precondition (and (not (busy grinder))
			      (not (scheduled ?x)))
	   :effect (and
		    (busy GRINDER)
		    (scheduled ?x)
		    (surface-condition ?x smooth)
		    (when (not (objscheduled))
		      (objscheduled))
		    (forall (?oldsurface - surface)
			    (when (and (surface-condition ?x ?oldsurface)
                                       (not (= ?oldsurface smooth)))
			      (not (surface-condition ?x ?oldsurface))))
		    (forall (?oldpaint - colour)
			    (when (painted ?x ?oldpaint)
			      (not (painted ?x ?oldpaint))))))

  (:action do-punch
	   :parameters (?x - block  - part ?width - block  - width  ?orient - block  - anorient)  
	   :precondition (and
			  (has-bit punch ?width)
			  (can-orient punch ?orient)
			  (temperature ?x cold)
			  (not (busy punch))
			  (not (scheduled ?x))
			  (not (has-hole ?x ?width ?orient)))
	   :effect (and
		    (busy punch)
		    (scheduled ?x)
		    (has-hole ?x ?width ?orient)
		    (surface-condition ?x rough)
		    (when (not (objscheduled))
		      (objscheduled))
		    (forall (?oldsurface - surface) 
			    (when (and (surface-condition ?x ?oldsurface)
                                       (not (= ?oldsurface rough)))
			      (not (surface-condition ?x ?oldsurface))))))

  (:action do-drill-press
	   :parameters (?x - block  - part ?width - block  - width ?orient - block  - anorient)
	   :precondition (and
			  (has-bit drill-press ?width)
			  (can-orient drill-press ?orient)
			  (temperature ?x cold)
			  (not (busy drill-press))
			  (not (scheduled ?x))
			  (not (has-hole ?x ?width ?orient)))
	   :effect (and
		    (busy drill-press)
		    (scheduled ?x)
		    (has-hole ?x ?width ?orient)
		    (when (not (objscheduled))
		      (objscheduled))))

  (:action do-spray-paint
	   :parameters (?x - block  - part ?newpaint - block  - colour) 
	   :precondition (and
			  (has-paint spray-painter ?newpaint)
			  (not (busy spray-painter))
			  (not (scheduled ?x))
			  (temperature ?x COLD))
	   :effect (and
		    (busy spray-painter)
		    (scheduled ?x)
		    (painted ?x ?newpaint)
		    (when (not (objscheduled))
		      (objscheduled))
		    (forall (?oldsurface - surface)
			    (when (surface-condition ?x ?oldsurface)
			      (not (surface-condition ?x ?oldsurface))))
		    (forall (?oldpaint - colour)
			    (when (and (painted ?x ?oldpaint)
                                       (not (= ?oldpaint ?newpaint)))
			      (not (painted ?x ?oldpaint))))))
  
  (:action do-immersion-paint     
           :parameters (?x - block  - part ?newpaint - block  - colour) 
           :precondition (and
                          (has-paint immersion-painter ?newpaint)
                          (not (busy immersion-painter))
                          (not (scheduled ?x)))
           :effect (and
                    (busy immersion-painter)
                    (scheduled ?x)
                    (painted ?x ?newpaint)
                    (when (not (objscheduled))
                      (objscheduled))
                    (forall (?oldpaint - colour)
                            (when (and (painted ?x ?oldpaint)
                                       (not (= ?oldpaint ?newpaint)))
                              (not (painted ?x ?oldpaint))))))
  
  (:action do-time-step
           :parameters ()
           :precondition (objscheduled)
           :effect (and
                    (forall (?x - part)
                            (when (scheduled ?x)
                              (not (scheduled ?x))))
                    (forall (?m - machine)
                            (when (busy ?m)
                              (not (busy ?m)))))))






