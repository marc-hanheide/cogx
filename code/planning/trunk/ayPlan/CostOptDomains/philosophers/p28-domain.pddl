 
;; This domain encodes the domain of a concurrent 
;; systems, especially communication protocols 
;; (c) Stefan Edelkamp, 2004 
 
(define (domain protocol)
 
(:requirements  :equality 
 ;; equality needed for blocked transition in case of a mismatch 
 
 :typing :action-costs)
 (:types block)
 
(:types process proctype state queue transition 
        number message
 ;; tags for size and messages 
 
        queuetype queue-state
        - object
)
(:predicates 
   (is-a-queue ?q - block  - queue ?pt - block  - queuetype) 
   ;; true if type of communication channel matches queuetype 
 
   (is-a-process ?q - block  - process ?pt - block  - proctype) 
   ;; true if process class matches process type 
 
   (at-process ?p - block  - process ?s - block  - state) 
   ;; true for current local state 
 
   (trans ?pt - block  - proctype ?t - block  - transition ?s1 - block  ?s2 - block  - state) 
   ;; local state transition 
 
   ;; false for all outgoing transition at a given local state 
 
   (writes ?p - block  - process ?q - block  - queue ?t - block  - transition) 
   ;; true if transition writes data in channel 
 
   (reads ?p - block  - process ?q - block  - queue ?t - block  - transition) 
   ;; true if transition reads data in channel 
 
   (enabled ?p - block  - process ?t - block  - transition) 
   ;; true if process can perform state transition 
 
   (pending ?p - block  - process) 
   ;; true if process is waiting for instructions 
 
   (activate ?p - block  - process ?t - block  - transition) 
   ;; if process is waiting for instructions 
 
   (queue-next ?qt - block  - queuetype ?qs1 - block  ?qs2 - block  - queue-state) 
   ;; true for two neighboring cells in a queue 
 
   (queue-head ?q - block  - queue ?qs - block  - queue-state) 
   ;; true if head is at given queue position 
 
   (queue-tail ?q - block  - queue ?qs - block  - queue-state) 
   ;; true if tail is at given queue position 
 
   (advance-tail ?q - block  - queue) 
   ;; true queue tail has yet to advance one cell 
 
   (advance-head ?q - block  - queue) 
   ;; true queue head has yet to advance one cell 
 
   (settled ?q - block  - queue) 
   ;; true if queue has completed restructuring 
 
   (blocked ?p - block  - process) 
   ;; true if no local state transition is applicable 
 
   (blocked-trans ?p - block  - process ?t - block  - transition) 
   ;; true if process is blocked 
 
   (queue-msg ?q - block  - queue ?p - block  - queue-state ?m - block  - message) 
   ;; true if queue cell is labelled with given message 
 
   (queue-head-msg ?q - block  - queue ?m - block  - message) 
   ;; true if queue at head cell is labelled with given message 
 
   (queue-tail-msg ?q - block  - queue ?m - block  - message) 
   ;; true if queue at tail cell is labelled with given message 
 
   (trans-msg ?t - block  - transition ?m - block  - message) 
   ;; true if message written or read by the transition 
 
   (inc ?n1 - block  ?n2 - block  - number) 
   ;; links successive integers 
 
   (dec ?n1 - block  ?n2 - block  - number) 
   ;; links successive integers 
 
   (queue-size ?q - block  - queue ?n1 - block  - number) 
   ;; current elements in communication channel 
 
   (queue-max ?qt - block  - queuetype ?n1 - block  - number) 
   ;; maximum elements in communication channel 
 
   (is-zero ?n1 - block  - number) 
   ;; true if number is equal to zero 
 
   (is-not-zero ?n1 - block  - number) 
   ;; true if number is not equal to zero 
 
   (is-max ?qt - block  - queuetype ?n1 - block  - number) 
   ;; true if number is equal to maximum queue size 
 
   (is-not-max ?qt - block  - queuetype ?n1 - block  - number) 
   ;; true if number is not equal to maximum queue size 
 
) 
 
  (:functions
     (total-cost) - number
  )
 
 
   ;; transition that reads data from channel 
 
(:action queue-read  
   :parameters (?p - block  - process 
                ?t - block  - transition 
                ?q - block  - queue  
                ?m - block  - message  
   ) 
   :precondition 
      (and  
        ;; matching transition activated 
 
        (activate ?p ?t) 
        (settled ?q)  
        (reads ?p ?q ?t) 
        ;; if messages match 
 
        (queue-head-msg ?q ?m) 
        (trans-msg ?t ?m) 
      ) 
    :effect  
      (and  
 
         (not (activate ?p ?t)) 
        ;; set flag for queue update  
 
	(advance-head ?q) 
	(not (settled ?q)) 
        ;; set flag for state transition  
 
	(enabled ?p ?t) 
	(increase (total-cost) 1)
      ) 
) 
 
 
;; determines if no local state transition can fire 
 
(:derived (blocked ?p - block  - process) 
   (and 
     (exists (?s - block  - state) 
      (exists (?pt - block  - proctype) 
        (and 
        ;; select local state 
 
        (at-process ?p - block  ?s - block ) 
        (is-a-process ?p - block  ?pt - block ) 
         ;; select active transition  
 
        (forall (?t - block  - transition) 
          (or (forall (?s2 - block  - state) 
                  (not (trans ?pt - block  ?t - block  ?s - block  ?s2 - block )))
             (blocked-trans ?p - block  ?t - block )))
(increase (total-cost) 1)
        )
        
        ))) 
) 
 
;; determines if a state transition cannot read, queue empty 
 
(:derived (blocked-trans ?p - block  - process ?t - block  - transition) 
   (and 
     (exists (?q - block  - queue) 
      (exists (?m - block  - message) 
       (exists (?n - block  - number) 
	(and (activate ?p - block  ?t - block ) 
	  (reads ?p - block  ?q - block  ?t - block ) 
	  (settled ?q - block ) 
        (trans-msg ?t - block  ?m - block ) 
        (queue-size ?q - block  ?n - block ) 
        (is-zero ?n - block ) 
	  (increase (total-cost) 1)
	  ))) 
	 )) 
   ) 
 
;; determines if a state transition cannot write, queue full 
 
(:derived (blocked-trans ?p - block  - process ?t - block  - transition) 
   (and 
     (exists (?q - block  - queue) 
      (exists (?qt - block  - queuetype) 
       (exists (?m - block  - message) 
        (exists (?n - block  - number) 
	 (and 
	  (activate ?p - block  ?t - block ) 
	  (writes ?p - block  ?q - block  ?t - block ) 
	  (settled ?q - block ) 
	  (trans-msg ?t - block  ?m - block ) 
  	  (is-a-queue ?q - block  ?qt - block ) 
        (queue-size ?q - block  ?n - block ) 
        (is-max ?qt - block  ?n - block ) 
          (increase (total-cost) 1)
          )))) 
          )) 
   ) 
 
;; determines if a state transition cannot read, wrong message 
 
(:derived (blocked-trans ?p - block  - process ?t - block  - transition) 
     (and 
        (exists (?q - block  - queue) 
        (exists (?m - block  - message) 
        (exists (?n - block  - message) 
          (and (activate ?p - block  ?t - block ) 
               (reads ?p - block  ?q - block  ?t - block ) 
               (settled ?q - block ) 
               (trans-msg ?t - block  ?m - block ) 
               (queue-head-msg ?q - block  ?n - block ) 
               (not (= ?m - block  ?n - block )) 
          (increase (total-cost) 1)
           )))           )))
 
;; state transition that writes message in communication channel 
 
(:action queue-write 
   :parameters (?p - block  - process  
                ?t - block  - transition 
                ?q - block  - queue 
	          ?m - block  - message 
   ) 
   :precondition  
      (and  
        (activate ?p ?t) 
        (settled ?q) 
        (writes ?p ?q ?t) 
        (trans-msg ?t ?m) 
      ) 
   :effect 
      (and  
          (not (activate ?p ?t)) 
	  (enabled ?p ?t) 
	  (not (settled ?q)) 
 
	  (advance-tail ?q) 
        (queue-tail-msg ?q ?m) 
      (increase (total-cost) 1)
      ) 
) 
 
 
;; update communication channel due to read operation 
 
(:action advance-queue-head  
   :parameters (?q - block  - queue ?qt - block  - queuetype  
	          ?qs1 - block  ?qs2 - block  - queue-state 
	          ?m - block  - message ?n1 - block  ?n2 - block  - number 
) 
   :precondition 
     (and  
	(queue-next ?qt ?qs1 ?qs2) 
	(is-a-queue ?q ?qt) 
	(queue-head ?q ?qs1) 
	(advance-head ?q) 
        (queue-msg ?q ?qs2 ?m) 
      (queue-size ?q ?n1) 
       (dec ?n1 ?n2) 
      ) 
   :effect 
     (and 	 
        (settled ?q) 
        (queue-head ?q ?qs2) 
        (not (advance-head ?q)) 
        (not (queue-head ?q ?qs1)) 
        (queue-head-msg ?q ?m) 
         (not (queue-size ?q ?n1)) 
         (queue-size ?q ?n2) 
         (increase (total-cost) 1)
      )       
) 
 
;; update communication channel due to write operation, empty queue 
 
(:action advance-empty-queue-tail    
   :parameters (?q - block  - queue ?qt - block  - queuetype  
	        ?qs1 - block  ?qs2 - block  - queue-state 
	        ?m - block  ?m1 - block  - message ?n1 - block  ?n2 - block  - number 
) 
   :precondition 
     (and  
	(queue-next ?qt ?qs1 ?qs2) 
	(is-a-queue ?q ?qt) 
	(queue-tail ?q ?qs1) 
	(advance-tail ?q) 
      (queue-tail-msg ?q ?m) 
      (queue-head-msg ?q ?m1) 
      (queue-size ?q ?n1) 
      (inc ?n1 ?n2) 
      (is-zero ?n1) 
     ) 
   :effect 
     (and 	 
	(settled ?q) 
	(not (advance-tail ?q)) 
      (queue-tail ?q ?qs2) 
      (not (queue-tail ?q ?qs1)) 
      (queue-msg ?q ?qs2 ?m) 
      (queue-head-msg ?q ?m) 
      (not (queue-head-msg ?q ?m1)) 
      (queue-size ?q ?n2) 
      (not (queue-size ?q ?n1)) 
      (increase (total-cost) 1)
     )       
) 
 
;; update communication channel due to write operation 
 
(:action advance-non-empty-queue-tail    
   :parameters ( 
        ?q - block  - queue ?qt - block  - queuetype 
	  ?qs1 - block  ?qs2 - block  - queue-state 
        ?m - block  - message 
        ?n1 - block  ?n2 - block  - number  
) 
   :precondition 
     (and  
	 (queue-next ?qt ?qs1 ?qs2) 
	 (is-a-queue ?q ?qt) 
	 (queue-tail ?q ?qs1) 
	 (advance-tail ?q) 
       (queue-tail-msg ?q ?m) 
       (queue-size ?q ?n1) 
       (inc ?n1 ?n2) 
       (is-not-zero ?n1) 
     ) 
   :effect 
     (and 	 
	  (settled ?q) 
	  (not (advance-tail ?q)) 
        (queue-tail ?q ?qs2) 
        (not (queue-tail ?q ?qs1)) 
        (queue-msg ?q ?qs2 ?m) 
        (not (queue-size ?q ?n1)) 
        (queue-size ?q ?n2)
        (increase (total-cost) 1) 
     )       
) 
 
;; execute local state transition 
 
(:action perform-trans 
   :parameters (?p - block  - process 
	 	  ?pt - block  - proctype 
                ?t - block  - transition ?s1 - block  ?s2 - block  - state 
   ) 
   :precondition  
      (and  
	(forall (?q - queue) (settled ?q)) 
        (trans ?pt ?t ?s1 ?s2)  
        (enabled ?p ?t)  
        (at-process ?p ?s1)  
      ) 
   :effect  
      (and  
        (at-process ?p ?s2) 
        (not (at-process ?p ?s1))  
        (not (enabled ?p ?t)) 
	(pending ?p)
	(increase (total-cost) 1) 
      ) 
) 
 
;; activate local state transition 
 
 
(:action activate-trans 
   :parameters (?p - block  - process 
                ?pt - block  - proctype 
                ?t - block  - transition ?s1 - block  ?s2 - block  - state 
   ) 
   :precondition  
      (and  
	(forall (?q - queue) (settled ?q)) 
        (trans ?pt ?t ?s1 ?s2)  
        (is-a-process ?p ?pt) 
        (at-process ?p ?s1) 
        (pending ?p)  
      ) 
   :effect  
      (and  
        (activate ?p ?t) 
        (not (pending ?p)) 
        (increase (total-cost) 1)
      ) 
) 

) 

