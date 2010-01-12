#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import tempfile
import os

import parser, domain
from parser import Parser, ParseError

logistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :object-fluents :mapl) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)
  
(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle))

(:sensor sense_package
         :agent         (?a - agent)
         :parameters    (?t - truck ?p - package)
         :sense         (= (location-of ?p) (location-of ?t)))

(:sensor sense_position
         :agent         (?a - agent)
         :parameters    (?t - truck)
         :sense         (location-of ?t))

(:action drive
         :agent         (?a - agent)
         :parameters    (?t - truck ?to - location)
         :precondition  (= (city-of (location-of ?t)) (city-of ?to))
         :effect        (assign (location-of ?t) ?to))

(:action fly
         :agent         (?a - agent)
         :parameters    (?a - airplane ?to - airport)
         :effect        (assign (location-of ?a) ?to))

(:action load
         :agent         (?a - agent)
         :parameters    (?p - package ?v - vehicle)
         :precondition  (= (location-of ?p) (location-of ?v))
         :effect        (assign (location-of ?p) ?v))

(:action unload
         :agent         (?a - agent)
         :parameters    (?p - package ?v - vehicle)
         :precondition  (= (location-of ?p) ?v)
         :effect        (assign (location-of ?p) (location-of ?v)))

)
"""

blocks = \
"""
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS-object-fluents)
  (:requirements :typing :equality :mapl) 
  (:types block agent)
  
  (:constants no-block - block)
  
  (:predicates (on-table ?x - block))

  (:functions	(in-hand) - block
		(on-block ?x - block) - block) ;;what is in top of block ?x

  (:action pick-up
             :agent (?a - agent)
	     :parameters (?x - block)
	     :precondition (and (= (on-block ?x) no-block) (on-table ?x) (= (in-hand) no-block))
	     :effect
	     (and (not (on-table ?x))
		   (assign (in-hand) ?x)))

  (:action put-down
             :agent (?a - agent)
	     :parameters (?x - block)
	     :precondition (= (in-hand) ?x)
	     :effect
	     (and (assign (in-hand) no-block)
		   (on-table ?x)))
  
  (:action stack
             :agent (?a - agent)
	     :parameters (?x - block ?y - block)
	     :precondition (and (= (in-hand) ?x) (= (on-block ?y) no-block))
	     :effect
	     (and (assign (in-hand) no-block)
	   	  (assign (on-block ?y) ?x)))

  (:action unstack
             :agent (?a - agent)
	     :parameters (?x - block ?y - block)
	     :precondition (and (= (on-block ?y) ?x) (= (on-block ?x) no-block) (= (in-hand) no-block))
	     :effect
	     (and (assign (in-hand) ?x)
		  (assign (on-block ?y) no-block))))"""

rovers = """
(define (domain Rover)
(:requirements :typing :durative-actions :fluents :mapl)

(:types rover waypoint store store_state camera mode lander objective)

(:constants empty full - store_state)

(:predicates 
             (can_traverse ?r - rover ?x - waypoint ?y - waypoint)
	(equipped_for_soil_analysis ?r - rover)
             (equipped_for_rock_analysis ?r - rover)
             (equipped_for_imaging ?r - rover)
              (have_rock_analysis ?r - rover ?w - waypoint)
             (have_soil_analysis ?r - rover ?w - waypoint)
 	(calibrated ?c - camera ?r - rover) 
	(supports ?c - camera ?m - mode)
             (available ?r - rover)
             (visible ?w - waypoint ?p - waypoint)
             (have_image ?r - rover ?o - objective ?m - mode)
             (communicated_soil_data ?w - waypoint)
             (communicated_rock_data ?w - waypoint)
             (communicated_image_data ?o - objective ?m - mode)
	(at_soil_sample ?w - waypoint)
	 (at_rock_sample ?w - waypoint)
             (visible_from ?o - objective ?w - waypoint)
	 (store_of ?s - store ?r - rover)
	 (calibration_target ?i - camera ?o - objective)
	 (on_board ?i - camera ?r - rover)
	 (channel_free ?l - lander)
	 (in_sun ?w - waypoint)
)


(:functions
	(energy ?r - rover) - number
	(recharge-rate ?x - rover) - number
	(at ?x - (either rover lander)) - waypoint
	(calibration_target ?i - camera) - objective
	(sstate ?s - store) - store_state)
	

(:durative-action navigate
:agent (?a - agent)
:parameters (?x - rover ?y - waypoint ?z - waypoint) 
:duration (= ?duration 5)
:condition (and (over all (can_traverse ?x ?y ?z))
	(at start (available ?x))
	(at start (= (at ?x) ?y))
	(at start (>= (energy ?x) 8))
                (over all (visible ?y ?z)))
:effect (and (at start (decrease (energy ?x) 8))
	(change (at ?x) ?z)))


(:durative-action recharge
:agent (?a - agent)
:parameters (?x - rover ?w - waypoint)
:duration (= ?duration (/ (- 80 (energy ?x)) (recharge-rate ?x)))
:condition (and 
	(over all (= (at ?x) ?w))
	(at start (in_sun ?w))
	(at start (<= (energy ?x) 80)))
:effect (and (at end (increase (energy ?x) (* ?duration (recharge-rate ?x)))))
)


(:durative-action sample_soil
:agent (?a - agent)
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 10)
:condition (and (over all (= (at ?x) ?p))
	(at start (at_soil_sample ?p))
	(at start (equipped_for_soil_analysis ?x))
	(at start (>= (energy ?x) 3))
	(at start (store_of ?s ?x))
	(at start (= (sstate ?s) empty)))
:effect (and (change (sstate ?s) full)
	(at start (decrease (energy ?x) 3))
	(at end (have_soil_analysis ?x ?p))
	(at end (not (at_soil_sample ?p))))
)


(:durative-action sample_rock
:agent (?a - agent)
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 8)
:condition (and (over all (= (at ?x) ?p))
	(at start (>= (energy ?x) 5))
	(at start (at_rock_sample ?p))
	(at start (equipped_for_rock_analysis ?x))
	(at start (store_of ?s ?x))
	(at start (= (sstate ?s) empty)))
:effect (and (change (sstate ?s) full)
	(at end (have_rock_analysis ?x ?p))
	(at start (decrease (energy ?x) 5))
	(at end (not (at_rock_sample ?p))))
)


(:durative-action drop
:agent (?a - agent)
:parameters (?x - rover ?y - store)
:duration (= ?duration 1)
:condition (and (at start (store_of ?y ?x))
	(at start (= (sstate ?y) full)))
:effect (change (sstate ?y) empty)
)


(:durative-action calibrate
 :agent (?a - agent)
 :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
 :duration (= ?duration 5)
 :condition (and (at start (equipped_for_imaging ?r))
	(at start (>= (energy ?r) 2))
	(at start (= (calibration_target ?i) ?t))
	(over all (= (at ?r) ?w))
	(at start (visible_from ?t ?w))
	(at start (on_board ?i ?r)))
 :effect (and (at end (calibrated ?i ?r))
	(at start (decrease (energy ?r) 2)))
)


(:durative-action take_image
 :agent (?a - agent)
 :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
 :duration (= ?duration 7)
 :condition (and (over all (calibrated ?i ?r))
		(at start (on_board ?i ?r))
                	(over all (equipped_for_imaging ?r))
         	             (over all (supports ?i ?m) )
		 (over all (visible_from ?o ?p))
                	(over all ( = (at ?r) ?p))
		(at start (>= (energy ?r) 1)))
 :effect (and (at end (have_image ?r ?o ?m))
	(at start (decrease (energy ?r) 1))
	(at end (not (calibrated ?i ?r))))
)


(:durative-action communicate_soil_data
 :agent (?a - agent)
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_soil_analysis ?r ?p))
	(at start (>= (energy ?r) 4))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_soil_data ?p))
	(at end (available ?r))
	(at start (decrease (energy ?r) 4)))
)

(:durative-action communicate_rock_data
 :agent (?a - agent)
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_rock_analysis ?r ?p))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l))
	(at start (>= (energy ?r) 4)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_rock_data ?p))
	(at end (available ?r))
	(at start (decrease (energy ?r) 4)))
)


(:durative-action communicate_image_data
 :agent (?a - agent)
 :parameters (?r - rover ?l - lander ?o - objective ?m - mode
	?x - waypoint ?y - waypoint)
 :duration (= ?duration 15)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_image ?r ?o ?m))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l))
	(at start (>= (energy ?r) 6)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_image_data ?o ?m))
	(at end (available ?r))
	(at start (decrease (energy ?r) 6))))
)

;; EOF
"""

class DomainTest(unittest.TestCase):
    
    def testLogistics(self):
        """Testing logistics domain"""
        p = Parser(logistics.split("\n"))
        dom = domain.Domain.parse(p.root)

        self.assertEqual(len(dom.actions), 4)
        self.assertEqual(len(dom.sensors), 2)

    def testBlocksworld(self):
        """Testing blocksworld domain"""
        p = Parser(blocks.split("\n"))
        dom = domain.Domain.parse(p.root)

        self.assertEqual(len(dom.actions), 4)
        
    def testRovers(self):
        """Testing rovers domain"""
        p = Parser(rovers.split("\n"))
        dom = domain.Domain.parse(p.root)

        self.assertEqual(len(dom.actions), 10)
        
if __name__ == '__main__':
    unittest.main()    
        
