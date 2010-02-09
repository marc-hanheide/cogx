#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import tempfile
import os

import predicates, parser, domain, problem, writer
import mapl
from parser import Parser, ParseError

domlogistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :mapl :typing :equality :object-fluents :durative-actions) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)

(:predicates (X ?f - (function object) ?v - (typeof ?f)))

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

(:durative-action dur_drive
         :agent         (?a - agent)
         :parameters    (?t - truck ?to - location)
         :duration      (= ?duration 4)
         :condition     (over all (= (city-of (location-of ?t)) (city-of ?to)))
         :effect        (at end (assign (location-of ?t) ?to)))

(:action fly
         :agent         (?a - agent)
         :parameters    (?ap - airplane ?to - airport)
         :effect        (assign (location-of ?ap) ?to))

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

problogistics = \
"""
(define (problem logistics-4-0)

(:domain logistics-object-fluents)

(:objects  apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 apt2 - airport
           pos1 pos2 - location
           cit1 cit2 - city)

(:init  (= (location-of apn1) apt2)
        (= (location-of tru1) pos1)
        (= (location-of tru2) pos2)
        (= (location-of obj11) pos1)
        (= (location-of obj12) pos1)
        (= (location-of obj13) pos1)
        (= (location-of obj21) pos2)
        (= (location-of obj22) pos2)
        (= (location-of obj23) pos2)
        (= (city-of apt1) cit1)
        (= (city-of apt2) cit2)
        (= (city-of pos1) cit1)
        (= (city-of pos2) cit2))

(:goal  (and (= (location-of obj11) apt1)
             (= (location-of obj13) apt1)
             (= (location-of obj21) pos1)
             (= (location-of obj23) pos1)))

)
"""

domblocks = \
"""
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS-object-fluents)
  (:requirements :typing :equality :object-fluents :mapl) 
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

probblocks = \
"""
(define (problem BLOCKS-4-0)

(:domain BLOCKS-object-fluents)

(:objects D B A C - block)

(:INIT 	(= (on-block C) no-block) 
	(= (on-block A) no-block) 
	(= (on-block B) no-block) 
	(= (on-block D) no-block)
	(on-table C) 
	(on-table A)
	(on-table B) 
	(on-table D)
	(= (in-hand) no-block))

(:goal (AND (= (on-block C) D) (= (on-block B) C) (= (on-block A) B))))

"""

domrovers = """
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

probrovers = """
(define (problem roverprob1234) (:domain Rover)
(:objects
	general - Lander
	colour high_res low_res - Mode
	rover0 - Rover
	rover0store - Store
	waypoint0 waypoint1 waypoint2 waypoint3 - Waypoint
	camera0 - Camera
	objective0 objective1 - Objective
	)
(:init
	(visible waypoint1 waypoint0)
	(visible waypoint0 waypoint1)
	(visible waypoint2 waypoint0)
	(visible waypoint0 waypoint2)
	(visible waypoint2 waypoint1)
	(visible waypoint1 waypoint2)
	(visible waypoint3 waypoint0)
	(visible waypoint0 waypoint3)
	(visible waypoint3 waypoint1)
	(visible waypoint1 waypoint3)
	(visible waypoint3 waypoint2)
	(visible waypoint2 waypoint3)
	(at_soil_sample waypoint0)
	(in_sun waypoint0)
	(at_rock_sample waypoint1)
	(at_soil_sample waypoint2)
	(at_rock_sample waypoint2)
	(at_soil_sample waypoint3)
	(at_rock_sample waypoint3)
	(= (at general) waypoint0)
	(channel_free general)
	(= (energy rover0) 50)
	(= (recharge-rate rover0) 11)
	(= (at rover0) waypoint3)
	(available rover0)
	(store_of rover0store rover0)
	(= (sstate rover0store) empty)
	(equipped_for_soil_analysis rover0)
	(equipped_for_rock_analysis rover0)
	(equipped_for_imaging rover0)
	(can_traverse rover0 waypoint3 waypoint0)
	(can_traverse rover0 waypoint0 waypoint3)
	(can_traverse rover0 waypoint3 waypoint1)
	(can_traverse rover0 waypoint1 waypoint3)
	(can_traverse rover0 waypoint1 waypoint2)
	(can_traverse rover0 waypoint2 waypoint1)
	(on_board camera0 rover0)
	(= (calibration_target camera0) objective1)
	(supports camera0 colour)
	(supports camera0 high_res)
	(visible_from objective0 waypoint0)
	(visible_from objective0 waypoint1)
	(visible_from objective0 waypoint2)
	(visible_from objective0 waypoint3)
	(visible_from objective1 waypoint0)
	(visible_from objective1 waypoint1)
	(visible_from objective1 waypoint2)
	(visible_from objective1 waypoint3)
)

(:goal (and
(communicated_soil_data waypoint2)
(communicated_rock_data waypoint3)
(communicated_image_data objective1 high_res))
)

(:metric minimize (total-time))

)
"""

class ProblemTest(unittest.TestCase):
    
    def testLogistics(self):
        """Testing logistics problem"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(problogistics.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        self.assertEqual(len(prob.init), 13)



    def testBlocksworld(self):
        """Testing blocksworld problem"""
        
        p = Parser(domblocks.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(probblocks.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        self.assertEqual(len(prob.init), 9)

    def testRovers(self):
        """Testing Rovers problem"""
        
        p = Parser(domrovers.split("\n"))
        dom = domain.Domain.parse(p.root)
        p = Parser(probrovers.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        self.assertEqual(len(prob.init), 48)

        
    def testWriterDomain(self):
        """Testing MAPLWriter domain roundtrip"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.Domain.parse(p.root)

        w = mapl.MAPLWriter()
        strings = w.write_domain(dom)
        
        p = Parser(strings)
        dom2 = domain.Domain.parse(p.root)

        self.assertEqual(dom.name, dom2.name)
        self.assertEqual(len(dom.constants), len(dom2.constants))
        self.assertEqual(len(dom.actions), len(dom2.actions))
        self.assertEqual(len(dom.sensors), len(dom2.sensors))
        for a1 in dom.actions:
            for a2 in dom2.actions:
                if a1.name != a2.name:
                    continue
                self.assertEqual(a1.agents, a2.agents)
                self.assertEqual(a1.args, a2.args)
                self.assertEqual(a1.vars, a2.vars)
                self.assertEqual(a1.precondition, a2.precondition)
                self.assertEqual(a1.replan, a2.replan)
                self.assertEqual(a1.effect, a2.effect)

        for s1 in dom.sensors:
            for s2 in dom2.sensors:
                if s1.name != s2.name:
                    continue
                self.assertEqual(s1.agents, s2.agents)
                self.assertEqual(s1.args, s2.args)
                self.assertEqual(s1.vars, s2.vars)
                self.assertEqual(s1.precondition, s2.precondition)
                self.assertEqual(s1.senses, s2.senses)
                
    def testWriterProblem(self):
        """Testing MAPLWriter problem roundtrip"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.Domain.parse(p.root)

        p = Parser(problogistics.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        w = mapl.MAPLWriter()
        strings = w.write_problem(prob)
        
        p = Parser(strings)
        prob2 = problem.Problem.parse(p.root, dom)

        self.assertEqual(prob.name, prob2.name)
        self.assertEqual(len(prob.objects), len(prob2.objects))
        self.assertEqual(len(prob.init), len(prob2.init))
        self.assertEqual(set(prob.init), set(prob2.init))
        self.assertEqual(prob.goal, prob2.goal)


    def testWriterDomainNumeric(self):
        """Testing MAPLWriter domain roundtrip with numeric fluents and durative actions"""
        
        p = Parser(domrovers.split("\n"))
        dom = domain.Domain.parse(p.root)

        w = mapl.MAPLWriter()
        strings = w.write_domain(dom)
        p = Parser(strings)
        dom2 = domain.Domain.parse(p.root)

        self.assertEqual(dom.name, dom2.name)
        self.assertEqual(len(dom.constants), len(dom2.constants))
        self.assertEqual(len(dom.actions), len(dom2.actions))
        self.assertEqual(len(dom.sensors), len(dom2.sensors))
        for a1 in dom.actions:
            for a2 in dom2.actions:
                if a1.name != a2.name:
                    continue
                self.assertEqual(a1.agents, a2.agents)
                self.assertEqual(a1.args, a2.args)
                self.assertEqual(a1.duration, a2.duration)
                self.assertEqual(a1.vars, a2.vars)
                self.assertEqual(a1.precondition, a2.precondition)
                self.assertEqual(a1.replan, a2.replan)
                self.assertEqual(a1.effect, a2.effect)

            
    def testWriterProblemNumeric(self):
        """Testing MAPLWriter problem roundtrip with numeric fluents and durative actions"""
        
        p = Parser(domrovers.split("\n"))
        dom = domain.Domain.parse(p.root)

        p = Parser(probrovers.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        w = mapl.MAPLWriter()
        strings = w.write_problem(prob)
        
        p = Parser(strings)
        prob2 = problem.Problem.parse(p.root, dom)

        self.assertEqual(prob.name, prob2.name)
        self.assertEqual(len(prob.objects), len(prob2.objects))
        self.assertEqual(len(prob.init), len(prob2.init))
        self.assertEqual(set(prob.init), set(prob2.init))
        self.assertEqual(prob.goal, prob2.goal)
        self.assertEqual(prob.optimization, prob2.optimization)
        self.assertEqual(prob.opt_func, prob2.opt_func)
        
        
if __name__ == '__main__':
    unittest.main()    
        
