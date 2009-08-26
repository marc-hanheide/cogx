#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import tempfile
import os

import parser, domain, problem, writer
from parser import Parser, ParseError

domlogistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :object-fluents) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)
  
(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle))

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
  (:requirements :typing :equality :object-fluents) 
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

class ProblemTest(unittest.TestCase):
    
    def testLogistics(self):
        """Testing logistics problem"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)
        p = Parser(problogistics.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        self.assertEqual(len(prob.init), 13)
        



    def testBlocksworld(self):
        """Testing blocksworld problem"""
        
        p = Parser(domblocks.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)
        p = Parser(probblocks.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        self.assertEqual(len(prob.init), 9)


        
    def testWriterDomain(self):
        """Testing MAPLWriter domain roundtrip"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)

        w = writer.MAPLWriter()
        strings = w.write_domain(dom)
        
        p = Parser(strings)
        dom2 = domain.MAPLDomain.parse(p.root)

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
                self.assertEqual(a1.effects, a2.effects)

                
    def testWriterProblem(self):
        """Testing MAPLWriter problem roundtrip"""
        
        p = Parser(domlogistics.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)

        p = Parser(problogistics.split("\n"))
        prob = problem.Problem.parse(p.root, dom)

        w = writer.MAPLWriter()
        strings = w.write_problem(prob)
        
        p = Parser(strings)
        prob2 = problem.Problem.parse(p.root, dom)

        self.assertEqual(prob.name, prob2.name)
        self.assertEqual(len(prob.objects), len(prob2.objects))
        self.assertEqual(len(prob.init), len(prob2.init))
        self.assertEqual(set(prob.init), set(prob2.init))
        self.assertEqual(prob.goal, prob2.goal)
        
        
if __name__ == '__main__':
    unittest.main()    
        
