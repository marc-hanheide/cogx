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

(:requirements :typing :equality :object-fluents) 

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

class DomainTest(unittest.TestCase):
    
    def testLogistics(self):
        """Testing logistics domain"""
        p = Parser(logistics.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)

        self.assertEqual(len(dom.actions), 4)
        self.assertEqual(len(dom.sensors), 2)

    def testBlocksworld(self):
        """Testing blocksworld domain"""
        p = Parser(blocks.split("\n"))
        dom = domain.MAPLDomain.parse(p.root)

        self.assertEqual(len(dom.actions), 4)
        
if __name__ == '__main__':
    unittest.main()    
        
