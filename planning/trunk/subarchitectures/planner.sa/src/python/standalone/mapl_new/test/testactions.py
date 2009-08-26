#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, domain, actions, state
from mapltypes import *
from actions import *
from effects import *
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
)"""
# (:action drive
#          :agent         (?a - agent)
#          :parameters    (?t - truck ?to - location)
#          :precondition  (= (city-of (location-of ?t)) (city-of ?to))
#          :effect        (assign (location-of ?t) ?to))

# (:action fly
#          :agent         (?a - agent)
#          :parameters    (?a - airplane ?to - airport)
#          :effect        (assign (location-of ?a) ?to))

# (:action load
#          :agent         (?a - agent)
#          :parameters    (?p - package ?v - vehicle)
#          :precondition  (= (location-of ?p) (location-of ?v))
#          :effect        (assign (location-of ?p) ?v))

# (:action unload
#          :agent         (?a - agent)
#          :parameters    (?p - package ?v - vehicle)
#          :precondition  (= (location-of ?p) ?v)
#          :effect        (assign (location-of ?p) (location-of ?v)))

# )
# """

drive = """
        (:action drive
                 :agent         (?a - agent)
                 :parameters    (?t - truck ?to - location)
                 :precondition  (= (city-of (location-of ?t)) (city-of ?to))
                 :effect        (assign (location-of ?t) ?to))
        """

dur_drive = """
        (:durative-action drive
                 :agent         (?a - agent)
                 :parameters    (?t - truck ?to - location)
                 :duration      (= ?duration 4)
                 :condition     (over all (= (city-of (location-of ?t)) (city-of ?to)))
                 :effect        (at end (assign (location-of ?t) ?to)))
        """

a_load = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

class ActionTest(unittest.TestCase):

    def setUp(self):
        p = Parser(logistics.split("\n"))
        self.domain = domain.MAPLDomain.parse(p.root)
        self.truck = TypedObject("truck1", self.domain.types["truck"])
        self.plane = TypedObject("plane", self.domain.types["airplane"])
        self.p = TypedObject("p1", self.domain.types["package"])
        self.airport = TypedObject("ap", self.domain.types["airport"])
        self.loc1 = TypedObject("loc1", self.domain.types["location"])
        self.loc2 = TypedObject("loc2", self.domain.types["location"])
        self.agent = TypedObject("agent", self.domain.types["agent"])

        self.domain.add([self.truck, self.plane, self.p, self.airport, self.loc1, self.loc2, self.agent])
        

    def testInstantiation(self):
        """Testing action instantiation"""
        
        action = Parser.parseAs(drive.split("\n"), Action, self.domain)
        action.instantiate({"?a" :self.agent, "?t" : self.truck, "?to" : self.loc1})
        #Can't create a fact from an assignment from one function to another.
        self.assertRaises(Exception, state.Fact.fromCondition, action.precondition)
        effect = state.Fact.fromEffect(action.effects[0])
        expected = state.Fact(state.StateVariable(self.domain.functions["location-of"][0], [self.truck]), self.loc1)
        unexpected = state.Fact(state.StateVariable(self.domain.functions["location-of"][0], [self.truck]), self.loc2)
        #test (in)equality
        self.assertEqual(effect, expected)
        self.assertNotEqual(effect, unexpected)
        action.uninstantiate()
        
        action.instantiate([self.agent, self.truck, self.loc1])
        effect2 = state.Fact.fromEffect(action.effects[0])
        self.assertEqual(effect2, expected)

        action.uninstantiate()
        #Test type checking
        self.assertRaises(Exception, action.instantiate, [self.truck, self.agent, self.loc1])

    def testDurativeAction(self):
        """Testing durative action parsing"""
        
        action = Parser.parseAs(dur_drive.split("\n"), DurativeAction, self.domain)

        self.assertEqual(action.precondition.__class__, conditions.TimedCondition)
        self.assertEqual(action.precondition.time, "all")
        self.assert_(isinstance(action.effects[0], TimedEffect))
        self.assertEqual(action.effects[0].time, "end")
        
    def testEffects(self):
        """Testing basic effect parsing"""
        
        action = Parser.parseAs(drive.split("\n"), Action, self.domain)
        self.assertEqual(len(action.effects), 1)
        self.assertEqual(len(action.effects), 1)

        
    def testAssertion(self):
        """Testing parsing of assertions"""
        
        action = Parser.parseAs(a_load.split("\n"), Action, self.domain)
        self.assert_(action.replan is not None)

        
if __name__ == '__main__':
    unittest.main()    
        
