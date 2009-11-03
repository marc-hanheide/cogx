#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, domain, actions
from mapltypes import *
from predicates import *
from actions import *
from effects import *
from parser import Parser, ParseError

logistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :mapl :typing :equality :object-fluents) 

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

cond_load = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :effect        (when (= (location-of ?p) (location-of ?v))
                                      (assign (location-of ?p) ?v)))
"""

prob_load = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        (and (probabilistic 0.4 (assign (location-of ?p) ?v)
                                               0.5 (assign (location-of ?p) (location-of ?v)))
                                      (assign-probabilistic (location-of ?p) ?v (location-of ?v))))
"""

prob_assign_load = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        
                                )
"""

univ_unload = """
        (:action dropall
                 :agent         (?a - agent)
                 :parameters    (?v - vehicle)
                 :effect        (forall (?p - package)
                                      (when (= (location-of ?p) (location-of ?v))
                                            (assign (location-of ?p) ?v))
                                ))
"""

modal_action = """
 	(:action tell_val
 	 :agent (?speaker - agent)
 	 :parameters (?hearer - agent ?var - (function object))
 	 :precondition (and
 		(KVAL ?speaker ?var)
                 (not (= ?speaker ?hearer))
 		)
 	 :effect (and
 		(KVAL ?hearer ?var)
 	))

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
        

    def testDurativeAction(self):
        """Testing durative action parsing"""
        
        action = Parser.parseAs(dur_drive.split("\n"), DurativeAction, self.domain)

        self.assertEqual(action.precondition.__class__, conditions.TimedCondition)
        self.assertEqual(action.precondition.time, "all")
        self.assert_(isinstance(action.effects[0], TimedEffect))
        self.assertEqual(action.effects[0].time, "end")
        
    def testModalAction(self):
        """Testing modal action parsing"""
        
        action = Parser.parseAs(modal_action.split("\n"), Action, self.domain)

        self.assertEqual(action.args[1].type, FunctionType(objectType))
        term = predicates.FunctionTerm(self.domain.functions["location-of"][0], [Parameter("?c", self.domain.types["city"])])
        action.instantiate({"?var" : term})
        
    def testEffects(self):
        """Testing basic effect parsing"""
        
        action = Parser.parseAs(drive.split("\n"), Action, self.domain)
        self.assertEqual(len(action.effects), 1)
        self.assertEqual(len(action.effects), 1)

    def testConditionalEffects(self):
        """Testing conditional effect parsing"""
        
        action = Parser.parseAs(cond_load.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effects[0], ConditionalEffect))
        self.assert_(isinstance(action.effects[0].condition, conditions.LiteralCondition))
        self.assert_(isinstance(action.effects[0].effects[0], SimpleEffect))

    def testUniversalEffects(self):
        """Testing conditional effect parsing"""
        
        action = Parser.parseAs(univ_unload.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effects[0], UniversalEffect))
        self.assertEqual(len(action.effects[0].args), 1)
        self.assert_(isinstance(action.effects[0].effects[0], ConditionalEffect))

    def testProbabilisticEffects(self):
        """Testing probabilistic effect parsing"""
        
        action = Parser.parseAs(prob_load.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effects[0], ProbabilisticEffect))
        p1, e1 = action.effects[0].effects[0]
        p2, e2 = action.effects[0].effects[1]

        ap1, ae1 = action.effects[1].effects[0]
        ap2, ae2 = action.effects[1].effects[1]

        self.assertEqual(p1, 0.4)
        self.assert_(isinstance(e1[0].args[0], FunctionTerm))
        self.assert_(isinstance(e1[0].args[1], VariableTerm))
        self.assertEqual(p2, 0.5)
        self.assert_(isinstance(e2[0].args[0], FunctionTerm))
        self.assert_(isinstance(e2[0].args[1], FunctionTerm))

        self.assertEqual(ae1, e1)
        self.assertEqual(ae2, e2)
        self.assertEqual(ap1, 0.5)
        self.assertEqual(ap2, 0.5)

        self.assertEqual(action.effects[0].getRandomEffect(0), e2)
        self.assertEqual(action.effects[0].getRandomEffect(1), e1)
        self.assertEqual(action.effects[0].getRandomEffect(2), [])

        import random
        random.seed(42)
        for r in xrange(30):
            self.assert_(action.effects[0].getRandomEffect() in (e1,e2,[]))
        
    def testAssertion(self):
        """Testing parsing of assertions"""
        
        action = Parser.parseAs(a_load.split("\n"), Action, self.domain)
        self.assert_(action.replan is not None)

        
if __name__ == '__main__':
    unittest.main()    
        
