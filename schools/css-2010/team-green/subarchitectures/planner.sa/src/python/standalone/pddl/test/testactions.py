#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, domain, actions, mapl, durative
from mapltypes import *
from predicates import *
from effects import *
from actions import Action
from parser import Parser, ParseError

logistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :adl :object-fluents :numeric-fluents) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)
  
(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle)
             (load_succ_prob) - number
)

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
                 :parameters    (?t - truck ?to - location)
                 :precondition  (= (city-of (location-of ?t)) (city-of ?to))
                 :effect        (assign (location-of ?t) ?to))
        """

mapl_drive = """
        (:action drive
                 :agent         (?a - agent)
                 :parameters    (?t - truck ?to - location)
                 :precondition  (= (city-of (location-of ?t)) (city-of ?to))
                 :effect        (assign (location-of ?t) ?to))
        """

dur_drive = """
        (:durative-action drive
                 :parameters    (?t - truck ?to - location)
                 :duration      (= ?duration 4)
                 :condition     (over all (= (city-of (location-of ?t)) (city-of ?to)))
                 :effect        (at end (assign (location-of ?t) ?to)))
        """

a_load = """
        (:action load
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

a_load_mapl = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

cond_load = """
        (:action load
                 :parameters    (?p - package ?v - vehicle)
                 :effect        (when (= (location-of ?p) (location-of ?v))
                                      (assign (location-of ?p) ?v)))
"""

prob_load = """
        (:action load
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        (and (probabilistic (load_succ_prob) (assign (location-of ?p) ?v)
                                               0.5 (assign (location-of ?p) (location-of ?v)))
                                      (assign-probabilistic (location-of ?p) 0.5 ?v (location-of ?v))))
"""

prob_assign_load = """
        (:action load
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        
                                )
"""

univ_unload = """
        (:action dropall
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

error_load1 = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

error_load2 = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

error_load3 = """
        (:action load
                 :agent         (?a - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :replan        (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v)
                 :effect        (assign (location-of ?p) ?v))
"""

error_load4 = """
        (:action load
                 :agent         (?p - agent)
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        (assign (location-of ?p) ?v))
"""

class ActionTest(unittest.TestCase):

    def setUp(self):
        p = Parser(logistics.split("\n"))
        self.domain = domain.Domain.parse(p.root)
        self.truck = TypedObject("truck1", self.domain.types["truck"])
        self.plane = TypedObject("plane", self.domain.types["airplane"])
        self.p = TypedObject("p1", self.domain.types["package"])
        self.airport = TypedObject("ap", self.domain.types["airport"])
        self.loc1 = TypedObject("loc1", self.domain.types["location"])
        self.loc2 = TypedObject("loc2", self.domain.types["location"])
        self.agent = TypedObject("agent", self.domain.types["agent"])

        self.domain.predicates.add(mapl.knowledge)

        self.domain.add([self.truck, self.plane, self.p, self.airport, self.loc1, self.loc2, self.agent])
        

    def testDurativeAction(self):
        """Testing durative action parsing"""
        
        action = Parser.parse_as(dur_drive.split("\n"), durative.DurativeAction, self.domain)

        self.assertEqual(action.precondition.__class__, durative.TimedCondition)
        self.assertEqual(action.precondition.time, "all")
        self.assert_(isinstance(action.effect, durative.TimedEffect))
        self.assertEqual(action.effect.time, "end")
        
    def testModalAction(self):
        """Testing modal action parsing"""
        
        action = Parser.parse_as(modal_action.split("\n"), mapl.MAPLAction, self.domain)

        self.assertEqual(action.maplargs[1].type, FunctionType(t_object))
        term = predicates.FunctionTerm(self.domain.functions["location-of"][0], [Parameter("?c", self.domain.types["city"])])
        action.instantiate({"?var" : term})
        
    def testEffects(self):
        """Testing basic effect parsing"""
        
        action = Parser.parse_as(drive.split("\n"), Action, self.domain)
        self.assert_(isinstance(action.effect, SimpleEffect))


    def testMAPLAction(self):
        """Testing basic effect parsing"""
        
        action = Parser.parse_as(mapl_drive.split("\n"), mapl.MAPLAction, self.domain)
        self.assertEqual(len(action.agents), 1)
        self.assertEqual(len(action.maplargs), 2)
        self.assertEqual(len(action.vars), 0)
        self.assertEqual(len(action.args), 3)
        self.assert_(isinstance(action.effect, SimpleEffect))
        
    def testConditionalEffects(self):
        """Testing conditional effect parsing"""
        
        action = Parser.parse_as(cond_load.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effect, ConditionalEffect))
        self.assert_(isinstance(action.effect.condition, conditions.LiteralCondition))
        self.assert_(isinstance(action.effect.effect, SimpleEffect))

    def testUniversalEffects(self):
        """Testing conditional effect parsing"""
        
        action = Parser.parse_as(univ_unload.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effect, UniversalEffect))
        self.assertEqual(len(action.effect.args), 1)
        self.assert_(isinstance(action.effect.effect, ConditionalEffect))

    def testProbabilisticEffects(self):
        """Testing probabilistic effect parsing"""

        action = Parser.parse_as(prob_load.split("\n"), Action, self.domain)

        self.assert_(isinstance(action.effect, ConjunctiveEffect))
        self.assert_(isinstance(action.effect.parts[0], ProbabilisticEffect))
        self.assert_(isinstance(action.effect.parts[1], ProbabilisticEffect))
        p1, e1 = action.effect.parts[0].effects[0]
        p2, e2 = action.effect.parts[0].effects[1]

        ap1, ae1 = action.effect.parts[1].effects[0]
        ap2, ae2 = action.effect.parts[1].effects[1]

        self.assert_(isinstance(p1, FunctionTerm))
        self.assertEqual(p1.function, self.domain.functions["load_succ_prob"][0])
        self.assert_(isinstance(e1.args[0], FunctionTerm))
        self.assert_(isinstance(e1.args[1], VariableTerm))
        self.assertEqual(p2, 0.5)
        self.assert_(isinstance(e2.args[0], FunctionTerm))
        self.assert_(isinstance(e2.args[1], FunctionTerm))

        self.assertEqual(ae1, e1)
        self.assertEqual(ae2, e2)
        self.assertEqual(ap1, 0.5)
        self.assertEqual(ap2, None)

        # self.assertEqual(action.effect.parts[0].getRandomEffect(0), e2)
        # self.assertEqual(action.effect.parts[0].getRandomEffect(1), e1)
        # self.assertEqual(action.effect.parts[0].getRandomEffect(2), None)

        # import random
        # random.seed(42)
        # for r in xrange(30):
        #     self.assert_(action.effect.parts[0].getRandomEffect() in (e1,e2,None))
        
    def testAssertion(self):
        """Testing parsing of assertions"""
        
        action = Parser.parse_as(a_load_mapl.split("\n"), mapl.MAPLAction, self.domain)
        self.assert_(action.replan is not None)

        action = Parser.parse_as(a_load.split("\n"), Action, self.domain)
        self.assert_(action.replan is not None)
        
    def testMaplErrorHandling(self):
        """Testing error handling of MaplAction"""
        try:
            action = Parser.parse_as(error_load1.split("\n"), mapl.MAPLAction, self.domain)
            self.fail("Action with duplicate precondition didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, ":precondition")
            self.assertEqual(e.token.line, 6)

        try:
            action = Parser.parse_as(error_load2.split("\n"), mapl.MAPLAction, self.domain)
            self.fail("Action with duplicate replan condition didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, ":replan")
            self.assertEqual(e.token.line, 7)

        try:
            action = Parser.parse_as(error_load3.split("\n"), mapl.MAPLAction, self.domain)
            self.fail("Action with duplicate effect statement didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, ":effect")
            self.assertEqual(e.token.line, 8)

        try:
            action = Parser.parse_as(error_load4.split("\n"), mapl.MAPLAction, self.domain)
            self.fail("Action with duplicate parameters didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "?p")
            self.assertEqual(e.token.line, 4)
            
        
if __name__ == '__main__':
    unittest.main()    
        
