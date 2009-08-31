#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest

import mapl_new as mapl
from mapl_new.predicates import *
from mapl_new.mapltypes import *
from mapl_new import domain, problem, conditions
from mapl_new.parser import Parser, ParseError
from state_new import *

domlogistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :mapl :typing :equality :object-fluents :durative-actions) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)
  
(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle))

(:derived (kval ?a - agent ?svar - (function object))
          (exists (?val - object) (= ?svar ?val)))

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

(:action a_load
         :agent         (?a - agent)
         :parameters    (?p - package ?v - vehicle)
         :replan        (kval ?a (location-of ?p))
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

(:objects  agent - agent
           apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 apt2 - airport
           pos1 pos2 - location
           cit1 cit2 - city)

(:init  (= (location-of apn1) apt2)
        (= (location-of tru1) pos1)
        (= (location-of tru2) pos2)
        (= (location-of obj11) pos1)
;;        (= (location-of obj12) pos1)
;;        (= (location-of obj13) pos1)
        (kval agent (location-of obj13))
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

class StateTest(unittest.TestCase):
    def setUp(self):
        p = Parser(domlogistics.split("\n"))
        self.dom = domain.MAPLDomain.parse(p.root)
        p = Parser(problogistics.split("\n"))
        self.prob = problem.Problem.parse(p.root, self.dom)
        
    def testActionInstantiation(self):
        """Testing action instantiation"""
        
        action = self.prob.getAction("drive")
        action.instantiate({"?a" :self.prob["agent"], "?t" : self.prob["tru1"], "?to" : self.prob["pos1"]})
        #Can't create a fact from an assignment from one function to another.
        self.assertRaises(Exception, Fact.fromCondition, action.precondition)
        effect = Fact.fromEffect(action.effects[0])
        expected = Fact(StateVariable(self.dom.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos1"])
        unexpected = Fact(StateVariable(self.dom.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos2"])
        
        #test (in)equality
        self.assertEqual(effect, expected)
        self.assertNotEqual(effect, unexpected)
        action.uninstantiate()
        
        action.instantiate([self.prob["agent"], self.prob["tru1"], self.prob["pos1"]])
        effect2 = Fact.fromEffect(action.effects[0])
        self.assertEqual(effect2, expected)
        action.uninstantiate()

        action.instantiate(["agent", "tru1", "pos1"])
        effect3 = Fact.fromEffect(action.effects[0])
        self.assertEqual(effect3, expected)
        action.uninstantiate()
                           
        #Test type checking
        self.assertRaises(Exception, action.instantiate, [self.prob["tru1"], self.prob["agent"], self.prob["pos1"]])
        
    def testConditionChecking(self):
        """Testing checking of conditions"""

        state = State.fromProblem(self.prob)
        
        drive = self.prob.getAction("drive")
        drive.instantiate(["agent", "tru1", "apt1"])
        self.assert_(state.isSatisfied(drive.precondition))
        drive.uninstantiate()

        drive.instantiate(["agent", "tru1", "apt2"])
        self.assertFalse(state.isSatisfied(drive.precondition))
        drive.uninstantiate()
        
    def testEffects(self):
        """Testing applying of effects"""

        state = State.fromProblem(self.prob)
        
        drive = self.prob.getAction("drive")
        drive.instantiate(["agent", "tru1", "apt1"])

        fold = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos1"])
        self.assert_(fold in state)
                           
        for e in drive.effects:
            state.applyEffect(e)
                
        fnew = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]]), self.prob["apt1"])
        self.assert_(fnew in state)
        self.assertFalse(fold in state)
        drive.uninstantiate()

    def testAxiomEvaluation(self):
        """Testing axiom evaluation"""
        state = State.fromProblem(self.prob)
        extstate = state.getExtendedState()

        kf11 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]], knowledge, [self.prob["agent"]]), TRUE)
        kf12 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj12"]], knowledge, [self.prob["agent"]]), TRUE)
        kf13 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj13"]], knowledge, [self.prob["agent"]]), TRUE)

        self.assert_(kf11 in extstate)
        self.assertFalse(kf12 in extstate)
        self.assert_(kf13 in extstate)
        self.assert_(kf13 in state)
        
        load = self.prob.getAction("a_load")
        load.instantiate(["agent", "obj11", "tru1"])
        self.assert_(extstate.isSatisfied(load.replan))
        load.uninstantiate()

        load.instantiate(["agent", "obj12", "tru1"])
        self.assertFalse(extstate.isSatisfied(load.replan))
        load.uninstantiate()

        load.instantiate(["agent", "obj13", "tru1"])
        self.assert_(extstate.isSatisfied(load.replan))
        load.uninstantiate()

        
if __name__ == '__main__':
    unittest.main()    
        
