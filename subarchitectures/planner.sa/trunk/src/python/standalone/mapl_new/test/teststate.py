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

(:requirements :mapl :typing :equality :fluents :durative-actions) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)

(:predicates (occupied ?l - location)
             (interesting ?l - location)
             (free ?l - location))

(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle)
             (num_packages ?v - vehicle) - number
             (capacity ?v - vehicle) - number
)

;(:derived (kval ?a - agent ?svar - (function object))
;          (exists (?val - object) (= ?svar ?val)))
;
;(:derived (in-domain ?svar - (function object) ?val - object)
;          (or (= ?svar ?val)
;              (and (i_in-domain ?svar ?val)
;                   (not (exists (?val - object) (= ?svar ?val))))
;          ))

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
         :precondition  (and (= (location-of ?p) (location-of ?v))
                             (< (num_packages ?v) (capacity ?v)))
         :effect        (and (assign (location-of ?p) ?v)
                             (increase (num_packages ?v) 1))
)

(:action a_load
         :agent         (?a - agent)
         :parameters    (?p - package ?v - vehicle)
         :precondition  (in-domain (location-of ?p) (location-of ?v))
         :replan        (kval ?a (location-of ?p))
         :effect        (assign (location-of ?p) ?v))

(:action unload
         :agent         (?a - agent)
         :parameters    (?p - package ?v - vehicle)
         :precondition  (= (location-of ?p) ?v)
         :effect        (and (assign (location-of ?p) (location-of ?v))
                             (decrease (num_packages ?v) (/ (+ (* 5 (- 2 1)) (- 3)) 2) ))

)
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
        (i_in-domain (location-of obj13) pos1)
        (i_in-domain (location-of obj13) pos2)
        (= (location-of obj21) pos2)
        (= (location-of obj22) pos2)
        (= (location-of obj23) pos2)
        (= (city-of apt1) cit1)
        (= (city-of apt2) cit2)
        (= (city-of pos1) cit1)
        (= (city-of pos2) cit2)
        (= (num_packages apn1) 0)
        (= (num_packages tru1) 0)
        (= (num_packages tru2) 0)

        (= (capacity apn1) 20)
        (= (capacity tru1) 4)
        (= (capacity tru2) 4)
)

(:goal  (and (= (location-of obj11) apt1)
             (= (location-of obj13) apt1)
             (= (location-of obj21) pos1)
             (= (location-of obj23) pos1)))

)
"""

strat1a = """
        (:derived (occupied ?loc - location)
                  (exists (?v - vehicle) (= (location-of ?v) ?loc))
        )
        """

strat1b = """
        (:derived (interesting ?loc - location)
                  (or (occupied ?loc)
                      (exists (?p - package) (= (location-of ?p) ?loc)))
        )
        """

strat2 = """
        (:derived (free ?loc - location)
                  (not (occupied ?loc))
        )
        """
        


class StateTest(unittest.TestCase):
    def setUp(self):
        p = Parser(domlogistics.split("\n"))
        self.dom = domain.Domain.parse(p.root)
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

    def testNumericConditions(self):
        """Testing numeric conditions"""

        state = State.fromProblem(self.prob)
        num_packages = StateVariable(self.prob.functions["num_packages"][0], [self.prob["tru2"]])
        
        load = self.prob.getAction("load")
        load.instantiate(["agent", "obj21", "tru2"])
        self.assert_(state.isSatisfied(load.precondition))
        
        state[num_packages] = TypedNumber(4)
        self.assertFalse(state.isSatisfied(load.precondition))

        state[num_packages] = TypedNumber(5)
        self.assertFalse(state.isSatisfied(load.precondition))
        
        load.uninstantiate()
        
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

    def testNumericEffects(self):
        """Testing numeric effects"""

        state = State.fromProblem(self.prob)
        num_packages = StateVariable(self.prob.functions["num_packages"][0], [self.prob["tru2"]])
        
        load = self.prob.getAction("load")
        load.instantiate(["agent", "obj21", "tru2"])

        fold = Fact(num_packages, TypedNumber(0))
        self.assert_(fold in state)
                           
        state.applyEffect(load.effects)
                
        fnew = Fact(num_packages, TypedNumber(1))
        self.assert_(fnew in state)
        self.assertFalse(fold in state)
        load.uninstantiate()
        
        load.instantiate(["agent", "obj22", "tru2"])
        state.applyEffect(load.effects)
        load.uninstantiate()

        fnew2 = Fact(num_packages, TypedNumber(2))
        fnew2b = Fact(num_packages, 2)
        self.assert_(fnew2 in state)
        self.assert_(fnew2b in state)
        
        unload = self.prob.getAction("unload")
        unload.instantiate(["agent", "obj21", "tru2"])
        state.applyEffect(unload.effects)
        unload.uninstantiate()

        self.assert_(fnew in state)
        
    def testConditionReasons(self):
        """Testing finding reason of satisfied conditions"""
        
        state = State.fromProblem(self.prob)

        relevantVars = []
        drive = self.prob.getAction("drive")
        drive.instantiate(["agent", "tru1", "apt1"])
        self.assert_(state.isSatisfied(drive.precondition, relevantVars))
        drive.uninstantiate()

        relevantVars = set(relevantVars)
        
        s1 = StateVariable(self.prob.functions["city-of"][0], [self.prob["pos1"]])
        s2 = StateVariable(self.prob.functions["city-of"][0], [self.prob["apt1"]])
        s3 = StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]])
        
        self.assertEqual(len(relevantVars), 3)
        self.assert_(s1 in relevantVars)
        self.assert_(s2 in relevantVars)
        self.assert_(s3 in relevantVars)

    def testAxiomEvaluation(self):
        """Testing axiom evaluation"""
        state = State.fromProblem(self.prob)
        extstate = state.getExtendedState()

        kf11 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]], knowledge, [self.prob["agent"]]), TRUE)
        kf12 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj12"]], knowledge, [self.prob["agent"]]), TRUE)
        kf13 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj13"]], knowledge, [self.prob["agent"]]), TRUE)

        id11 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]], indomain, [self.prob["pos1"]]), TRUE)
        
        self.assert_(kf11 in extstate)
        self.assertFalse(kf12 in extstate)
        self.assert_(kf13 in extstate)
        self.assert_(kf13 in state)

        self.assert_(id11 in extstate)
        
        load = self.prob.getAction("a_load")
        load.instantiate(["agent", "obj11", "tru1"])
        self.assert_(extstate.isSatisfied(load.replan))
        self.assert_(extstate.isSatisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj12", "tru1"])
        self.assertFalse(extstate.isSatisfied(load.replan))
        self.assertFalse(extstate.isSatisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj13", "tru1"])
        self.assert_(extstate.isSatisfied(load.replan))
        self.assert_(extstate.isSatisfied(load.precondition))
        load.uninstantiate()

    def testPartialAxiomEvaluation(self):
        """Testing partial axiom evaluation"""
        state = State.fromProblem(self.prob)

        load = self.prob.getAction("a_load")
        load.instantiate(["agent", "obj11", "tru1"])
        extstate = state.getExtendedState(state.getRelevantVars(load.replan) | state.getRelevantVars(load.precondition))
        self.assert_(extstate.isSatisfied(load.replan))
        self.assert_(extstate.isSatisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj12", "tru1"])
        extstate = state.getExtendedState(state.getRelevantVars(load.replan) | state.getRelevantVars(load.precondition))
        self.assertFalse(extstate.isSatisfied(load.replan))
        self.assertFalse(extstate.isSatisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj13", "tru1"])
        extstate = state.getExtendedState(state.getRelevantVars(load.replan) | state.getRelevantVars(load.precondition))
        self.assert_(extstate.isSatisfied(load.replan))
        self.assert_(extstate.isSatisfied(load.precondition))
        load.uninstantiate()
        
    def testAxiomReasoning(self):
        """Testing finding reasons of derived predicates """
        
        state = State.fromProblem(self.prob)
        extstate, reasons, universalReasons = state.getExtendedState(getReasons=True)

        relevantVars = []
        relevantReplanVars = []
        load = self.prob.getAction("a_load")
        load.instantiate(["agent", "obj11", "tru1"])
        self.assert_(extstate.isSatisfied(load.precondition, relevantVars))
        self.assert_(extstate.isSatisfied(load.replan, relevantReplanVars))

        all_reasons = set(relevantVars)
        for v in relevantVars:
            all_reasons |= reasons[v]

        all_replan_reasons = set(relevantReplanVars)
        for v in relevantReplanVars:
            all_replan_reasons |= reasons[v]
            
        s1 = StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]])
        s2 = StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]])

        self.assert_(s1 in all_reasons)
        self.assert_(s2 in all_reasons)
        self.assertFalse(s1 in all_replan_reasons)
        self.assert_(s2 in all_replan_reasons)
                
    def testStratifiedAxioms(self):
        """Testing evaluation of stratified axioms"""
        
        a1a = Parser.parseAs(strat1a.split("\n"), mapl.axioms.Axiom, self.prob)
        a1b = Parser.parseAs(strat1b.split("\n"), mapl.axioms.Axiom, self.prob)
        a2 = Parser.parseAs(strat2.split("\n"), mapl.axioms.Axiom, self.prob)
        
        self.prob.axioms = [a1a, a1b, a2]
        self.prob.stratifyAxioms()
        state = State.fromProblem(self.prob).getExtendedState()
        
if __name__ == '__main__':
    unittest.main()    
        
