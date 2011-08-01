#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import common

import predicates, domain, problem, conditions, axioms, mapl
from parser import Parser, ParseError
from builtin import *
from mapltypes import *
from state import *

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
        


class StateTest(common.PddlTest):
    def setUp(self):
        self.dom, self.prob = self.load("testdata/logistics-ext.domain.mapl", "testdata/logistics-ext.problem.mapl")
        
    def testActionInstantiation(self):
        """Testing action instantiation"""
        
        action = self.dom.get_action("drive")
        action.instantiate({"?a" :self.prob["agent"], "?t" : self.prob["tru1"], "?to" : self.prob["pos1"]}, self.prob)
        #Can't create a fact from an assignment from one function to another.
        self.assertRaises(Exception, Fact.from_condition, action.precondition)
        effect = Fact.from_effect(action.effect)[0]
        expected = Fact(StateVariable(self.dom.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos1"])
        unexpected = Fact(StateVariable(self.dom.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos2"])
        
        #test (in)equality
        self.assertEqual(effect, expected)
        self.assertNotEqual(effect, unexpected)
        action.uninstantiate()

        #Test alternative instantiation syntax
        action.instantiate([self.prob["agent"], self.prob["tru1"], self.prob["pos1"]], self.prob)
        effect2 = Fact.from_effect(action.effect)[0]
        self.assertEqual(effect2, expected)
        action.uninstantiate()

        #Test name only instantiation
        action.instantiate(["agent", "tru1", "pos1"], self.prob)
        effect3 = Fact.from_effect(action.effect)[0]
        self.assertEqual(effect3, expected)
        action.uninstantiate()
                           
        #Test type checking
        self.assertRaises(Exception, action.instantiate, [self.prob["tru1"], self.prob["agent"], self.prob["pos1"]])
        
    def testConditionChecking(self):
        """Testing checking of conditions"""

        state = State.from_problem(self.prob)
        
        drive = self.dom.get_action("drive")
        drive.instantiate(["agent", "tru1", "apt1"], self.prob)
        self.assert_(state.is_satisfied(drive.precondition))
        drive.uninstantiate()

        drive.instantiate(["agent", "tru1", "apt2"], self.prob)
        self.assertFalse(state.is_satisfied(drive.precondition))
        drive.uninstantiate()

    def testNumericConditions(self):
        """Testing numeric conditions"""

        state = State.from_problem(self.prob)
        num_packages = StateVariable(self.prob.functions["num_packages"][0], [self.prob["tru2"]])
        
        load = self.dom.get_action("load")
        load.instantiate(["agent", "obj21", "tru2"], self.prob)
        self.assert_(state.is_satisfied(load.precondition))
        
        state[num_packages] = TypedNumber(4)
        self.assertFalse(state.is_satisfied(load.precondition))

        state[num_packages] = TypedNumber(5)
        self.assertFalse(state.is_satisfied(load.precondition))
        
        load.uninstantiate()
        
    def testEffects(self):
        """Testing applying of effects"""

        state = State.from_problem(self.prob)
        
        drive = self.dom.get_action("drive")
        drive.instantiate(["agent", "tru1", "apt1"], self.prob)

        fold = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]]), self.prob["pos1"])
        self.assert_(fold in state)
                           
        state.apply_effect(drive.effect)
                
        fnew = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["tru1"]]), self.prob["apt1"])
        self.assert_(fnew in state)
        self.assertFalse(fold in state)
        drive.uninstantiate()

    def testNumericEffects(self):
        """Testing numeric effects"""

        state = State.from_problem(self.prob)
        num_packages = StateVariable(self.prob.functions["num_packages"][0], [self.prob["tru2"]])
        
        load = self.dom.get_action("load")
        load.instantiate(["agent", "obj21", "tru2"], self.prob)

        fold = Fact(num_packages, TypedNumber(0))
        self.assert_(fold in state)
                           
        state.apply_effect(load.effect)
                
        fnew = Fact(num_packages, TypedNumber(1))
        self.assert_(fnew in state)
        self.assertFalse(fold in state)
        load.uninstantiate()
        
        load.instantiate(["agent", "obj22", "tru2"], self.prob)
        state.apply_effect(load.effect)
        load.uninstantiate()

        fnew2 = Fact(num_packages, TypedNumber(2))
        self.assert_(fnew2 in state)
        self.assertEqual(state[num_packages].value, 2)
        
        unload = self.dom.get_action("unload")
        unload.instantiate(["agent", "obj21", "tru2"], self.prob)
        state.apply_effect(unload.effect)
        unload.uninstantiate()

        self.assert_(fnew in state)

        capacity = StateVariable(self.prob.functions["capacity"][0], [self.prob["tru1"]])
        fold = Fact(capacity, TypedNumber(4))
        fnew = Fact(capacity, TypedNumber(8))
        self.assert_(fold in state)
        
        double = self.dom.get_action("double_capacity")
        double.instantiate(["agent", "tru1"], self.prob)
        
        state.apply_effect(double.effect)
        
        self.assertFalse(fold in state)
        self.assert_(fnew in state)
        double.uninstantiate()

        halve = self.dom.get_action("halve_capacity")
        halve.instantiate(["agent", "tru1"], self.prob)

        state.apply_effect(halve.effect)

        self.assertFalse(fnew in state)
        self.assert_(fold in state)
        halve.uninstantiate()
        
        
    def testConditionReasons(self):
        """Testing finding reason of satisfied conditions"""
        
        state = State.from_problem(self.prob)

        relevantVars = []
        drive = self.dom.get_action("drive")
        drive.instantiate(["agent", "tru1", "apt1"], self.prob)
        self.assert_(state.is_satisfied(drive.precondition, relevantVars))
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
        state = State.from_problem(self.prob)
        extstate = state.get_extended_state()

        kf11 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]], mapl.knowledge, [self.prob["agent"]]), TRUE)
        kf12 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj12"]], mapl.knowledge, [self.prob["agent"]]), TRUE)
        kf13 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj13"]], mapl.knowledge, [self.prob["agent"]]), TRUE)

        id11 = Fact(StateVariable(self.prob.functions["location-of"][0], [self.prob["obj11"]], mapl.indomain, [self.prob["pos1"]]), TRUE)
        
        self.assert_(kf11 in extstate)
        self.assertFalse(kf12 in extstate)
        self.assert_(kf13 in extstate)
        self.assert_(kf13 in state)

        self.assert_(id11 in extstate)
        
        load = self.dom.get_action("a_load")
        load.instantiate(["agent", "obj11", "tru1"], self.prob)
        self.assert_(extstate.is_satisfied(load.replan))
        self.assert_(extstate.is_satisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj12", "tru1"], self.prob)
        self.assertFalse(extstate.is_satisfied(load.replan))
        self.assertFalse(extstate.is_satisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj13", "tru1"], self.prob)
        self.assert_(extstate.is_satisfied(load.replan))
        self.assert_(extstate.is_satisfied(load.precondition))
        load.uninstantiate()

    def testPartialAxiomEvaluation(self):
        """Testing partial axiom evaluation"""
        state = State.from_problem(self.prob)

        load = self.dom.get_action("a_load")
        load.instantiate(["agent", "obj11", "tru1"], self.prob)
        extstate = state.get_extended_state(state.get_relevant_vars(load.replan) | state.get_relevant_vars(load.precondition))
        self.assert_(extstate.is_satisfied(load.replan))
        self.assert_(extstate.is_satisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj12", "tru1"], self.prob)
        extstate = state.get_extended_state(state.get_relevant_vars(load.replan) | state.get_relevant_vars(load.precondition))
        self.assertFalse(extstate.is_satisfied(load.replan))
        self.assertFalse(extstate.is_satisfied(load.precondition))
        load.uninstantiate()

        load.instantiate(["agent", "obj13", "tru1"], self.prob)
        extstate = state.get_extended_state(state.get_relevant_vars(load.replan) | state.get_relevant_vars(load.precondition))
        self.assert_(extstate.is_satisfied(load.replan))
        self.assert_(extstate.is_satisfied(load.precondition))
        load.uninstantiate()
        
    def testAxiomReasoning(self):
        """Testing finding reasons of derived predicates """
        
        state = State.from_problem(self.prob)
        extstate, reasons, universalReasons = state.get_extended_state(getReasons=True)

        relevantVars = []
        relevantReplanVars = []
        load = self.dom.get_action("a_load")
        load.instantiate(["agent", "obj11", "tru1"], self.prob)
        self.assert_(extstate.is_satisfied(load.precondition, relevantVars))
        self.assert_(extstate.is_satisfied(load.replan, relevantReplanVars))

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
        
        a1a = Parser.parse_as(strat1a.split("\n"), axioms.Axiom, self.prob)
        a1b = Parser.parse_as(strat1b.split("\n"), axioms.Axiom, self.prob)
        a2 = Parser.parse_as(strat2.split("\n"), axioms.Axiom, self.prob)
        
        self.dom.axioms = [a1a, a1b, a2]
        self.dom.stratify_axioms()

        self.assert_(a1a.predicate in self.dom.stratification[1])
        self.assert_(a1b.predicate in self.dom.stratification[1])
        self.assert_(a2.predicate in self.dom.stratification[2])
        
        state = State.from_problem(self.prob).get_extended_state()

        oc1 = StateVariable(self.prob.predicates["occupied"][0], [self.prob["pos1"]])
        oc2 = StateVariable(self.prob.predicates["occupied"][0], [self.prob["pos2"]])
        oc3 = StateVariable(self.prob.predicates["occupied"][0], [self.prob["apt1"]])
        oc4 = StateVariable(self.prob.predicates["occupied"][0], [self.prob["apt2"]])

        self.assertEqual(state[oc1], TRUE)
        self.assertEqual(state[oc2], TRUE)
        self.assertEqual(state[oc3], FALSE)
        self.assertEqual(state[oc4], TRUE)

        int1 = StateVariable(self.prob.predicates["interesting"][0], [self.prob["pos1"]])
        int2 = StateVariable(self.prob.predicates["interesting"][0], [self.prob["pos2"]])
        int3 = StateVariable(self.prob.predicates["interesting"][0], [self.prob["apt1"]])
        int4 = StateVariable(self.prob.predicates["interesting"][0], [self.prob["apt2"]])
        
        self.assertEqual(state[int1], TRUE)
        self.assertEqual(state[int2], TRUE)
        self.assertEqual(state[int3], FALSE)
        self.assertEqual(state[int4], TRUE)
        
        free1 = StateVariable(self.prob.predicates["free"][0], [self.prob["pos1"]])
        free2 = StateVariable(self.prob.predicates["free"][0], [self.prob["pos2"]])
        free3 = StateVariable(self.prob.predicates["free"][0], [self.prob["apt1"]])
        free4 = StateVariable(self.prob.predicates["free"][0], [self.prob["apt2"]])

        self.assertEqual(state[free1], FALSE)
        self.assertEqual(state[free2], FALSE)
        self.assertEqual(state[free3], TRUE)
        self.assertEqual(state[free4], FALSE)

        
if __name__ == '__main__':
    unittest.main()    
        
