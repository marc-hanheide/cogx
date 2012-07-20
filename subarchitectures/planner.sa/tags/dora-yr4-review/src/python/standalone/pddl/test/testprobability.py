#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import common

import predicates, domain, problem, conditions, actions, mapl
from parser import Parser, ParseError
from prob_state import *
        
prob_load = """
        (:action load
                 :parameters    (?p - package ?v - vehicle)
                 :precondition  (= (location-of ?p) (location-of ?v))
                 :effect        (probabilistic (p_load_success ?v) (assign (location-of ?p) ?v))
)
"""

class ProbStateTest(common.PddlTest):
    def testProbState(self):
        """Testing probabilistic state representation"""
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.pprob.mapl")
        state = ProbabilisticState.from_problem(prob)

        for v in state.itervalues():
            self.assert_(isinstance(v, ValueDistribution))
        
        svar_loc11 = StateVariable(prob.functions["location-of"][0], [prob["obj11"]])
        svar_loc21 = StateVariable(prob.functions["location-of"][0], [prob["obj21"]])
        svar_loc22 = StateVariable(prob.functions["location-of"][0], [prob["obj22"]])

        self.assertEqual(state[svar_loc21], prob["pos2"])
        self.assertEqual(state.dist(svar_loc21)[prob["pos2"]], 1.0)
        
        self.assertEqual(state[svar_loc11][prob["pos1"]], 0.3)
        self.assertEqual(state[svar_loc11][prob["pos2"]], 0.4)
        self.assertEqual(state[svar_loc11][prob["apt2"]], 0.3)

        self.assert_(Fact(svar_loc11, prob["pos1"]) in state)
        self.assert_(Fact(svar_loc11, prob["pos2"]) in state)
        self.assert_(Fact(svar_loc11, prob["apt2"]) in state)
        self.assertFalse(Fact(svar_loc11, prob["apt1"]) in state)
        self.assert_(Fact(svar_loc21, prob["pos2"]) in state)
        self.assertFalse(Fact(svar_loc21, prob["pos1"]) in state)
        self.assertFalse(Fact(svar_loc21, prob["apt2"]) in state)

        self.assertEqual(state[svar_loc22][prob["pos1"]], 0.25)
        self.assertEqual(state[svar_loc22][prob["pos2"]], 0.25)
        self.assertEqual(state[svar_loc22][prob["apt1"]], 0.25)
        self.assertEqual(state[svar_loc22][prob["apt2"]], 0.25)
        
        svar_pload_apn1 = StateVariable(prob.functions["p_load_success"][0], [prob["apn1"]])
        self.assertEqual(state[svar_pload_apn1].value, 0.8)
        
        
    def testStateDeterminisation(self):
        """Testing state determinisation"""
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.pprob.mapl")
        state = ProbabilisticState.from_problem(prob)
        dstate = state.determinized_state(0.1, 0.9)

        svar_loc11 = StateVariable(prob.functions["location-of"][0], [prob["obj11"]])
        svar_loc12 = StateVariable(prob.functions["location-of"][0], [prob["obj12"]])
        svar_loc13 = StateVariable(prob.functions["location-of"][0], [prob["obj13"]])
        svar_loc21 = StateVariable(prob.functions["location-of"][0], [prob["obj21"]])

        self.assert_(svar_loc21 in dstate)
        self.assertEqual(dstate[svar_loc21], prob["pos2"])

        self.assertFalse(svar_loc11 in dstate)
        self.assert_(svar_loc11.as_modality(mapl.i_indomain, [prob["pos1"]]) in dstate)
        self.assert_(svar_loc11.as_modality(mapl.i_indomain, [prob["pos2"]]) in dstate)
        self.assert_(svar_loc11.as_modality(mapl.i_indomain, [prob["apt2"]]) in dstate)

        self.assert_(svar_loc12 in dstate)
        self.assertEqual(dstate[svar_loc12], prob["pos1"])

        self.assertFalse(svar_loc13 in dstate)
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["apn1"]]) in dstate)
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["tru1"]]) in dstate)
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["tru2"]]) in dstate)
        
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["apt1"]]) in dstate)
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["pos1"]]) in dstate)
        self.assert_(svar_loc13.as_modality(mapl.i_indomain, [prob["pos2"]]) in dstate)
        self.assertFalse(svar_loc13.as_modality(mapl.i_indomain, [prob["apt2"]]) in dstate)

    def testProbabilisticEffect(self):
        """Testing probabilistic updates"""
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.pprob.mapl")
        state = ProbabilisticState.from_problem(prob)
        
        action = Parser.parse_as(prob_load.split("\n"), actions.Action, prob)

        s2 = state.copy()
        with action.instantiate(["obj11", "apn1"]):
            s2.apply_effect(action.effect)

            svar_loc11 = StateVariable(prob.functions["location-of"][0], [prob["obj11"]])

            self.assertAlmostEqual(s2[svar_loc11][prob["apn1"]], 0.8)
            self.assertAlmostEqual(s2[svar_loc11][prob["pos1"]], 0.3*0.2)
            self.assertAlmostEqual(s2[svar_loc11][prob["pos2"]], 0.4*0.2)
            self.assertAlmostEqual(s2[svar_loc11][prob["apt2"]], 0.3*0.2)

            s2.apply_effect(action.effect)

            self.assertAlmostEqual(s2[svar_loc11][prob["apn1"]], 0.8+(0.2*0.8))
            self.assertAlmostEqual(s2[svar_loc11][prob["pos1"]], 0.3*0.2**2)
            self.assertAlmostEqual(s2[svar_loc11][prob["pos2"]], 0.4*0.2**2)
            self.assertAlmostEqual(s2[svar_loc11][prob["apt2"]], 0.3*0.2**2)
        
        s2 = state.copy()
        with action.instantiate(["obj11", "tru1"]):
            s2.apply_effect(action.effect)
        
        self.assertAlmostEqual(s2[svar_loc11][prob["tru1"]], 0.7)
        self.assertAlmostEqual(s2[svar_loc11][prob["pos1"]], 0.3*0.3)
        self.assertAlmostEqual(s2[svar_loc11][prob["pos2"]], 0.4*0.3)
        self.assertAlmostEqual(s2[svar_loc11][prob["apt2"]], 0.3*0.3)

    def testProbabilisticEffectDet(self):
        """Testing probabilistic effects on deterministic states"""
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.pprob.mapl")
        state = ProbabilisticState.from_problem(prob)
        dstate = state.determinized_state(0.1, 0.9)
        dstate.set_random_seed(1)
        
        action = Parser.parse_as(prob_load.split("\n"), actions.Action, prob)

        with action.instantiate(["obj11", "apn1"]):
            svar_loc11 = StateVariable(prob.functions["location-of"][0], [prob["obj11"]])

            eff = dstate.get_effect_facts(action.effect)
            self.assertEqual(eff[svar_loc11], prob["apn1"])
            eff = dstate.get_effect_facts(action.effect)
            self.assertEqual(eff, {})
            eff = dstate.get_effect_facts(action.effect)
            self.assertEqual(eff[svar_loc11], prob["apn1"])

            dstate = state.determinized_state(0.1, 0.9)
            dstate.set_random_seed(1)
            dstate.apply_effect(action.effect)
            self.assertEqual(dstate[svar_loc11], prob["apn1"])

            dstate = state.determinized_state(0.1, 0.9)
            dstate.set_random_seed(2)
            dstate.apply_effect(action.effect)
            self.assertEqual(dstate[svar_loc11], UNKNOWN)
        
        
if __name__ == '__main__':
    unittest.main()    
        
