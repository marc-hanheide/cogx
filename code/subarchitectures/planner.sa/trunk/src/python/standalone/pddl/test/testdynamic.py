#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import common
import os

import parser, domain, state, actions, mapl, durative, dynamic_objects
from mapltypes import *
from predicates import *
from effects import *
from actions import Action
from builder import Builder
from parser import Parser, ParseError

create = """
        (:action create
                 :agent         (?a - agent)
                 :parameters    (?t - truck)
                 :effect        (create (?p - package) (assign (location-of ?p) ?t)))
        """

destroy_package = """
        (:action destroy_package
                 :agent         (?a - agent)
                 :parameters    (?p - package ?t - truck)
                 :precondition  (= (location-of ?p) (location-of ?t))
                 :effect        (destroy ?p))
        """

destroy_loc = """
        (:action destroy_loc
                 :agent         (?a - agent)
                 :parameters    (?t - truck ?to - location)
                 :precondition  (and (= (city-of (location-of ?t)) (city-of ?to))
                                     (not (= (location-of ?t) ?to)))
                 :effect        (and (destroy (location-of ?t))
                                     (assign (location-of ?t) ?to)))
        """

class DynamicTest(common.PddlTest):

    def testParsing(self):
        """Testing parsing of create and destroy effects"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        dom.add_requirement("dynamic-objects")
        
        a_create = Parser.parse_as(create.split("\n"), mapl.MAPLAction, dom)
        a_pdestroy = Parser.parse_as(destroy_package.split("\n"), mapl.MAPLAction, dom)
        a_locdestroy = Parser.parse_as(destroy_loc.split("\n"), mapl.MAPLAction, dom)
        dom.actions += [a_create, a_pdestroy, a_locdestroy]

        self.roundtrip(dom, prob, print_result=False)
        
    def testTranslation(self):
        """Testing translation of create and destroy effects"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        dom.add_requirement("dynamic-objects")
        
        a_create = Parser.parse_as(create.split("\n"), mapl.MAPLAction, dom)
        a_pdestroy = Parser.parse_as(destroy_package.split("\n"), mapl.MAPLAction, dom)
        a_locdestroy = Parser.parse_as(destroy_loc.split("\n"), mapl.MAPLAction, dom)

        dom.actions += [a_create, a_pdestroy, a_locdestroy]
        
        t = dynamic_objects.DynamicObjectsCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.roundtrip(dom2, prob2, print_result=False)

    def testCreateEffect(self):
        """Testing application of create effects"""
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        dom.add_requirement("dynamic-objects")
        
        a_create = Parser.parse_as(create.split("\n"), mapl.MAPLAction, dom)
        
        self.assert_("package0" not in prob)
        
        st = state.State.from_problem(prob)
        oldlen = len(prob)
        with a_create.instantiate([prob["agent"], prob["tru1"]], prob):
            st.apply_effect(a_create.effect)

            b = Builder(prob)

            self.assertEqual(len(prob), oldlen+1)
            self.assert_("package0" in prob)
            svar = b.svar("location-of", "package0")
            self.assert_(st[svar] == prob["tru1"])

            st.apply_effect(a_create.effect)
            st.apply_effect(a_create.effect)

            self.assertEqual(len(prob), oldlen+3)
            self.assert_("package1" in prob)
            self.assert_("package2" in prob)
            svar1 = b.svar("location-of", "package2")
            self.assert_(st[svar1] == prob["tru1"])
        
    # def testModalAction(self):
    #     """Testing modal action parsing"""
        
    #     action = Parser.parse_as(modal_action.split("\n"), mapl.MAPLAction, self.domain)

    #     self.assertEqual(action.params[1].type, FunctionType(t_object))
    #     term = predicates.FunctionTerm(self.domain.functions["location-of"][0], [Parameter("?c", self.domain.types["city"])])
    #     action.instantiate({"?var" : term})
        
    # def testEffects(self):
    #     """Testing basic effect parsing"""
        
    #     action = Parser.parse_as(drive.split("\n"), Action, self.domain)
    #     self.assert_(isinstance(action.effect, SimpleEffect))


    # def testMAPLAction(self):
    #     """Testing basic effect parsing"""
        
    #     action = Parser.parse_as(mapl_drive.split("\n"), mapl.MAPLAction, self.domain)
    #     self.assertEqual(len(action.agents), 1)
    #     self.assertEqual(len(action.params), 2)
    #     self.assertEqual(len(action.vars), 0)
    #     self.assertEqual(len(action.args), 3)
    #     self.assert_(isinstance(action.effect, SimpleEffect))
        
    # def testConditionalEffects(self):
    #     """Testing conditional effect parsing"""
        
    #     action = Parser.parse_as(cond_load.split("\n"), Action, self.domain)

    #     self.assert_(isinstance(action.effect, ConditionalEffect))
    #     self.assert_(isinstance(action.effect.condition, conditions.LiteralCondition))
    #     self.assert_(isinstance(action.effect.effect, SimpleEffect))

    # def testUniversalEffects(self):
    #     """Testing conditional effect parsing"""
        
    #     action = Parser.parse_as(univ_unload.split("\n"), Action, self.domain)

    #     self.assert_(isinstance(action.effect, UniversalEffect))
    #     self.assertEqual(len(action.effect.args), 1)
    #     self.assert_(isinstance(action.effect.effect, ConditionalEffect))

    # def testProbabilisticEffects(self):
    #     """Testing probabilistic effect parsing"""

    #     action = Parser.parse_as(prob_load.split("\n"), Action, self.domain)

    #     self.assert_(isinstance(action.effect, ConjunctiveEffect))
    #     self.assert_(isinstance(action.effect.parts[0], ProbabilisticEffect))
    #     self.assert_(isinstance(action.effect.parts[1], ProbabilisticEffect))
    #     p1, e1 = action.effect.parts[0].effects[0]
    #     p2, e2 = action.effect.parts[0].effects[1]

    #     ap1, ae1 = action.effect.parts[1].effects[0]
    #     ap2, ae2 = action.effect.parts[1].effects[1]

    #     self.assert_(isinstance(p1, FunctionTerm))
    #     self.assertEqual(p1.function, self.domain.functions["load_succ_prob"][0])
    #     self.assert_(isinstance(e1.args[0], FunctionTerm))
    #     self.assert_(isinstance(e1.args[1], VariableTerm))
    #     self.assertEqual(p2, 0.5)
    #     self.assert_(isinstance(e2.args[0], FunctionTerm))
    #     self.assert_(isinstance(e2.args[1], FunctionTerm))

    #     self.assertEqual(ae1, e1)
    #     self.assertEqual(ae2, e2)
    #     self.assertEqual(ap1, 0.5)
    #     self.assertEqual(ap2, None)

    #     # self.assertEqual(action.effect.parts[0].getRandomEffect(0), e2)
    #     # self.assertEqual(action.effect.parts[0].getRandomEffect(1), e1)
    #     # self.assertEqual(action.effect.parts[0].getRandomEffect(2), None)

    #     # import random
    #     # random.seed(42)
    #     # for r in xrange(30):
    #     #     self.assert_(action.effect.parts[0].getRandomEffect() in (e1,e2,None))
        
    # def testAssertion(self):
    #     """Testing parsing of assertions"""
        
    #     action = Parser.parse_as(a_load_mapl.split("\n"), mapl.MAPLAction, self.domain)
    #     self.assert_(action.replan is not None)

    #     action = Parser.parse_as(a_load.split("\n"), Action, self.domain)
    #     self.assert_(action.replan is not None)
        
    # def testMaplErrorHandling(self):
    #     """Testing error handling of MaplAction"""
    #     try:
    #         action = Parser.parse_as(error_load1.split("\n"), mapl.MAPLAction, self.domain)
    #         self.fail("Action with duplicate precondition didn't raise exception")
    #     except ParseError, e:
    #         self.assertEqual(e.token.string, ":precondition")
    #         self.assertEqual(e.token.line, 6)

    #     try:
    #         action = Parser.parse_as(error_load2.split("\n"), mapl.MAPLAction, self.domain)
    #         self.fail("Action with duplicate replan condition didn't raise exception")
    #     except ParseError, e:
    #         self.assertEqual(e.token.string, ":replan")
    #         self.assertEqual(e.token.line, 7)

    #     try:
    #         action = Parser.parse_as(error_load3.split("\n"), mapl.MAPLAction, self.domain)
    #         self.fail("Action with duplicate effect statement didn't raise exception")
    #     except ParseError, e:
    #         self.assertEqual(e.token.string, ":effect")
    #         self.assertEqual(e.token.line, 8)

    #     try:
    #         action = Parser.parse_as(error_load4.split("\n"), mapl.MAPLAction, self.domain)
    #         self.fail("Action with duplicate parameters didn't raise exception")
    #     except ParseError, e:
    #         self.assertEqual(e.token.string, "?p")
    #         self.assertEqual(e.token.line, 4)

    # def testActionCosts(self):
    #     """Testing setting/getting/deleting of action costs"""
    #     from builder import Builder

    #     action = Parser.parse_as(cost_load.split("\n"), Action, self.domain)
    #     b = Builder(action)
        
    #     expected_term = b("load-costs", "?v")
    #     self.assertEqual(action.get_total_cost(), expected_term)

    #     action.set_total_cost(25)
    #     self.assertEqual(action.get_total_cost(), b(25))

    #     action.set_total_cost(None)
    #     self.assertEqual(len(action.effect.parts), 1)

    #     action.set_total_cost(b("+", ("load-costs", "?v"), 5))
    #     self.assertEqual(len(action.effect.parts), 2)
        
        
if __name__ == '__main__':
    unittest.main()    
        
