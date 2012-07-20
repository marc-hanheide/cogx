#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import common

import parser, domain, effects, conditions, translators
from parser import Parser, ParseError
from dtpddl import *

obs1 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (load ?a ?p ?v)
 :precondition (kval ?a (location-of ?p))
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

obs2 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (or (not (load ?a ?p ?v))
                (unload ?a ?p ?v))
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

obs_error1 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (load ?a ?v ?p)
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

obs_error2 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (load ?a ?p)
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

obs_error3 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (load ?a ?p ?v ?v)
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

obs_error4 = """
(:observe package
 :agent (?a - agent)
 :parameters (?p - package ?v - vehicle)
 :execution (notexists ?a ?p)
 :effect (when (= (location-of ?p) (location-of ?v)) (observed (location-of ?p) (location-of ?v)))
)
"""

adl_support = ["strips", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "adl", "derived-predicated"]

class DTTest(common.PddlTest):

    def setUp(self):
        self.dom, self.prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        self.dom.predicates.add(default_predicates)

    def testObserveParsing(self):
        """Testing parsing of observation models"""
        
        ob1 = Parser.parse_as(obs1.split("\n"), Observation, self.dom)
        ob2 = Parser.parse_as(obs2.split("\n"), Observation, self.dom)

        self.assert_(len(ob1.execution), 1)
        self.assert_(ob1.execution[0].action.name, "load")
        self.assertFalse(ob1.execution[0].negated)
        self.assert_(isinstance(ob1.effect, effects.ConditionalEffect))
        self.assert_(isinstance(ob1.precondition, conditions.LiteralCondition))

        self.assert_(len(ob2.execution), 2)
        self.assert_(ob2.execution[0].action.name, "load")
        self.assert_(ob2.execution[0].negated)
        self.assert_(ob2.execution[1].action.name, "unload")
        self.assertFalse(ob2.execution[1].negated)

    def testDTDomain(self):
        """Testing parsing of domain with observation models"""
        self.dom = self.load("testdata/logistics.dtpddl")

        self.assert_(len(self.dom.observe), 2)

    def testMAPLTranslation(self):
        """Testing translation of  observation models to mapl actions"""
        self.dom = self.load("testdata/logistics.dtpddl")
        
        prob_functions = set(self.dom.functions['location-of'])
        dom2 = DT2MAPLCompiler().translate(self.dom, prob_functions=prob_functions)
        #self.roundtrip(dom2, print_result=True)

        s1 = dom2.get_action("sense_package")
        self.assertEqual(len(s1.sensors), 1)

        s2 = dom2.get_action("sense_position")
        self.assertEqual(len(s2.sensors), 1)

    def testADLTranslation(self):
        """Testing translation of mapl/dtpddl to adl/dtpddl"""
        self.dom = self.load("testdata/logistics.dtpddl")
        dom2 = translators.ModalPredicateCompiler().translate(self.dom)
        self.roundtrip(dom2, print_result=False)

        dom2 = translators.ObjectFluentCompiler().translate(self.dom)
        self.roundtrip(dom2, print_result=False)

        dom2 = translators.ADLCompiler().translate(self.dom)
        self.roundtrip(dom2, print_result=False)
        
        #s1 = dom2.get_action("sense_package")
        #self.assertEqual(len(s1.sensors), 1)

        #s2 = dom2.get_action("sense_position")
        #self.assertEqual(len(s2.sensors), 1)

        
    def testDtErrorHandling(self):
        """Testing error handling of observation models"""
        try:
            obs = Parser.parse_as(obs_error1.split("\n"), Observation, self.dom)
            self.fail("Observation model with wrong execution parameter didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "?v")
            self.assertEqual(e.token.line, 5)

        try:
            obs = Parser.parse_as(obs_error2.split("\n"), Observation, self.dom)
            self.fail("Observation model with missing execution parameter didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, ")")
            self.assertEqual(e.token.line, 5)

        try:
            obs = Parser.parse_as(obs_error3.split("\n"), Observation, self.dom)
            self.fail("Observation model with too many execution parameters didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "?v")
            self.assertEqual(e.token.line, 5)

        try:
            obs = Parser.parse_as(obs_error4.split("\n"), Observation, self.dom)
            self.fail("Observation model referring to a nonexisting action didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "notexists")
            self.assertEqual(e.token.line, 5)
            

    def testMDTTPDDLtoMAPL(self):
        """Testing compilation of DTPDDL/MAPL to MAPL"""
        import dtpddl
        
        dom, prob = self.load("testdata/switchtest-tfd.pddl", "testdata/switchtest-tfd-problem.pddl")

        t = dtpddl.DT2MAPLCompilerFD()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.assertEqual(len(dom2.observe), 0)
        
        self.roundtrip(dom2, prob2)
        
    def testMDTTPDDLtoDTPDDL(self):
        """Testing compilation of DTPDDL/MAPL to DTPDDL"""
        import dtpddl

        # import pdb
        dom, prob = self.load("testdata/switchtest-tfd.pddl", "testdata/switchtest-tfd-problem.pddl")
        # try:
        #     dom, prob = pdb.runcall(self.load, "testdata/switchtest-tfd.pddl", "testdata/switchtest-tfd-problem.pddl")
        # except:
        #     pdb.post_mortem()

        supported = adl_support + ['action-costs', 'partial-observability', 'fluents', 'mapl']
        t1 = dom.compile_to(supported)
        t2 = dtpddl.DTPDDLCompiler()
        t = translators.ChainingTranslator(t1, t2)

        prob2 = t.translate(prob)
        dom2 = prob2.domain
        self.assertEqual(len(dom2.observe), 1)
        
        self.roundtrip(dom2, prob2, print_result=False)

    def testMDTTPDDLtoSimpleDTPDDL(self):
        """Testing compilation of DTPDDL/MAPL to DTPDDL/ADL"""
        import dtpddl
        
        dom, prob = self.load("testdata/switchtest-tfd.pddl", "testdata/switchtest-tfd-problem.pddl")

        supported = adl_support + ['action-costs', 'partial-observability', 'fluents', 'mapl']
        t1 = dom.compile_to(supported)
        t2 = dtpddl.DTPDDLCompiler()
        t3 = dtpddl.ProbADLCompiler()
        t = translators.ChainingTranslator(t1, t2, t3)

        prob2 = t.translate(prob)
        dom2 = prob2.domain

        self.assertEqual(len(dom2.observe), 1)
        
        self.roundtrip(dom2, prob2)
            
if __name__ == '__main__':
    unittest.main()    
        
