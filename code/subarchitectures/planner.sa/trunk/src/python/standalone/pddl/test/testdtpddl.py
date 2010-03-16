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
 :execution (and (not (load ?a ?p ?v))
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

class DTTest(common.PddlTest):

    def setUp(self):
        self.dom, self.prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        self.dom.predicates.add(modal_predicates)

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
        """Testing parsing of domain with observation models"""
        self.dom = self.load("testdata/logistics.dtpddl")
        dom2 = DT2MAPLCompiler().translate_domain(self.dom)
        self.roundtrip(dom2, print_result=True)

        
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
            
        
if __name__ == '__main__':
    unittest.main()    
        
