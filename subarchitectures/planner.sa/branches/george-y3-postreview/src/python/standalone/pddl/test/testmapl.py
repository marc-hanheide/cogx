#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import common
import os

import parser, domain, problem, state, actions, mapl, durative, dynamic_objects
# from mapltypes import *
# from predicates import *
# from effects import *
from builder import Builder
from parser import Parser, ParseError

create_rule = """
  (:init-rule objects
              :effect (forall (?c - city) (create (?a - airport)
                                                   (assign (city-of ?a) ?c)))
              )
"""

problem_str = """
(define (problem logistics-4-0)

(:domain logistics-object-fluents)

(:objects  apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 - airport
           pos1 pos2 - location
           cit1 cit2 - city)

(:init  (= (location-of apn1) apt1)
        (= (location-of tru1) pos1)
        (= (location-of tru2) pos2)
        (= (location-of obj11) pos1)
        (= (location-of obj12) pos1)
        (= (location-of obj13) pos1)
        (= (location-of obj21) pos2)
        (= (location-of obj22) pos2)
        (= (location-of obj23) pos2)
        (= (city-of apt1) cit1)
        (= (city-of pos1) cit1)
        (= (city-of pos2) cit2))

(:goal  (and (= (location-of obj11) pos2)
             (= (location-of obj13) pos2)
             (= (location-of obj21) pos1)
             (= (location-of obj23) pos1)))

)
"""

class DynamicTest(common.PddlTest):

    def testInitRuleParsing(self):
        """Testing parsing of initial rules"""

        dom = self.load("testdata/logistics.domain.mapl")
        dom.add_requirement("dynamic-objects")
        
        create = Parser.parse_as(create_rule.split("\n"), mapl.InitRule, dom)

    def testInitRuleApplication(self):
        """Testing application of initial rules"""

        dom = self.load("testdata/logistics.domain.mapl")
        dom.add_requirement("dynamic-objects")
        
        create = Parser.parse_as(create_rule.split("\n"), mapl.InitRule, dom)

        prob = Parser.parse_as(problem_str.split("\n"), problem.Problem, dom)
        b = Builder(prob)
        
        st = state.State.from_problem(prob)
        st.apply_init_rules(rules=[create])

        airports = list(prob.get_all_objects(prob.types["airport"]))
        cities = list(prob.get_all_objects(prob.types["city"]))
        self.assertEqual(len(airports), 3)
        
        svar1 = b.svar("city-of", "airport0")
        svar2 = b.svar("city-of", "airport1")

        self.assert_(st[svar1] in cities)
        self.assert_(st[svar2] in cities)
        
        
    # def testTranslation(self):
    #     """Testing translation of create and destroy effects"""

    #     dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
    #     dom.add_requirement("dynamic-objects")
        
    #     a_create = Parser.parse_as(create.split("\n"), mapl.MAPLAction, dom)
    #     a_pdestroy = Parser.parse_as(destroy_package.split("\n"), mapl.MAPLAction, dom)
    #     a_locdestroy = Parser.parse_as(destroy_loc.split("\n"), mapl.MAPLAction, dom)

    #     dom.actions += [a_create, a_pdestroy, a_locdestroy]
        
    #     t = dynamic_objects.DynamicObjectsCompiler()
    #     dom2 = t.translate(dom)
    #     prob2 = t.translate(prob)

    #     self.roundtrip(dom2, prob2, print_result=False)

    # def testCreateEffect(self):
    #     """Testing application of create effects"""
    #     dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
    #     prob.add_requirement("dynamic-objects")
        
    #     a_create = Parser.parse_as(create.split("\n"), mapl.MAPLAction, prob)
        
    #     self.assert_("package0" not in prob)
        
    #     st = state.State.from_problem(prob)
    #     oldlen = len(prob)
    #     a_create.instantiate([prob["agent"], prob["tru1"]])
    #     st.apply_effect(a_create.effect)

    #     b = Builder(prob)
        
    #     self.assertEqual(len(prob), oldlen+1)
    #     self.assert_("package0" in prob)
    #     svar = b.svar("location-of", "package0")
    #     self.assert_(st[svar] == prob["tru1"])

    #     st.apply_effect(a_create.effect)
    #     st.apply_effect(a_create.effect)

    #     self.assertEqual(len(prob), oldlen+3)
    #     self.assert_("package1" in prob)
    #     self.assert_("package2" in prob)
    #     svar1 = b.svar("location-of", "package2")
    #     self.assert_(st[svar1] == prob["tru1"])

            
if __name__ == '__main__':
    unittest.main()    
