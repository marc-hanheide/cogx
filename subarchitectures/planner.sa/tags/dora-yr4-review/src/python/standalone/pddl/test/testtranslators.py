#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import common

import parser, domain, problem, actions, writer, translators #, sas_translate
import mapl

multi_params = """
        (:action load_both
                 :parameters   (?t1 ?t2 - truck ?p1 ?p2 - package)
                 :precondition (and (= (location-of ?t1) (location-of ?p1))
                                    (= (location-of ?t2) (location-of ?p2)))
                 :effect       (and (assign (location-of ?p1) ?t1)
                                    (assign (location-of ?p2) ?t2)))
"""

univ_fly = """
        (:action safe_fly
                 :parameters    (?a - airplane ?to - airport)
                 :precondition  (forall (?t - truck)
                                        (not (= (location-of ?a) (location-of ?t))))
                 :effect        (assign (location-of ?a) ?to))
        """

ex_fly = """
        (:action fly
                 :parameters    (?a - airplane ?to - airport)
                 :precondition  (exists (?t - truck)
                                        (= (city-of (location-of ?t)) (city-of ?to)))
                 :effect        (assign (location-of ?a) ?to))
        """

univ_load = """
        (:action loadall
                 :parameters    ()
                 :effect        (forall (?v - truck ?p - package)
                                      (when (= (location-of ?p) (location-of ?v))
                                            (assign (location-of ?p) ?v))
                                ))
"""

adl_support = ["strips", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "adl", "derived-predicated"]

class TranslateTests(common.PddlTest):

    def testMAPLFluentNormalisation(self):
        """Testing nomalisation of object fluents"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t = mapl.MAPLObjectFluentNormalizer()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.roundtrip(dom2, prob2)

        
    def testQuantifiedNormalisation(self):
        """Testing nomalisation of object fluents in quantified expressions"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        t = translators.ObjectFluentNormalizer()

        uni_cond = parser.Parser.parse_as(univ_fly.split("\n"), actions.Action, dom)
        ex_cond = parser.Parser.parse_as(ex_fly.split("\n"), actions.Action, dom)
        uni_eff = parser.Parser.parse_as(univ_load.split("\n"), actions.Action, dom)

        uni_cond2 = t.translate(uni_cond, domain=dom)
        ex_cond2 = t.translate(ex_cond, domain=dom)
        uni_eff2 = t.translate(uni_eff, domain=dom)
        
        #TODO: test that the results are correct
        print "univ pre", uni_cond2.precondition.pddl_str()
        print "exists pre", ex_cond2.precondition.pddl_str()
        print "univ eff", uni_eff2.effect.pddl_str()

    # def testQuantifiedModalCompilation(self):
    #     """Testing compilation of modal predicates in quantified expressions"""

    #     dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
    #     t = translators.ModalPredicateCompiler()

    #     uni_cond = parser.Parser.parse_as(univ_fly.split("\n"), actions.Action, dom)
    #     ex_cond = parser.Parser.parse_as(ex_fly.split("\n"), actions.Action, dom)
    #     uni_eff = parser.Parser.parse_as(univ_load.split("\n"), actions.Action, dom)

    #     uni_cond2 = t.translate(uni_cond, domain=dom)
    #     ex_cond2 = t.translate(ex_cond, domain=dom)
    #     uni_eff2 = t.translate(uni_eff, domain=dom)
        
    #     #TODO: test that the results are correct
    #     print "univ pre", uni_cond2.precondition.pddl_str()
    #     print "exists pre", ex_cond2.precondition.pddl_str()
    #     print "univ eff", uni_eff2.precondition.pddl_str()

    def testQuantifiedFluentCompilation(self):
        """Testing compilation of object fluents in quantified expressions"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        t = translators.ObjectFluentCompiler()

        uni_cond = parser.Parser.parse_as(univ_fly.split("\n"), actions.Action, dom)
        ex_cond = parser.Parser.parse_as(ex_fly.split("\n"), actions.Action, dom)
        uni_eff = parser.Parser.parse_as(univ_load.split("\n"), actions.Action, dom)

        dom.actions.append(uni_cond)
        dom.actions.append(ex_cond)
        dom.actions.append(uni_eff)

        dom2 = t.translate(dom)
        
        uni_cond2 = dom2.get_action(uni_cond.name)
        ex_cond2 = dom2.get_action(ex_cond.name)
        uni_eff2 = dom2.get_action(uni_eff.name)
        
        #TODO: test that the results are correct
        print "univ pre", uni_cond2.precondition.pddl_str()
        print "exists pre", ex_cond2.precondition.pddl_str()
        print "univ eff", uni_eff2.effect.pddl_str()
        
    def testMAPLtoPDDL(self):
        """Testing basic mapl to pddl translation"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t = translators.MAPLCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.roundtrip(dom2, prob2)
        
    def testObjectFluentCompilation(self):
        """Testing compilation of object fluents to propositional pddl"""
        
        dom, prob = self.load("testdata/blocksworld.domain.pddl", "testdata/blocksworld.problem.pddl")

        t = translators.ObjectFluentCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)
        
        self.roundtrip(dom2, prob2)

    def testObjectFluentCompilation2(self):
        """Testing compilation of object fluents to propositional pddl (2)"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        mult = parser.Parser.parse_as(multi_params.split("\n"), actions.Action, dom)
        dom.actions.append(mult)

        t = translators.ObjectFluentCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)
        
        self.roundtrip(dom2, prob2)
        
        
    def testModalPredicateCompilation(self):
        """Testing compilation of modal predicates"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t = translators.ModalPredicateCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.roundtrip(dom2, prob2)
        
    def testMAPLtoADL(self):
        """Testing compilation of MAPL to simple ADL"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t = translators.ADLCompiler()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)
        
        self.roundtrip(dom2, prob2)
      
        
    def testRequirementCompilation(self):
        """Testing selective compilation of requirements"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t1 = dom.compile_to(['adl', 'modal-predicates', 'fluents'])
        prob1 = t1.translate(prob)
        self.assert_('mapl' not in prob1.domain.requirements)
        self.assert_('modal-predicates' in prob1.domain.requirements)
        self.assert_('object-fluents' in prob1.domain.requirements)
        self.assert_('numeric-fluents' in prob1.domain.requirements)
        
        t2 = dom.compile_to(['adl', 'fluents'])
        prob2 = t2.translate(prob)
        self.assert_('mapl' not in prob2.domain.requirements)
        self.assert_('modal-predicates' not in prob2.domain.requirements)
        self.assert_('object-fluents' in prob2.domain.requirements)
        self.assert_('numeric-fluents' in prob2.domain.requirements)
        
        t3 = dom.compile_to(['adl', 'numeric-fluents'])
        prob3 = t3.translate(prob)
        self.assert_('mapl' not in prob3.domain.requirements)
        self.assert_('modal-predicates' not in prob3.domain.requirements)
        self.assert_('object-fluents' not in prob3.domain.requirements)
        self.assert_('numeric-fluents' in prob3.domain.requirements)
        
        dom, prob = self.load("testdata/rovers.domain.mapl", "testdata/rovers.problem.mapl")

        t4 = dom.compile_to(['adl', 'durative-actions', 'fluents'])
        prob4 = t4.translate(prob)
        self.assert_('mapl' not in prob4.domain.requirements)
        self.assert_('modal-predicates' not in prob4.domain.requirements)
        self.assert_('durative-actions' in prob4.domain.requirements)
        self.assert_('object-fluents' in prob4.domain.requirements)

        t5 = dom.compile_to(['adl', 'fluents'])
        prob5 = t5.translate(prob)
        self.assert_('mapl' not in prob5.domain.requirements)
        self.assert_('modal-predicates' not in prob5.domain.requirements)
        self.assert_('durative-actions' not in prob5.domain.requirements)
        self.assert_('object-fluents' in prob5.domain.requirements)
        
        
        
    # def testMAPLtoSAS(self):
    #     """Testing compilation of MAPL to simple SAS"""
        
    #     dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

    #     t = sas_translate.SASTranslator()
    #     dom2 = t.translate(dom)
    #     prob2 = t.translate(prob)
        
    #     self.roundtrip(dom2, prob2)
        
        
if __name__ == '__main__':
    unittest.main()    
        
