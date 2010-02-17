#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest
import common

import parser, domain, problem, writer, translators #, sas_translate
import mapl

class TranslateTests(common.PddlTest):

    def testMAPLFluentNormalisation(self):
        """Testing nomalisation of object fluents"""

        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

        t = mapl.MAPLObjectFluentNormalizer()
        dom2 = t.translate(dom)
        prob2 = t.translate(prob)

        self.roundtrip(dom2, prob2)
        
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

    # def testMAPLtoSAS(self):
    #     """Testing compilation of MAPL to simple SAS"""
        
    #     dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")

    #     t = sas_translate.SASTranslator()
    #     dom2 = t.translate(dom)
    #     prob2 = t.translate(prob)
        
    #     self.roundtrip(dom2, prob2)
        
        
if __name__ == '__main__':
    unittest.main()    
        
