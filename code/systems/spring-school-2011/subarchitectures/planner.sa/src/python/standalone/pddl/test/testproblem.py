#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import unittest, common

import predicates, parser, domain, problem, writer
import mapl

from parser import Parser

class ProblemTest(common.PddlTest):
    
    def testLogistics(self):
        """Testing logistics problem"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        self.assertEqual(len(prob.init), 13)

    def testBlocksworld(self):
        """Testing blocksworld problem"""
        
        dom, prob = self.load("testdata/blocksworld.domain.pddl", "testdata/blocksworld.problem.pddl")
        self.assertEqual(len(prob.init), 9)

    def testRovers(self):
        """Testing Rovers problem"""
        
        dom, prob = self.load("testdata/rovers.domain.mapl", "testdata/rovers.problem.mapl")
        self.assertEqual(len(prob.init), 48)

        
    def testWriterDomain(self):
        """Testing MAPLWriter domain roundtrip"""
        
        dom = self.load("testdata/logistics.domain.mapl")
        dom2 = self.roundtrip(dom)

        self.assertEqual(dom.name, dom2.name)
        self.assertEqual(len(dom.constants), len(dom2.constants))
        self.assertEqual(len(dom.actions), len(dom2.actions))
        for a1 in dom.actions:
            for a2 in dom2.actions:
                if a1.name != a2.name:
                    continue
                self.assertEqual(a1.agents, a2.agents)
                self.assertEqual(a1.args, a2.args)
                self.assertEqual(a1.vars, a2.vars)
                self.assertEqual(a1.precondition, a2.precondition)
                self.assertEqual(a1.replan, a2.replan)
                self.assertEqual(a1.effect, a2.effect)
                self.assertEqual(a1.sensors, a2.sensors)

                
    def testWriterProblem(self):
        """Testing MAPLWriter problem roundtrip"""
        
        dom, prob = self.load("testdata/logistics.domain.mapl", "testdata/logistics.p1.mapl")
        dom2, prob2 = self.roundtrip(dom, prob)

        self.assertEqual(prob.name, prob2.name)
        self.assertEqual(len(prob.objects), len(prob2.objects))
        self.assertEqual(len(prob.init), len(prob2.init))
        self.assertEqual(set(prob.init), set(prob2.init))
        self.assertEqual(prob.goal, prob2.goal)


    def testWriterDomainNumeric(self):
        """Testing MAPLWriter domain roundtrip with numeric fluents and durative actions"""
        
        dom = self.load("testdata/rovers.domain.mapl")
        dom2 = self.roundtrip(dom)

        self.assertEqual(dom.name, dom2.name)
        self.assertEqual(len(dom.constants), len(dom2.constants))
        self.assertEqual(len(dom.actions), len(dom2.actions))
        for a1 in dom.actions:
            for a2 in dom2.actions:
                if a1.name != a2.name:
                    continue
                self.assertEqual(a1.agents, a2.agents)
                self.assertEqual(a1.args, a2.args)
                self.assertEqual(a1.duration, a2.duration)
                self.assertEqual(a1.vars, a2.vars)
                self.assertEqual(a1.precondition, a2.precondition)
                self.assertEqual(a1.replan, a2.replan)
                self.assertEqual(a1.effect, a2.effect)

            
    def testWriterProblemNumeric(self):
        """Testing MAPLWriter problem roundtrip with numeric fluents and durative actions"""
        
        dom, prob = self.load("testdata/rovers.domain.mapl", "testdata/rovers.problem.mapl")
        dom2, prob2 = self.roundtrip(dom, prob)

        self.assertEqual(prob.name, prob2.name)
        self.assertEqual(len(prob.objects), len(prob2.objects))
        self.assertEqual(len(prob.init), len(prob2.init))
        self.assertEqual(set(prob.init), set(prob2.init))
        self.assertEqual(prob.goal, prob2.goal)
        self.assertEqual(prob.optimization, prob2.optimization)
        self.assertEqual(prob.opt_func, prob2.opt_func)
        
        
if __name__ == '__main__':
    unittest.main()    
        
