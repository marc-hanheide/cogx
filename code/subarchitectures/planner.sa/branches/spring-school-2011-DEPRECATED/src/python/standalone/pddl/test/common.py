#! /usr/bin/env python
# -*- coding: latin-1 -*-
#from __future__ import absolute_import

import os, unittest
from os.path import abspath, dirname
import parser, writer, domain, problem

test_path = abspath(dirname(__file__)) 

import sys, logging
if "-v" in sys.argv:
    logging.basicConfig(level=logging.DEBUG)

class PddlTest(unittest.TestCase):
    
    def load(self, domfile, probfile=None):
        p = parser.Parser.parse_file(os.path.join(test_path, domfile))
        dom = domain.Domain.parse(p.root)
        if not probfile:
            return dom
        
        p = parser.Parser.parse_file(os.path.join(test_path, probfile))
        prob = problem.Problem.parse(p.root, dom)

        return dom, prob

    def roundtrip(self, dom, prob=None, print_result=False):
        import mapl, dtpddl
        if "mapl" in dom.requirements:
            w = mapl.MAPLWriter()
        elif "partial-observability" in dom.requirements:
            w = dtpddl.DTPDDLWriter()
        else:
            w = writer.Writer()

        s = w.write_domain(dom)
        if print_result:
            print "\n".join(s)
        p = parser.Parser(s)
        dom2 = domain.Domain.parse(p.root)

        if prob is None:
            return dom2
        
        s = w.write_problem(prob)
        if print_result:
            print "\n\n------------------------------------------------------\n"
            print "\n".join(s)
        p = parser.Parser(s)
        prob2 = problem.Problem.parse(p.root, dom2)
        return dom2, prob2
    
