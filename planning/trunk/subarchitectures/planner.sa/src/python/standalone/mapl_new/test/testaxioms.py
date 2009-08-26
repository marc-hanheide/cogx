#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, domain, actions, state
from mapltypes import *
from axioms import *
from parser import Parser, ParseError

logistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :object-fluents) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)

(:predicates (occupied ?l - location)
             (axiom2 ?v - vehicle ?l - location))

(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle))
)"""

testaxiom = """
        (:derived (occupied ?loc - location)
                  (exists (?v - vehicle) (= (location-of ?v) ?loc))
        )
        """

class AxiomTest(unittest.TestCase):

    def setUp(self):
        p = Parser(logistics.split("\n"))
        self.domain = domain.MAPLDomain.parse(p.root)
        

    def testAxiomParsing(self):
        """Testing axiom parsing"""
        axiom = Parser.parseAs(testaxiom.split("\n"), Axiom, self.domain)
        
if __name__ == '__main__':
    unittest.main()    
        
