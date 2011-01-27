#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, domain, actions
from mapltypes import *
from axioms import *
from parser import Parser, ParseError

logistics = \
"""
;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :adl :object-fluents) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing agent - object)

(:predicates (occupied ?l - location)
             (interesting ?l - location)
             (free ?l - location))

(:functions  (city-of ?l - (either location vehicle)) - city
             (location-of ?t - thing) - (either location vehicle))
)"""

testaxiom = """
        (:derived (occupied ?loc - location)
                  (exists (?v - vehicle) (= (location-of ?v) ?loc))
        )
        """

strat1a = testaxiom

strat1b = """
        (:derived (interesting ?loc - location)
                  (or (occupied ?loc)
                      (exists (?p - package) (= (location-of ?p) ?loc)))
        )
        """

strat2 = """
        (:derived (free ?loc - location)
                  (not (occupied ?loc))
        )
        """

strat_invalid = """
        (:derived (occupied ?loc - location)
                  (not (free ?loc))
        )
        """

class AxiomTest(unittest.TestCase):

    def setUp(self):
        p = Parser(logistics.split("\n"))
        self.domain = domain.Domain.parse(p.root)
        

    def testAxiomParsing(self):
        """Testing axiom parsing"""
        axiom = Parser.parse_as(testaxiom.split("\n"), Axiom, self.domain)

    def testStratification(self):
        """Testing axiom stratification"""
        a1a = Parser.parse_as(strat1a.split("\n"), Axiom, self.domain)
        a1b = Parser.parse_as(strat1b.split("\n"), Axiom, self.domain)
        a2 = Parser.parse_as(strat2.split("\n"), Axiom, self.domain)
        aerror = Parser.parse_as(strat_invalid.split("\n"), Axiom, self.domain)
        
        self.domain.axioms += [a1a, a1b, a2]
        self.domain.stratify_axioms()

        self.assert_(a1a.predicate in self.domain.stratification[1])
        self.assert_(a1a.predicate in self.domain.nonrecursive)
        self.assert_(a1b.predicate in self.domain.stratification[1])
        self.assertFalse(a1b.predicate in self.domain.nonrecursive)
        self.assert_(a2.predicate in self.domain.stratification[2])
        self.assert_(a2.predicate in self.domain.nonrecursive)

        self.domain.axioms.append(aerror)
        def test_wrong_axtioms():
            self.domain.stratify_axioms()
            x = self.domain.stratification
        self.assertRaises(Exception, test_wrong_axtioms)
        
        
        
if __name__ == '__main__':
    unittest.main()    
        
