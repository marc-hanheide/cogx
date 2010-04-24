#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, builtin, mapl, scope
from mapltypes import *
from predicates import *
from conditions import *
from parser import Parser, ParseError

from builtin import TRUE, FALSE, t_object, t_boolean
from mapl import t_agent

test = \
"""
(= (func1 ?p1 ?p2 ?p3) ?v)
"""

test2 = \
"""(
(pred1 c1 ?p2 ?p3)
(= (func1 ?p1 ?p2 ?p3) true)
(= (func1 ?p1 ?p2 c1) false)
(= (func1 ?p1 ?p2 c1) c1)
)
"""

test3 = \
"""(
(pred1 ?p1 ?p2 ?p3)
(pred1 ?p3 ?p2 false)
(pred1 ?p3 ?p2 ?p1)
)
"""

negtest = \
"""(
(not (pred1 ?p1 ?p2 ?p3))
(not (pred1 ?p1 ?p2 ?p3))
(not (pred1 ?p1 ?p2 ?p3))
)
"""

junctiontest = \
"""(and
(not (= (func1 ?p1 ?p2 ?p3)  true))
(or (pred1 ?p1 ?p2 ?p3)
(not (= (func1 ?p1 ?p2 ?p3) false)))
)
"""

implicationtest = \
"""(imply (= (func1 ?p1 ?p2 ?p3)  true) (pred1 ?p1 ?p2 ?p3) )
"""

quanttest = \
"""(and
(forall (?a - type1 ?b - type2) (= (func2 ?a ?b) ?p1))
(exists (?x - boolean) (= (func1 ?p1 ?p2 ?p3)  ?x))
)
"""

mixedtest = \
"""(and
(forall (?a - type1 ?b - type2) (= (func2 ?a ?b) ?p1))
(or (exists (?x - object) (pred1 ?p1 ?p2 ?x))
(not (= (func1 ?p1 ?p2 ?p3) false)))
)
"""

class ConditionsTest(unittest.TestCase):
    def setUp(self):
        self.type1 = Type("type1")
        self.type2 = Type("type2", [self.type1])
        self.c1 = TypedObject("c1", self.type1)
        self.c2 = TypedObject("c2", self.type2)
        args = [Parameter("?a", self.type1),
                Parameter("?b", self.type2),
                Parameter("?c", t_object)]
        val = Parameter("?v", t_boolean)
        self.func1 =  predicates.Function("func1", args, t_boolean)
        self.func2 =  predicates.Function("func2", args[:-1], self.type1)

        self.pred1 =  predicates.Predicate("pred1", args)
        self.pred2 =  predicates.Predicate("pred2", args[:-1])
        
        self.scope = scope.Scope([TRUE, FALSE, self.c1, self.c2], None)
        self.scope.types["type1"] = self.type1
        self.scope.types["type2"] = self.type2
        self.scope.types["boolean"] = t_boolean
        self.scope.types["object"] = t_object
        self.scope.types["agent"] = t_agent
        self.scope.predicates.add(builtin.equals)
        self.scope.predicates.add(self.pred1)
        self.scope.predicates.add(self.pred2)
        self.scope.predicates.add(mapl.knowledge)
        self.scope.predicates.add(mapl.indomain)
        self.scope.functions.add(self.func1)
        self.scope.functions.add(self.func2)

    def getLocalScope(self):
        params = [Parameter("?p1", self.type1),
                  Parameter("?p2", self.type2),
                  Parameter("?p3", t_object),
                  Parameter("?v", t_boolean)]

        return scope.Scope(params, self.scope)
        

    def testSimpleParsing(self):
        """Testing basic literal parsing"""
        
        p = Parser(test.split("\n"))
        localScope = self.getLocalScope()

        cond = Condition.parse(iter(p.root), localScope)

        self.assert_(isinstance(cond, LiteralCondition))
        self.assertEqual(cond.predicate.name, "=")
        self.assert_(isinstance(cond.args[0], Term))
        self.assert_(isinstance(cond.args[1], VariableTerm))
        self.assertEqual(cond.args[0].function.name, "func1")
        
    def testConstantParsing(self):
        """Testing parsing of constants in conditions"""

        p = Parser(test2.split("\n"))
        localScope = self.getLocalScope()

        cond1 = Condition.parse(iter(p.root[0]), localScope)
        cond2 = Condition.parse(iter(p.root[1]), localScope)
        cond3 = Condition.parse(iter(p.root[2]), localScope)

        self.assert_(isinstance(cond1, LiteralCondition))
        self.assertEqual(cond1.args[0].object.name, "c1")
        self.assertEqual(cond2.args[1].object, TRUE)
        self.assertEqual(cond3.args[0].args[2].object.name, "c1")
        self.assertEqual(cond3.args[1].object, FALSE)

    def testSubtypesParsing(self):
        """Testing typechecking"""
        
        p = Parser(test3.split("\n"))

        params = [Parameter("?p1", self.type2),
                  Parameter("?p2", self.type2),
                  Parameter("?p3", self.type1),
                  Parameter("?v", t_boolean)]

        localScope = scope.Scope(params, self.scope)

        cond1 = Condition.parse(iter(p.root[0]), localScope)
        cond2 = Condition.parse(iter(p.root[1]), localScope)

        self.assert_(isinstance(cond1, LiteralCondition))
        self.assertEqual(cond1.predicate.name, cond2.predicate.name)
        self.assertEqual(cond2.args[2].object, FALSE)

    def testNegationParsing(self):
        """Testing negation parsing"""
        
        p = Parser(negtest.split("\n"))
        localScope = self.getLocalScope()

        cond1 = Condition.parse(iter(p.root[0]), localScope)
        cond2 = Condition.parse(iter(p.root[1]), localScope)
        cond3 = Condition.parse(iter(p.root[2]), localScope)

        self.assert_(isinstance(cond1, LiteralCondition))
        self.assertEqual(cond1.predicate.name, cond2.predicate.name)
        self.assert_(cond1.negated)
        self.assert_(cond2.negated)
        self.assert_(cond3.negated)
        
    def testJunctionParsing(self):
        """Testing parsing of and/or"""
        
        p = Parser(junctiontest.split("\n"))
        localScope = self.getLocalScope()

        cond = Condition.parse(iter(p.root), localScope)

        self.assert_(isinstance(cond, Conjunction))
        self.assert_(isinstance(cond.parts[0], LiteralCondition))
        self.assert_(isinstance(cond.parts[1], Disjunction))
        self.assert_(isinstance(cond.parts[1].parts[0], LiteralCondition))
        self.assert_(isinstance(cond.parts[1].parts[1], LiteralCondition))

    def testImplicationParsing(self):
        """Testing parsing of imply"""
        
        p = Parser(implicationtest.split("\n"))
        localScope = self.getLocalScope()

        cond = Condition.parse(iter(p.root), localScope)

        self.assert_(isinstance(cond, Disjunction))
        self.assert_(isinstance(cond.parts[0], LiteralCondition))
        self.assert_(isinstance(cond.parts[1], LiteralCondition))
        self.assert_(cond.parts[0].negated)
        self.assertFalse(cond.parts[1].negated)
        
    def testQuantifierParsing(self):
        """Testing parsing of quantified conditions"""
        
        p = Parser(quanttest.split("\n"))
        localScope = self.getLocalScope()

        cond = Condition.parse(iter(p.root), localScope)

        self.assert_(isinstance(cond, Conjunction))
        self.assert_(isinstance(cond.parts[0], UniversalCondition))
        self.assert_(isinstance(cond.parts[1], ExistentialCondition))

    def testVisitors(self):
        """Testing visitors"""
        
        p = Parser(mixedtest.split("\n"))
        localScope = self.getLocalScope()

        cond = Condition.parse(iter(p.root), localScope)

        def countLiteralsVisitor(cond, results=[]):
            if cond.__class__ == LiteralCondition:
                return 1
            return sum(results)
        
        def printVisitor(cond, results=[]):
            if cond.__class__ == LiteralCondition:
                if cond.negated:
                    return "(not (%s))" % cond.predicate.name
                return "(%s)" % cond.predicate.name
            if cond.__class__ == Conjunction:
                return "(and %s)" % " ".join(results)
            if cond.__class__ == Disjunction:
                return "(or %s)" % " ".join(results)
            if cond.__class__ == UniversalCondition:
                args = " ".join(sorted(cond.iterkeys()))
                return "(forall (%s) %s)" % (args, " ".join(results))
            if cond.__class__ == ExistentialCondition:
                args = " ".join(sorted(cond.iterkeys()))
                return "(exists (%s) %s)" % (args, " ".join(results))

        def copyVisitor(cond, results=[]):
            if cond.__class__ == LiteralCondition:
                return LiteralCondition(cond.predicate, cond.args, None, cond.negated)
            if isinstance(cond, JunctionCondition):
                return cond.__class__(results)
            if isinstance(cond, QuantifiedCondition):
                return cond.__class__(cond.args, results[0], cond.parent)
            
        self.assertEqual(cond.visit(countLiteralsVisitor), 3)
        self.assertEqual(cond.visit(printVisitor), "(and (forall (?a ?b) (=)) (or (exists (?x) (pred1)) (not (=))))")
        #print cond.visit(printVisitor)
        copy = cond.visit(copyVisitor)
        self.assertEqual(cond.visit(printVisitor), copy.visit(printVisitor))
        
    def testCopy(self):
        """Testing copying"""
        
        localScope = self.getLocalScope()
        cond = Parser.parse_as(mixedtest.split("\n"), Condition, localScope)
        copy = cond.copy()

        self.assertEqual(cond, copy)

        copy2 = cond.copy()
        del copy2.parts[0]
        self.assertNotEqual(copy, copy2)
        self.assertEqual(cond, copy)

        
        
    def testTypeMismatch(self):
        """Testing type checking"""
        p = Parser(test.split("\n"))

        params = [Parameter("?p1", self.type2),
                  Parameter("?p2", self.type1),
                  Parameter("?p3", t_object),
                  Parameter("?v", t_boolean)]

        localScope = scope.Scope(params, self.scope)

        try:
            cond = Condition.parse(iter(p.root), localScope)
        except ParseError, e:
            self.assertEqual(e.token.string, "func1")
            self.assertEqual(e.token.line,  2)
            return
        
        self.fail("Mismatched types triggered no error")

        
    def testCreationFromScratch(self):
        """Testing type checking when creating conditions form scratch"""
        params = [Parameter("?p1", self.type1),
                  Parameter("?p2", self.type2),
                  Parameter("?p3", t_object),
                  Parameter("?v", t_boolean)]

        localScope = scope.Scope(params, self.scope)

        c1 = LiteralCondition(self.pred1, [Term(params[0]), Term(Parameter("?p2", self.type2)), Term(TypedObject("test", t_object))])
        c2 = LiteralCondition(self.pred1, [Term(Parameter("?p1", self.type1)), Term(Parameter("?p2", self.type2)), Term(Parameter("?p3", t_object))])
        c3 = LiteralCondition(self.pred1, ["?p1", "?p2", "?p3"], localScope)
        self.assertRaises(KeyError, LiteralCondition, self.pred1, ["?p1", "?p2", "?p5"], localScope)
        c1copy = c1.copy()
        self.assertRaises(KeyError, c1.copy, localScope)
        c2copy = c2.copy(localScope)
        
        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        self.assertFalse(c2.args[0].object.is_instantiated())
        self.assert_(c2copy.args[0].object.is_instantiated())
        self.assert_(c3.args[0].object.is_instantiated())
        

    def testArityMismatch(self):
        """Testing arity mismatches 1"""
        aritytest1 = \
        """
        (pred1 ?p1 ?p2 ?p3 ?p4)
        """
        p = Parser(aritytest1.split("\n"))

        params = [Parameter("?p1", self.type1),
                  Parameter("?p2", self.type2),
                  Parameter("?p3", t_object),
                  Parameter("?p4", t_boolean),
                  Parameter("?v", t_boolean)]

        localScope = scope.Scope(params, self.scope)

        try:
            cond = Condition.parse(iter(p.root), localScope)
        except ParseError, e:
            self.assertEqual(e.token.string, "pred1")
            self.assertEqual(e.token.line,  2)
            return

        self.fail("Too many arguments triggered no error")

        
    def testArityMismatch2(self):
        """Testing arity mismatches 2"""

        aritytest2 = \
        """
        (pred1 ?p1 ?p2)
        """
        p = Parser(aritytest2.split("\n"))
        localScope = self.getLocalScope()

        try:
            cond = Condition.parse(iter(p.root), localScope)
        except ParseError, e:
            self.assertEqual(e.token.string, "pred1")
            self.assertEqual(e.token.line,  2)
            return
        
        self.fail("Not enough arguments triggered no error")
       

    def testModalPredicates(self):
        """Testing modal predicates"""

        test = \
        """(and
        (KVAL ?a (func1 ?p1 ?p2 ?p3))
        (not (in-domain (func2 ?p1 ?p2) c1))
        )
        """
        
        localScope = self.getLocalScope()
        localScope.add(Parameter("?a", t_agent))
        cond = Parser.parse_as(test.split("\n"), Condition, localScope)

    def testModalPredicatesChecks(self):
        """Testing modal predicates syntax"""

        test = \
        """(and
        (not (in-domain (func2 ?p1 ?p2) c1))
        (KVAL ?a false)
        )
        """

        test2 = \
        """(and
        (not (in-domain (func2 ?p1 ?p2) false))
        )
        """
        
        localScope = self.getLocalScope()
        localScope.add(Parameter("?a", t_agent))
        try:
            cond = Parser.parse_as(test.split("\n"), Condition, localScope)
            self.fail("Modal predicate without function didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "kval")
            self.assertEqual(e.token.line, 3)

        try:
            cond = Parser.parse_as(test2.split("\n"), Condition, localScope)
            self.fail("Modal predicate without function didn't raise exception")
        except ParseError, e:
            self.assertEqual(e.token.string, "in-domain")
            self.assertEqual(e.token.line, 2)
            return
        
        
        
        
if __name__ == '__main__':
    unittest.main()    
        
