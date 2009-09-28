#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest

import mapl_new as mapl
from mapl_new.predicates import *
from mapl_new.mapltypes import *
from mapl_new import conditions, scope 
from mapl_new.parser import Parser, ParseError
from state_new import *


class StateTest(unittest.TestCase):
    def setUp(self):
        self.type1 = Type("type1")
        self.type2 = Type("type2", [self.type1])
        self.c1 = TypedObject("c1", self.type1)
        self.c2 = TypedObject("c2", self.type2)
        self.agent = TypedObject("agent", agentType)
        args = [Parameter("?a", self.type1),
                Parameter("?b", self.type2),
                Parameter("?c", objectType)]
        val = Parameter("?v", booleanType)
        eq =  Predicate("=", [Parameter("?o1", objectType), Parameter("?o2", objectType)])
        self.func1 =  Function("func1", args, booleanType)
        self.func2 =  Function("func2", args[:-1], self.type1)

        self.pred1 =  Predicate("pred1", args)
        self.pred2 =  Predicate("pred2", args[:-1])
        
        self.scope = scope.Scope([TRUE, FALSE, self.c1, self.c2, self.agent], None)
        self.scope.types["type1"] = self.type1
        self.scope.types["type2"] = self.type2
        self.scope.types["boolean"] = booleanType
        self.scope.types["object"] = objectType
        self.scope.types["agent"] = agentType
        self.scope.predicates.add(eq)
        self.scope.predicates.add(self.pred1)
        self.scope.predicates.add(self.pred2)
        self.scope.predicates.add(knowledge)
        self.scope.predicates.add(indomain)
        self.scope.functions.add(self.func1)
        self.scope.functions.add(self.func2)

    def getLocalScope(self):
        params = [Parameter("?p1", self.type1),
                  Parameter("?p2", self.type2),
                  Parameter("?p3", objectType),
                  Parameter("?v", booleanType)]

        return scope.Scope(params, self.scope)
        

    def testLiteralInstantiation(self):
        """Testing instantiation of literals"""
        
        test = \
        """
        (= (func1 ?p1 ?p2 ?p3) ?v)
        """
        localScope = self.getLocalScope()
        cond = Parser.parseAs(test.split("\n"), conditions.Condition, localScope)

        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        fact = Fact.fromLiteral(cond)

        #check that svar and value are correct
        self.assertEqual(fact.value, TRUE)
        self.assertEqual(fact.svar, StateVariable(self.func1, [self.c1, self.c2, self.c1]))
        #check that comparison with a manually constructed fact work
        self.assertEqual(fact, Fact(StateVariable(self.func1, [self.c1, self.c2, self.c1]), TRUE))
        self.assertNotEqual(fact, Fact(StateVariable(self.func1, [self.c1, self.c2, self.c1]), FALSE))
        self.assertNotEqual(fact, Fact(StateVariable(self.func1, [self.c2, self.c2, self.c1]), TRUE))
        self.assertNotEqual(fact, Fact(StateVariable(self.pred1, [self.c1, self.c2, self.c1]), TRUE))
        #check that comparison with a tuple work
        self.assertEqual(fact, (StateVariable(self.func1, [self.c1, self.c2, self.c1]), TRUE))
        self.assertNotEqual(fact, (StateVariable(self.func1, [self.c1, self.c2, self.c1]), FALSE))
        self.assertNotEqual(fact, (StateVariable(self.func1, [self.c2, self.c2, self.c1]), TRUE))
        self.assertNotEqual(fact, (StateVariable(self.pred1, [self.c1, self.c2, self.c1]), TRUE))

        #check that trying to get a fact from an uninstantiated literal will fail
        localScope.uninstantiate()
        self.assertRaises(Exception, Fact.fromLiteral, cond)
        
        #instancing via objects and via names should be equivalent
        localScope.instantiate({"?p1" : "c1", "?p2" : "c2", "?p3" : "c1", "?v" : "true"})
        fact2 = Fact.fromLiteral(cond)

        self.assertEqual(fact, fact2)

        
    def testConditionInstantiation(self):
        """Testing instantiation of conditions"""

        test = \
        """(and
        (pred1 c1 ?p2 ?p3)
        (not (pred2 c1 ?p2))
        (= (func1 ?p1 ?p2 ?p3) ?v)
        (= (func1 ?p1 ?p2 c2) false)
        )
        """
        
        localScope = self.getLocalScope()
        cond = Parser.parseAs(test.split("\n"), conditions.Condition, localScope)

        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        facts = Fact.fromCondition(cond)
        f1 = Fact(StateVariable(self.pred1, [self.c1, self.c2, self.c1]), TRUE)
        f2 = Fact(StateVariable(self.pred2, [self.c1, self.c2]), FALSE)
        f3 = Fact(StateVariable(self.func1, [self.c1, self.c2, self.c1]), TRUE)
        f4 = Fact(StateVariable(self.func1, [self.c1, self.c2, self.c2]), FALSE)
        self.assertEqual(len(facts), 4)
        self.assert_(f1 in facts)
        self.assert_(f2 in facts)
        self.assert_(f3 in facts)
        self.assert_(f4 in facts)
        

    def testModalInstantiation(self):
        """Testing modal predicates"""

        test = \
        """(and
        (KVAL ?a (func1 ?p1 ?p2 ?p3))
        (not (in-domain (func2 ?p1 ?p2) c1))
        )
        """
        
        localScope = self.getLocalScope()
        localScope.add(Parameter("?a", agentType))
        cond = Parser.parseAs(test.split("\n"), conditions.Condition, localScope)
        
        localScope.instantiate({"?a" : self.agent, "?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        f1 = Fact(StateVariable(self.func1, [self.c1, self.c2, self.c1], modality=knowledge, modal_args=[self.agent]), TRUE)
        f2 = Fact(StateVariable(self.func2, [self.c1, self.c2], modality=indomain, modal_args=[self.c1]), FALSE)

        facts = Fact.fromCondition(cond)
        self.assert_(f1 in facts)
        self.assert_(f2 in facts)

    def testNestedFunctionInstantiation(self):
        """Testing instantiation of literals with nested functions"""
        
        test = \
        """(and
        (pred1 (func2 (func2 ?p1 ?p2) ?p2) ?p2 ?p3)
        (= (func1 (func2 c2 ?p2) ?p2 (func2 (func2 ?p1 ?p2) ?p2)) false)
        )
        """
        
        localScope = self.getLocalScope()
        cond = Parser.parseAs(test.split("\n"), conditions.Condition, localScope)
        s1 = Fact(StateVariable(self.func2, [self.c1, self.c2]), self.c1)
        s2 = Fact(StateVariable(self.func2, [self.c2, self.c2]), self.c2)
        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        
        state = State([s1, s2])

        self.assertRaises(Exception, Fact.fromCondition, cond)
        facts = state.factsFromCondition(cond)
        
        f1 = Fact(StateVariable(self.pred1, [self.c1, self.c2, self.c1]), TRUE)
        f2 = Fact(StateVariable(self.func1, [self.c2, self.c2, self.c1]), FALSE)
        self.assert_(f1 in facts)
        self.assert_(f2 in facts)

        self.assertFalse(state.isSatisfied(cond))
        state.set(f1)
        self.assertFalse(state.isSatisfied(cond))
        state.set(f2)
        self.assert_(state.isSatisfied(cond))

        state2 = State([s1])
        self.assertRaises(Exception, state2.factsFromCondition, cond)
        
        
        
if __name__ == '__main__':
    unittest.main()    
        
