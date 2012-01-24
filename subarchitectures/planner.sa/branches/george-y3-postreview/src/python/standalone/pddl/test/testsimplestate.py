#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest

import builtin, conditions, mapl, scope
from parser import Parser, ParseError
from predicates import Term, Literal
from mapltypes import *
from state import *


class StateTest(unittest.TestCase):
    def setUp(self):
        self.type1 = Type("type1")
        self.type2 = Type("type2", [self.type1])
        self.c1 = TypedObject("c1", self.type1)
        self.c2 = TypedObject("c2", self.type2)
        self.agent = TypedObject("agent", mapl.t_agent)
        args = [Parameter("?a", self.type1),
                Parameter("?b", self.type2),
                Parameter("?c", t_object)]
        val = Parameter("?v", t_boolean)
        eq =  Predicate("=", [Parameter("?o1", t_object), Parameter("?o2", t_object)])
        self.func1 =  Function("func1", args, t_boolean)
        self.func2 =  Function("func2", args[:-1], self.type1)

        self.pred1 =  Predicate("pred1", args)
        self.pred2 =  Predicate("pred2", args[:-1])
        
        self.scope = scope.Scope([TRUE, FALSE, self.c1, self.c2, self.agent], None)
        self.scope.types["type1"] = self.type1
        self.scope.types["type2"] = self.type2
        self.scope.types["boolean"] = t_boolean
        self.scope.types["object"] = t_object
        self.scope.types["agent"] = mapl.t_agent
        self.scope.predicates.add(eq)
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
        

    def testStateVariables(self):
        """Testing basic StateVariable functions"""
        
        svar1 = StateVariable(self.func1, [self.c1, self.c2, self.c1])
        svar2 = StateVariable(self.func2, [self.c1, self.c2])
        term2 = Term(self.func2, [self.c1, self.c2])
        svar3 = StateVariable(self.pred1, [self.c1, self.c2, self.c1])
        kvar2 = StateVariable(self.func2, [self.c1, self.c2], mapl.knowledge, [self.agent])    
        ivar2 = StateVariable(self.func2, [self.c1, self.c2], mapl.indomain, [self.c1])

        self.assertEqual(svar1.get_type(), t_boolean)
        self.assertEqual(svar2.get_type(), self.type1)
        self.assertEqual(svar3.get_type(), t_boolean)
        self.assertEqual(kvar2.get_type(), t_boolean)
        self.assertEqual(ivar2.get_type(), t_boolean)

        self.assertEqual(svar1.get_predicate(), None)
        self.assertEqual(svar2.get_predicate(), None)
        self.assertEqual(svar3.get_predicate(), self.pred1)
        self.assertEqual(kvar2.get_predicate(), mapl.knowledge)
        self.assertEqual(ivar2.get_predicate(), mapl.indomain)

        self.assertEqual(kvar2.nonmodal(), svar2)
        self.assertEqual(ivar2.nonmodal(), svar2)

        self.assertEqual(svar2.as_modality(mapl.knowledge, [self.agent]), kvar2)
        self.assertEqual(svar2.as_modality(mapl.indomain, [self.c1]), ivar2)

        self.assertEqual(svar1.get_args(), [self.c1, self.c2, self.c1])
        self.assertEqual(kvar2.get_args(), [self.agent, term2])
        self.assertEqual(ivar2.get_args(), [Term(self.func2, [self.c1, self.c2]), self.c1])

        self.assertEqual(svar1, StateVariable.from_term(Term(self.func1, [self.c1, self.c2, self.c1])))
        self.assertEqual(svar2, StateVariable.from_term(term2))
        self.assertEqual(svar3, StateVariable.from_literal(Literal(self.pred1, [self.c1, self.c2, self.c1])))
        self.assertEqual(kvar2, StateVariable.from_literal(Literal(mapl.knowledge, [self.agent, term2])))
        self.assertEqual(ivar2, StateVariable.from_literal(Literal(mapl.indomain, [term2, self.c1])))

        try:
            lit1 = svar1.as_literal()
            self.fail("No exception when trying to create a literal from a fluent variable")
        except AssertionError, e:
            pass
            
        lit3 = svar3.as_literal()
        klit2 = kvar2.as_literal()
        
        self.assertEqual(lit3.predicate, self.pred1)
        self.assertEqual(klit2.predicate, mapl.knowledge)
        
        self.assertEqual(lit3.args, [Term(self.c1), Term(self.c2), Term(self.c1)])
        self.assertEqual(klit2.args, [Term(self.agent), Term(self.func2, [self.c1, self.c2])])

        self.assertEqual(str(svar1), "func1(c1 c2 c1)")
        self.assertEqual(str(svar3), "pred1(c1 c2 c1)")
        self.assertEqual(str(kvar2), "kval(func2(c1 c2), agent)")
        self.assertEqual(str(ivar2), "in-domain(func2(c1 c2), c1)")
        
        
    def testFacts(self):
        """Testing basic Fact functions"""

        svar1 = StateVariable(self.func1, [self.c1, self.c2, self.c1])
        term1 = Term(self.func1, [self.c1, self.c2, self.c1])
        svar2 = StateVariable(self.func2, [self.c1, self.c2])
        term2 = Term(self.func2, [self.c1, self.c2])
        svar3 = StateVariable(self.pred1, [self.c1, self.c2, self.c1])
        kvar2 = StateVariable(self.func2, [self.c1, self.c2], mapl.knowledge, [self.agent])    

        fact1 = Fact(svar1, TRUE)
        fact2 = Fact(svar2, self.c1)
        fact3 = Fact(svar3, FALSE)

        kfact2 = Fact(kvar2, TRUE)
        
        self.assertEqual(fact1.as_literal(), Literal(builtin.assign, [term1, TRUE]))
        self.assertEqual(fact2.as_literal(useEqual=True), Literal(builtin.equal_assign, [term2, self.c1]))
        self.assertEqual(fact3.as_literal(), Literal(self.pred1, [self.c1, self.c2, self.c1], negated=True))
        self.assertEqual(kfact2.as_literal(), Literal(mapl.knowledge, [self.agent, term2]))

        self.assertEqual(fact1, Fact.from_literal(Literal(builtin.assign, [term1, TRUE])))
        self.assertEqual(fact2, Fact.from_literal(Literal(builtin.equals, [term2, self.c1])))
        self.assertEqual(fact3, Fact.from_literal(Literal(self.pred1, [self.c1, self.c2, self.c1], negated=True)))
        self.assertEqual(kfact2, Fact.from_literal(Literal(mapl.knowledge, [self.agent, term2])))
        
        self.assertEqual(str(fact1), "func1(c1 c2 c1) = true")
        self.assertEqual(str(fact2), "func2(c1 c2) = c1")
        self.assertEqual(str(fact3), "pred1(c1 c2 c1) = false")
        self.assertEqual(str(kfact2), "kval(func2(c1 c2), agent) = true")
        
    def testLiteralInstantiation(self):
        """Testing instantiation of literals"""
        
        test = \
        """
        (= (func1 ?p1 ?p2 ?p3) ?v)
        """
        localScope = self.getLocalScope()
        cond = Parser.parse_as(test.split("\n"), conditions.Condition, localScope)

        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        fact = Fact.from_literal(cond)

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
        self.assertRaises(Exception, Fact.from_literal, cond)
        
        #instancing via objects and via names should be equivalent
        localScope.instantiate({"?p1" : "c1", "?p2" : "c2", "?p3" : "c1", "?v" : "true"})
        fact2 = Fact.from_literal(cond)

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
        cond = Parser.parse_as(test.split("\n"), conditions.Condition, localScope)

        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        facts = Fact.from_condition(cond)
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
        localScope.add(Parameter("?a", mapl.t_agent))
        cond = Parser.parse_as(test.split("\n"), conditions.Condition, localScope)
        
        localScope.instantiate({"?a" : self.agent, "?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        f1 = Fact(StateVariable(self.func1, [self.c1, self.c2, self.c1], mapl.knowledge, [self.agent]), TRUE)
        f2 = Fact(StateVariable(self.func2, [self.c1, self.c2], mapl.indomain, [self.c1]), FALSE)

        facts = Fact.from_condition(cond)
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
        cond = Parser.parse_as(test.split("\n"), conditions.Condition, localScope)
        s1 = Fact(StateVariable(self.func2, [self.c1, self.c2]), self.c1)
        s2 = Fact(StateVariable(self.func2, [self.c2, self.c2]), self.c2)
        localScope.instantiate({"?p1" : self.c1, "?p2" : self.c2, "?p3" : self.c1, "?v" : TRUE})
        
        state = State([s1, s2])

        self.assertRaises(Exception, Fact.from_condition, cond)
        facts = Fact.from_condition(cond, state)
        
        f1 = Fact(StateVariable(self.pred1, [self.c1, self.c2, self.c1]), TRUE)
        f2 = Fact(StateVariable(self.func1, [self.c2, self.c2, self.c1]), FALSE)
        self.assert_(f1 in facts)
        self.assert_(f2 in facts)

        self.assertFalse(state.is_satisfied(cond))
        state.set(f1)
        self.assertFalse(state.is_satisfied(cond))
        state.set(f2)
        self.assert_(state.is_satisfied(cond))

        #state2 = State([s1])
        #self.assertRaises(Exception, Fact.from_condition, cond, state2)
        
        
        
if __name__ == '__main__':
    unittest.main()    
        
