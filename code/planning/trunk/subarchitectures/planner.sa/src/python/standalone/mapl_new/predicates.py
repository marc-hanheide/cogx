#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser, scope
from mapltypes import *

from parser import ParseError, UnexpectedTokenError

def parseArgList(it, typeDict):
    args = []
    
    def leftFunc(elem):
        if elem.token.string[0] != "?":
            raise UnexpectedTokenError(elem.token, "parameter name")
        return elem
    
    def rightFunc(elem):
        return Type.parse(elem, typeDict)

    args = []
    for params, type in parser.parseTypedList(it, leftFunc, rightFunc, "parameter name", "type specification", True):
        for p in params:
            args.append(Parameter(p.token.string, type))

    return args

class Function(object):
    def __init__(self, name, args, type, builtin=False):
        self.name = name
        self.args = args
        self.type = type
        self.builtin=builtin

        self.arity = len(args)
        
    @staticmethod
    def parse(it, type, types):
        name = it.get("terminal", "function identifier").token.string
        args = []

        args = parseArgList(it, types)
        return Function(name, args, type)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.name == other.name \
            and self.type == other.type and all(map(lambda a,b: a == b, self.args, other.args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "(%s %s) - %s" % (self.name, " ".join(str(a) for a in self.args), self.type)
    
class Predicate(Function):
    def __init__(self, name, args, builtin=False):
        Function.__init__(self, name, args, booleanType, builtin)
        
    @staticmethod
    def parse(it, types):
        name = it.get("terminal", "predicate identifier").token.string
        args = []

        args = parseArgList(it, types)
        return Predicate(name, args)

    def __str__(self):
        return "(%s %s)" % (self.name, " ".join(str(a) for a in self.args))

#basic predicates
equals = Predicate("=", [Parameter("?o1", objectType), Parameter("?o2", objectType)], builtin=True)

assign = Predicate("assign", [Parameter("?f", FunctionType(objectType)), Parameter("?v", objectType)], builtin=True)
change = Predicate("change", [Parameter("?f", FunctionType(objectType)), Parameter("?v", objectType)], builtin=True)
equalAssign = Predicate("=", [Parameter("?f", FunctionType(objectType)), Parameter("?v", objectType)], builtin=True)
num_equalAssign = Predicate("=", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)

#numeric predicates
num_assign = Predicate("assign", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)
scale_up = Predicate("scale-up", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)
scale_down = Predicate("scale-down", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)
increase = Predicate("increase", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)
decrease = Predicate("decrease", [Parameter("?f", FunctionType(numberType)), Parameter("?v", numberType)], builtin=True)

numericOps = [num_assign, scale_up, scale_down, increase, decrease]
assignmentOps = [assign, num_assign, equalAssign, num_equalAssign]

gt = Predicate(">", [Parameter("?n1", numberType), Parameter("?n2", numberType)], builtin=True)
lt = Predicate("<", [Parameter("?n1", numberType), Parameter("?n2", numberType)], builtin=True)
eq = Predicate("=", [Parameter("?n1", numberType), Parameter("?n2", numberType)], builtin=True)
ge = Predicate(">=", [Parameter("?n1", numberType), Parameter("?n2", numberType)], builtin=True)
le = Predicate("<=", [Parameter("?n1", numberType), Parameter("?n2", numberType)], builtin=True)

numericComparators = [gt, lt, eq, ge, le]

#numeric functions
minus = Function("-", [Parameter("?n1", numberType), Parameter("?n2", numberType)], numberType, builtin=True)
plus = Function("+", [Parameter("?n1", numberType), Parameter("?n2", numberType)], numberType, builtin=True)
mult = Function("*", [Parameter("?n1", numberType), Parameter("?n2", numberType)], numberType, builtin=True)
div = Function("/", [Parameter("?n1", numberType), Parameter("?n2", numberType)], numberType, builtin=True)

neg = Function("-", [Parameter("?n", numberType)], numberType, builtin=True)

numericFunctions = [minus, plus, mult, div, neg]

#default minimization function
total_time = Function("total-time", [], numberType, builtin=True)

#mapl predicates
knowledge = Predicate("kval", [Parameter("?a", agentType), Parameter("?f", FunctionType(objectType))], builtin=True)
indomain = Predicate("in-domain", [Parameter("?f", FunctionType(objectType)), Parameter("?v", objectType), ], builtin=True)

mapl_modal_predicates = [knowledge, indomain]

is_planning_agent = Predicate("is_planning_agent", [Parameter("?a", agentType)], builtin=True)
achieved = Predicate("achieved", [Parameter("?sg", subgoalType)], builtin=True)
commited_to_plan = Predicate("commited_to_plan", [Parameter("?a", agentType)], builtin=True)
can_talk_to = Predicate("can_talk_to", [Parameter("?a1", agentType), Parameter("?a2", agentType)], builtin=True)

mapl_predicates = mapl_modal_predicates + [is_planning_agent, achieved, commited_to_plan, can_talk_to]


class Literal(object):
    def __init__(self, predicate, args, scope=None, negated=False):
        self.predicate = predicate
        self.negated = negated

        if scope:
            self.args = scope.lookup(args)
        else:
            self.args = args

    def __str__(self):
        s = "(%s %s)" % (self.predicate.name, " ".join(str(a) for a in self.args))
        if self.negated:
            s = "(not %s)" % s
        return s
            
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.predicate == other.predicate and self.negated == other.negated and all(map(lambda a,b: a==b, self.args, other.args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.predicate, self.negated ) + tuple(self.args))
        
    @staticmethod
    def parse(it, scope, negate=False, maxNesting=999):
        first = it.get("terminal", "predicate").token

        if first.string == "not":
            j = iter(it.get(list, "literal"))
            it.noMoreTokens()
            return Literal.parse(j, scope, not negate, maxNesting)

        if first.string not in scope.predicates:
            raise ParseError(first, "Unknown predicate: %s" % first.string)
            
        args = []
        while True:
            try:
                args.append(Term.parse(it, scope, maxNesting))
            except parser.EndOfListError:
                break

        predicate = scope.predicates.get(first.string, args)
        if not predicate:
            type_str = " ".join(map(lambda a: str(a.getType()), args))
            candidates = scope.predicates[first.string]
            c_str = "\n  ".join(str(p) for p in candidates)
            raise ParseError(first, "no matching predicate found for (%s %s). Candidates are:\n  %s" % (first.string, type_str, c_str))

        #check type constraints for assignments
        if predicate in assignmentOps:
            term = args[0]
            value = args[1]
            if not value.getType().equalOrSubtypeOf(term.getType().type):
                raise ParseError(first, "Can't assign object of type %s to %s." % (value.getType(), term.function.name))

        #check nesting constraints
        if maxNesting <= 0:
            for i, (arg, parg) in enumerate(zip(args, predicate.args)):
                if isinstance(arg, FunctionTerm) and not isinstance(parg.type, FunctionType):
                    raise ParseError(first, "Error in Argument %d: Maximum nesting depth for functions exceeded or no functions allowed here." % (i+1))

        return Literal(predicate, args, scope, negate)
    
class Term(object):
    def __init__(self, function, args):
        raise NotImplementedError()
    
    def getType(self):
        raise NotImplementedError()

    def __eq__(self, other):
        return self.__class__ == other.__class__;

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.__class__)
    
    @staticmethod
    def parse(it, scope, maxNesting=999):
        term = it.get(None, "function term, variable or constant")

        if term.isTerminal():
            if term.token.string in scope:
                obj = scope[term.token.string]
            else:
                try:
                    value = float(term.token.string)
                except:
                    raise ParseError(term.token, "Unkown identifier: '%s'" % term.token.string)
                obj = TypedObject(value, numberType)
                
            return ConstantTerm(obj)
                
        
        return FunctionTerm.parse(iter(term), scope, maxNesting-1)

    
class FunctionTerm(Term):
    def __init__(self, function, args, scope=None):
        self.function = function

        if scope:
            self.args = scope.lookup(args)
        else:
            self.args = args
    
    def getType(self):
        return FunctionType(self.function.type)

    def __str__(self):
        return "FunctionTerm: %s(%s)" % (self.function.name, " ".join(str(s) for s in self.args))

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.function == other.function and all(map(lambda a,b: a==b, self.args, other.args))

    def __hash__(self):
        return hash((self.function, ) + tuple(self.args))

    @staticmethod
    def parse(it, scope, maxNesting=999):
        name = it.get(None, "function identifier")
        if name.token.string not in scope.functions:
            raise ParseError(name.token, "Unkown function: '%s'" % name.token.string)
        
        args = []
        i = 0
        while True:
            try:
                term = Term.parse(it, scope, maxNesting)
                if maxNesting <= 0 and isinstance(term, FunctionTerm):
                    raise ParseError(name.token, "Error in Argument %d: Maximum nesting depth for functions exceeded or no functions allowed here." % i+1)
                args.append(term)
                i += 1
            except parser.EndOfListError:
                break

        func = scope.functions.get(name.token.string, args)
        if not func:
            type_str = " ".join(map(lambda a: str(a.getType()), args))
            candidates = scope.functions[name.token.string]
            c_str = "\n  ".join(str(p) for p in candidates)
            raise ParseError(name.token, "no matching function found for (%s %s). Candidates are:\n  %s" % (name.token.string, type_str, c_str))

        return FunctionTerm(func, args)

    
class ConstantTerm(Term):
    def __init__(self, obj):
        self.object = obj
    
    def getType(self):
        return self.object.type

    def __str__(self):
        return "Term: %s" % self.object.name
    
    def __eq__(self, other):
        if isinstance(other, ConstantTerm):
            return self.object == other.object
        return self.object == other

    def __hash__(self):
        return hash((self.__class__, self.object))
