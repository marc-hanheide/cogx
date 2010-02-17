#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser, scope
from mapltypes import Type, FunctionType, TypedObject, TypedNumber, Parameter
from mapltypes import TRUE, FALSE, t_object, t_boolean

from parser import ParseError, UnexpectedTokenError

def parse_arg_list(it, typeDict, parentScope=None, previous_params=[], require_types=False):
    tempScope = scope.Scope([], parentScope)
    
    def left_func(elem):
        if elem.token.string[0] != "?":
            raise UnexpectedTokenError(elem.token, "parameter name")
        return elem
    
    def right_func(elem):
        return Type.parse(elem, typeDict, tempScope)

    args = []
    names = set(p.name for p in previous_params)
    for params, type in parser.parse_typed_list(it, left_func, right_func, "parameter name", "type specification", require_types):
        if type is None:
            type = t_object
        for p in params:
            if p.token.string in names:
                raise ParseError(p.token, "Duplicate parameter name: %s" % p.token.string)
            param = Parameter(p.token.string, type)
            args.append(param)
            names.add(p.token.string)
            tempScope.add(param)

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

        args = parse_arg_list(it, types)
        return Function(name, args, type)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.name == other.name \
            and self.type == other.type and all(map(lambda a,b: a == b, self.args, other.args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "(%s %s) - %s" % (self.name, " ".join(str(a) for a in self.args), self.type)

    def __hash__(self):
        return hash((self.name, self.type)+tuple(self.args))
    
class Predicate(Function):
    def __init__(self, name, args, builtin=False):
        Function.__init__(self, name, args, t_boolean, builtin)
        
    @staticmethod
    def parse(it, types):
        name = it.get("terminal", "predicate identifier").token.string
        args = []

        args = parse_arg_list(it, types)
        return Predicate(name, args)

    def __str__(self):
        return "(%s %s)" % (self.name, " ".join(str(a) for a in self.args))


class Literal(object):
    def __init__(self, predicate, args, scope=None, negated=False):
        self.predicate = predicate
        self.negated = negated
        assert isinstance(predicate, Function)

        if scope:
            self.args = scope.lookup(args)
        else:
            self.args = [Term(a) for a in args]

    def pddl_str(self, instantiated=True):
        s = "(%s %s)" % (self.predicate.name, " ".join(a.pddl_str(instantiated) for a in self.args))
        if self.negated:
            return "(not %s)" % s
        return s

    def negate(self):
        return self.__class__(self.predicate, self.args[:], None, not self.negated)

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        if copy_instance:
            l = self.copy_instance()
            if new_scope:
                l.set_scope(new_scope)
            return l
        
        return self.__class__(self.predicate, self.args, new_scope, self.negated)

    def set_scope(self, new_scope):
        self.args = new_scope.lookup(self.args)
    
    def copy_instance(self):
        return self.__class__(self.predicate, [a.copy_instance() for a in self.args], negated=self.negated)

    def collect_arguments(self):
        return sum([term.visit(collect_args_visitor) for term in self.args], [])
            
    def __str__(self):
        s = "(%s %s)" % (self.predicate.name, " ".join(str(a) for a in self.args))
        if self.negated:
            s = "(not %s)" % s
        return s
            
    def __eq__(self, other):
        if not isinstance(other, Literal):
            return False
#        return self.__class__ == other.__class__ and self.predicate == other.predicate and self.negated == other.negated and all(map(lambda a,b: a==b, self.args, other.args))
        return self.predicate == other.predicate and self.negated == other.negated and all(map(lambda a,b: a==b, self.args, other.args))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.predicate, self.negated ) + tuple(self.args))
        
    @staticmethod
    def parse(it, scope, negate=False, maxNesting=999):
        import builtin
        first = it.get("terminal", "predicate").token

        if first.string == "not":
            j = iter(it.get(list, "literal"))
            it.no_more_tokens()
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
            type_str = " ".join(str(a.get_type()) for a in args)
            candidates = scope.predicates[first.string]
            c_str = "\n  ".join(str(p) for p in candidates)
            raise ParseError(first, "no matching predicate found for (%s %s). Candidates are:\n  %s" % (first.string, type_str, c_str))

        #check type constraints for assignments
        if predicate in builtin.assignment_ops:
            term = args[0]
            value = args[1]
            if not value.get_type().equal_or_subtype_of(term.get_type().type):
                raise ParseError(first, "Can't assign object of type %s to %s." % (value.get_type(), term.function.name))

        #check nesting constraints
        if maxNesting <= 0:
            for i, (arg, parg) in enumerate(zip(args, predicate.args)):
                if isinstance(arg, FunctionTerm) and not isinstance(parg.type, FunctionType):
                    raise ParseError(first, "Error in Argument %d: Maximum nesting depth for functions exceeded or no functions allowed here." % (i+1))

        return Literal(predicate, args, scope, negate)


class Term(object):
    def __init__(self, *params):
        if len(params) == 1:
            obj = params[0]
            if isinstance(obj, (ConstantTerm, VariableTerm)):
                obj = obj.object
            elif isinstance(obj, FunctionTerm):
                self.__class__ = FunctionTerm
                FunctionTerm.__init__(self, obj.function, obj.args)
                return
                
            if isinstance(obj, Parameter):
                if isinstance(obj.type, FunctionType):
                    self.__class__ = FunctionVariableTerm
                    FunctionVariableTerm.__init__(self, obj)
                else:
                    self.__class__ = VariableTerm
                    VariableTerm.__init__(self, obj)
            elif isinstance(obj, TypedObject):
                self.__class__ = ConstantTerm
                ConstantTerm.__init__(self, obj)
            elif isinstance(obj, (int, float, long)):
                self.__class__ = ConstantTerm
                ConstantTerm.__init__(self, TypedNumber(obj))
            else:
                raise Exception("Unexpected Argument for Term: %s" % str(obj))
        elif len(params) == 2:
            func = params[0]
            args = params[1]
            assert isinstance(func, Function)
            assert isinstance(args, list)
            self.__class__ = FunctionTerm
            FunctionTerm.__init__(self, func, [Term(a) for a in args])
        else:
            raise Exception("Too many arguments for Term()")
    
    def visit(self, fn):
        return fn(self, [])
    
    def pddl_str(self, instantiated=True):
        def printVisitor(term, results=[]):
            if isinstance(term, VariableTerm) and term.is_instantiated() and instantiated:
                term = term.get_instance()
                
            if term.__class__ == FunctionTerm:
                return "(%s %s)" % (term.function.name, " ".join(results))
            elif isinstance(term, Term):
                return term.object.name
            return term.name
        return self.visit(printVisitor)
    
    def get_type(self):
        raise NotImplementedError()
    
    def is_instance_of(self, type):
        return self.get_type().equal_or_subtype_of(type)

    def __eq__(self, other):
        return self.__class__ == other.__class__

    def __ne__(self, other):
        return not self.__eq__(other)
    
    @staticmethod
    def parse(it, scope, maxNesting=999):
        if isinstance(it, parser.ElementIterator):
            term = it.get(None, "function term, variable or constant")
        else:
            term = it

        if term.is_terminal():
            if term.token.string in scope:
                obj = scope[term.token.string]
            else:
                try:
                    value = float(term.token.string)
                except:
                    raise ParseError(term.token, "Unknown identifier: '%s'" % term.token.string)
                obj = TypedNumber(value)

            if isinstance(obj, Parameter):
                if isinstance(obj.type, FunctionType):
                    return FunctionVariableTerm(obj)
                return VariableTerm(obj)
            return ConstantTerm(obj)
        
        return FunctionTerm.parse(iter(term), scope, maxNesting-1)

    
class FunctionTerm(Term):
    def __init__(self, function, args, scope=None):
        self.function = function

        if scope:
            self.args = scope.lookup(args)
        else:
            self.args = args
    
    def get_type(self):
        return FunctionType(self.function.type)
    
    def copy_instance(self):
        return FunctionTerm(self.function, [a.copy_instance() for a in self.args])

    def visit(self, fn):
        return fn(self, [a.visit(fn) for a in self.args])
    
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
            raise ParseError(name.token, "Unknown function: '%s'" % name.token.string)
        
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
            type_str = " ".join(str(a.get_type()) for a in args)
            candidates = scope.functions[name.token.string]
            c_str = "\n  ".join(str(p) for p in candidates)
            raise ParseError(name.token, "no matching function found for (%s %s). Candidates are:\n  %s" % (name.token.string, type_str, c_str))

        return FunctionTerm(func, args)

class VariableTerm(Term):
    def __init__(self, parameter):
        assert isinstance(parameter, Parameter)
        self.object = parameter
        
    def is_instantiated(self):
        return self.object.is_instantiated()

    def get_instance(self):
        if self.is_instantiated():
            return self.object.get_instance()
        raise Exception, "Term %s is not instantiated" % str(self.object)

    def copy_instance(self):
        if self.is_instantiated():
            return ConstantTerm(self.get_instance())
        return VariableTerm(self.object)
    
    def get_type(self):
        return self.object.type

    def __eq__(self, other):
        if isinstance(other, VariableTerm):
            return self.object == other.object
        return self.object == other
    
    def __hash__(self):
        return hash((self.__class__, self.object))
    
class FunctionVariableTerm(FunctionTerm, VariableTerm):
    def __init__(self, obj):
        self.object = obj

    def get_type(self):
        return self.object.type

    def visit(self, fn):
        if self.is_instantiated():
            return self.object.get_instance().visit(fn)
        return fn(self, [])
    
    def copy_instance(self):
        if self.is_instantiated():
            return FunctionTerm(self.get_instance().function, [a for a in self.get_instance().args])
        return FunctionVariableTerm(self.object)
    
    def __get_function(self):
        if self.is_instantiated():
            return self.object.get_instance().function
        raise Exception, "Term %s is not instantiated" % str(self.object)

    def __get_args(self):
        if self.is_instantiated():
            return self.object.get_instance().args
        raise Exception, "Term %s is not instantiated" % str(self.object)
        
    function = property(__get_function)
    args = property(__get_args)

    def __str__(self):
        return "FunctionVariableTerm: %s" % self.object.name
    
    def __eq__(self, other):
        if isinstance(other, FunctionVariableTerm):
            return self.object == other.object
        return self.object == other

    def __hash__(self):
        return hash((self.__class__, self.object))

    
    
class ConstantTerm(Term):
    def __init__(self, obj):
        assert isinstance(obj, TypedObject)
        self.object = obj
    
    def get_type(self):
        return self.object.type

    def copy_instance(self):
        return ConstantTerm(self.object)
    
    def __str__(self):
        return "Term: %s" % self.object.name
    
    def __eq__(self, other):
        if isinstance(other, ConstantTerm):
            return self.object == other.object
        return self.object == other

    def __hash__(self):
        return hash((self.__class__, self.object))


def collect_args_visitor(term, results):
    if isinstance(term, (VariableTerm, ConstantTerm)):
        return [term.object]
    if isinstance(term, FunctionTerm):
        return sum(results, [])
