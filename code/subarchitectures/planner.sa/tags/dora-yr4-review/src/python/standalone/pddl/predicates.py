#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser, scope
from mapltypes import Type, FunctionType, TypedObject, TypedNumber, Parameter
from mapltypes import t_object, t_boolean

from parser import ParseError, UnexpectedTokenError

def parse_arg_list(it, typeDict, parentScope=None, previous_params=[], require_types=False):
    """parses a PDDL argument list and returns it as a list of Parameter objects.

    Arguments:
    it -- the ElementIterator of the list to parse.
    typeDict -- a dictionary of typename to Type objects that contains the defined types.
    parentScope -- The Scope this element lives in, None by default
    previous_params -- List of already parsed Parameters, empty by default
    require_types -- If True, requires that a typename is specified for each parameter. Otherwise t_object is assumed as default.
    """
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
    """A Function object represents any type of PDDL function or
    predicate."""
    
    def __init__(self, name, args, type, builtin=False, function_scope=scope.SCOPE_ALL):
        """Create a new Function object.

        Arguments:
        name -- the name of the function
        args -- list of Parameter objects of this function
        type -- Type object of this function
        builtin -- True if this function is a builtin PDDL/MAPL/whateverDDL function
        """
        
        self._name = name
        self._args = args
        self._type = type
        self.builtin=builtin
        self.function_scope=function_scope

        self.arity = len(args)
        self.hash = hash((self.name, self.type)+tuple(self.args))

    name = property(lambda self: self._name)
    type = property(lambda self: self._type)
    args = property(lambda self: self._args)
        
    @staticmethod
    def parse(it, type, types):
        name = it.get("terminal", "function identifier").token.string
        args = []

        args = parse_arg_list(it, types)
        return Function(name, args, type)

    def is_modal(self):
        return any(isinstance(a.type, FunctionType) for a in self.args)
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.hash == other.hash

    def __ne__(self, other):
        return not self.__eq__(other)

    def __str__(self):
        return "(%s %s) - %s" % (self.name, " ".join(str(a) for a in self.args), self.type)

    def __hash__(self):
        return self.hash
    
class Predicate(Function):
    """A Predicate is just a Function with a boolean type."""
    
    def __init__(self, name, args, builtin=False, function_scope=scope.SCOPE_ALL):
        """Create a new Predicate object.

        Arguments:
        name -- the name of the predicate
        args -- list of Parameter objects of this predicate
        builtin -- True if this function is a builtin PDDL/MAPL/whateverDDL predicate
        """
        Function.__init__(self, name, args, t_boolean, builtin, function_scope)
        
    @staticmethod
    def parse(it, types):
        name = it.get("terminal", "predicate identifier").token.string
        args = []

        args = parse_arg_list(it, types)
        return Predicate(name, args)

    def __str__(self):
        return "(%s %s)" % (self.name, " ".join(str(a) for a in self.args))


class Literal(object):
    """This class represents a PDDL literal.

    The arguments of the literal can be functions, variables and
    constants and are stored as Term objects in the "args" member
    variable."""
    
    def __init__(self, predicate, args, _scope=None, negated=False):
        """Create a new Literal object.

        Arguments:
        predicate -- the Predicate object this literal refers to.
        args -- list of TypesObjects or Terms that are arguments of this literal.
        _scope -- Scope this literal is in. If not None, all arguments will be looked up in this Scope befor the Literal is created.
        negated -- True if the literal is negated.
        """
        assert _scope is None or isinstance(_scope, scope.Scope)
        self.predicate = predicate
        self.negated = negated
        self.scope = _scope
        assert isinstance(predicate, Function), "not a function: %s" % str(predicate)

        self.hash = None

        if _scope:
            self.args = _scope.lookup(args)
        else:
            self.args = [Term(a) for a in args]

    def visit(self, fn):
        return fn(self, [])
            
    def pddl_str(self, instantiated=True):
        """Return a pddl text representation of this Literal.
        
        Arguments:
        instantiated -- if True (which is the default) resolves
        instantiated Parameters before printing the string."""
        s = "(%s %s)" % (self.predicate.name, " ".join(a.pddl_str(instantiated) for a in self.args))
        if self.negated:
            return "(not %s)" % s
        return s

    def negate(self):
        """Return the negated version of this Literal."""
        return self.__class__(self.predicate, self.args[:], None, not self.negated)

    def copy(self, new_scope=None, new_parts=None, copy_instance=False):
        """Create a copy of this Literal.

        Arguments:
        new_scope -- if not None, the arguments will be looked up in this Scope.
        copy_instance -- if True, return a Literal with all instantiated Parameters resolved.
        """
        if copy_instance:
            l = self.copy_instance()
            if new_scope:
                l.set_scope(new_scope)
            return l
        if not new_scope:
            new_scope = self.scope
        
        return self.__class__(self.predicate, self.args, new_scope, self.negated)

    def new_literal(self, predicate=None, args=None, scope=False, negated=None):
        if predicate is None:
            predicate = self.predicate
        if args is None:
            args = self.args
        if scope is False:
            scope = self.scope
        if negated is None:
            negated = self.negated
        return self.__class__(predicate, args, scope, negated)

    def set_scope(self, new_scope):
        self.scope = new_scope
        self.args = new_scope.lookup(self.args)
    
    def copy_instance(self):
        """Return a copy of this Literal where all instantiated
        Parameters are resolved to their ground objects."""
        return self.__class__(self.predicate, [a.copy_instance() for a in self.args], negated=self.negated)

    def collect_arguments(self):
        """Return a list of all arguments of this Literal, including
        all arguments of any nested FunctionTerms"""
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
        if not self.hash:
            self.hash = hash((self.predicate, self.negated ) + tuple(self.args))
        return self.hash
        
    @staticmethod
    def parse(it, scope, negate=False, maxNesting=999, function_scope=scope.SCOPE_ALL):
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
                args.append(Term.parse(it, scope, maxNesting, function_scope))
            except parser.EndOfListError:
                break

        predicate = scope.predicates.get(first.string, args, function_scope)
        if not predicate:
            type_str = " ".join(str(a.get_type()) for a in args)
            candidates = scope.predicates[first.string]
            c_str = "\n  ".join(str(p) for p in candidates if p.function_scope & function_scope)
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
    """This is an abstract superclass for all terms in a PDDL
    functional expression."""
    
    def __init__(self, *params):
        """Return different subclasses of Term, depending on
        arguments:

        One Term: Returns a copy of that Term
        One Parameter: Returns a VariableTerm with the Parameter as the argument
        One TypedObject: Returns a ConstantTerm with the TypedObject as the argument
        One int or float: Returns a ConstantTerm with a new TypedNumber of the argument

        One Function and a list of arguments: Returns a FunctionTerm
        """
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
        """visit this Term and all its children.

        Arguments:
        fn -- visitor function that will be called with the current
        Term and a list of results of previous calls."""
        
        return fn(self, [])

    def copy(self, new_scope=None):
        raise NotImplementedError()
    
    def pddl_str(self, instantiated=True):
        """Return a pddl representation of this Term.
        
        Arguments:
        instantiated -- if True (which is the default) resolves
        instantiated Parameters before printing the string."""
        
        def printVisitor(term, results=[]):
            if isinstance(term, VariableTerm) and term.is_instantiated() and instantiated:
                term = term.get_instance()
                
            if term.__class__ == FunctionTerm:
                return "(%s %s)" % (term.function.name, " ".join(results))
            elif isinstance(term, Term):
                return str(term.object.name)
            return str(term.name)
        return self.visit(printVisitor)
    
    def get_type(self):
        """Return the PDDL type of this Term"""
        raise NotImplementedError()
    
    def is_instance_of(self, type):
        """Return True if the PDDL type of this Term is equal to
        "type" or a subtype."""
        return self.get_type().equal_or_subtype_of(type)

    def __eq__(self, other):
        return self.__class__ == other.__class__

    def __ne__(self, other):
        return not self.__eq__(other)
    
    @staticmethod
    def parse(it, scope, maxNesting=999, function_scope=scope.SCOPE_ALL):
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
        
        return FunctionTerm.parse(iter(term), scope, maxNesting-1, function_scope)

    
class FunctionTerm(Term):
    """This class represents a function term. It is similar to a
    Literal, but it can be nested inside other Terms or Literals.
    """
    
    def __init__(self, function, args, scope=None):
        """Create a new FunctionTerm object.

        Arguments:
        predicate -- the Function of this term.
        args -- list of TypesObjects or Terms that are arguments of this function.
        scope -- Scope this function term is in. If not None, all arguments will be looked up in this Scope befor the FunctionTerm is created.
        """
        self._function = function

        if scope:
            self._args = scope.lookup(args)
        else:
            assert all(isinstance(a, Term) for a in args)
            self._args = args

    args = property(lambda self: self._args)
    function = property(lambda self: self._function)
            
    def get_type(self):
        """Return the PDDL type of this Term. For FunctionTerms, this is always a FunctionType."""
        return FunctionType(self.function.type)
    
    def copy_instance(self):
        """Return a copy of this FunctionTerm where all instantiated
        Parameters are resolved to their ground objects."""
        return FunctionTerm(self.function, [a.copy_instance() for a in self.args])

    def visit(self, fn):
        """visit this Term and all its children.

        Arguments:
        fn -- visitor function that will be called with the current
        Term and a list of results of previous calls."""
        
        return fn(self, [a.visit(fn) for a in self.args])

    def copy(self, new_scope=None):
        return FunctionTerm(self.function, [a.copy(new_scope) for a in self.args], new_scope)
    
    def __str__(self):
        return "FunctionTerm: %s(%s)" % (self.function.name, " ".join(str(s) for s in self.args))

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.function == other.function and all(map(lambda a,b: a==b, self.args, other.args))

    def __hash__(self):
        return hash((self.function, ) + tuple(self.args))

    @staticmethod
    def parse(it, scope, maxNesting=999, function_scope=scope.SCOPE_ALL):
        name = it.get(None, "function or predicate identifier")
        if name.token.string in scope.functions:
            table = scope.functions
        elif name.token.string in scope.predicates:
            table = scope.predicates
        else:
            raise ParseError(name.token, "Unknown function or predicate: '%s'" % name.token.string)
        
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

        func = table.get(name.token.string, args, function_scope)
        if not func:
            type_str = " ".join(str(a.get_type()) for a in args)
            candidates = table[name.token.string]
            c_str = "\n  ".join(str(p) for p in candidates if p.function_scope & function_scope)
            raise ParseError(name.token, "no matching function or predicate found for (%s %s). Candidates are:\n  %s" % (name.token.string, type_str, c_str))
        
        if func.is_modal():
            raise ParseError(name.token, "nested modal predicates are not allowed.")

        return FunctionTerm(func, args)

class VariableTerm(Term):
    """This class represents a variable term. It is always bound to a
    Parameter object, which can be accessed by the "object" member.
    """
    
    def __init__(self, parameter):
        """Create a new VariableTerm object.

        Arguments:
        parameter -- The Parameter object that this term refers to
        """
        assert isinstance(parameter, Parameter)
        self._object = parameter

    object = property(lambda self: self._object)
        
    def is_instantiated(self):
        """Returns True if this terms parameter is instantiated."""
        return self.object.is_instantiated()

    def get_instance(self):
        """Returns the ground object if this terms parameter is
        instantiated."""
        if self._object.is_instantiated():
            return self._object.get_instance()
        raise Exception, "Term %s is not instantiated" % str(self.object)

    def copy_instance(self):
        """Returns a ConstantTerm referring to the ground object if
        this terms parameter is instantiated. Return a copy of this
        VariableTerm otherwise."""
        if self.is_instantiated():
            if isinstance(self.get_instance(), TypedObject):
                return Term(self.get_instance())
            return self.get_instance().copy_instance()
        return VariableTerm(self.object)
    
    def get_type(self):
        """Return the PDDL type of this Term. This is the type of the
        terms parameter."""
        return self.object.type

    def copy(self, new_scope=None):
        if new_scope:
            return VariableTerm(new_scope[self.object])
        return VariableTerm(self.object)

    def __str__(self):
        if self.is_instantiated():
            return "VariableTerm: %s (instance: %s)" % (str(self.object), str(self.object.get_instance()))
        return "VariableTerm: %s" % str(self.object)
    
    def __eq__(self, other):
        if isinstance(other, VariableTerm):
            return self.object == other.object
        return self.object == other
    
    def __hash__(self):
        return hash((self.__class__, self.object))
    
class FunctionVariableTerm(FunctionTerm, VariableTerm):
    """This class represents a variable term with a function type. It
    is always bound to a Parameter object, which can be accessed by
    the "object" member.

    If the parameter is instantiated with a FunctionTerm, it will
    behave like a FunctionTerm.
    """
    
    def __init__(self, parameter):
        """Create a new FunctionVariableTerm object.

        Arguments:
        parameter -- The Parameter object that this term refers to
        """
        self._object = parameter

    object = property(lambda self: self._object)
        
    def get_type(self):
        """Return the PDDL type of this Term. This is the type of the
        terms parameter."""
        return self.object.type

    def visit(self, fn):
        """visit this Term and all its children. If this terms
        Parameter is instantiated, it will visit the FunctionTerm it
        is instantiated with.

        Arguments:
        fn -- visitor function that will be called with the current
        Term and a list of results of previous calls."""
        
        if self.is_instantiated():
            return self.object.get_instance().visit(fn)
        return fn(self, [])
    
    def copy_instance(self):
        """Returns a FunctionTerm referring to the instantiated term
        if this terms parameter is instantiated. Return a copy of this
        VariableTerm otherwise."""
        if self.is_instantiated():
            return FunctionTerm(self.get_instance().function, [a for a in self.get_instance().args])
        return FunctionVariableTerm(self.object)

    def copy(self, new_scope=None):
        if new_scope:
            return FunctionVariableTerm(new_scope[self.object])
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
    """This class represents a constant term. It is always bound to a
    TypedObject, which can be accessed by the "object" member.
    """
    
    def __init__(self, obj):
        """Create a new ConstantTerm object.

        Arguments:
        obj -- The TypedObject that this term refers to
        """
        assert isinstance(obj, TypedObject)
        self._object = obj

    object = property(lambda self: self._object)
        
    def get_type(self):
        """Return the PDDL type of this Term. This is the type of the
        object."""
        return self.object.type

    def copy_instance(self):
        """Returns a copy of this term."""
        return ConstantTerm(self.object)

    def copy(self, new_scope=None):
        if new_scope:
            return ConstantTerm(new_scope[self.object])
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
    """This visitor collects all TypedObjects that occur in this Term
    (and it's children) and returns them as a list."""
    
    if isinstance(term, (VariableTerm, ConstantTerm)):
        return [term.object]
    if isinstance(term, FunctionTerm):
        return sum(results, [])
