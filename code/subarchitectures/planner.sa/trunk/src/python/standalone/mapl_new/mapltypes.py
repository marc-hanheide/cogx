#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError

class Type(object):
    def __init__(self, typename, supertypes=None):
        self.name = typename
        
        if supertypes is None:
            supertypes = [objectType]
        self.supertypes = set(supertypes)

    def isSubtypeOf(self, other):
        if self.__class__ != other.__class__:
            return other.isSupertypeOf(self)
        
        if other in self.supertypes:
            return True
        return any(sup.isSubtypeOf(other) for sup in self.supertypes)

    def equalOrSubtypeOf(self, other):
        if self.__class__ != other.__class__:
            return other.equalOrSupertypeOf(self)
        
        if other in self.supertypes or self == other:
            return True
        return any(sup.isSubtypeOf(other) for sup in self.supertypes)
    
    def isSupertypeOf(self, other):
        return other.isSubtypeOf(self)

    def equalOrSupertypeOf(self, other):
        return other.equalOrSubtypeOf(self)
    
    def __str__(self):
        return self.name

    def __hash__(self):
        return hash((self.__class__, self.name))

    def __eq__(self, other):
        try:
            return self.name == other.name
        except:
            return False;

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def parse(it, types, scope=None):
        if isinstance(it, parser.Element):
            next = it
        else:
            next = it.get(None, "type specification")
            
        if next.isTerminal():
            if next.token.string not in types:
                raise ParseError(next.token, "Unknown type: '%s'" % next.token.string)
            return types[next.token.string]
        
        j = iter(next)
        first = j.get("terminal").token
        if first.string == "either":
            ctypes = []
            for elem in j:
                ctypes.append(Type.parse(elem, types))
            return CompositeType(ctypes)

        elif first.string == "function":
            ftype = Type.parse(j, types)
            j.noMoreTokens()
            return FunctionType(ftype)
        elif first.string == "typeof" and scope:
            param = j.get("terminal", "parameter").token
            j.noMoreTokens()
            if param.string not in scope:
                raise ParseError(param, "Unknown identifier: '%s'" % param.string)
            return ProxyType(scope[param.string])
            

class CompositeType(Type):
    def __init__(self, types):
        self.name = "-".join(t.name for t in types)
        self.types = types

    def isSubtypeOf(self, other):
        if isinstance(other, CompositeType):
            return other.isSupertypeOf(self)

#        print self, other, all(map(lambda t: t.equalOrSubtypeOf(other), self.types))
        return all(t.equalOrSubtypeOf(other) for t in self.types)

    def equalOrSubtypeOf(self, other):
        return self == other or self.isSubtypeOf(other)
    
    def isSupertypeOf(self, other):
        if isinstance(other, CompositeType):
            strictSupertype = any(any(t.isSupertypeOf(t2) for t2 in other.types) for t in self.types)
            return all(self.equalOrSupertypeOf(t) for t in other.types) and strictSupertype
        
        return any(t.equalOrSupertypeOf(other) for t in self.types)

    def equalOrSupertypeOf(self, other):
        return self == other or self.isSupertypeOf(other)
        
    def __str__(self):
        return "(either %s)" % " ".join(t.name for t in self.types)

    def __hash__(self):
        return hash((self.__class__,)+tuple(self.types))
    
    def __eq__(self, other):
        try:
            return all(map(lambda s,o: s == o, self.types, other.types))
        except:
            return False;

class FunctionType(Type):
    def __init__(self, type):
        self.name = "function(%s)" % str(type)
        self.type = type

    def isSubtypeOf(self, other):
        if isinstance(other, FunctionType):
            return self.type.isSubtypeOf(other.type)
        return self.type.equalOrSubtypeOf(other)

    def equalOrSubtypeOf(self, other):
        if isinstance(other, FunctionType):
            return self.type.equalOrSubtypeOf(other.type)
        return self.type.equalOrSubtypeOf(other)
    
    def isSupertypeOf(self, other):
        if isinstance(other, FunctionType):
            return self.type.isSupertypeOf(other.type)
        return False

    def equalOrSupertypeOf(self, other):
        if isinstance(other, FunctionType):
            return self.type.equalOrSupertypeOf(other.type)
        return False
        
    def __str__(self):
        return "(function of %s)" % str(self.type)

    def __hash__(self):
        return hash((self.__class__, self.type))
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.type == other.type

class ProxyType(Type):
    def __init__(self, param):
        assert isinstance(param.type, FunctionType)
        self.name = "typeof(%s)" % str(param.name)
        self.parameter = param

    def effectiveType(self):
        if self.parameter.isInstantiated():
            return self.parameter.getInstance().function.type
        return self.parameter.type.type
        
    def isSubtypeOf(self, other):
        return self.effectiveType().isSubtypeOf(other)

    def equalOrSubtypeOf(self, other):
        return self.effectiveType().equalOrSubtypeOf(other)
    
    def isSupertypeOf(self, other):
        return self.effectiveType().isSupertypeOf(other)

    def equalOrSupertypeOf(self, other):
        return self.effectiveType().equalOrSupertypeOf(other)
        
    def __str__(self):
        return "(type of %s)" % self.parameter.name

    def __hash__(self):
        return hash((self.__class__, self.parameter))
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.parameter == other.parameter

class AnyType(Type):
    def __init__(self, name="any"):
        self.name = name

    def isSubtypeOf(self, other):
        return True

    def equalOrSubtypeOf(self, other):
        return True
    
    def isSupertypeOf(self, other):
        return True

    def equalOrSupertypeOf(self, other):
        return True
        
    def __eq__(self, other):
        return isinstance(other, Type)
    
#basic types for all pddl representations
objectType = Type("object", [])
numberType = Type("number", [])
booleanType = Type("boolean", [objectType])

default_types = [objectType, booleanType]

#basic mapl types
agentType = Type("agent")
planningAgentType = Type("planning_agent", [agentType])
phys_objType = Type("phys_obj")
subgoalType = Type("subgoal")
featureType = Type("feature")

mapl_types = [agentType, planningAgentType, phys_objType, subgoalType, featureType]

class TypedObject(object):
    def __init__(self, name, _type):
        if type(name) in (int, float, long):
            self.__class__ = TypedNumber
            TypedNumber.__init__(self, name)
            return
        self.name = name
        self.type = _type

    def isInstanceOf(self, type):
        return self.type.equalOrSubtypeOf(type)

    def copy(self):
        return self.__class__(self.name, self.type)

    def __str__(self):
        return "%s - %s" % (self.name, self.type)

    def __hash__(self):
        return hash((self.__class__, self.name, self.type))

    def __eq__(self, other):
        try:
            return self.name == other.name and self.type == other.type
        except:
            return False;

    def __ne__(self, other):
        return not self.__eq__(other)

class TypedNumber(TypedObject):
    def __init__(self, number):
        self.value = number
        self.type = numberType

    name = property(lambda self: self.value)

    def copy(self):
        return self.__class__(self.value)

    def __str__(self):
        return "%f" % self.value

    def __eq__(self, other):
        try:
            return self.value == other.value and self.type == other.type
        except:
            return self.value == other
        

TRUE = TypedObject("true", booleanType)
FALSE = TypedObject("false", booleanType)

UNKNOWN = TypedObject("unknown", AnyType())
UNDEFINED = TypedObject("undefined", AnyType())

class Parameter(TypedObject):
    def __init__(self, name, type):
        assert name[0] == "?"
        self.name = name
        self.type = type
        self.instantiated = None

    def instantiate(self, value):
        if value is not None:
            assert value.isInstanceOf(self.type)
        self.instantiated = value

    def isInstantiated(self):
        return self.instantiated is not None

    def getInstance(self):
        return self.instantiated
    
    # def __hash__(self):
    #     return hash(id(self))

    # def __eq__(self, other):
    #     return id(self) == id(other)

        
    
def parse_typelist(it):
    types = {}

    def checkFunc(elem):
        if not elem.isTerminal():
            raise UnexpectedTokenError(elem.token, "identifier")
        return elem.token
    
    for subtypes, super in parser.parseTypedList(it, checkFunc, checkFunc):
        if super is None:
            super = parser.Token("object", 0, None)
        for type in subtypes:
            types[type] = super
    
    return types
    
