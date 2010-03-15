#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError

class Type(object):
    def __init__(self, typename, supertypes=None):
        self.name = typename
        
        if supertypes is None:
            supertypes = [t_object]
        self.supertypes = set(supertypes)

    def get_supertypes(self):
        return self.supertypes | reduce(lambda a,b: a|b, (t.get_supertypes() for t in self.supertypes), set())

    def is_subtype_of(self, other):
        if self.__class__ != other.__class__:
            return other.is_supertype_of(self)
        
        if other in self.supertypes:
            return True
        return any(sup.is_subtype_of(other) for sup in self.supertypes)

    def equal_or_subtype_of(self, other):
        if self.__class__ != other.__class__:
            return other.equal_or_supertype_of(self)
        
        if other in self.supertypes or self == other:
            return True
        return any(sup.is_subtype_of(other) for sup in self.supertypes)
    
    def is_supertype_of(self, other):
        return other.is_subtype_of(self)

    def equal_or_supertype_of(self, other):
        return other.equal_or_subtype_of(self)
    
    def __str__(self):
        return self.name

    def __hash__(self):
        return hash((self.__class__, self.name))

    def __eq__(self, other):
        try:
            return self.name == other.name
        except:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def parse(it, types, scope=None):
        if isinstance(it, parser.Element):
            next = it
        else:
            next = it.get(None, "type specification")
            
        if next.is_terminal():
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
            j.no_more_tokens()
            return FunctionType(ftype)
        elif first.string == "typeof" and scope:
            param = j.get("terminal", "parameter").token
            j.no_more_tokens()
            if param.string not in scope:
                raise ParseError(param, "Unknown identifier: '%s'" % param.string)
            return ProxyType(scope[param.string])
            

class CompositeType(Type):
    def __init__(self, types):
        self.name = "-".join(t.name for t in types)
        self.types = types

    def is_subtype_of(self, other):
        if isinstance(other, CompositeType):
            return other.is_supertype_of(self)

#        print self, other, all(map(lambda t: t.equal_or_subtype_of(other), self.types))
        return all(t.equal_or_subtype_of(other) for t in self.types)

    def equal_or_subtype_of(self, other):
        return self == other or self.is_subtype_of(other)
    
    def is_supertype_of(self, other):
        if isinstance(other, CompositeType):
            strictSupertype = any(any(t.is_supertype_of(t2) for t2 in other.types) for t in self.types)
            return all(self.equal_or_supertype_of(t) for t in other.types) and strictSupertype
        
        return any(t.equal_or_supertype_of(other) for t in self.types)

    def equal_or_supertype_of(self, other):
        return self == other or self.is_supertype_of(other)
        
    def __str__(self):
        return "(either %s)" % " ".join(t.name for t in self.types)

    def __hash__(self):
        return hash((self.__class__,)+tuple(self.types))
    
    def __eq__(self, other):
        try:
            return all(map(lambda s,o: s == o, self.types, other.types))
        except:
            return False

class FunctionType(Type):
    def __init__(self, type):
        self.name = "function(%s)" % str(type)
        self.type = type

    def is_subtype_of(self, other):
        if isinstance(other, FunctionType):
            return self.type.is_subtype_of(other.type)
        return self.type.equal_or_subtype_of(other)

    def equal_or_subtype_of(self, other):
        if isinstance(other, FunctionType):
            return self.type.equal_or_subtype_of(other.type)
        return self.type.equal_or_subtype_of(other)
    
    def is_supertype_of(self, other):
        if isinstance(other, FunctionType):
            return self.type.is_supertype_of(other.type)
        return False

    def equal_or_supertype_of(self, other):
        if isinstance(other, FunctionType):
            return self.type.equal_or_supertype_of(other.type)
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

    def effective_type(self):
        if self.parameter.is_instantiated():
            return self.parameter.get_instance().function.type
        return self.parameter.type.type
        
    def is_subtype_of(self, other):
        return self.effective_type().is_subtype_of(other)

    def equal_or_subtype_of(self, other):
        return self.effective_type().equal_or_subtype_of(other)
    
    def is_supertype_of(self, other):
        return self.effective_type().is_supertype_of(other)

    def equal_or_supertype_of(self, other):
        return self.effective_type().equal_or_supertype_of(other)
        
    def __str__(self):
        return "(type of %s)" % self.parameter.name

    def __hash__(self):
        return hash((self.__class__, self.parameter))
    
    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.parameter == other.parameter

class AnyType(Type):
    def __init__(self, name="any"):
        self.name = name

    def is_subtype_of(self, other):
        return True

    def equal_or_subtype_of(self, other):
        return True
    
    def is_supertype_of(self, other):
        return True

    def equal_or_supertype_of(self, other):
        return True
        
    def __eq__(self, other):
        return isinstance(other, Type)
    
class TypedObject(object):
    def __init__(self, name, _type):
        if type(name) in (int, float, long):
            self.__class__ = TypedNumber
            TypedNumber.__init__(self, name)
            return
        self.name = name
        self.type = _type
        self.hash = hash((self.__class__, self.name, self.type))

    def is_instance_of(self, type):
        return self.type.equal_or_subtype_of(type)

    def copy(self):
        return self.__class__(self.name, self.type)

    def __str__(self):
        return "%s - %s" % (self.name, self.type)

    def __hash__(self):
        return self.hash

    def __eq__(self, other):
        try:
            return self.name == other.name and self.type == other.type
        except:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

class TypedNumber(TypedObject):
    def __init__(self, number):
        self.value = number
        self.type = t_number
        self.hash = hash((self.__class__, self.name, self.type))

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
        
class Parameter(TypedObject):
    def __init__(self, name, type):
        assert name[0] == "?"
        self.name = name
        self.type = type
        self.instantiated = None
        self.hash = hash((self.__class__, self.name, self.type))

    def instantiate(self, value):
        if value is not None:
            assert value.is_instance_of(self.type)
        self.instantiated = value

    def is_instantiated(self):
        return self.instantiated is not None

    def get_instance(self):
        return self.instantiated
    
    # def __hash__(self):
    #     return hash(id(self))

    # def __eq__(self, other):
    #     return id(self) == id(other)


#basic types for all pddl representations
t_object = Type("object", [])
t_number = Type("number", [])
t_boolean = Type("boolean", [t_object])

#predefined constants

TRUE = TypedObject("true", t_boolean)
FALSE = TypedObject("false", t_boolean)

UNKNOWN = TypedObject("unknown", AnyType())
UNDEFINED = TypedObject("undefined", AnyType())

   
def parse_typelist(it):
    types = {}

    def check_func(elem):
        if not elem.is_terminal():
            raise UnexpectedTokenError(elem.token, "identifier")
        return elem.token
    
    for subtypes, super in parser.parse_typed_list(it, check_func, check_func):
        if super is None:
            super = parser.Token("object", 0, None)
        for type in subtypes:
            types[type] = super
    
    return types
    
