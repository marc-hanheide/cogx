#! /usr/bin/env python
# -*- coding: latin-1 -*-

import parser
from parser import ParseError, UnexpectedTokenError

class Type(object):
    """This class represents (normal) PDDL types. Each type can have a
    unlimited number of supertypes, the default supertype is t_object.
    """
    
    def __init__(self, typename, supertypes=None):
        """Create a new Type object.

        Arguments:
        typename -- the name of the type
        supertypes -- list of Type objects that are supertypes of this
        Type. Defaults to [t_object].
        """
        self.name = typename
        
        if supertypes is None:
            supertypes = [t_object]
        self.supertypes = set(supertypes)

    def get_supertypes(self):
        """Returns a list of all supertypes of this type. This will also return
        supertypes of supertypes and so on.
        """
        return self.supertypes | reduce(lambda a,b: a|b, (t.get_supertypes() for t in self.supertypes), set())

    def is_subtype_of(self, other):
        """Returns True if self is a proper subtype of other.

        Arguments:
        other -- Type object.
        """
        
        if self.__class__ != other.__class__:
            return other.is_supertype_of(self)
        
        if other in self.supertypes:
            return True
        return any(sup.is_subtype_of(other) for sup in self.supertypes)

    def equal_or_subtype_of(self, other):
        """Returns True if self is equal to or a a subtype of other.

        Arguments:
        other -- Type object.
        """
        
        if self.__class__ != other.__class__:
            return other.equal_or_supertype_of(self)
        
        if other in self.supertypes or self == other:
            return True
        return any(sup.is_subtype_of(other) for sup in self.supertypes)
    
    def is_supertype_of(self, other):
        """Returns True if self is a proper supertype of other.

        Arguments:
        other -- Type object.
        """
        return other.is_subtype_of(self)

    def equal_or_supertype_of(self, other):
        """Returns True if self is equal to or a a supertype of other.

        Arguments:
        other -- Type object.
        """
        return other.equal_or_subtype_of(self)

    def is_compatible(self, other):
        """Returns True if self is either equal to, a subtype or a supertype of other.

        Arguments:
        other -- Type object.
        """
        return self.equal_or_subtype_of(other) or self.equal_or_supertype_of(other)
    
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
    """This class represents a PDDL type that is composed of other
    types using the "(either a b c)" syntax.
    """
    
    def __init__(self, types):
        """Create a new CompositeType object.

        Arguments:
        types -- a list of Type objects that this CompositeType is composed of
        """
        
        self.name = "-".join(t.name for t in types)
        self.types = types

    def is_subtype_of(self, other):
        """Returns True if self is a proper subtype of other.

        Arguments:
        other -- Type object.
        """
        
        if isinstance(other, CompositeType):
            return other.is_supertype_of(self)

#        print self, other, all(map(lambda t: t.equal_or_subtype_of(other), self.types))
        return all(t.equal_or_subtype_of(other) for t in self.types)

    def equal_or_subtype_of(self, other):
        """Returns True if self is equal to or a a subtype of other.

        Arguments:
        other -- Type object.
        """
        return self == other or self.is_subtype_of(other)
    
    def is_supertype_of(self, other):
        """Returns True if self is a proper supertype of other.

        Arguments:
        other -- Type object.
        """
        
        if isinstance(other, CompositeType):
            strictSupertype = any(any(t.is_supertype_of(t2) for t2 in other.types) for t in self.types)
            return all(self.equal_or_supertype_of(t) for t in other.types) and strictSupertype
        
        return any(t.equal_or_supertype_of(other) for t in self.types)

    def equal_or_supertype_of(self, other):
        """Returns True if self is equal to or a a supertype of other.

        Arguments:
        other -- Type object.
        """
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
    """This class represents a function type as specified by the
    "(function x)" syntax. Function types are used to model modal
    predicates/actions/axioms in MAPL.
    """
    
    def __init__(self, type):
        """Create a new FunctionType object.

        Arguments:
        type -- a Type object that specifies the type of function this
        FunctionType represents.
        """
        
        self.name = "function(%s)" % str(type)
        self.type = type

    def is_subtype_of(self, other):
        """Returns True if self is a proper subtype of other.

        Arguments:
        other -- Type object.
        """

        if isinstance(other, FunctionType):
            return self.type.is_subtype_of(other.type)
        return self.type.equal_or_subtype_of(other)

    def equal_or_subtype_of(self, other):
        """Returns True if self is equal to or a a subtype of other.

        Arguments:
        other -- Type object.
        """
        if isinstance(other, FunctionType):
            return self.type.equal_or_subtype_of(other.type)
        return self.type.equal_or_subtype_of(other)
    
    def is_supertype_of(self, other):
        """Returns True if self is a proper supertype of other.

        Arguments:
        other -- Type object.
        """
        
        if isinstance(other, FunctionType):
            return self.type.is_supertype_of(other.type)
        return False

    def equal_or_supertype_of(self, other):
        """Returns True if self is equal to or a a supertype of other.

        Arguments:
        other -- Type object.
        """
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
    """This class represents a proxy type as specified by the "(typeof
    ?x)" syntax. It is used to refer to types of function parameters
    which might not be known at the time of parsing.

    A ProxyType created with ProxyType(param) (with param.type being a
    FunctionType) will behave like the base type of param when param is not
    instantiated. When param is instantiated with a FunctionTerm, it
    will behave like the type of this FunctionTerm's function.
    """
    
    def __init__(self, param):
        """Create a new ProxyType object.

        Arguments:
        param -- the Parameter object that this ProxyType refers to.
        """
        assert isinstance(param.type, FunctionType)
        self.name = "typeof(%s)" % str(param.name)
        self.parameter = param

    def effective_type(self):
        """Return the base type of param if param is not instantiated.
        Return the type of the function of param's instance if it is instantiated"""
        
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
    """This class is used to represent types that match all other
    types. Used to model the "unknown" constant.
    """
    
    def __init__(self, name="any"):
        """Create a new AnyType object.

        Arguments:
        name -- the name of the type
        """
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
    """This class represents all constants or objects in a PDDL
    domain.
    """
    
    def __init__(self, name, _type):
        """Create a new TypedObject.

        Arguments:
        name -- name of the object
        _type -- Type object that represents the type of the object.
        """
        if type(name) in (int, float, long):
            self.__class__ = TypedNumber
            TypedNumber.__init__(self, name)
            return
        self._name = name
        self._type = _type
        self.hash = hash((self.name, self.type))

    name = property(lambda self: self._name)
    type = property(lambda self: self._type)

    def rename(self, name):
        self._name = name
        self.hash = hash((self.name, self.type))

    def change_type(self, type):
        assert self.type.equal_or_subtype_of(type) or type.equal_or_subtype_of(self.type), "invalid type change from %s to %s." % (str(self.type), str(type))
        self._type = type
        self.hash = hash((self.name, self.type))

    def get_type(self):
        return self.type
        
    def is_instance_of(self, type):
        """Returns true if this TypedObject's type is equal to "type"
        or a subtype."""
        return self.type.equal_or_subtype_of(type)

    def copy(self):
        """Create a copy of the object."""
        return self.__class__(self.name, self.type)

    def __str__(self):
        # return "%s - %s" % (self.name, self.type)
        return "%s" % (self.name)

    def __hash__(self):
        return self.hash

    def __eq__(self, other):
        return isinstance(other, TypedObject) and self.hash == other.hash

    def __ne__(self, other):
        return not isinstance(other, TypedObject) or self.hash != other.hash

class TypedNumber(TypedObject):
    """Specialized subclass of TypedObject to represent numeric
    values. It will automatically have a type of t_number.
    """
    def __init__(self, number):
        """Create a new numberic object.

        Arguments:
        value -- number represented by the new object.
        """
        self._value = number
        self._type = t_number
        self.hash = hash((self._value, self.type))

    value = property(lambda self: self._value)
    name = property(lambda self: self._value)

    def copy(self):
        return self.__class__(self.value)

    def __str__(self):
        return "%f" % self.value
    
class Parameter(TypedObject):
    """Subclass of TypedObject that represents parameters of actions or
    quantified conditions/effects.

    A Parameter object can be instantiated with a object whose type is
    compatible with the type of the Parameter.
    """
    
    def __init__(self, name, type):
        """Create a new Parameter.

        Arguments:
        name -- name of the parameter (needs to start with a "?")
        type -- Type object of the Parameter's type
        """
        assert name[0] == "?"
        self._name = name
        self._type = type
        self.instantiated = None
        self.hash = hash((self.__class__, self.name, self.type))

    name = property(lambda self: self._name)
    type = property(lambda self: self._type)

    def instantiate(self, value):
        """Instantiate this Parameter with the object value. Throws an
        exception if the type of value is not compatible with the
        Parameter.

        Arguments:
        value -- TypedObject the Parameter should be instantiated with.
        """
        
        if value is not None:
            assert value.is_instance_of(self.type), "%s not of type %s" % (str(value.get_type()), str(self.type))
        self.instantiated = value

    def is_instantiated(self):
        """Returns True if this Parameter is instantiated"""
        return self.instantiated is not None

    def get_instance(self):
        """Returns the TypedObject this Parameter is instantiated with
        (None if it is not instantiated)."""
        return self.instantiated

    def __str__(self):
        return "%s - %s" % (self.name, self.type)
    
    # def __hash__(self):
    #     return hash(id(self))

    # def __eq__(self, other):
    #     return id(self) == id(other)


#basic types for all pddl representations
t_object = Type("object", [])
t_number = Type("number", [])
t_boolean = Type("boolean", [t_object])
t_any = AnyType()

#predefined constants

TRUE = TypedObject("true", t_boolean)
FALSE = TypedObject("false", t_boolean)

UNKNOWN = TypedObject("unknown", t_any)
UNDEFINED = TypedObject("undefined", t_any)

   
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
    
