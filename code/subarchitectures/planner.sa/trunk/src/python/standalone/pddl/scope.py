#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools

import mapltypes as types
import predicates

SCOPE_CONDITION = 1
SCOPE_EFFECT = 2
SCOPE_INIT = 4
SCOPE_ALL = 0xffffffff


class FunctionTable(dict):
    """This class is used to store and retrieve PDDL Function objects
    according to name and argument types."""
    
    def __init__(self, functions=[]):
        """Create a new FunctionTable.

        Arguments:
        functions -- List of Function objects this table should contain."""
        for f in functions:
            self.add(f)

    def copy(self):
        """Create a copy of this table."""
        c =  FunctionTable()
        for f in self:
            c.add(f)
        return c

    def add(self, function):
        """Add a new Function to the table. If a function with
        identical name and arguments exists, an Exception will be
        raised.

        Arguments:
        function -- Function object or list of Function objects .
        """
        if isinstance(function, (list, tuple, set)):
            for f in function:
                self.add(f)
            return
                
        if function.name not in self:
            dict.__setitem__(self, function.name, set())
        else:
            if self.get(function.name, function.args, function.function_scope):
                raise Exception("A function with this name and arguments already exists: " + str(function))

        dict.__getitem__(self, function.name).add(function)
        

    def remove(self, function):
        """Remove Functions from this table.

        Arguments:
        function -- Function object or list of Function objects .
        """
        if isinstance(function, (list, tuple, set)):
            for f in function:
                self.remove(f)
            return
        
        if function not in self:
            return
        
        fs = dict.__getitem__(self, function.name)
        fs.remove(function)
        if not fs:
            del self[function.name]
                
        
    def get(self, name, args, function_scope=SCOPE_ALL):
        """Get all functions matching the provided name and argument
        types and which can be used in the given scope.

        If excactly one function matches the supplied information, it
        will be returned. Otherwise a list of matching Functions will
        be returned.

        Arguments:
        name -- the name of the function to look for
        args -- a list of Terms, TypedObjects or Types that describe
        the arguments of the function that is searched."""
        
        if name not in self:
            return []
        
        fs = dict.__getitem__(self, name.lower())
        
        argtypes = []
        for arg in args:
            if isinstance(arg, predicates.Term):
                argtypes.append(arg.get_type())
            elif isinstance(arg, types.TypedObject):
                argtypes.append(arg.type)
            elif isinstance(arg, types.Type):
                argtypes.append(arg)
            else:
                raise Exception("Wrong type for argument list:" + str(type(arg)))
                
        result = []
#        print name, map(str, args)
        for f in fs:
            if not (f.function_scope & function_scope):
                continue
            #in case one tries to check the function argument, too
            if len(argtypes) == len(f.args):
                funcargs = {}
                matches = True
                for t, arg in zip(argtypes, f.args):
                    argtype = arg.type
                    #make sure that "typeof(?f)" parameters are checked correctly
                    if isinstance(arg.type, types.ProxyType) and arg.type.parameter in funcargs:
                        argtype = funcargs[arg.type.parameter]
                        
                    if not t.equal_or_subtype_of(argtype):
                        matches = False
                        break
                    
                    if isinstance(arg.type, types.FunctionType):
                        funcargs[arg] = t.type
                if matches:
#            if len(argtypes) == len(f.args) and all(map(lambda t, fa: , argtypes, f.args)):
#                print f
#                print map(lambda t, fa: "%s <= %s: %s" %(str(t), str(fa.type),str(t.equal_or_subtype_of(fa.type))), argtypes, f.args)
                    result.append(f)

        if len(result) == 1:
            return iter(result).next()
        return list(result)

    def __iter__(self):
        return itertools.chain(*self.itervalues())

    def __contains__(self, key):
        if isinstance(key, predicates.Function):
            if dict.__contains__(self, key.name):
                return key in dict.__getitem__(self, key.name)
            return False
        
        key = key.lower()
        return dict.__contains__(self, key)
        
    def __getitem__(self, key):
        if isinstance(key, predicates.Function):
            key = key.name
        key = key.lower()

        result = dict.__getitem__(self, key)
        #if len(result) == 1:
        #    return iter(result).next()
        return list(result)
        
    def __setitem__(self, key, value):
        raise NotImplementedError


UNKNOWN_OBJECT_ERROR = 0
UNKNOWN_OBJECT_IGNORE = 1
UNKNOWN_OBJECT_ADD = 2
    
class Scope(dict):
    """This class represents any PDDL object that can define variables
    or constants. It implements a lookup table to get the associated
    Parameters or TypedObjects given their name. Most methods of this
    class can take strings, TypedObjects or Terms as keys. For the
    latter two, the name of the objects will be used as a key.

    Scopes can be nested; when a symbol is not found in one scope, it
    will be looked up in it's parent Scope.

    Usually, a Scope object will not be used directly but be inherited
    by classes representing PDDL elements, e.g. domains, actions,
    quantified conditions/effects.
    """
    
    def __init__(self, objects, parent):
        """Create a new Scope object.

        Arguments:
        objects -- List of Parameters and TypedObjects that should be defined in this scope.
        parent -- Scope that this scope resides in. May be None if no parent exists.
        """
        
        self.set_parent(parent)
        self.original_parent = None
        self.termcache = {}

        self.tags = {}
        self.inherited_tags = set()
        self.lazy_param = UNKNOWN_OBJECT_ERROR
        self.lazy_constant = UNKNOWN_OBJECT_ERROR
        
        for obj in objects:
            dict.__setitem__(self, obj.name, obj)

    def set_parent(self, parent):
        """Change the parent of this Scope object.

        Arguments:
        parent -- Scope object."""
        self.parent = parent
        self.termcache = {}
        if parent is not None:
            self.predicates = parent.predicates
            self.functions = parent.functions
            self.types = parent.types
            assert not "false" in self.types
            self.requirements = parent.requirements
            self.parse_handlers = parent.parse_handlers
        else:
            self.predicates = FunctionTable()
            self.functions = FunctionTable()
            self.types = {}
            self.requirements = set()
            self.parse_handlers = []

    def set_lazy_mode(self, constants=None, params=None, all=None):
        if all is not None:
            self.lazy_param = all
            self.lazy_constant = all
        if constants is not None:
            self.lazy_constant = constants
        if params is not None:
            self.lazy_param = params
            
    def lookup(self, args):
        """Lookup a list of symbols in this Scope. Returns a list of
        Terms corresponding to the supplied symbols.

        Arguments:
        args -- list of strings, TypesObjects or Terms to look up"""
        result = []
        for arg in args:
            if arg.__class__ == predicates.FunctionTerm:
                result.append(predicates.FunctionTerm(arg.function, self.lookup(arg.args)))
            else:
                res = self[arg]
                if res not in self.termcache:
                    self.termcache[res] = predicates.Term(res)
                result.append(self.termcache[res])
                #result.append(predicates.Term(self[arg]))

        return result

    def add(self, obj):
        """Add an object to this Scope.

        Arguments:
        obj -- TypedObject or Parameter to add."""
        if isinstance(obj, (tuple, list, set)):
            for o in obj:
                dict.__setitem__(self, o.name, o)
        else:
            dict.__setitem__(self, obj.name, obj)

    def rename(self, obj, name):
        """Rename an object inside the scope.
        
        Arguments:
        obj -- TypedObject or Parameter to rename.
        name -- new name of the object"""

        my_obj = dict.__getitem__(self, obj.name)
        del self[my_obj.name]
        my_obj.rename(name)
        self.add(my_obj)

        
    # def merge(self, other):
    #     #assume unique variables for now
    #     for entry in other.values():
    #         name = entry.name
    #         #reassign existing variables
    #         if name in self:
    #             raise KeyError, "Variable %s already exists" % name
                
    #         dict.__setitem__(self, entry.name, entry)

    def tryInstantiate(self, mapping, parent=None):
        """Try to instantiate parameters in this scope. If
        instantiation fails (see instantiate() method), this method
        will return False, otherwise True.

        Arguments:
        mapping -- dictionary from parameter to object."""
        try:
            self.instantiate(mapping, parent)
        except:
            self.uninstantiate()
            return False
        return True
            
    def instantiate(self, mapping, parent=None):
        """Instantiate Parameters. All parameters and values must be
        defined in this Scope or one of its ancestors. An exception is
        instantiating a functional variable with a FunctionTerm, here
        the FunctionTerm is not looked up in the scope.

        If any of the objects are not defined, an Exception will be raised.

        Arguments:
        mapping -- dictionary from parameter to object.
        parent -- scope object (usually a pddl.Problem) that should be the base for instantiation"""
        self.uninstantiate()

        if parent:
            self.original_parent = self.parent
            self.set_parent(parent)
        
        nonfunctions = []
        #instantiate function variables first, as they can affect the types of other parameters
        for key, val in mapping.iteritems():
            key = self[key]
            
            if not isinstance(key, types.Parameter):
                raise TypeError("Cannot instantiate %s: it is no Parameter but %s" % (key, type(key)))
            
            if isinstance(val, predicates.FunctionTerm):
                key.instantiate(val)
            else:
                nonfunctions.append((key, val))
            
        for key,val in nonfunctions:
            if not isinstance(val, types.Parameter):
                val = self[val]
            key.instantiate(val)

    def smart_instantiate(self, func, args, arglists, parent=None, partial_mapping={}):
        self.uninstantiate()

        if parent:
            self.original_parent = self.parent
            self.set_parent(parent)
        
        values = {}
        mapping = {}
        stack = []
        remaining = set(args)
        for a, val in partial_mapping.iteritems():
            values[a] = [val]
            mapping[a] = val
            # stack.append((a,0))
            a.instantiate(val)
            remaining.discard(a)
            
        for a, l in zip(args, arglists):
            if a in partial_mapping:
                continue
            if not l:
                return
            l = list(l)
            values[a] = l
            if len(l) == 1:
                mapping[a] = l[0]
                stack.append((a,0))
                a.instantiate(l[0])
                remaining.discard(a)

        args = [a for a in args if a not in partial_mapping]
        
        curr_arg = None
        curr_index = -1
        while True:
            next, nextval = func(mapping, [s[0] for s in stack])

            if next is not None and next not in values:
                next = True
                nextval = None

            # print next, [str(v) for v,_ in stack], len(stack), len(args)
            if next == True and len(stack) == len(args):
                # print ["%s=%s" % (s[0].name, mapping[s[0]]) for s in stack]
                yield mapping
                next = False
            elif nextval:
                if nextval not in values[next]:
                    #print "illegal var:", nextval.name
                    next = False
                else:
                    curr_arg = next
                    curr_index = -1
            elif next:
                if next == True:
                    next = iter(remaining).next()
                curr_arg = next
                curr_index = 0
                # print curr_arg
                if values[curr_arg]:
                    nextval = values[curr_arg][curr_index]
                else:
                    next = None
                
            if not next:
                curr_index = -1
                while (curr_index == -1 or curr_index >= len(values[curr_arg])):
                    if not stack:
                        self.uninstantiate()
                        # print calls
                        return
                    curr_arg, curr_index = stack.pop()
                    # print ["%s = %s" % (str(k),str(v)) for k,v in mapping.iteritems() ]
                    # print curr_arg
                    del mapping[curr_arg]
                    remaining.add(curr_arg)
                    curr_arg.instantiate(None)
                    if curr_index != -1:
                        curr_index += 1
                    #print "bt:", curr_arg, curr_index
                nextval = values[curr_arg][curr_index]

            assert curr_arg not in set(s[0] for s in stack)
            mapping[curr_arg] = nextval
            curr_arg.instantiate(nextval)
            remaining.discard(curr_arg)

            # print ["%s = %s" % (str(k),str(mapping[k])) for k,v in stack ]
            # print "push:", curr_arg, mapping[curr_arg]
            stack.append((curr_arg, curr_index))
            
    def uninstantiate(self):
        """Uninstantiate all Parameters defined in this Scope."""
        for val in self.itervalues():
            if isinstance(val, types.Parameter):
                val.instantiate(None)
        if self.original_parent:
            self.set_parent(self.original_parent)
            self.original_parent = None

    def uniquify_variables(self):
        """Rename objects to that there are no name collisions."""
        renamings = {}
        for entry in self.values():
            if entry.name in self.parent:
                i = 2
                while "%s%d" % (entry.name, i) in self.parent:
                    i += 1
                del self[entry.name]
                newname =  "%s%d" % (entry.name, i)
                renamings[entry.name] = newname
                entry.name = newname
                dict.__setitem__(self, newname, entry)
        return renamings

    def copy_args(self, args, copy_instance=False):
        used_names = set()
        result = []
        for arg in args:
            if isinstance(arg.type, types.ProxyType):
                if copy_instance and arg.type.parameter.is_instantiated():
                    type = arg.type.effective_type()
                else:
                    type = types.ProxyType(self[arg.type.parameter])
            else:
                type = arg.type

            name = arg.name
            i=0
            while name in used_names:
                name = "%s%d" % (arg.name, i)
                i += 1

            used_names.add(name)
            arg = types.Parameter(name, type)
            self.add(arg)
            result.append(arg)
        return result

    def set_tag(self, key, value, inherit=True):
        self.tags[key] = value
        self.inherited_tags.add(key)

    def get_tag(self, key, from_parents=True, inherited_only=False):
        if key in self.tags:
            if inherited_only and key not in self.inherited_tags:
                return None
            return self.tags[key]
        if from_parents and self.parent:
            return self.parent.get_tag(key, inherited_only=True)
        return None

    def __contains__(self, key):
        if isinstance(key, (predicates.ConstantTerm, predicates.VariableTerm)):
            key = key.object
        if isinstance(key, (float, int)):
            return types.TypedNumber(key)
        if isinstance(key, types.TypedObject):
            if key.type == types.t_number:
                return True
            key = key.name
            
        key = key.lower()

        if dict.__contains__(self, key):
            return True
        if self.parent:
            return key in self.parent
        
    def __getitem__(self, key):
        #print type(key)
        obj = None
        if isinstance(key, predicates.VariableTerm):
            obj = key.object
            key = key.object.name
        elif isinstance(key, str):
            pass
        elif isinstance(key, predicates.ConstantTerm):
            if key.object.type == types.t_number:
                return key.object
            obj = key.object
            key = key.object.name
        elif isinstance(key, types.TypedObject):
            if key.type == types.t_number:
                return key
            obj = key
            key = key.name
        elif isinstance(key, (float, int)):
            return types.TypedNumber(key)
        # if isinstance(key, types.TypedObject):
        #     if key.type == types.t_number:
        #         return key
        #     key = key.name
        
        key = key.lower()

        if dict.__contains__(self, key):
            return dict.__getitem__(self, key)
        if self.parent:
            try:
                return self.parent[key]
            except KeyError:
                pass
            
        if obj is not None:
            if isinstance(obj, types.Parameter):
                if self.lazy_param == UNKNOWN_OBJECT_IGNORE:
                    return obj
                elif self.lazy_param == UNKNOWN_OBJECT_ADD:
                    self.add(obj)
                    return obj
            elif isinstance(obj, types.TypedObject):
                if self.lazy_constant == UNKNOWN_OBJECT_IGNORE:
                    return obj
                elif self.lazy_constant == UNKNOWN_OBJECT_ADD:
                    self.add(obj)
                    return obj

        raise KeyError, "Symbol %s not found." % key
        
    def __setitem__(self, key, value):
        raise NotImplementedError

    def __hash__(self):
        return object.__hash__(self)
    def __eq__(self, other):
        return self.__hash__() == other.__hash__()
    def __nonzero__(self):
        return True


# class ParserContext(object):
#     def __init__(self):
#         self.stack = []
#         self.scope = None
#         self.handlers = defaultdict(list)

#     def register_handler(self, tag, handler, class_=None):
#         self.handlers[tag].append((class_, handler))

#     def handle(self, tag, iterator):
#         if tag in self.handlers:
#             for class_, handler in self.handlers[tag]:
#                 if not class_ or isinstance(self.current, class_):
#                     if handler(iterator, self):
#                         return True
#                     else:
#                         iterator.reset()
#         return False
    
#     @property
#     def current(self):
#         return self.stack[-1]
        
#     def push(self, obj):
#         self.stack.append(obj)
#         if isinstance(obj, Scope):
#             self.scope = obj
            
#     def pop(self):
#         obj = self.types.pop()
#         if isinstance(obj, Scope):
#             self.scope = obj.parent
#         return obj

#     def __contains__(self, key):
#         return key in self.scope

#     def __getitem__(self, key):
#         return self.scope[key]
    
