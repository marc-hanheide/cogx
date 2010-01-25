#! /usr/bin/env python
# -*- coding: latin-1 -*-

import itertools

import mapltypes as types
import predicates

class FunctionTable(dict):
    def __init__(self, functions=[]):
        for f in functions:
            self.add(f)

    def copy(self):
        c =  FunctionTable()
        for f in self:
            c.add(f)
        return c

    def add(self, function):
        if isinstance(function, (list, tuple, set)):
            for f in function:
                self.add(f)
            return
                
        if function.name not in self:
            dict.__setitem__(self, function.name, set())
        else:
            if self.get(function.name, function.args):
                raise Exception("A function with this name and arguments already exists: " + str(function))

        dict.__getitem__(self, function.name).add(function)
        

    def remove(self, function):
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
                
        
    def get(self, name, args):
        """Get all functions matching the provided name and argument types"""
        
        if name not in self:
            return []
        
        fs = dict.__getitem__(self, name)
        
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

class Scope(dict):
    def __init__(self, objects, parent):
        self.set_parent(parent)
        for obj in objects:
            dict.__setitem__(self, obj.name, obj)

    def set_parent(self, parent):
        self.parent = parent
        if parent is not None:
            self.predicates = parent.predicates
            self.functions = parent.functions
            self.types = parent.types
            assert not "false" in self.types
            self.requirements = parent.requirements
        else:
            self.predicates = FunctionTable()
            self.functions = FunctionTable()
            self.types = {}
            self.requirements = set()
        
    def lookup(self, args):
        result = []
        for arg in args:
            if arg.__class__ == predicates.FunctionTerm:
                result.append(predicates.FunctionTerm(arg.function, self.lookup(arg.args)))
            else:
                result.append(predicates.Term(self[arg]))

        return result

    def add(self, obj):
        if isinstance(obj, (tuple, list)):
            for o in obj:
                dict.__setitem__(self, o.name, o)
        else:
            dict.__setitem__(self, obj.name, obj)

    def merge(self, other):
        #assume unique variables for now
        for entry in other.values():
            name = entry.name
            #reassign existing variables
            if name in self:
                raise KeyError, "Variable %s already exists" % name
                
            dict.__setitem__(self, entry.name, entry)

    def tryInstantiate(self, mapping):
        try:
            self.instantiate(mapping)
        except:
            self.uninstantiate()
            return False
        return True
            
    def instantiate(self, mapping):
        self.uninstantiate()
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
            val = self[val]
            key.instantiate(val)

            
    def uninstantiate(self):
        for val in self.itervalues():
            if isinstance(val, types.Parameter):
                val.instantiate(None)

    def uniquify_variables(self):
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
        if isinstance(key, (predicates.ConstantTerm, predicates.VariableTerm)):
            key = key.object
        if isinstance(key, (float, int)):
            return types.TypedNumber(key)
        if isinstance(key, types.TypedObject):
            if key.type == types.t_number:
                return key
            key = key.name
        
        key = key.lower()

        if dict.__contains__(self, key):
            return dict.__getitem__(self, key)
        if self.parent:
            return self.parent[key]

        raise KeyError, "Symbol %s not found." % key
        
    def __setitem__(self, key, value):
        raise NotImplementedError

    def __hash__(self):
        return object.__hash__(self)
    def __eq__(self, other):
        return self.__hash__() == other.__hash__()
    def __nonzero__(self):
        return True


