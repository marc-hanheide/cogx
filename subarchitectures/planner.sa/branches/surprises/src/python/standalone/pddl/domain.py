#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict

import parser
import mapltypes, builtin
import predicates, axioms

from mapltypes import Type, TypedObject
from parser import ParseError, UnexpectedTokenError
from scope import Scope, FunctionTable
from actions import Action
from axioms import Axiom

supported = set(["modal-predicates", "strips", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "universal-effects", "adl", "derived-predicated", "fluents", "numeric-fluents", "object-fluents", "action-costs", "preferences"])

support_depends = {"adl" : ["typing", "negative-preconditions", "disjunctive-preconditions", "quantified-preconditions", "equality", "conditional-effects"],
                   "fluents" : ["numeric-fluents", "object-fluents"]}

registered_modules = {}

class ModuleDescription(object):
    def __init__(self, module, **kwargs):
        d = {}
        if module:
            d.update(module.__dict__)
        d.update(kwargs)
        
        self.dependencies = d.get("dependencies", [])
        self.types = d.get("default_types", [])
        self.constants = d.get("default_constants", [])
        self.predicates = d.get("default_predicates", [])
        self.functions = d.get("default_functions", [])

        self.prepare_domain = d.get("prepare_domain", None)
        self.parse_handlers = d.get("parse_handlers", [])
        self.post_parse = d.get("post_parse", None)
        self.default_compiler = d.get("default_compiler", None)
        self.compilers = d.get("compilers", {})
        self.domain_hooks = d.get("domain_hooks", {})

    @staticmethod
    def create_module(name, **kwargs):
        desc = ModuleDescription(None, **kwargs)
        desc.name = name
        desc.modulename = None
        registered_modules[name] = desc
        
    @staticmethod
    def get_module(name):
        if name in registered_modules:
            return registered_modules[name]
        
        modulename = name.replace("-", "_")
        try:
            exec("import %s as _pddl_module" % modulename)
            if not _pddl_module.__dict__.get("pddl_module", False):
                return False
            desc = ModuleDescription(_pddl_module)
            desc.name = name
            desc.modulename = modulename
            registered_modules[name] = desc
            return desc
        except ImportError:
            return False

def hook(f):
    import functools
    @functools.wraps(f)
    def wrapper(self, *args, **kwargs):
        result = f(self, *args, **kwargs)
        if f.__name__ in self.hooks:
            for h in self.hooks[f.__name__]:
                hres = h(self, result, *args, **kwargs)
                result = hres if hres is not None else result
        return result
    return wrapper
    
        
class Domain(Scope):
    """This class represents a PDDL domain."""
    
    def __init__(self, name, reqs,  types, constants, predicates, functions, actions, axioms):
        """Create a new PDDL domain.

        Arguments:
        name -- name of the domain
        req -- domain requirements
        types -- dictionary from typename to Type objects
        constants -- sequence of TypedObjects
        predicates -- FunctionTable with the domain's predicates
        functions -- FunctionTable with the domain's functions
        actions -- List of Action objects
        axioms -- List of Axiom objects
        """
        Scope.__init__(self, constants, None)
        self.name = name
        self.requirements = reqs
        self.types = types
        self.constants = constants
        self.predicates = predicates
        self.functions = functions
        self.actions = actions
        self.axioms = axioms

        self.hooks = defaultdict(list)
        for req in self.requirements:
            if req in supported:
                continue
            m = ModuleDescription.get_module(req)
            assert m, "%s is not supported." % req
            
            if m.parse_handlers:
                self.parse_handlers.append(m.parse_handlers)
            for f, h in m.domain_hooks.iteritems():
                self.hooks[f].append(h)
            if m.prepare_domain:
                m.prepare_domain(self)

        self.stratify_axioms()
        self.name2action = None
        self.objects_by_type = {}

    @hook
    def copy(self):
        """Create a deep copy of this Domain."""
        dom = Domain(self.name, set(self.requirements), self.types.copy(), self.constants.copy(), self.predicates.copy(), self.functions.copy(), [], [])
        dom.actions = [a.copy(dom) for a in self.actions]
        dom.axioms = [a.copy(dom) for a in self.axioms]
            
        return dom

    @hook
    def copy_skeleton(self):
        """Create a deep copy of the essential elements of this
        Domain. This includes predicates, functions, constants but
        excludes actions and axioms."""
        
        return Domain(self.name, set(self.requirements), self.types.copy(), self.constants.copy(), self.predicates.copy(), self.functions.copy(), [], [])
        
    def get_action(self, name):
        """Return the action with the given name.

        Arguments:
        name -- name of the Action to look up"""
        
        if not self.name2action:
            self.name2action = dict((a.name, a) for a in self.actions)
        try:
            return self.name2action[name]
        except:
            # print [a.name for a in self.actions]
            # print [n for n in self.name2action.iterkeys()]
            raise

    def add_constant(self, object):
        if object.name in self:
            self.constants.remove(self[object.name])
        self.constants.add(object)
        self.add(object)
        for typ, objs in self.objects_by_type.iteritems():
            if object.is_instance_of(typ):
                objs.add(object)

    def remove_constant(self, object):
        if dict.__contains__(self, object.name):
            self.constants.remove(self[object.name])
            del self[object.name]
            for typ, objs in self.objects_by_type.iteritems():
                if object.is_instance_of(typ):
                    objs.discard(object)

    def get_all_objects(self, type):
        if type not in self.objects_by_type:
            self.objects_by_type[type] = set(o for o in self.constants if o.is_instance_of(type) and o != builtin.UNKNOWN)
        for obj in self.objects_by_type[type]:
            yield obj

    def get_nonstatic_functions(self):
        import visitors, utils
        def get_nonstatic(a):
            return [utils.get_function(l) for l in  visitors.visit(a.effect, visitors.collect_literals, [])]
        return set(sum((get_nonstatic(a) for a in self.actions), [])) | set(a.predicate for a in self.axioms)
        
    @hook
    def add_action(self, action):
        if action.__class__ == Action:
            self.actions.append(action)
            self.name2action = None

    @hook
    def clear_actions(self):
        self.actions = []
        self.name2action = None

    def set_actions(self, actions):
        """Set the actions for this domain.

        Arguments:
        actions -- the new set of actions for this domain.
        """
        self.actions = list(actions)
        #for a in self.actions:
        #    a.set_scope(self)
        self.name2action = None

    @hook
    def get_action_like(self):
        return self.actions

    def get_stratification(self):
        if self._stratification is None:
            self._stratification, self._nonrecursive = axioms.stratify(self.axioms)
        return self._stratification

    def get_nonrecursive(self):
        if self._nonrecursive is None:
            self._stratification, self._nonrecursive = axioms.stratify(self.axioms)
        return self._nonrecursive

    def get_derived(self):
        if self._derived is None:
            self._derived = set(ax.predicate for ax in self.axioms)
        return self._derived
    
    stratification = property(get_stratification)
    nonrecursive = property(get_nonrecursive)
    derived = property(get_derived)

    def stratify_axioms(self):
        """Compute lazily stratification layers for the axioms in this domain."""
        self._derived = None
        self._stratification = None
        self._nonrecursive = None

    def add_requirement(self, req):
        if req not in supported:
            module = ModuleDescription.get_module(req)
            assert module, "%s is not supported." % req
            
            for r in module.dependencies:
                self.add_requirement(r)
        
            for t in module.types:
                self.types[t.name] = t
            for obj in module.constants:
                self.add_constant(obj)
            self.predicates.add(module.predicates)
            self.functions.add(module.functions)

            if module.parse_handlers:
                self.parse_handlers.append(module.parse_handlers)
            for f, h in module.domain_hooks.iteritems():
                self.hooks[f].append(h)
            if module.prepare_domain:
                module.prepare_domain(self)
            
        self.requirements.add(req)

    def compile_to(self, supported, **kwargs):
        """Returns a translator that compiles to the given set of supported features (if possible)"""

        import translators
        
        new = set(supported)
        for s in supported:
            if s in support_depends:
                new |= set(support_depends[s])
                
        remove = list(self.requirements - new)
        deps = defaultdict(set)
        for r in remove:
            desc = ModuleDescription.get_module(r)
            if not desc:
                #print "not found:",r
                continue
            for d in desc.dependencies:
                if d not in new:
                    deps[d].add(r)
            
        compile_seq = []
        while remove:
            req = remove.pop(0)
            if deps[req] & set(remove): #only compile things away that nothing else depends on
                remove.append(req)
                continue
            desc = ModuleDescription.get_module(req)
            if not desc:
                #print "not found:",r
                continue

            compiler = None
            for target, comp in desc.compilers.iteritems():
                if target in supported:
                    compiler = comp
            if not compiler:
                compiler = desc.default_compiler
            if compiler:
                compile_seq.append(compiler)

        compilers = [c(**kwargs) for c in compile_seq]
        t = translators.ChainingTranslator(*compilers)
        return t
        
    @staticmethod
    def parse(root, supp = supported):
        it = iter(root)
        it.get("define")
        j = iter(it.get(list, "(domain 'domain identifier')"))
        j.get("domain")
        domname = j.get(None, "domain identifier").token.string

        typeDict = dict((t.name, t) for t in builtin.default_types)
        constants = set()
        preds = FunctionTable([builtin.equals, builtin.assign, builtin.equal_assign])
        functions = FunctionTable()

        default_functions = []

        modules = []
        
        req = iter(it.get(list, "requirement definition"))
        req.get(":requirements")
        requirements = []
        for r in req:
            if not r.is_terminal() or r.token.string[0] != ":":
                raise UnexpectedTokenError(r.token, "requirement identifier")
            requirement = r.token.string[1:]
            requirements.append(requirement)
            if requirement  not in supp:
                module = ModuleDescription.get_module(requirement)
                if not module:
                    raise ParseError(r.token, "%s is not supported." % r.token.string)
                modules.append(module)
                requirements += module.dependencies

            elif requirement in support_depends:
                requirements += support_depends[requirement]

        for module in modules:
            for t in module.types:
                typeDict[t.name] = t
            constants |= set(module.constants)
            preds.add(module.predicates)
            functions.add(module.functions)
            default_functions += module.predicates + module.functions

        if "fluents" in requirements or "numeric-fluents" in requirements:
            typeDict[builtin.t_number.name] = builtin.t_number
            preds.add(builtin.numeric_comparators)
            preds.add(builtin.numeric_ops)
            preds.add(builtin.num_equal_assign)
            functions.add(builtin.numeric_functions)

        if "action-costs" in requirements:
            try:
                preds.add(builtin.increase)
            except:
                pass
            functions.add(builtin.total_cost)
            default_functions.append(builtin.total_cost)
        
        domain = None
        
        for elem in it:
            j = iter(elem)
            type = j.get("terminal").token

            #If domain is not already constructed, create it now
            if type not in (":types", ":constants", ":predicates", ":functions") and domain is None:
                domain = Domain(domname, set(requirements), typeDict, constants, preds, functions, [], [])
                
            handled = False
            for module in modules:
                if type.string in module.parse_handlers:
                    if module.parse_handlers[type.string](j.reset(), domain):
                        handled = True
                        break

            if handled:
                continue
                    
            if type == ":types":
                tlist = mapltypes.parse_typelist(j)
                for key, value in tlist.iteritems():
                    if key.string == "object":
                        continue
                    if value.string not in typeDict:
                        typeDict[value.string] = Type(value.string, [builtin.t_object])
                    if key.string not in typeDict:
                        typeDict[key.string] = Type(key.string, [builtin.t_object])
                        
                    typeDict[key.string].supertypes.add(typeDict[value.string])

            elif type == ":constants":
                clist = mapltypes.parse_typelist(j)
                for key, value in clist.iteritems():
                    if value.string not in typeDict:
                        raise ParseError(value, "undeclared type")

                    constants.add(TypedObject(key.string, typeDict[value.string]))

                if domain:
                    domain.constants = set(constants)
                    for c in constants:
                        if c not in domain:
                            domain.add(c)

            elif type == ":predicates":
                for elem in j:
                    if elem.is_terminal():
                        raise UnexpectedTokenError(elem, "predicate declaration")
                    f = predicates.Predicate.parse(iter(elem), typeDict)
                    try:
                        preds.add(f)
                    except Exception, e:
                        if f not in default_functions:
                            raise e
                
            elif type == ":functions":
                def leftFunc(elem):
                    if elem.is_terminal():
                        raise UnexpectedTokenError(elem.token, "function declaration")
                    return elem
                def rightFunc(elem):
                    return Type.parse(elem, typeDict)

                for funcs, type in parser.parse_typed_list(j, leftFunc, rightFunc, "function declarations", "type specification", True):
                    for elem in funcs:
                        f = predicates.Function.parse(iter(elem), type, typeDict)
                        try:
                            functions.add(f)
                        except Exception, e:
                            if f not in default_functions:
                                raise e

            elif type == ":action":
                domain.actions.append(Action.parse(j.reset(), domain))
                
            elif type == ":derived":
                domain.axioms.append(Axiom.parse(j.reset(), domain))
                                
            else:
                raise ParseError(type, "Unknown section identifier: '%s'." % type.string)
            
        if domain is None:
            domain = Domain(domname, set(requirements), typeDict, constants, preds, functions, [], [])

        for m in modules:
            if m.post_parse:
                m.post_parse(domain)
                
        domain.stratify_axioms()

        return domain
    
