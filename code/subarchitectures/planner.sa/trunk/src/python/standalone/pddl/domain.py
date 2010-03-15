#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict
import itertools

import parser
import mapltypes, builtin
import predicates, axioms

import builtin

from mapltypes import Type, TypedObject
from parser import Parser, ParseError, UnexpectedTokenError
from scope import Scope, FunctionTable
from actions import Action
from axioms import Axiom

supported = set(["mapl",  "modal-predicates", "strips", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "adl", "derived-predicated", "fluents", "numeric-fluents", "object-fluents", "durative-actions", "partial-observability"])

support_depends = {"mapl" : ["object-fluents", "modal-predicates"],
                   "adl" : ["typing", "negative-preconditions", "disjunctive-preconditions", "quantified-preconditions", "equality", "conditional-effects"],
                   "fluents" : ["numeric-fluents", "object-fluents"]}

class Domain(Scope):
    def __init__(self, name, types, constants, predicates, functions, actions, axioms, sensors=None, observe=None):
        Scope.__init__(self, constants, None)
        self.name = name
        self.types = types
        self.constants = constants
        self.predicates = predicates
        self.functions = functions
        self.actions = actions
        self.axioms = axioms

        if sensors is None:
            self.sensors = []
        else:
            self.sensors = sensors
        if observe is None:
            self.observe = []
        else:
            self.observe = observe

        self.requirements = set()
        self.stratify_axioms()
        self.name2action = None

    def copy(self):
        dom = Domain(self.name, self.types.copy(), self.constants.copy(), self.predicates.copy(), self.functions.copy(), [], [])
        dom.actions = [a.copy(dom) for a in self.actions]
        dom.axioms = [a.copy(dom) for a in self.axioms]
        if self.sensors:
            dom.sensors = [s.copy(dom) for s in self.sensors]
        if self.observe:
            dom.observe = [s.copy(dom) for s in self.observe]
            
        dom.stratify_axioms()
        dom.name2action = None

        dom.requirements = set(self.requirements)
        return dom

    def get_action(self, name):
        if not self.name2action:
            if "mapl" in self.requirements:
                self.name2action = dict((a.name, a) for a in itertools.chain(self.actions, self.sensors))
            else:
                self.name2action = dict((a.name, a) for a in self.actions)
        return self.name2action[name]

    def stratify_axioms(self):
        self.stratification, self.nonrecursive = axioms.stratify(self.axioms)
    
    @staticmethod
    def parse(root, supp = supported):
        it = iter(root)
        it.get("define")
        j = iter(it.get(list, "(domain 'domain identifier')"))
        j.get("domain")
        domname = j.get(None, "domain identifier").token.string
        
        typeDict = dict((t.name, t) for t in builtin.default_types)
        constants = set()
        preds = FunctionTable([builtin.equals])
        functions = FunctionTable()
        
        req = iter(it.get(list, "requirement definition"))
        req.get(":requirements")
        requirements = []
        for r in req:
            if not r.is_terminal() or r.token.string[0] != ":":
                raise UnexpectedTokenError(r.token, "requirement identifier")
            requirements.append(r.token.string[1:])
            if r.token.string[1:] not in supp:
                raise ParseError(r.token, "%s is not supported." % r.token.string)
            if r.token.string[1:] in support_depends:
                requirements += support_depends[r.token.string[1:]]

        if "mapl" in requirements:
            import mapl
            constants = set([builtin.TRUE, builtin.FALSE, builtin.UNKNOWN])
            
        if "fluents" in requirements or "numeric-fluents" in requirements:
            typeDict[builtin.t_number.name] = builtin.t_number
            preds.add(builtin.numeric_comparators)
            functions.add(builtin.numeric_functions)

        if "mapl" in requirements:
            for t in mapl.mapl_types:
                typeDict[t.name] = t
            preds.add(mapl.mapl_predicates)

        if "partial-observability" in requirements:
            import dtpddl
            preds.add(dtpddl.modal_predicates)
            
        domain = None
            
        for elem in it:
            j = iter(elem)
            type = j.get("terminal").token

            #If domain is not already constructed, create it now
            if type in (":action", ":durative-action", ":sensor", ":derived") and domain is None:
                domain = Domain(domname, typeDict, constants, preds, functions, [], [])
                domain.requirements = set(requirements)
            
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

            elif type == ":predicates":
                for elem in j:
                    if elem.is_terminal():
                        raise UnexpectedTokenError(elem, "predicate declaration")
                    preds.add(predicates.Predicate.parse(iter(elem), typeDict))
                
            elif type == ":functions":
                def leftFunc(elem):
                    if elem.is_terminal():
                        raise UnexpectedTokenError(elem.token, "function declaration")
                    return elem
                def rightFunc(elem):
                    return Type.parse(elem, typeDict)

                for funcs, type in parser.parse_typed_list(j, leftFunc, rightFunc, "function declarations", "type specification", True):
                    for elem in funcs:
                        functions.add(predicates.Function.parse(iter(elem), type, typeDict))

            elif type == ":action":
                if "mapl" in requirements:
                    domain.actions.append(mapl.MAPLAction.parse(j.reset(), domain))
                else:
                    domain.actions.append(Action.parse(j.reset(), domain))

            elif type == ":durative-action" and "durative-actions" in requirements :
                if "mapl" in requirements:
                    domain.actions.append(mapl.MAPLDurativeAction.parse(j.reset(), domain))
                else:
                    import durative
                    domain.actions.append(durative.DurativeAction.parse(j.reset(), domain))
                
            elif type == ":sensor" and "mapl" in requirements:
                from sensors import Sensor
                domain.sensors.append(Sensor.parse(j.reset(), domain))
                
            elif type == ":derived":
                domain.axioms.append(Axiom.parse(j.reset(), domain))
                
            elif type == ":observe" and "partial-observability" in requirements:
                domain.observe.append(dtpddl.Observation.parse(j.reset(), domain))
                
            else:
                raise ParseError(type, "Unknown section identifier: '%s'." % type.string)
            
        if domain is None:
            domain = Domain(domname, typeDict, constants, preds, functions, [], [])

        if "mapl" in requirements:
            for axiom_str in mapl.mapl_axioms:
                axiom = Parser.parse_as(axiom_str.split("\n"), Axiom, domain)
                domain.axioms.append(axiom)

        return domain
