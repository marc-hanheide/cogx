#! /usr/bin/env python
# -*- coding: latin-1 -*-

from collections import defaultdict
import itertools

import parser
import mapltypes
import scope
import predicates, conditions, actions, axioms

from mapltypes import *
from predicates import *
from parser import Parser, ParseError, UnexpectedTokenError
from actions import Action
from axioms import Axiom

supported = set(["mapl",  "modal-predicates", "typing", "equality", "negative-preconditions", "disjunctive-preconditions", "existential-preconditions", "universal-preconditions", "quantified-preconditions", "conditional-effects", "adl", "derived-predicated", "fluents", "numeric-fluents", "object-fluents", "durative-actions"])

support_depends = {"mapl" : ["object-fluents", "modal-predicates"],
                   "adl" : ["typing", "negative-preconditions", "disjunctive-preconditions", "quantified-preconditions", "equality", "conditional-effects"],
                   "fluents" : ["numeric-fluents", "object-fluents"]}

class Domain(scope.Scope):
    def __init__(self, name, types, constants, predicates, functions, actions, axioms):
        scope.Scope.__init__(self, constants, None)
        self.name = name
        self.types = types
        self.constants = constants
        self.predicates = predicates
        self.functions = functions
        self.actions = actions
        self.axioms = axioms

        self.stratifyAxioms()
        self.name2action = None

    def copy(self):
        dom = Domain(self.name, self.types.copy(), self.constants.copy(), self.predicates.copy(), self.functions.copy(), [], [])
        dom.actions = [a.copy(self) for a in self.actions]
        dom.axioms = [a.copy(self) for a in self.axioms]
        dom.stratifyAxioms()
        dom.name2action = None
            
        dom.requirements = set(self.requirements)
        return dom

    def getAction(self, name):
        if not self.name2action:
            self.name2action = dict((a.name, a) for a in itertools.chain(self.actions, self.sensors))
        return self.name2action[name]

    def stratifyAxioms(self):
        self.stratification, self.nonrecursive = axioms.stratify(self.axioms)
    
    @staticmethod
    def parse(root, supp = supported):
        it = iter(root)
        it.get("define")
        j = iter(it.get(list, "(domain 'domain identifier')"))
        j.get("domain")
        domname = j.get(None, "domain identifier").token.string
        
        typeDict = dict((t.name, t) for t in default_types)
        constants = set()
        preds = scope.FunctionTable([predicates.equals])
        functions = scope.FunctionTable()
        
        req = iter(it.get(list, "requirement definition"))
        req.get(":requirements")
        requirements = []
        for r in req:
            if not r.isTerminal() or r.token.string[0] != ":":
                raise UnexpectedTokenError(r.token, "requirement identifier")
            requirements.append(r.token.string[1:])
            if r.token.string[1:] not in supp:
                raise ParseError(r.token, "%s is not supported." % r.token.string)
            if r.token.string[1:] in support_depends:
                requirements += support_depends[r.token.string[1:]]

        if "mapl" in requirements:
            constants = set([TRUE, FALSE, UNKNOWN])
                
        if "fluents" in requirements or "numeric-fluents" in requirements:
            typeDict[numberType.name] = numberType
            preds.add(predicates.numericComparators)
            functions.add(predicates.numericFunctions)

        if "mapl" in requirements:
            for t in mapl_types:
                typeDict[t.name] = t
            preds.add(mapl_predicates)

        domain = None
            
        for elem in it:
            j = iter(elem)
            type = j.get("terminal").token

            #If domain is not already constructed, create it now
            if type in (":action", ":durative-action", ":sensor", ":derived") and domain is None:
                if "mapl" in requirements:
                    import mapl
                    domain = mapl.MAPLDomain(domname, typeDict, constants, preds, functions, [], [], [])
                else:
                    domain = Domain(domname, typeDict, constants, preds, functions, [], [])
                domain.requirements = set(requirements)
            
            if type == ":types":
                tlist = mapltypes.parse_typelist(j)
                for key, value in tlist.iteritems():
                    if key.string == "object":
                        continue
                    if value.string not in typeDict:
                        typeDict[value.string] = Type(value.string, [objectType])
                    if key.string not in typeDict:
                        typeDict[key.string] = Type(key.string, [objectType])
                        
                    typeDict[key.string].supertypes.add(typeDict[value.string])

            elif type == ":constants":
                clist = mapltypes.parse_typelist(j)
                for key, value in clist.iteritems():
                    if value.string not in typeDict:
                        raise ParseError(value, "undeclared type")

                    constants.add(TypedObject(key.string, typeDict[value.string]))

            elif type == ":predicates":
                for elem in j:
                    if elem.isTerminal():
                        raise UnexpectedTokenError(elem, "predicate declaration")
                    preds.add(predicates.Predicate.parse(iter(elem), typeDict))
                
            elif type == ":functions":
                def leftFunc(elem):
                    if elem.isTerminal():
                        raise UnexpectedTokenError(elem.token, "function declaration")
                    return elem
                def rightFunc(elem):
                    return Type.parse(elem, typeDict)

                for funcs, type in parser.parseTypedList(j, leftFunc, rightFunc, "function declarations", "type specification", True):
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
                
            else:
                raise ParseError(type, "Unknown section identifier: '%s'." % type.string)
            
        if domain is None:
            if "mapl" in requirements:
                import mapl
                domain = mapl.MAPLDomain(domname, typeDict, constants, preds, functions, [], [], [])
            else:
                domain = Domain(domname, typeDict, constants, preds, functions, [], [])

        if "mapl" in requirements:
            for axiom_str in axioms.mapl_axioms:
                axiom = Parser.parseAs(axiom_str.split("\n"), Axiom, domain)
                domain.axioms.append(axiom)

        return domain
