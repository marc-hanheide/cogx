#! /usr/bin/env python
# -*- coding: latin-1 -*-

import sys
import copy
import sets

import conditions
import effects
import types

import utils
from constants import *

tmp_vars = 0

UNQUANT_TMPL = """
	(forall ($vname - $vtype)
      (when $fact (not $fact)))
"""


def invent_parameters(lit, predicates):
    """invent missing parameters for values of this literal"""
    global tmp_vars
    pname = lit.predicate
    declaration = predicates[pname]
    ts = [p.type for p in declaration.value]
    params = []
    for t in ts:
        params.append(types.TypedObject("?_v%d" % tmp_vars, t))
        tmp_vars += 1
    return params

def check_if_template(alist):
    assert alist[0] in (":action", ":sensor")
    if alist[2].startswith("??"):
        del alist[2]
        return True
    return False

def copyreplace(alist, p, is_boolean):
    arg_names = [a.name for a in to_tmp_var_names(p.arguments)]
    val_names = [a.name for a in to_tmp_var_names(p.value)]
    val_types = [a.type for a in p.value]
    repl = {"??svar":p.name, "??args":" ".join(arg_names)}
    if is_boolean:
        repl["??val"] = ""
        repl["??dom_type"] = ""
    else:
        repl["??val"] = " ".join(val_names)
        repl["??dom_type"] = " ".join(val_types)
    alist_copy = copy.deepcopy(alist)
    name = alist[1]
    alist_copy[1] = "%s_%s" % (name, p.name)
    replacements = utils.rec_replace(alist_copy, repl)
    return alist_copy, replacements

def to_typed_list(list_of_typed_objects):
    l = []
    for o in list_of_typed_objects:
        l.extend([o.name, "-", o.type])
    return l

def to_tmp_var_names(list_of_typed_objects):
    l = []
    for o in list_of_typed_objects:
        assert o.name.startswith("?")
        new_name = "?_%s" % o.name[1:]
        obj = types.TypedObject(new_name, o.type)
        l.append(obj)
    return l




class Action(object):
    def __init__(self, name, parameters, precondition, effects, agents=[], replan=None, variables=[]):
        self.name = name
        self.agents = agents
        self.parameters = parameters
        self.variables = variables
        self.precondition = precondition
        self.replan_condition = replan
        self.effects = effects
        self.remove_double_declarations()
        self.uniquify_variables()
        self.eliminate_ex_quantifiers()

    def is_assertion (self):
        return bool(self.replan_condition)

    def remove_double_declarations(self):
        def unknown_vars(some_vars, known_vars):
            for v in some_vars:
                if v not in known_vars:
                    known_vars.add(v)
                    yield v
        known_vars = set(self.agents)
        self.parameters = list(unknown_vars(self.parameters, known_vars))
        self.variables = list(unknown_vars(self.variables, known_vars))
    
    def copy(self):
        n = self.name
        ps = list(self.parameters)
        pre = self.precondition.copy()
        agts = list(self.agents)
        es = [e.copy() for e in self.effects]
        rc = None
        if self.replan_condition:
            rc = self.replan_condition.copy()
        vs = list(self.variables)
        a = Action(n, ps, pre, es, agents=agts, replan=rc, variables=vs)
        return a

    def translate2pddl(self, pred_dict):
        action = self.copy()
        # ignore the following for now. TODO: refactor to use new Literal representation
        #action.add_commitment_precond()
        #action.add_delete_effects(pred_dict)
        return action

    def add_commitment_precond(self):
        if isinstance(self.precondition, conditions.Conjunction):
            old_parts = self.precondition.parts
        else:
            old_parts = [self.precondition]
        agent_name = self.agents[0].name
        new_condition = [COMMITED_TO_PLAN, agent_name]
        new_parts = [conditions.parse_literal(new_condition)]
        new_parts.extend(old_parts)
        self.precondition = conditions.Conjunction(new_parts)
        
        
    def add_delete_effects(self, predicates):
        """looks for mapl effects and extends parameters, (pre-)conditions,
        and delete effects such that the action works in PDDL, too."""

        #print "adding dels for", self.name
        new_effects = list()
        for effect in self.effects:
            svar = (effect.literal.predicate, effect.literal.args)
            eff_val = effect.literal.value
            svar_name = effect.literal.predicate
##             print "eff::", effect.literal
            if eff_val in (TRUE_TUP, FALSE_TUP):
                continue
            if isinstance(effect.condition, conditions.Conjunction):
                cond_conjuncts = effect.condition.parts
            elif isinstance(effect.condition, conditions.Truth):
                cond_conjuncts = []
            else:
                cond_conjuncts = [effect.condition]
            params = invent_parameters(effect.literal, predicates)
            pnames = [p.name for p in params]

##             # when the svar has any value(s) pnames (usually exactly one value)
            prop = conditions.Atom(svar[0], list(svar[1]), pnames)
            cond_conjuncts.append(prop)
##             # and is different from the value we actually want to set
##             unequal_effval = conditions.Atom("=", [pnames[0], eff_val[0]], FALSE_TUP)
##             cond_conjuncts.append(unequal_effval)
            condition = conditions.Conjunction(cond_conjuncts)

            # then we can delete this value
            new_effect_literal = conditions.Atom(svar[0], list(svar[1])+list(pnames), FALSE_TUP)
            new_params = effect.parameters + params
            new_effects.append(effects.Effect(new_params, condition, new_effect_literal))
        self.effects.extend(new_effects)
    
    
    
    def parse(alist, predicates):
        global tmp_vars
        tmp_vars = 0
        is_template = check_if_template(alist)
        if not is_template:
            return [Action._parse(alist)]
        else:
            acts = []
##             print "\naction", alist[1]
##             print "len", len(alist), alist
            expected_length = 12
            if ":precondition" not in alist:
                expected_length -= 2
            if ":parameters" not in alist:
                alist[4:4] = [":parameters", []]
            if ":variables" not in alist:
                alist[6:6] = [":variables", []]
            newarg_pos = 7  # add to ":agent" field, we can be sure it exists
##             print "len", len(alist), alist
            assert len(alist)==expected_length, "len only %s" % len(alist)
            for pred in predicates:
                is_bool = pred.is_boolean()
##                 print "next pred", pred, "is bool:", is_bool
                alist_copy, replacements = copyreplace(alist, pred, pred.is_boolean())
                oldargs = alist_copy[newarg_pos]
                newargs = []
##                 print "repls", replacements
                if "??args" in replacements:
                    newargs.extend(to_typed_list(to_tmp_var_names(pred.arguments)))
                if "??val" in replacements and not is_bool:
                    newargs.extend(to_typed_list(to_tmp_var_names(pred.value)))
                alist_copy[newarg_pos] = oldargs + newargs
                new_action = Action._parse(alist_copy)
                acts.append(new_action)
##                 print "newargs", newargs
##                 print "newaction", new_action.dump_pddl(sys.stdout)
##                 print "alist", alist_copy
##             print "exit"
##             sys.exit()
            return acts
    parse = staticmethod(parse)                      

    @staticmethod
    def _parse(alist):
        iterator = iter(alist)
        atype = iterator.next()
        assert atype == ":action"
        name = iterator.next()
        precondition = conditions.Conjunction([])
        replan = None
        agents = None
        parameters = []
        variables = []
        tag = iterator.next()
        while tag != ":effect":
            if tag == ":parameters":
                parameters = types.parse_typed_list(iterator.next(), only_variables=True)
            elif tag == ":variables":
                variables = types.parse_typed_list(iterator.next(), only_variables=True)
            elif tag == ":agent":
                agents = types.parse_typed_list(iterator.next(), only_variables=True)
            elif tag == ":precondition":
                precondition = conditions.parse_condition(iterator.next())
            elif tag == ":replan":
                replan = conditions.parse_condition(iterator.next())
            tag = iterator.next()
        assert tag == ":effect"
        effect_list = iterator.next()
        if effect_list[0] == "and":
            effect_list = effect_list[1:]
        else:
            effect_list = [effect_list]
        eff = []
        for item in effect_list:
            effects.parse_effects(item, eff)
        for rest in iterator:
            assert False, rest
        assert agents is not None, "':agent' must be defined in every MAPL operator!"
        a = Action(name, parameters, precondition, eff, agents=agents, replan=replan, variables=variables)
        #print "actiondump:", a.dump_pddl(sys.stdout)
        return a
    
    def dump(self):
        print "\n%s(%s)" % (self.name, ", ".join(map(str, self.parameters)))
        print "Precondition:"
        self.precondition.dump()
        print "Effects:"
        for eff in self.effects:
            eff.dump()
    def dump_pddl(self, stream, keep_assertions=True):
        print >> stream, "\n".join(self.pddl_gen(keep_assertions))

    def pddl_gen(self, keep_assertions=True):
        replan2precond = self.replan_condition and not keep_assertions
        yield "\n(:action %s" % self.name
        params = self.agents + self.parameters + self.variables
        yield " :parameters (%s)" % " ".join(map(types.pddl_str, params))
        indent = "    "
        yield " :precondition"
        if replan2precond:
            yield "   (and"
            indent += "  "
        yield self.precondition.pddl_str(indent)
        if self.replan_condition:
            if not replan2precond:
                yield " :replan"
            yield self.replan_condition.pddl_str(indent)
            if replan2precond:
                yield "    )"
        yield " :effect (and"
        for effect in self.effects:
            yield effect.pddl_str(indent="    ")
#             if self.name == "test":
#                 print
#                 print effect.literal
#                 print effect.pddl_str(indent="    ")
#                 asdf
        yield "))"
    def uniquify_variables(self):
        self.type_map = dict([(par.name, par.type) for par in self.parameters])
        self.precondition = self.precondition.uniquify_variables(self.type_map)
        for effect in self.effects:
            effect.uniquify_variables(self.type_map)
    def unary_actions(self):
        # TODO: An neue Effect-Repr?sentation anpassen.
        result = []
        for i, effect in enumerate(self.effects):
            unary_action = copy.copy(self)
            unary_action.name += "@%d" % i
            if isinstance(effect, effects.UniversalEffect):
                # Careful: Create a new parameter list, the old one might be shared.
                unary_action.parameters = unary_action.parameters + effect.parameters
                effect = effect.effect
            if isinstance(effect, effects.ConditionalEffect):
                unary_action.precondition = conditions.Conjunction([unary_action.precondition,
                effect.condition]).simplified()
                effect = effect.effect
            unary_action.effects = [effect]
            result.append(unary_action)
        return result
    def relaxed(self):
        new_effects = []
        for eff in self.effects:
            relaxed_eff = eff.relaxed()
            if relaxed_eff:
                new_effects.append(relaxed_eff)
        return Action(self.name, self.parameters,
        self.precondition.relaxed().simplified(),
        new_effects)
    def untyped(self):
        # We do not actually remove the types from the parameter lists,
        # just additionally incorporate them into the conditions.
        # Maybe not very nice.
        result = copy.copy(self)
        parameter_atoms = [par.to_untyped_strips() for par in self.parameters]
        new_precondition = self.precondition.untyped()
        result.precondition = conditions.Conjunction(parameter_atoms + [new_precondition])
        result.effects = [eff.untyped() for eff in self.effects]
        return result
    def untyped_strips_preconditions(self):
        # Used in instantiator for converting unary actions into prolog rules.
        return [par.to_untyped_strips() for par in self.parameters] + \
           self.precondition.to_untyped_strips()

    def ground_mapl_str(self, constants, keep_assertions=True):
        params = self.agents + self.parameters + self.variables
        def gen():
            replan2precond = self.replan_condition and not keep_assertions
            yield "\n(:action %s" % self.name
            yield " :parameters (%s)" % " ".join(map(types.mapl_str, params))
            indent = "    "
            yield " :precondition"
            if replan2precond:
                yield "   (and"
                indent += "  "
            yield self.precondition.mapl_str(indent)
            if self.replan_condition:
                if not replan2precond:
                    yield " :replan"
                yield self.replan_condition.mapl_str(indent)
                if replan2precond:
                    yield "    )"
            yield " :effect (and"
            for effect in self.effects:
                yield effect.mapl_str(indent="    ")    
            yield "))"
        s = "\n".join(gen())
        assert len(params) == len(constants)
        param_names = [param.name for param in params]
        replacements = dict(zip(param_names, constants))
        result = utils.multiple_replace(s, replacements)
        return result
        
    def grounded_read_write_data(self, constants):
        """ Instantiate MAPL operator (only STRIPS subset currently
        supported).  Return a list of preconditions and a list of effects,
        both as state variable assignments.  Treat replan condtions as
        normal preconditions."""
        
        params = self.agents + self.parameters + self.variables
        assert len(params) == len(constants)
        param_names = [param.name for param in params]
        replacements = dict(zip(param_names, constants))
        pre_conjuncts = self.precondition.as_svar_assignments(replacements)
        if self.replan_condition:
            pre_conjuncts.extend(self.replan_condition.as_svar_assignments(replacements))
        pre_conjuncts = dict(pre_conjuncts)
        eff_conjuncts = dict(eff.literal.as_svar_assignment(replacements) for eff in self.effects)
        return pre_conjuncts, eff_conjuncts
        
    def eliminate_ex_quantifiers(self):
        def elim_ex_quant(condition):
            if isinstance(condition, (conditions.UniversalCondition, conditions.Disjunction)):
                if condition.has_existential_part():
                    print "Warning: couldn't remove exists quantifier inside universal quantifier or disjunction"
                return
            new_parts = ()
            was_changed = False
            for i, part in enumerate(condition.parts):
                if isinstance(part, conditions.ExistentialCondition):
                    new_parts += part.parts
                    self.parameters += part.parameters
                    was_changed = True
                else:
                    new_parts += (part,)
                    elim_ex_quant(part)
            if was_changed:
                condition.parts = new_parts

        if self.precondition:
            elim_ex_quant(self.precondition)
            
    
    def instantiate(self, var_mapping, init_facts, fluent_facts, objects_by_type):
        """Return a PropositionalAction which corresponds to the instantiation of
        this action with the arguments in var_mapping. Only fluent parts of the
        conditions (those in fluent_facts) are included. init_facts are evaluated
        whilte instantiating.
        Precondition and effect conditions must be normalized for this to work.
        Returns None if var_mapping does not correspond to a valid instantiation
        (because it has impossible preconditions or an empty effect list.)"""
        arg_list = [var_mapping[par.name] for par in self.parameters]
        name = "(%s %s)" % (self.name, " ".join(arg_list))
        
        precondition = []
        try:
            self.precondition.instantiate(var_mapping, init_facts,
            fluent_facts, precondition)
        except conditions.Impossible:
            return None
        effects = []
        for eff in self.effects:
            eff.instantiate(var_mapping, init_facts, fluent_facts,
            objects_by_type, effects)
        if effects:
            return PropositionalAction(name, precondition, effects)
        else:
            return None


class PropositionalAction:
    def __init__(self, name, precondition, effects):
        self.name = name
        self.precondition = precondition
        self.add_effects = []
        self.del_effects = []
        for (condition, effect) in effects:
            if effect.negated:
                self.del_effects.append((condition, effect.negate()))
            else:
                self.add_effects.append((condition, effect))
    def dump(self):
        print self.name
        for fact in self.precondition:
            print "PRE: %s" % fact
        for cond, fact in self.add_effects:
            print "ADD: %s -> %s" % (", ".join(map(str, cond)), fact)
        for cond, fact in self.del_effects:
            print "DEL: %s -> %s" % (", ".join(map(str, cond)), fact)
