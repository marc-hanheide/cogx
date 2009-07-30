#! /usr/bin/env python
# -*- coding: latin-1 -*-

import sys
from string import Template

import parser
import actions
import conditions
#import predicates
import effects
import types
from constants import *
import utils
import state

class SensorModel(actions.Action):
    def __init__(self, name, parameters, precondition, replan, negcondition, svar, agents=[], variables=[]):
        self.name = name
        self.agents = agents
        self.parameters = parameters
        self.variables = variables
        self.precondition = precondition
        self.replan_condition = replan
        self.negcondition = negcondition
        self.svar = svar
        self.effects = [] # to make uniquify happy
        self.uniquify_variables()
        self.eliminate_ex_quantifiers()
        self.params = self.agents + self.parameters + self.variables
        self.posmap = dict((p.name, i) for i,p in enumerate(self.params))

    @staticmethod
    def parse(alist):
        expected_length = 16
        if ":precondition" not in alist:
            expected_length -= 2
        if ":replan" not in alist:
            expected_length -= 2
        if ":negcondition" not in alist:
            expected_length -= 2
        if ":parameters" not in alist:
            alist[4:4] = [":parameters", []]
        if ":variables" not in alist:
            alist[6:6] = [":variables", []]
        assert len(alist)==expected_length, "len only %s for %s" % (len(alist), alist)
        iterator = iter(alist)
        atype = iterator.next()
        assert atype == ":sensor"
        name = iterator.next()
        assert iterator.next() == ":agent"
        agents = types.parse_typed_list(iterator.next(), only_variables=True)
        precondition = conditions.Conjunction([])
        replan = None
        negcondition = None
        tag = iterator.next()
        while tag != ":sense":
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
            elif tag == ":negcondition":
                negcondition = conditions.parse_condition(iterator.next())
            tag = iterator.next()
        assert tag == ":sense"
        svar = iterator.next()
        svars = []
        effects.parse_effects(svar, svars)
        for rest in iterator:
            assert False, rest
        assert len(svars) == 1
        svar = svars[0].literal.mapsim_svar()
        a = SensorModel(name, parameters, precondition, replan, negcondition, svar, agents=agents, variables=variables)
#         print "sensordump:"
#         a.dump_pddl(sys.stdout, keep_assertions=True)
        return a

    def instantiate_neg_condition(self, params, mark_negation=True):
        nc = self.negcondition
        if isinstance(nc, conditions.Conjunction):
            assert len(nc.parts)==1, "negconditions may only consists of a single literal (currently)..."
            nc = nc.parts[0]
        assert isinstance(nc, conditions.Literal)
        assert nc.is_negated(), "negconditions must negate an expectation, ie must be a negative literal (currently)..."
        svar = nc.mapsim_svar()
        v2p = dict((v.name,p) for (v,p) in zip(self.params,params))
        new_name = str(svar)
        if mark_negation:
            new_name = "!" + new_name
        s = utils.multiple_replace(new_name, v2p).split()
        svar, val = state.StateVariable(s[:-1]), s[-1]
        return svar, val

    def dump_mapl(self, stream):
        print >> stream, "\n(:sensor %s" % self.name
        print >> stream, " :agent (%s)" % " ".join(map(types.pddl_str, self.parameters))
        print >> stream, " :parameters (%s)" % " ".join(map(types.pddl_str, self.parameters))
        print >> stream, " :precondition"
        self.precondition.dump_pddl(stream, indent="    ")
        print >> stream, " :negcondition"
        self.negcondition.dump_pddl(stream, indent="    ")
        print >> stream, " :sense (%s)" % self.svar.pddl_str()
        print >> stream, "))"

    def dump_pddl(self, stream, keep_assertions=True):
        print >> stream, "\n".join(self.pddl_gen(keep_assertions))

    def pddl_gen(self, keep_assertions=True):
        actions = [self.build_sensing_action(negative=False)]
        if self.negcondition:
            actions.append(self.build_sensing_action(negative=True))
        for action in actions:
            for line in action.pddl_gen(keep_assertions):
                yield line
        
    def build_sensing_action(self, negative):
        pref = PDDL_SENSOR_PREFIX
        precondition = self.precondition.make_conjunction()
        names = [a.name for a in self.agents]
        eff_str = "(kval (%s) %s)" % (" ".join(names), self.svar.as_mapl_str(parens=True))
        alist = parser.parse_nested_list(eff_str)
        effs = []
        eff = effects.parse_effects(alist, effs)
        if negative:
            pref = PDDL_NEG_SENSOR_PREFIX
        name = pref + self.name
        action =  actions.Action(name, self.params, precondition, effs, agents=self.agents, replan=self.replan_condition, variables=self.variables)
        if negative and self.negcondition:
            negcondition = self.negcondition.make_conjunction()
            parts = list(action.precondition.parts)
            parts.extend(negcondition.parts)
            action.precondition.parts = parts
        return action

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
        pre_conjuncts = dict(pre_conjuncts)

        k_var = conditions.Atom(K_PREFIX+PREFIX_SEP+self.svar.predicate, (constants[0],)+self.svar.args)
        eff_conjuncts = dict([k_var.as_svar_assignment(replacements)])

        return pre_conjuncts, eff_conjuncts
        
    def analyze_sensor_command(self, scommand):
        # find out which state var is sensed by this sensor (a little hackish)
        scommand_list = scommand.split()
        svar = self.svar
        svar_name = svar.name
        orig_args = svar.args
        new_args = [svar_name] + [scommand_list[self.posmap[arg]+1] for arg in orig_args]
        svar = state.StateVariable(new_args)
        return svar
            
sensor_template = Template("""
(:action $sensorpref$name
 :parameters ($paramdecl)
 :precondition
    (and $precond
    )
 :replan (and $replan)
 :effect (and $eff)
)
""")

neg_sensor_template = Template("""
(:action $negpref$name
 :parameters ($paramdecl)
 :precondition
    (and $precond$negcond
    )
 :replan (and $replan)
 :effect (and $eff)
)
""")

